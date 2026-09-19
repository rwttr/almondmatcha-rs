"""Lane detection pipeline, ported from the ROS 2 node's
`lane_detector.py` and the per-frame state that used to live in
`lane_detection_node.py`.

The ROS 2 tree those came from has been removed from this branch, but the
detector itself is vendored verbatim at `perception/tests/oracle/` as the
frozen parity oracle — so every path this docstring cites is still readable,
and `tests/test_lane_parity.py` still compares this file against the real
original rather than against a description of it.

# FIDELITY — read this before touching a constant or a line of algorithm code

This pipeline is field-tuned, not derived from first principles, and the
white-segmentation approach (LAB `L*`, not chroma) was corrected and
validated on real D415 footage in commit `6a49552` after a chroma-based
version silently misclassified ~45% of a frame as "white" on that sensor.
Repeating that validation costs a field session, not a code review.

**Every function from `segment_track_colors` through `compute_lane_params`
is a behaviour-preserving port**: same algorithm, same constants, same order
of operations, same default parameter values (`LaneConfig` below mirrors
`vision_navigation.config.LaneDetectionConfig` field-for-field). Type hints
and module reorganisation are the only changes. `tests/test_lane_parity.py`
runs both the original and this port over the same synthetic frames and
asserts the outputs match — that test, not code review, is what proves this
file didn't drift.

Do not "clean up" the segmentation thresholds, the ROI geometry, the sliding
window search, or the polyfit step. If one of them looks wrong, it might be —
see `docs/RUST_REWRITE_PLAN.md` §10 and report it upstream instead of
changing it here.

# What is intentionally NOT ported

- `plot_lane_lines` (pure `cv2.imshow` debug visualisation): no numeric
  effect on `process_frame`'s return value in the original either — it was
  called for its side effect only, after the values to return were already
  computed. Dropped as ROS/GUI plumbing, per the porting brief.
- The `lane_detection_node.py` stale-frame check (`MAX_FRAME_AGE_SEC`,
  comparing a capture timestamp across a DDS hop): that check existed
  because the camera and detector were two ROS processes connected by a
  queued topic, so a frame already in flight could go stale before this
  node acted on it. Camera capture and detection now run in the same loop
  iteration of one process (see `docs/RUST_REWRITE_PLAN.md` §1.1) — there is
  no queue and nothing for a frame to go stale in. This is a consequence of
  the architecture change, not a relaxation of the detector's tolerance for
  bad input.

# The one deliberate behaviour change: curvature to 1/m at the source

`compute_lane_params` still returns `curvature` as the raw polyfit
coefficient `A` in BEV pixels (1/px) — unchanged, because that keeps
`test_lane_parity.py` comparing like with like against the original.
`LaneDetector.detect` (below), the boundary of this module, is where the
port diverges on purpose: it converts to a physical 1/m curvature before
building the outgoing `LaneMeasurement`, instead of shipping pixels and
leaving every consumer to fold in `BEV_PX_PER_M` itself (which is what the
ROS 2 system did — see `docs/RUST_REWRITE_PLAN.md` §3.3 and
`docs/CONTROL_LAW.md` §1.7, which flags this as an asymmetry against `b`,
which *was* converted before publication).

Derivation, matching `docs/CONTROL_LAW.md`'s `R = 1/(2*A*S)`:

    The fit is x_px = A*y_px^2 + B*y_px + C in the BEV canvas, which is
    isotropic (`perspective_transform` builds it with the same px/metre
    scale on both axes -- see that function's docstring). So x_m = x_px/S,
    y_m = y_px/S with S = BEV_PX_PER_M:

        S*x_m = A*(S*y_m)^2 + B*(S*y_m) + C
        x_m   = (A*S)*y_m^2 + B*y_m + C/S

    The metric parabola coefficient is therefore A_m = A*S (units 1/m,
    since x_m is metres and y_m^2 is metres^2). For a parabola x = A_m*y^2,
    curvature kappa = x'' / (1+x'^2)^1.5 ~= 2*A_m for the small slopes this
    pipeline operates at (the same small-angle assumption the EKF and the
    static-gain feedforward already make -- see RUST_REWRITE_PLAN.md §2.2
    and CONTROL_LAW.md §1.7's `k_ff` derivation, which needs this identical
    factor of `2*S`). So:

        curvature_inv_m = kappa = 2*A_m = 2*A*S

    R = 1/kappa = 1/(2*A*S), matching CONTROL_LAW.md exactly. This is also
    bit-for-bit the same formula `rover_kinematic_control_node.py` already
    computed downstream (`curvature_m_ema = 2.0 * bev_px_per_m *
    curvature_ema`) -- moving it here doesn't change the arithmetic, only
    where it happens: once, at the source, instead of independently in
    every consumer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


class LaneConfig:
    """Mirrors `vision_navigation.config.LaneDetectionConfig` field-for-field.
    Values are field-tuned or measured constants (see `RUST_REWRITE_PLAN.md`
    §10) — do not change one without the field session that would justify it.
    """

    # ===== Color segmentation =====
    SEGMENTATION_MORPH_KERNEL_PX = 9

    # ===== Line shape filter (bird's-eye view) =====
    LINE_MAX_WIDTH_M = 0.10
    LINE_MIN_DEPTH_FRAC = 0.35
    LINE_MIN_ASPECT = 1.5
    MIN_LINE_COMPONENT_AREA_PX = 15

    # ===== Lane finding =====
    MIN_LANE_PIXELS = 50
    SLIDING_WINDOWS = 9
    MIN_WINDOW_PIXELS = 50
    WINDOW_MARGIN = 40
    SEARCH_BAND_PX = 45.0
    MAX_ABS_B_M = 0.50

    # ===== Perspective transform =====
    ROI_BASE_POINTS = np.float32([
        [39, 458],
        [1241, 458],
        [915, 270],
        [365, 270],
    ])
    ROI_BASE_WIDTH = 1280.0
    ROI_BASE_HEIGHT = 720.0
    CROP_MARGIN_PX = 20.0

    # ===== Bird's-eye canvas (metric) =====
    BEV_PX_PER_M = 200.0
    BEV_WIDTH_PX = 720
    BEV_HEIGHT_PX = 340

    LINE_MAX_WIDTH_PX = LINE_MAX_WIDTH_M * BEV_PX_PER_M

    # ===== Lookahead geometry (camera calibration, not a tuning knob) =====
    # y=0 in compute_lane_params's fit frame is the near edge of the ROI --
    # 1.22 m ahead of the front axle with the shipped camera mount. See
    # lane_detector.py's compute_lane_params docstring in the original tree.
    LOOKAHEAD_M = 1.22


# ================================
# 1. Adaptive color segmentation
# ================================
def segment_track_colors(
    frame_bgr: np.ndarray,
    morph_kernel_px: int = LaneConfig.SEGMENTATION_MORPH_KERNEL_PX,
) -> Tuple[np.ndarray, np.ndarray]:
    """Split the cropped ROI frame into red track surface / white line
    candidates / background, using Otsu's method to re-fit both splits every
    frame instead of a fixed threshold.

    Red uses LAB `a*` (green<->red axis). White uses LAB `L*` (lightness),
    Otsu-thresholded only among pixels already on/near the red blob. `L*`
    was chosen over chroma distance from neutral after validating against
    real D415 footage (`6a49552`): on this sensor the `a*`/`b*` channels
    span too narrow a range for Otsu to find a real bimodal chroma split,
    misclassifying ~45% of the frame (most of the plain track) as white
    candidate, where `L*` gives a clean bimodal split (track ~110-115,
    paint >~170). Do not revert this without re-validating on hardware.

    Returns:
        `(red_track_mask, white_mask)`, both uint8 0/1, same size as
        `frame_bgr`. `white_mask` is not yet shape-filtered — callers doing
        lane finding warp it to BEV and pass it through
        `filter_line_candidates_bev` first.
    """
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    L = lab[:, :, 0]

    A_u8 = lab[:, :, 1].astype(np.uint8)
    _, red_otsu = cv2.threshold(A_u8, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    red_raw = red_otsu.astype(np.uint8)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_kernel_px, morph_kernel_px))
    red_closed = cv2.morphologyEx(red_raw, cv2.MORPH_CLOSE, kernel)
    red_closed = cv2.morphologyEx(red_closed, cv2.MORPH_OPEN, kernel)

    # White vs. red on the D415 is a BRIGHTNESS distinction (see module and
    # function docstrings for the measurements this replaced a chroma-based
    # split with). Restricted to near_track so a bright unrelated patch
    # elsewhere in the ROI can't shift the white cut point.
    near_track = cv2.dilate(red_closed, kernel, iterations=2).astype(bool)
    white_raw = np.zeros_like(red_raw)
    if np.count_nonzero(near_track) >= 50:
        L_u8 = np.clip(L, 0, 255).astype(np.uint8)
        l_samples = L_u8[near_track].reshape(-1, 1)
        # THRESH_BINARY (not _INV): Otsu's cut separates low-L (red track,
        # below the cut) from high-L (white paint, above it); take the high
        # side, same sense as the red_otsu split above.
        _, white_otsu = cv2.threshold(l_samples, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        white_raw[near_track] = white_otsu.reshape(-1)

    corridor_raw = (red_closed.astype(bool) | white_raw.astype(bool)).astype(np.uint8)
    corridor_closed = cv2.morphologyEx(corridor_raw, cv2.MORPH_CLOSE, kernel)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(corridor_closed, connectivity=8)
    if num_labels > 1:
        largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        corridor_mask = (labels == largest_label).astype(np.uint8)
    else:
        corridor_mask = corridor_closed

    red_track_mask = (corridor_mask.astype(bool) & red_closed.astype(bool) & ~white_raw.astype(bool)).astype(np.uint8)
    white_mask = (corridor_mask & white_raw).astype(np.uint8)

    return red_track_mask, white_mask


def filter_line_candidates_bev(
    warped_white: np.ndarray,
    bev_height: int,
    max_width_px: float = LaneConfig.LINE_MAX_WIDTH_PX,
    min_height_frac: float = LaneConfig.LINE_MIN_DEPTH_FRAC,
    min_aspect: float = LaneConfig.LINE_MIN_ASPECT,
    min_area_px: int = LaneConfig.MIN_LINE_COMPONENT_AREA_PX,
) -> np.ndarray:
    """Keep only white-candidate blobs shaped like a painted line; reject
    broad/short blobs such as a sun-glare patch on the track.

    Must run AFTER warping to BEV, not on the raw camera frame — see the
    original `lane_detector.py` docstring for why this is a geometric test
    that only holds in the metric top-down view.
    """
    # Pass 1 -- prune per-ROW, not per-component (see original docstring:
    # a per-row run-length opening erases a too-wide run in its own row
    # without destroying a genuinely thin row from the same connected blob).
    kw = int(max_width_px) + 1
    row_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kw, 1))
    too_wide = cv2.morphologyEx(warped_white, cv2.MORPH_OPEN, row_kernel)
    thin_only = (warped_white.astype(bool) & ~too_wide.astype(bool)).astype(np.uint8)

    # Pass 2 -- keep components that persist over enough of the ROI's depth
    # and are tall/narrow overall.
    n, labels, stats, _ = cv2.connectedComponentsWithStats(thin_only, connectivity=8)
    keep = np.zeros_like(warped_white)
    for label in range(1, n):
        _, _, bw, bh, area = stats[label]
        if area < min_area_px:
            continue
        aspect = bh / max(bw, 1)
        if bh >= min_height_frac * bev_height and aspect >= min_aspect:
            keep[labels == label] = 1
    return keep


# ================================
# 2. ROI scaling & cropping
# ================================
def get_scaled_roi_points(
    frame_width: int,
    frame_height: int,
    roi_base_points: np.ndarray,
    roi_base_width: float = LaneConfig.ROI_BASE_WIDTH,
    roi_base_height: float = LaneConfig.ROI_BASE_HEIGHT,
) -> np.ndarray:
    """Scale ROI corner points (authored at `roi_base_width x
    roi_base_height`) to the actual frame size."""
    roi_base_points = np.float32(roi_base_points)
    sx = frame_width / roi_base_width
    sy = frame_height / roi_base_height
    return np.float32([[p[0] * sx, p[1] * sy] for p in roi_base_points])


def crop_to_roi(
    frame_bgr: np.ndarray,
    roi_points: np.ndarray,
    margin_x: float = 0.0,
    margin_y: float = 0.0,
) -> Tuple[np.ndarray, int, int]:
    """Crop a frame to the bounding box of the ROI polygon (plus margin).

    Runs before the expensive per-pixel preprocessing so CPU cost scales
    with the ROI area, without reducing pixel density inside it.

    Returns:
        `(cropped_frame, x_offset, y_offset)` — offsets are the crop's
        top-left corner in the original frame's coordinate system.
    """
    frame_height, frame_width = frame_bgr.shape[:2]

    x_min = int(np.clip(np.min(roi_points[:, 0]) - margin_x, 0, frame_width))
    x_max = int(np.clip(np.max(roi_points[:, 0]) + margin_x, 0, frame_width))
    y_min = int(np.clip(np.min(roi_points[:, 1]) - margin_y, 0, frame_height))
    y_max = int(np.clip(np.max(roi_points[:, 1]) + margin_y, 0, frame_height))

    cropped_frame = frame_bgr[y_min:y_max, x_min:x_max]
    return cropped_frame, x_min, y_min


# ================================
# 3. Perspective transform
# ================================
def perspective_transform(
    binary: np.ndarray,
    frame_size: Tuple[int, int],
    roi_points: np.ndarray,
    bev_size: Optional[Tuple[int, int]] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Warp to a fixed-size bird's-eye canvas, independent of the crop size.

    A fixed canvas with the same px/metre scale on both axes is what makes
    `theta` a real heading angle, `b` a real cross-track offset, and
    `curvature` a real 1/m arc once converted — see this module's docstring
    for the curvature derivation, which depends on this isotropy.

    Returns:
        `(warped, M, M_inv)`.
    """
    w, h = frame_size if bev_size is None else bev_size

    dst = np.float32([
        [w * 0.25, h * 1.0],
        [w * 0.75, h * 1.0],
        [w * 0.75, h * 0.0],
        [w * 0.25, h * 0.0],
    ])

    M = cv2.getPerspectiveTransform(roi_points, dst)
    M_inv = cv2.getPerspectiveTransform(dst, roi_points)

    # INTER_NEAREST: `binary` is a 0/1 mask feeding connected-component
    # analysis downstream. Linear interpolation would blend 0/1 neighbours
    # into fractional values that truncate to 0, thinning or dropping thin
    # lines right where the shape filter most needs true width preserved.
    warped = cv2.warpPerspective(binary, M, (w, h), flags=cv2.INTER_NEAREST)

    return warped, M, M_inv


# ================================
# 4. Lane finding (single center line)
# ================================
def find_center_line(
    binary_warped: np.ndarray,
    num_windows: int = 9,
    window_margin: int = 100,
    min_pixels: int = 50,
    search_center: Optional[float] = None,
    search_band: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Sliding-window lane pixel search, bounded to a band around
    `search_center` so the tracker stays on the line it was already
    following instead of re-picking whichever of several parallel painted
    lines carries the most pixels this frame.

    Returns:
        `(x_coords, y_coords)` of detected lane pixels.
    """
    height, width = binary_warped.shape[:2]

    if search_center is None or not np.isfinite(search_center):
        search_center = width / 2.0
    search_center = float(np.clip(search_center, 0, width - 1))

    if search_band is None or search_band <= 0:
        band_low, band_high = 0, width
    else:
        band_low = int(max(0, np.floor(search_center - search_band)))
        band_high = int(min(width, np.ceil(search_center + search_band) + 1))
        if band_high <= band_low:
            band_low, band_high = 0, width

    histogram = np.sum(binary_warped[height // 2:, band_low:band_high], axis=0)
    if histogram.size == 0 or histogram.max() == 0:
        # Nothing in the band. Hold the expected position; the windows will
        # come back empty and min_lane_pixels reports "not detected" rather
        # than seeding the search from an arbitrary column.
        base_x = int(round(search_center))
    else:
        base_x = int(np.argmax(histogram)) + band_low

    window_height = binary_warped.shape[0] // num_windows
    nonzero = binary_warped.nonzero()
    nonzero_y, nonzero_x = np.array(nonzero[0]), np.array(nonzero[1])

    current_x = base_x
    lane_indices_list = []

    for window_idx in range(num_windows):
        win_y_low = binary_warped.shape[0] - (window_idx + 1) * window_height
        win_y_high = binary_warped.shape[0] - window_idx * window_height
        win_x_low = current_x - window_margin
        win_x_high = current_x + window_margin

        good_indices = (
            (nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
            (nonzero_x >= win_x_low) & (nonzero_x < win_x_high)
        ).nonzero()[0]
        lane_indices_list.append(good_indices)

        if len(good_indices) > min_pixels:
            current_x = int(np.mean(nonzero_x[good_indices]))

    lane_indices = np.concatenate(lane_indices_list) if lane_indices_list else np.array([], dtype=int)
    x_coords = nonzero_x[lane_indices]
    y_coords = nonzero_y[lane_indices]

    return x_coords, y_coords


# ================================
# 5. Fit single line & params
# ================================
def compute_lane_params(
    binary_warped: np.ndarray,
    sliding_windows: int = LaneConfig.SLIDING_WINDOWS,
    window_margin: int = LaneConfig.WINDOW_MARGIN,
    min_window_pixels: int = LaneConfig.MIN_WINDOW_PIXELS,
    min_lane_pixels: int = LaneConfig.MIN_LANE_PIXELS,
    search_center: Optional[float] = None,
    search_band: float = LaneConfig.SEARCH_BAND_PX,
) -> dict:
    """Fit `x = A*y^2 + B*y + C` in a frame shifted so y=0 is the canvas
    bottom row (the fit's lookahead point) and x=0 is the canvas horizontal
    centre — so the fit coefficients themselves are the quantities needed:
    `B` is the heading slope, `C` is the lateral offset, with no separate
    polynomial-evaluation step afterward.

    Returns:
        `{"curvature": A (1/px, NaN if undetected), "theta": degrees,
        "b": metres, "detected": bool}`. `curvature` is deliberately left in
        BEV pixels here, unconverted — see this module's docstring for
        where and why the 1/m conversion happens instead.
    """
    x_coords, y_coords = find_center_line(
        binary_warped,
        num_windows=sliding_windows,
        window_margin=window_margin,
        min_pixels=min_window_pixels,
        search_center=search_center,
        search_band=search_band,
    )
    height, width = binary_warped.shape[:2]

    result = {"curvature": np.nan, "theta": np.nan, "b": np.nan, "detected": False}

    if len(x_coords) >= min_lane_pixels:
        y_shifted = y_coords.astype(np.float64) - height
        x_shifted = x_coords.astype(np.float64) - (width / 2.0)

        coeff_a, coeff_b, coeff_c = np.polyfit(y_shifted, x_shifted, 2)

        # theta needs no unit conversion -- arctan of a dimensionless slope
        # is already a real angle. b is converted to metres here, at the
        # single point it is produced, same as the original.
        theta = np.degrees(np.arctan(coeff_b))
        b_centered = coeff_c / LaneConfig.BEV_PX_PER_M

        result = {
            "curvature": coeff_a,
            "theta": theta,
            "b": b_centered,
            "detected": True,
        }

    return result


# ================================
# 6. Full pipeline (behaviour-preserving; matches lane_detector.process_frame)
# ================================
def process_frame(
    frame_bgr: np.ndarray,
    roi_base_points: Optional[np.ndarray] = None,
    roi_base_width: float = LaneConfig.ROI_BASE_WIDTH,
    roi_base_height: float = LaneConfig.ROI_BASE_HEIGHT,
    crop_margin_px: float = LaneConfig.CROP_MARGIN_PX,
    sliding_windows: int = LaneConfig.SLIDING_WINDOWS,
    window_margin: int = LaneConfig.WINDOW_MARGIN,
    min_window_pixels: int = LaneConfig.MIN_WINDOW_PIXELS,
    min_lane_pixels: int = LaneConfig.MIN_LANE_PIXELS,
    bev_width: int = LaneConfig.BEV_WIDTH_PX,
    bev_height: int = LaneConfig.BEV_HEIGHT_PX,
    search_center: Optional[float] = None,
    search_band: float = LaneConfig.SEARCH_BAND_PX,
) -> Tuple[float, float, float, bool]:
    """Crop -> segment colors -> transform to BEV -> shape-filter -> detect
    -> compute params. Bit-exact port of `lane_detector.process_frame`
    (minus the `plot_lane_lines` visualisation call, which had no effect on
    this return value — see module docstring).

    Returns:
        `(curvature, theta, b, detected)` — curvature in BEV pixels (1/px),
        theta in degrees, b in metres, exactly as the original.
    """
    if roi_base_points is None:
        roi_base_points = LaneConfig.ROI_BASE_POINTS

    frame_width, frame_height = frame_bgr.shape[1], frame_bgr.shape[0]

    roi_points_full = get_scaled_roi_points(
        frame_width, frame_height, roi_base_points, roi_base_width, roi_base_height
    )
    margin_x = crop_margin_px * (frame_width / roi_base_width)
    margin_y = crop_margin_px * (frame_height / roi_base_height)

    cropped_frame, x_offset, y_offset = crop_to_roi(frame_bgr, roi_points_full, margin_x, margin_y)
    crop_width, crop_height = cropped_frame.shape[1], cropped_frame.shape[0]

    roi_points_cropped = roi_points_full - np.float32([x_offset, y_offset])

    _red_track_mask, white_candidates = segment_track_colors(cropped_frame)
    warped_white, _M, _M_inv = perspective_transform(
        white_candidates, (crop_width, crop_height), roi_points_cropped, bev_size=(bev_width, bev_height)
    )
    warped = filter_line_candidates_bev(warped_white, bev_height=bev_height)
    params = compute_lane_params(
        warped,
        sliding_windows=sliding_windows,
        window_margin=window_margin,
        min_window_pixels=min_window_pixels,
        min_lane_pixels=min_lane_pixels,
        search_center=search_center,
        search_band=search_band,
    )

    return params["curvature"], params["theta"], params["b"], params["detected"]


# ===========================================================================
# 7. Per-frame tracking state, ported from lane_detection_node.py
# ===========================================================================

@dataclass
class LaneResult:
    """Physical-unit lane geometry, ready to go on the wire. See this
    module's docstring for the curvature conversion."""

    curvature_inv_m: float
    heading_err_rad: float
    cross_track_m: float
    valid: bool


class LaneDetector:
    """Owns the frame-to-frame state `lane_detection_node.py` kept as node
    members: the tracked search column, and the implausible-lock rejection
    that guards it.

    Not just `process_frame` wrapped in a loop — the plausibility check and
    the search-center seed are part of what made this detector track
    correctly on a three-line track in the field, and both are ported
    unchanged from `lane_detection_node.py`'s `_on_rgb_frame`.
    """

    def __init__(self, config: type = LaneConfig) -> None:
        self._config = config
        # Column the lane was found at last frame, warped-canvas pixels.
        # None = search the canvas centre (startup, or after a reset).
        self._search_center: Optional[float] = None

    def reset(self) -> None:
        self._search_center = None

    def detect(self, frame_bgr: np.ndarray) -> LaneResult:
        """Run one frame through the pipeline and return physical-unit
        geometry, with the same implausible-lock rejection and search-seed
        tracking `lane_detection_node.py` applied around `process_frame`.
        """
        cfg = self._config
        curvature_px, theta_deg, b_m, detected = process_frame(
            frame_bgr,
            search_center=self._search_center,
        )

        # Reject an implausible lock: the per-frame search band stops jumps
        # but permits a steady walk off the line being followed (or onto
        # off-track content in the canvas margin) while still reporting
        # "detected" every frame. See lane_detection_node.py's identical
        # check and the field log that motivated MAX_ABS_B_M.
        if detected and math.isfinite(b_m) and abs(b_m) > cfg.MAX_ABS_B_M:
            detected = False
            curvature_px = theta_deg = b_m = float("nan")

        # Track the lane across frames: `b` is the fitted offset at the
        # canvas bottom row, exactly where the next frame's first search
        # window starts, so it is the correct seed with no re-derivation.
        # Converted back to pixels here (the one place that needs canvas
        # coordinates) since `b` itself is metres. Dropped, not held, on a
        # lost frame -- a stale seed after a gap is more likely wrong than
        # the centre.
        if detected and b_m is not None and math.isfinite(b_m):
            self._search_center = (cfg.BEV_WIDTH_PX / 2.0) + (b_m * cfg.BEV_PX_PER_M)
        else:
            self._search_center = None

        if not detected:
            return LaneResult(curvature_inv_m=0.0, heading_err_rad=0.0, cross_track_m=0.0, valid=False)

        # ---- The one deliberate behaviour change: convert at the source ----
        # See module docstring for the derivation. b is already metres from
        # compute_lane_params; theta is degrees and needs only unit
        # conversion, not a scale conversion.
        curvature_inv_m = 2.0 * curvature_px * cfg.BEV_PX_PER_M
        heading_err_rad = math.radians(theta_deg)

        return LaneResult(
            curvature_inv_m=curvature_inv_m,
            heading_err_rad=heading_err_rad,
            cross_track_m=b_m,
            valid=True,
        )
