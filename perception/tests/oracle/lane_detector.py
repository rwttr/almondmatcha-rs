"""
lane_detector.py
Workspace:  ws_jetson  |  Package: vision_navigation

Lane Detection Image Processing Pipeline

Provides image processing functions for detecting lane markers in RGB frames.
Includes preprocessing, perspective transformation, and lane fitting algorithms.

Author: AlmondMatcha Rover Team
Date: February 27, 2026
"""

import cv2
import numpy as np
from typing import Tuple

from .config import LaneDetectionConfig

# ================================
# 1. Adaptive color segmentation
# ================================
def segment_track_colors(
    frame_bgr: np.ndarray,
    morph_kernel_px: int = LaneDetectionConfig.SEGMENTATION_MORPH_KERNEL_PX,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Split the cropped ROI frame into red track surface / white line
    candidates / background, using Otsu's method to re-fit both splits every
    frame instead of a fixed threshold.

    A fixed cutoff tuned against one camera's color pipeline (its white
    balance, saturation curve, JPEG compression) does not transfer to a
    different sensor looking at the same physical track. Confirmed against a
    phone photo of the actual track: a sun-glare patch on the red surface
    reads almost identically to real white paint in HSV brightness and
    saturation (both ~S=55-60, V=185-195), so a fixed HSV threshold cannot
    tell them apart regardless of what the cutoff is set to. Otsu instead
    finds wherever red-vs-not-red and white-vs-not splits fall in THIS
    frame's own histogram, which at least tracks whatever global exposure
    and white balance the current camera/lighting produced -- it does not by
    itself fix local color collisions like the glare patch (nothing operating
    on a single pixel's color can); that is what filter_line_candidates_bev
    is for.

    Red uses LAB a* (the green<->red axis): track red sits well above grass,
    water, and concrete on this axis regardless of overall brightness, which
    plain HSV saturation does not guarantee (saturation is measured relative
    to brightness, so a bright-but-still-reddish pixel can misleadingly read
    as "desaturated"). White uses LAB L* (lightness), Otsu-thresholded only
    among pixels already on/near the red blob -- restricting the histogram
    this way keeps sunlit grass or a dirt-bank highlight elsewhere in the ROI
    from shifting the white cut point.

    L* was chosen over chroma distance from neutral (sqrt((a*-128)^2 +
    (b*-128)^2)) after validating against real D415 footage of the actual
    track (see dev/ captures, 1280x720 @ 30 FPS) -- the earlier chroma
    approach was designed and tuned only against a phone photo, and on the
    D415 it fails outright: the ROI's a*/b* channels span only ~35-40 levels
    total on this sensor (vs. the phone's much wider gamut), so Otsu finds no
    real bimodal structure there and the "white" cut lands near the middle of
    that narrow band -- in testing this misclassified ~45% of the frame
    (most of the plain red track) as white candidate, rather than the ~1-5%
    the actual painted lines cover. L* on the same D415 footage has a clean
    bimodal split (track ~gray 110-115, paint >~170), consistent with the
    old pre-Otsu pipeline's hardcoded `gray > 180` cutoff -- Otsu on L* just
    makes that cutoff adaptive per frame instead of a fixed constant.
    Confirmed this does NOT reintroduce the sun-glare-reads-as-white problem
    the phone-photo audit found: restricting the L* histogram to
    near_track (see below) keeps a glare patch's shape rejected downstream
    by filter_line_candidates_bev() the same way a chroma-based candidate
    would be -- this only changes which pixels become white *candidates*,
    not how glare-shaped blobs get filtered out.

    The white center line splits the track into a left half and a right
    half. Taking "the single largest red blob" would silently keep only one
    half and report the other as background -- a bug that showed up in
    testing as the rover trusting only one side of the lane. Fixed here by
    running connectivity on (red | white) TOGETHER before splitting them
    back into separate masks.

    Args:
        frame_bgr: Cropped ROI frame (see crop_to_roi), BGR
        morph_kernel_px: Structuring element size for the close/open passes

    Returns:
        Tuple of (red_track_mask, white_mask), both uint8 0/1, same size as
        frame_bgr. white_mask is NOT yet shape-filtered -- callers doing lane
        finding should warp it to BEV and pass it through
        filter_line_candidates_bev() first.
    """
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    L = lab[:, :, 0]

    A_u8 = lab[:, :, 1].astype(np.uint8)
    _, red_otsu = cv2.threshold(A_u8, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    red_raw = red_otsu.astype(np.uint8)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_kernel_px, morph_kernel_px))
    red_closed = cv2.morphologyEx(red_raw, cv2.MORPH_CLOSE, kernel)
    red_closed = cv2.morphologyEx(red_closed, cv2.MORPH_OPEN, kernel)

    # White vs. red on the D415 is a BRIGHTNESS distinction: L* has a clean
    # bimodal split on real footage of this track (plain track ~110-115,
    # painted line >~170; see segment_track_colors' docstring for the
    # measurements this replaced a chroma-distance split with, and why).
    # Restricted to near_track for the same reason the old chroma version
    # was: keeps a bright unrelated patch elsewhere in the ROI (sunlit grass,
    # a dirt-bank highlight) from shifting the white cut point, since Otsu
    # only sees the histogram of pixels already on/near the red blob.
    near_track = cv2.dilate(red_closed, kernel, iterations=2).astype(bool)
    white_raw = np.zeros_like(red_raw)
    if np.count_nonzero(near_track) >= 50:
        L_u8 = np.clip(L, 0, 255).astype(np.uint8)
        l_samples = L_u8[near_track].reshape(-1, 1)
        # THRESH_BINARY (not _INV): Otsu's cut separates low-L (red track,
        # below the cut) from high-L (white paint, above it); we want the
        # high side, same sense as the red_otsu split above.
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
    max_width_px: float = LaneDetectionConfig.LINE_MAX_WIDTH_PX,
    min_height_frac: float = LaneDetectionConfig.LINE_MIN_DEPTH_FRAC,
    min_aspect: float = LaneDetectionConfig.LINE_MIN_ASPECT,
    min_area_px: int = LaneDetectionConfig.MIN_LINE_COMPONENT_AREA_PX,
) -> np.ndarray:
    """
    Keep only white-candidate blobs shaped like a painted line; reject
    broad/short blobs such as a sun-glare patch on the track.

    Must run AFTER warping to the metric bird's-eye view, not on the raw
    camera frame: in the raw perspective, lines converge and shrink with
    distance, so a fixed width/aspect rule can't tell "thin far line" from
    "thick near glare" -- they overlap in raw pixel dimensions. In the BEV
    canvas (fixed px/metre in both axes), a real line has near-constant
    physical width and runs the full depth of the ROI (tall, narrow -> high
    height/width ratio); a glare or reflection blob is localized and does
    not persist over that depth range (short, comparatively wide -> low
    ratio). This is a geometric test, not a color one, so it holds
    regardless of which camera or lighting produced the color-stage
    candidates in segment_track_colors().

    Args:
        warped_white: White-candidate mask (uint8 0/1) already warped to the
            BEV canvas
        bev_height: Height of the BEV canvas in pixels
        max_width_px: Reject components wider than this (a real line stays
            under it; a merged line+glare or line+curb blob does not)
        min_height_frac: Reject components spanning less than this fraction
            of bev_height (a real line persists across most of the ROI's
            depth; a localized blob or a fragment of gravel-texture speckle
            does not)
        min_aspect: Reject components with height/width below this
        min_area_px: Drop components smaller than this before the shape test

    Returns:
        uint8 0/1 mask, same size as warped_white, containing only the
        components that passed the shape test
    """
    # Pass 1 -- prune per-ROW, not per-component. A glare patch that happens
    # to touch a real line (no color gap between them, e.g. the glare washes
    # out right up to the paint) 8-connects into ONE component with it; a
    # whole-component width/aspect test then either keeps or rejects the
    # glare pixels AND the genuine line pixels together, since it cannot see
    # that only part of the blob is line-shaped. Opening with a purely
    # horizontal (kw x 1) kernel is a per-row run-length test: a run
    # narrower than the kernel is erased entirely, a run at least that wide
    # survives untouched -- so subtracting the opened result from the
    # original leaves exactly the pixels that belonged to a too-wide run in
    # their own row, regardless of what a taller neighbouring blob does.
    # This turns "glare merged with the real line" into "glare erased, real
    # line's own thin rows survive", instead of losing the whole component.
    kw = int(max_width_px) + 1
    row_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kw, 1))
    too_wide = cv2.morphologyEx(warped_white, cv2.MORPH_OPEN, row_kernel)
    thin_only = (warped_white.astype(bool) & ~too_wide.astype(bool)).astype(np.uint8)

    # Pass 2 -- of what's left, keep components that persist over enough of
    # the ROI's depth and are tall/narrow overall (rejects isolated speckle
    # that happens to be thin at any single row but doesn't form a line).
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
# 2. ROI Scaling & Cropping
# ================================
def get_scaled_roi_points(
    frame_width: int,
    frame_height: int,
    roi_base_points: np.ndarray,
    roi_base_width: float = LaneDetectionConfig.ROI_BASE_WIDTH,
    roi_base_height: float = LaneDetectionConfig.ROI_BASE_HEIGHT,
) -> np.ndarray:
    """
    Scale ROI corner points (authored at roi_base_width x roi_base_height) to
    the actual frame size.

    Args:
        frame_width: Actual frame width (pixels)
        frame_height: Actual frame height (pixels)
        roi_base_points: Array-like, shape (4, 2) -- ROI corners in base-resolution coordinates
        roi_base_width: Reference width roi_base_points were authored against
        roi_base_height: Reference height roi_base_points were authored against

    Returns:
        np.float32 array, shape (4, 2): ROI corners scaled to (frame_width, frame_height)
    """
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
    """
    Crop a frame to the bounding box of the ROI polygon (plus margin).

    Runs before the expensive per-pixel preprocessing so CPU cost scales
    with the ROI area the perspective transform actually uses, rather than
    the full camera frame -- unlike a sensor-level resolution cut, this
    doesn't reduce pixel density (ground-sample-distance) inside the ROI,
    so it doesn't cost curve-fitting accuracy the way downscaling does.

    Args:
        frame_bgr: Input BGR frame
        roi_points: np.float32 array, shape (4, 2) -- ROI corners already
            scaled to this frame's size (see get_scaled_roi_points)
        margin_x: Extra margin added left/right of the ROI bounding box (pixels)
        margin_y: Extra margin added above/below the ROI bounding box (pixels)

    Returns:
        Tuple of (cropped_frame, x_offset, y_offset):
            - cropped_frame: BGR sub-image
            - x_offset, y_offset: top-left corner of the crop, in the
              original frame's coordinate system (subtract from roi_points
              to re-express them relative to the cropped frame)
    """
    frame_height, frame_width = frame_bgr.shape[:2]

    x_min = int(np.clip(np.min(roi_points[:, 0]) - margin_x, 0, frame_width))
    x_max = int(np.clip(np.max(roi_points[:, 0]) + margin_x, 0, frame_width))
    y_min = int(np.clip(np.min(roi_points[:, 1]) - margin_y, 0, frame_height))
    y_max = int(np.clip(np.max(roi_points[:, 1]) + margin_y, 0, frame_height))

    cropped_frame = frame_bgr[y_min:y_max, x_min:x_max]
    return cropped_frame, x_min, y_min


# ================================
# 3. Perspective Transform
# ================================
def perspective_transform(
    binary: np.ndarray,
    frame_size: Tuple[int, int],
    roi_points: np.ndarray,
    bev_size: Tuple[int, int] = None
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Apply perspective transformation to bird eye view.

    Transforms the image from camera view to top-down view for easier
    lane detection in the transformed coordinate system.

    The output canvas is `bev_size`, independent of the input crop size. This
    matters: sizing the canvas from the crop made the bird's-eye view
    anisotropic (the crop is much wider than it is tall), which inflated the
    reported heading angle by the canvas aspect ratio and left `theta` as a
    canvas-dependent number rather than an angle. With a fixed canvas chosen so
    that both axes carry the same metres-per-pixel, `theta` is a real heading
    angle, `b` is a real cross-track offset and `curvature` is a real 1/m arc.

    Args:
        binary: Binary input image
        frame_size: Tuple of (width, height) of `binary`
        roi_points: np.float32 array, shape (4, 2) -- source ROI quadrilateral
            corners [bottom-left, bottom-right, top-right, top-left], already
            scaled (and, if cropped upstream, shifted) to match `binary`
        bev_size: Tuple of (width, height) for the warped output canvas.
            Defaults to `frame_size`, preserving the previous behaviour.

    Returns:
        Tuple of (warped, M, M_inv):
            - warped: Transformed binary image (bird eye view)
            - M: Perspective transformation matrix
            - M_inv: Inverse transformation matrix
    """
    w, h = frame_size if bev_size is None else bev_size

    dst = np.float32([
        [w * 0.25, h * 1.0],
        [w * 0.75, h * 1.0],
        [w * 0.75, h * 0.0],
        [w * 0.25, h * 0.0]
    ])

    M = cv2.getPerspectiveTransform(roi_points, dst)
    M_inv = cv2.getPerspectiveTransform(dst, roi_points)

    # INTER_NEAREST: `binary` is a 0/1 mask, both callers of this function
    # feed it into connected-component analysis downstream. Linear
    # interpolation blends 0/1 neighbours into fractional values that
    # truncate back to 0 on cast, thinning or dropping thin lines right where
    # the shape filter most needs the true width preserved.
    warped = cv2.warpPerspective(binary, M, (w, h), flags=cv2.INTER_NEAREST)

    return warped, M, M_inv


# ================================
# 4. Lane Finding (single center line)
# ================================
def find_center_line(
    binary_warped: np.ndarray,
    num_windows: int = 9,
    window_margin: int = 100,
    min_pixels: int = 50,
    search_center: float = None,
    search_band: float = None
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Find lane center line pixels using sliding window technique.

    The search starts from the strongest column of the bottom-half histogram,
    but only within `search_band` pixels of `search_center`. On a track with
    several parallel painted lines in view (e.g. two lane edges plus a centre
    line) an unrestricted argmax has no way to tell them apart -- it picks
    whichever line happens to carry the most pixels that frame, and that choice
    flips as lighting, wear or the rover's own drift changes the balance. Each
    flip is a step change in the reported offset of roughly one line spacing,
    which no downstream filter can absorb because it is a genuine step, not
    noise. Bounding the search keeps the detector on the line it was already
    following, and bounds how far that line can appear to move per frame.

    Args:
        binary_warped: Binary bird eye view image
        num_windows: Number of horizontal windows
        window_margin: Width of search window (plus-minus margin). Keep below
            half the spacing between adjacent painted lines, otherwise one
            window straddles two lines and recenters into the gap between them.
        min_pixels: Minimum pixels to recenter window
        search_center: Column the lane is expected at (typically the previous
            frame's result). Defaults to the canvas centre.
        search_band: Half-width of the band around `search_center` the initial
            histogram may search. None or <= 0 searches the full width (the
            previous, unbounded behaviour).

    Returns:
        Tuple of (x_coords, y_coords) of detected lane pixels
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
        if band_high <= band_low:            # degenerate band -- fall back
            band_low, band_high = 0, width

    histogram = np.sum(binary_warped[height // 2:, band_low:band_high], axis=0)
    if histogram.size == 0 or histogram.max() == 0:
        # Nothing in the band. Hold the expected position; the windows will
        # come back empty and min_lane_pixels will report "not detected",
        # rather than seeding the search from an arbitrary column.
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
# 5. Fit single line & Params
# ================================
def compute_lane_params(
    binary_warped: np.ndarray,
    sliding_windows: int = LaneDetectionConfig.SLIDING_WINDOWS,
    window_margin: int = LaneDetectionConfig.WINDOW_MARGIN,
    min_window_pixels: int = LaneDetectionConfig.MIN_WINDOW_PIXELS,
    min_lane_pixels: int = LaneDetectionConfig.MIN_LANE_PIXELS,
    search_center: float = None,
    search_band: float = LaneDetectionConfig.SEARCH_BAND_PX,
) -> dict:
    """
    Compute lane parameters (curvature, steering angle, lateral offset) from binary image.

    Detected pixels are translated into a lookahead-centered frame before
    fitting: y is measured from the bottom row of the canvas (y=0) and x from
    the canvas's horizontal center (x=0). Fitting the parabola
    x = A*y^2 + B*y + C directly in this frame means the fit coefficients
    themselves ARE the quantities the controller needs at that point -- B is
    the heading slope and C is the lateral offset -- with no separate
    "evaluate the polynomial at y=height" step (and its extra floating-point
    error) afterward.

    NOTE: y=0 is the NEAR EDGE OF THE ROI, not the rover. With the shipped
    camera geometry that is 1.30 m ahead of the camera, i.e. 1.22 m ahead of
    the front axle and 1.71 m ahead of the rear axle. So `b` is a lookahead
    cross-track error, not the error at the rover: on a curve it is non-zero
    even when the rover is perfectly on the line. That is a usable
    (pure-pursuit-like) signal, but it overlaps with `curvature`, so k_e2 and
    k_ff act on partly the same information when tuning.

    sliding_windows/window_margin/min_window_pixels/min_lane_pixels are
    exposed as arguments (not hardcoded) because their correct values are
    tied to the working canvas size -- since the ROI is now cropped before
    this point, the canvas is much smaller than the un-cropped frame, so
    these need to be tunable without editing code.

    Args:
        binary_warped: Binary bird eye view image
        sliding_windows: Number of vertical search windows (see find_center_line)
        window_margin: Search window half-width in pixels (see find_center_line)
        min_window_pixels: Minimum pixels to recenter a window (see find_center_line)
        min_lane_pixels: Minimum total detected pixels required for a valid fit
        search_center: Column the lane is expected at, in the warped canvas
            (see find_center_line). Defaults to the canvas centre.
        search_band: Half-width of the search band around `search_center`

    Returns:
        Dictionary with keys:
            - curvature: Parabola coefficient A (x = A*y^2 + B*y + C), in
              BEV pixels (1/px) -- NOT converted to metres here; consumers
              that need a real radius fold BEV_PX_PER_M into their own
              formula (see rover_kinematic_control's k_ff derivation).
              NaN if not detected
            - theta: Steering angle (degrees) at the fit origin, NaN if not detected
            - b: Lateral offset from center, in METRES, at the fit origin,
              NaN if not detected
            - detected: Boolean flag indicating valid detection
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
        # Translate to rover-centered frame: y=0 at the bottom row (rover
        # position), x=0 at the horizontal center, before fitting.
        y_shifted = y_coords.astype(np.float64) - height
        x_shifted = x_coords.astype(np.float64) - (width / 2.0)

        # Fit parabola: x = A*y^2 + B*y + C in the shifted frame
        coeff_a, coeff_b, coeff_c = np.polyfit(y_shifted, x_shifted, 2)

        # At y_shifted = 0 (the rover's position), the fit coefficients are
        # directly the heading slope (B) and lateral offset (C). C comes out
        # of the fit in BEV pixels (the fit runs in the warped canvas's pixel
        # coordinates); convert to metres here, at the single point `b` is
        # produced, so every consumer (the ROS topic, CSV logs, the control
        # law's k_e2 weight) gets a physical distance rather than a value
        # that silently means something different if BEV_PX_PER_M ever
        # changes. theta needs no such conversion -- arctan of a dimensionless
        # slope is already a real angle regardless of pixel scale.
        theta = np.degrees(np.arctan(coeff_b))
        b_centered = coeff_c / LaneDetectionConfig.BEV_PX_PER_M

        result = {
            "curvature": coeff_a,
            "theta": theta,
            "b": b_centered,
            "detected": True,
        }

    return result

# ================================
# 6. Full Pipeline
# ================================
def process_frame(
    frame_bgr: np.ndarray,
    roi_base_points: np.ndarray = None,
    roi_base_width: float = LaneDetectionConfig.ROI_BASE_WIDTH,
    roi_base_height: float = LaneDetectionConfig.ROI_BASE_HEIGHT,
    crop_margin_px: float = LaneDetectionConfig.CROP_MARGIN_PX,
    sliding_windows: int = LaneDetectionConfig.SLIDING_WINDOWS,
    window_margin: int = LaneDetectionConfig.WINDOW_MARGIN,
    min_window_pixels: int = LaneDetectionConfig.MIN_WINDOW_PIXELS,
    min_lane_pixels: int = LaneDetectionConfig.MIN_LANE_PIXELS,
    bev_width: int = LaneDetectionConfig.BEV_WIDTH_PX,
    bev_height: int = LaneDetectionConfig.BEV_HEIGHT_PX,
    search_center: float = None,
    search_band: float = LaneDetectionConfig.SEARCH_BAND_PX,
) -> Tuple[float, float, float, bool]:
    """
    Complete lane detection pipeline: crop -> segment colors -> transform to
    BEV -> shape-filter -> detect -> compute params.

    Crops to the ROI's bounding box (plus margin) before the expensive
    per-pixel preprocessing runs, so CPU cost scales with the ROI area the
    perspective transform actually uses rather than the full camera frame,
    without reducing pixel density inside the ROI the way a sensor-level
    resolution cut would.

    The white-candidate mask is warped to BEV BEFORE the line-shape filter
    runs (not after, and not filtered in camera view) -- see
    filter_line_candidates_bev()'s docstring for why the shape test only
    makes sense in the metric top-down view.

    Args:
        frame_bgr: Input BGR frame from camera
        roi_base_points: ROI trapezoid corners at (roi_base_width x roi_base_height);
            defaults to LaneDetectionConfig.ROI_BASE_POINTS
        roi_base_width: Reference width roi_base_points were authored against
        roi_base_height: Reference height roi_base_points were authored against
        crop_margin_px: Extra margin (in roi_base_width/height units) added
            around the ROI bounding box before cropping
        sliding_windows: Number of vertical search windows
        window_margin: Search window half-width in pixels
        min_window_pixels: Minimum pixels to recenter a search window
        min_lane_pixels: Minimum total detected pixels required for a valid fit
        bev_width: Warped output canvas width (pixels)
        bev_height: Warped output canvas height (pixels). Together with
            bev_width this fixes the bird's-eye scale, so the reported values
            are physical rather than crop-size-dependent.
        search_center: Column the lane is expected at in the warped canvas,
            normally `bev_width/2 + b * LaneDetectionConfig.BEV_PX_PER_M`
            from the previous detected frame -- `b` is in metres, this is a
            pixel column, the conversion is not optional. Defaults to the
            canvas centre.
        search_band: Half-width of the search band around `search_center`

    Returns:
        Tuple of (curvature, theta, b, detected):
            - curvature: Parabola coefficient A (x = A*y^2 + B*y + C), BEV
              pixels (1/px), not converted to a real 1/m arc here
            - theta: Steering angle error (degrees)
            - b: Lateral offset, metres
            - detected: Boolean detection flag
    """
    if roi_base_points is None:
        roi_base_points = LaneDetectionConfig.ROI_BASE_POINTS

    frame_width, frame_height = frame_bgr.shape[1], frame_bgr.shape[0]

    # Scale the ROI (and its margin) from the base calibration resolution to
    # the actual frame size.
    roi_points_full = get_scaled_roi_points(
        frame_width, frame_height, roi_base_points, roi_base_width, roi_base_height
    )
    margin_x = crop_margin_px * (frame_width / roi_base_width)
    margin_y = crop_margin_px * (frame_height / roi_base_height)

    # Crop to the ROI bounding box before the expensive per-pixel preprocessing
    cropped_frame, x_offset, y_offset = crop_to_roi(frame_bgr, roi_points_full, margin_x, margin_y)
    crop_width, crop_height = cropped_frame.shape[1], cropped_frame.shape[0]

    # Re-express the ROI corners relative to the cropped frame's new origin
    roi_points_cropped = roi_points_full - np.float32([x_offset, y_offset])

    _red_track_mask, white_candidates = segment_track_colors(cropped_frame)
    warped_white, M, M_inv = perspective_transform(
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
    plot_lane_lines(cropped_frame, warped, M_inv, params["theta"], params["b"], params["detected"])

    return params["curvature"], params["theta"], params["b"], params["detected"]


def plot_lane_lines(
    frame_bgr: np.ndarray,
    warped: np.ndarray,
    M_inv: np.ndarray,
    theta: float,
    b: float,
    detected: bool
) -> None:
    """
    Visualize detected lane lines in bird eye and original views.
    
    Args:
        frame_bgr: Original BGR frame
        warped: Bird eye view binary image
        M_inv: Inverse perspective transformation matrix
        theta: Steering angle (degrees)
        b: Lateral offset, metres
        detected: Detection flag
    """
    height, width = warped.shape[:2]

    # Generate y-coordinates
    y_vals = np.linspace(0, height - 1, num=height)
    if detected:
        # b arrives in metres; this function draws entirely in the BEV
        # canvas's pixel coordinates, so convert back here rather than
        # mixing units in the line below.
        b_px = b * LaneDetectionConfig.BEV_PX_PER_M
        slope = np.tan(np.radians(theta))
        x_vals = slope * y_vals + (b_px + width // 2)
    else:
        x_vals = np.array([])
        y_vals = np.array([])

    # Bird eye view visualization
    bird_eye_vis = cv2.cvtColor((warped * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    bird_eye_vis = bird_eye_vis[:, 100:width - 100]
    width_cropped = width - 200
    x_vals = x_vals - 100

    if len(x_vals) > 0:
        pts = np.vstack([x_vals, y_vals]).T.astype(np.int32)
        cv2.polylines(bird_eye_vis, [pts], isClosed=False, color=(0, 0, 255), thickness=3)
        cv2.line(bird_eye_vis, (width_cropped // 2, 0), (width_cropped // 2, height), (0, 255, 0), 1)
        cv2.line(bird_eye_vis, (0, height // 2), (width_cropped, height // 2), (255, 0, 0), 1)

    # Original view visualization
    orig_vis = frame_bgr.copy()
    if len(x_vals) > 0:
        pts_warped = np.vstack([x_vals, y_vals]).T.reshape(-1, 1, 2).astype(np.float32)
        pts_original = cv2.perspectiveTransform(pts_warped, M_inv)
        pts_int = pts_original.astype(np.int32)
        cv2.polylines(orig_vis, [pts_int], isClosed=False, color=(0, 0, 255), thickness=3)
        cv2.line(orig_vis, (width // 2, 0), (width // 2, height), (0, 255, 0), 1)
        cv2.line(orig_vis, (0, height // 2), (width, height // 2), (255, 0, 0), 1)

    # Combined visualization
    if orig_vis.shape[1] != bird_eye_vis.shape[1]:
        orig_vis = cv2.resize(orig_vis, (bird_eye_vis.shape[1], bird_eye_vis.shape[0]))
    
    screen_width = 720
    combined_vis = np.vstack([bird_eye_vis, orig_vis])
    h, w = combined_vis.shape[:2]
    if w > screen_width:
        scale = screen_width / w
        combined_vis = cv2.resize(combined_vis, (int(w * scale), int(h * scale)))