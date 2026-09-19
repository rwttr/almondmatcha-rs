"""Parity test: `rover_perception.lane.process_frame` against the frozen ROS 2
oracle in `tests/oracle/lane_detector.py`.

`lane.py`'s module docstring promises this file exists and is what proves the
port did not drift from the algorithm it was copied from — not code review.
See `tests/oracle/README.md` for what the oracle is and the rule that it is
never "fixed": a mismatch found here is a finding about `lane.py`, to be
reported and investigated, not a discrepancy to paper over by loosening this
test's tolerances.

# Comparison boundary

This compares at `process_frame`, NOT `LaneDetector.detect`. `process_frame`
is the behaviour-preserving boundary — same algorithm, same constants, same
operation order as the oracle. `LaneDetector.detect` is documented (see
`lane.py`'s module docstring, "The one deliberate behaviour change") to
diverge on purpose: it converts `curvature` from BEV pixels to a physical
1/m quantity and `theta` from degrees to radians before it leaves the
module. Comparing at `detect` would make this test fail on an intentional,
already-documented change instead of catching accidental drift — the wrong
tool for the job. That intentional divergence gets its own pinning test
below instead (`test_lane_detector_detect_diverges_exactly_as_documented`),
so it cannot silently drift either.

# Parameter alignment

The oracle's `process_frame` takes explicit keyword args for every
ROI/window/BEV parameter, defaulted from `LaneDetectionConfig`
(`tests/oracle/config.py`). The port's `process_frame` takes the *same*
parameter names in the *same* order, defaulted from `LaneConfig`
(`rover_perception/lane.py`) instead. The two config classes hold equal
values today, but nothing enforces that they stay equal — `LaneConfig` is a
free-standing class in the port, not derived from or checked against
`LaneDetectionConfig`. So this test does not lean on the two functions'
own defaults lining up: `_common_kwargs()` below reads every value straight
off `LaneDetectionConfig` (the oracle's own config) and passes it
*explicitly* to both functions. A parity test that silently let the two
sides run different configurations would be worse than no test at all —
see the module docstring on `tests/oracle/README.md`.

# Equality, not tolerance

Both implementations run the identical sequence of numpy/OpenCV operations
against the identical input array, so exact equality is the right
assertion — a real algorithmic divergence should show up as *any* bit
difference, not just one past some epsilon. `_assert_same_result` uses `==`
(equivalent to `math.isclose(rel_tol=0, abs_tol=0)`, spelled out in one
place) and treats "both NaN" as equal, since `nan == nan` is `False` in
Python/numpy but `NaN` is exactly what `compute_lane_params` returns for
"undetected" in both implementations. If a future change ever needs a
nonzero tolerance here, that would mean the two sides no longer run bit-
identical operations in bit-identical order, which is itself worth
understanding before relaxing anything.
"""

from __future__ import annotations

import math
from typing import Optional

import cv2
import numpy as np
import pytest

from rover_perception.lane import LaneConfig, LaneDetector, process_frame as port_process_frame
from tests.oracle.config import LaneDetectionConfig
from tests.oracle.lane_detector import process_frame as oracle_process_frame

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

# The ROI trapezoid (LaneConfig.ROI_BASE_POINTS / LaneDetectionConfig.
# ROI_BASE_POINTS) spans y=270 (top, narrow) to y=458 (bottom, wide) at this
# base resolution. Drawing lines a little outside that band (260..470) keeps
# them visible right up to the ROI's own edges after crop + warp, rather than
# fading out exactly at the boundary the pipeline itself uses.
_LINE_Y_TOP = 260
_LINE_Y_BOTTOM = 470


def _track_background(rng: np.random.Generator) -> np.ndarray:
    """A uniform reddish "track surface" (BGR, roughly matching the real
    D415 footage this pipeline was tuned against — see `lane.py`'s module
    docstring) plus small per-pixel noise, so Otsu's threshold on the LAB
    `a*`/`L*` channels has real (if slight) variance to split on instead of
    a degenerate constant image. The noise amplitude is small enough that it
    never crosses into "looks like a line" territory on its own."""
    frame = np.empty((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    frame[:] = (40, 40, 170)  # BGR
    noise = rng.integers(-6, 7, size=frame.shape)
    return np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)


def _draw_line(
    frame: np.ndarray,
    x_top: float,
    x_bottom: float,
    *,
    curvature: float = 0.0,
    thickness: int = 6,
) -> None:
    """Paint one white line from `(x_top, _LINE_Y_TOP)` to
    `(x_bottom, _LINE_Y_BOTTOM)`, optionally bowed by `curvature` (a
    dimensionless fraction of frame width added as `curvature * t**2`, `t`
    running 0..1 down the line) to produce a genuinely curved lane marker
    for the curvature test case rather than only ever exercising the
    straight-line path."""
    ys = np.linspace(_LINE_Y_TOP, _LINE_Y_BOTTOM, num=60)
    t = (ys - _LINE_Y_TOP) / (_LINE_Y_BOTTOM - _LINE_Y_TOP)
    xs = x_top + (x_bottom - x_top) * t + curvature * (t**2) * frame.shape[1]
    pts = np.stack([xs, ys], axis=1).astype(np.int32)
    cv2.polylines(frame, [pts], isClosed=False, color=(255, 255, 255), thickness=thickness)


def straight_centered_frame(seed: int = 1) -> np.ndarray:
    """A straight line running down the middle of the ROI."""
    frame = _track_background(np.random.default_rng(seed))
    _draw_line(frame, x_top=640, x_bottom=640)
    return frame


def offset_line_frame(seed: int = 2) -> np.ndarray:
    """A straight line offset well off centre, but still inside the ROI
    trapezoid at every y (its narrowest point, the top edge, spans x=365..915
    at this resolution) — chosen empirically (780 detects; 800+ does not,
    since the shape/depth filters need a fully-inside-ROI line to see enough
    of its length) to give a clean nonzero, non-edge-case `b`."""
    frame = _track_background(np.random.default_rng(seed))
    _draw_line(frame, x_top=780, x_bottom=780)
    return frame


def curved_line_frame(seed: int = 3) -> np.ndarray:
    """A line with real curvature, to exercise the quadratic (not just
    linear) term of the polyfit both implementations share."""
    frame = _track_background(np.random.default_rng(seed))
    _draw_line(frame, x_top=640, x_bottom=750, curvature=0.05)
    return frame


def blank_frame(seed: int = 4) -> np.ndarray:
    """All-black: no red track, no white line, nothing to segment. Expected
    to come back not-detected on both sides — asserted directly in
    `test_blank_frame_is_not_detected`, on top of the parity check every
    scenario gets."""
    return np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)


def noise_frame(seed: int = 5) -> np.ndarray:
    """Pure per-pixel random noise, no structure at all — the adversarial
    case for the Otsu/connected-components machinery, which must degrade
    identically in both implementations even when there is no "right"
    answer to detect."""
    return np.random.default_rng(seed).integers(0, 256, size=(FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)


def two_parallel_lines_frame(seed: int = 6) -> np.ndarray:
    """Two straight lines, standing in for the real track's three painted
    lines (two edges + a centre line — see `LaneConfig.WINDOW_MARGIN`'s
    comment on the measured geometry). Exercises the sliding-window search
    actually having to choose one line over another via the histogram, the
    part of the algorithm `WINDOW_MARGIN`/`SEARCH_BAND_PX` exist to
    constrain."""
    frame = _track_background(np.random.default_rng(seed))
    _draw_line(frame, x_top=500, x_bottom=500)
    _draw_line(frame, x_top=800, x_bottom=800)
    return frame


SCENARIOS = {
    "straight_centered": straight_centered_frame,
    "offset_line": offset_line_frame,
    "curved_line": curved_line_frame,
    "blank": blank_frame,
    "noise": noise_frame,
    "two_parallel_lines": two_parallel_lines_frame,
}


def _common_kwargs(search_center: Optional[float] = None) -> dict:
    """Every ROI/window/BEV parameter `process_frame` takes, read explicitly
    off `LaneDetectionConfig` (the oracle's own config class) and handed to
    *both* implementations identically. See the module docstring's
    "Parameter alignment" section for why this does not rely on the two
    functions' own defaults agreeing, even though they do today."""
    cfg = LaneDetectionConfig
    return dict(
        # A plain list, not `np.float32(...)`: both `process_frame`
        # implementations convert internally (`get_scaled_roi_points` does
        # `np.float32(roi_base_points)`), so passing the oracle's own
        # unconverted list keeps this literally "the value LaneDetectionConfig
        # holds," not a value reshaped for one side's convenience.
        roi_base_points=list(cfg.ROI_BASE_POINTS),
        roi_base_width=cfg.ROI_BASE_WIDTH,
        roi_base_height=cfg.ROI_BASE_HEIGHT,
        crop_margin_px=cfg.CROP_MARGIN_PX,
        sliding_windows=cfg.SLIDING_WINDOWS,
        window_margin=cfg.WINDOW_MARGIN,
        min_window_pixels=cfg.MIN_WINDOW_PIXELS,
        min_lane_pixels=cfg.MIN_LANE_PIXELS,
        bev_width=cfg.BEV_WIDTH_PX,
        bev_height=cfg.BEV_HEIGHT_PX,
        search_center=search_center,
        search_band=cfg.SEARCH_BAND_PX,
    )


def _assert_same_result(port_result, oracle_result) -> None:
    """Assert the two `(curvature, theta, b, detected)` tuples are exactly
    equal, NaN-in-both counted as equal. See the module docstring's
    "Equality, not tolerance" section for why `==` (not an epsilon
    comparison) is the correct check here."""
    p_curvature, p_theta, p_b, p_detected = port_result
    o_curvature, o_theta, o_b, o_detected = oracle_result

    assert p_detected == o_detected, f"detected: port={p_detected!r} oracle={o_detected!r}"

    for name, p_val, o_val in (
        ("curvature", p_curvature, o_curvature),
        ("theta", p_theta, o_theta),
        ("b", p_b, o_b),
    ):
        if math.isnan(o_val):
            assert math.isnan(p_val), f"{name}: oracle is NaN but port={p_val!r}"
        else:
            # math.isclose(p_val, o_val, rel_tol=0, abs_tol=0) is exactly
            # `==` for non-NaN floats; spelled as `==` directly since that is
            # what it reduces to and NaN is already handled above.
            assert p_val == o_val, f"{name}: port={p_val!r} oracle={o_val!r}"


@pytest.mark.parametrize("name", sorted(SCENARIOS))
def test_process_frame_matches_oracle(name: str) -> None:
    frame = SCENARIOS[name]()
    kwargs = _common_kwargs()
    port_result = port_process_frame(frame, **kwargs)
    oracle_result = oracle_process_frame(frame, **kwargs)
    _assert_same_result(port_result, oracle_result)


def test_blank_frame_is_not_detected() -> None:
    """Sanity check on top of the parity check above: an all-black frame
    really must come back "not detected" on both sides, not merely "the same
    wrong answer on both sides"."""
    frame = blank_frame()
    _curvature, _theta, _b, detected = port_process_frame(frame, **_common_kwargs())
    assert detected is False


def test_process_frame_matches_oracle_with_search_center_tracking() -> None:
    """Exercises the `search_center`/`search_band` tracking path (as
    `LaneDetector.detect` uses it frame-to-frame) rather than only ever
    hitting the `search_center is None` startup branch that every scenario
    above takes. 340px is near, but not equal to, this canvas's natural
    centre (360px = `BEV_WIDTH_PX / 2`), so the band-limited histogram in
    `find_center_line` actually has to do its job, and is still close enough
    to the frame's line (drawn through the true centre) that both
    implementations still detect it — chosen empirically the same way
    `offset_line_frame`'s x-position was.
    """
    frame = straight_centered_frame(seed=7)
    kwargs = _common_kwargs(search_center=340.0)
    port_result = port_process_frame(frame, **kwargs)
    oracle_result = oracle_process_frame(frame, **kwargs)
    _assert_same_result(port_result, oracle_result)
    assert port_result[3] is True, "test needs a frame this search band actually detects on"


def test_lane_detector_detect_diverges_exactly_as_documented() -> None:
    """Pin the ONE intentional behaviour change `LaneDetector.detect` makes
    relative to `process_frame` (see `lane.py`'s module docstring, "The one
    deliberate behaviour change"), so that change itself cannot silently
    drift even though it is deliberately excluded from the parity comparison
    above.
    """
    frame = straight_centered_frame(seed=8)
    curvature_px, theta_deg, b_m, detected = port_process_frame(frame)
    assert detected, "test needs a frame process_frame actually detects on"

    result = LaneDetector().detect(frame)

    assert result.valid
    assert result.curvature_inv_m == 2.0 * curvature_px * LaneConfig.BEV_PX_PER_M
    assert result.heading_err_rad == math.radians(theta_deg)
    # b is carried straight through unconverted (already metres) -- not part
    # of the documented divergence, but worth pinning here too since this
    # test already has both values in hand.
    assert result.cross_track_m == b_m
