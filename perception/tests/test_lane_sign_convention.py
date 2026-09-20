"""Tests the detector's `heading_err_rad` sign convention against the model
that consumes it — `RoverState::at_lookahead` and `Ekf::correct_camera` in
`crates/rover-msgs`/`crates/rover-navigation`, both of which encode
`d(cross_track)/d(distance) = +heading_err_rad`.

This is the test D5's own post-mortem says never existed
(`docs/RUST_REWRITE_PLAN.md` §13.3b D5: "Nothing anywhere tests the
detector's convention against the model's."). `test_lane_parity.py` could
not have caught the D5 defect and is not a substitute for this file: it
proves the port (`process_frame`) and the frozen ROS 2 oracle agree with
each other, bit-for-bit. They do agree — and they agreed the whole time the
sign was wrong, because the port is a faithful, behaviour-preserving copy of
the oracle's algorithm, defect included. Parity between two implementations
of the same (buggy) fit says nothing about whether that fit's output means
what its consumers assume it means. That is a claim about the *model*, not
about drift between two copies of the *code*, and only a test that
independently reconstructs the model's own quantity — `d(cross_track)/d
(distance)`, from the fitted polynomial coefficients, per D5's derivation —
and compares it against what the detector actually reports can catch it.

`LaneDetector.detect` is where this port negates the raw fit's `theta`
before it becomes `heading_err_rad` (see `lane.py`'s module docstring,
section 2, and the comment at that conversion) so that it matches the
`cross_track_m`/`steer` convention: positive means "the lane heads right,
steer right". This file checks that property directly, from first
principles, rather than trusting the implementation to have gotten its own
fix right.
"""

from __future__ import annotations

import math

import numpy as np

from rover_perception.lane import LaneConfig, LaneDetector, process_frame
from tests.test_lane_parity import _draw_line, _track_background

# Lookahead window used to finite-difference the reconstructed cross-track
# curve. 0.8 m matched the measurement in D5's write-up.
_LOOKAHEAD_M = 0.8


def _reconstruct_cross_track(s_m: float, curvature_px: float, theta_deg: float, b_m: float) -> float:
    """Reconstruct cross-track offset at forward distance `s_m` from the raw
    fit coefficients, using the relation derived in `lane.py`'s module
    docstring (section 2) and D5: the fit is `x' = A*y'^2 + B*y' + C` with
    `y' = y - height` zero at the canvas bottom and negative going forward,
    so forward distance is `s = -y'` and, in BEV pixels,

        x'_px = A*s_px^2 - B*s_px + C

    with `A = curvature_px` (already in 1/px, unconverted -- see
    `compute_lane_params`), `B = tan(radians(theta_deg))` (the fit's raw
    slope, before this module's sign correction), and `C = b_m *
    BEV_PX_PER_M` (`b` converted back to the pixels the fit coefficients are
    in). Dividing back by `BEV_PX_PER_M` gives cross-track in metres.
    """
    S = LaneConfig.BEV_PX_PER_M
    A = curvature_px
    B = math.tan(math.radians(theta_deg))
    C = b_m * S
    s_px = s_m * S
    return (A * s_px**2 - B * s_px + C) / S


def _model_slope(curvature_px: float, theta_deg: float, b_m: float) -> float:
    """`d(cross_track)/d(distance)`, by finite difference of the
    reconstruction above over `[0, _LOOKAHEAD_M]` -- the same model quantity
    `RoverState::at_lookahead` and `Ekf::correct_camera` both assume equals
    `+heading_err_rad`."""
    c0 = _reconstruct_cross_track(0.0, curvature_px, theta_deg, b_m)
    c1 = _reconstruct_cross_track(_LOOKAHEAD_M, curvature_px, theta_deg, b_m)
    return (c1 - c0) / _LOOKAHEAD_M


def _veering_frame(*, x_top: float, x_bottom: float, seed: int) -> np.ndarray:
    """A straight lane line from `(x_top, near-top-of-ROI)` to `(x_bottom,
    near-bottom-of-ROI)`. The BEV canvas's top row is ahead of the rover and
    its bottom row is nearest (confirmed by measurement in D5), so
    `x_top > x_bottom` draws a lane whose x-position increases as it
    recedes into the distance -- it veers **right** as it recedes.
    `x_top < x_bottom` is the mirror: veers **left**."""
    frame = _track_background(np.random.default_rng(seed))
    _draw_line(frame, x_top=x_top, x_bottom=x_bottom)
    return frame


def test_right_veering_lane_has_positive_slope_and_positive_heading_err() -> None:
    """A lane veering right as it recedes must have both
    `d(cross_track)/d(distance) > 0` (by direct reconstruction from the fit)
    and `heading_err_rad > 0` (the detector's actual output) -- same sign,
    matching the "positive means steer right" convention `cross_track_m`
    already satisfies."""
    frame = _veering_frame(x_top=850, x_bottom=550, seed=101)

    curvature_px, theta_deg, b_m, detected = process_frame(frame)
    assert detected, "test needs a frame process_frame actually detects on"
    assert abs(theta_deg) > 1.0, "theta_deg too close to zero to pin a sign"

    slope = _model_slope(curvature_px, theta_deg, b_m)
    assert slope > 0.0, f"expected a right-veering lane to have positive slope, got {slope!r}"

    result = LaneDetector().detect(frame)
    assert result.valid
    assert result.heading_err_rad > 0.0, (
        f"right-veering lane: model slope={slope!r} (positive) but "
        f"heading_err_rad={result.heading_err_rad!r} (not positive) -- signs disagree"
    )


def test_left_veering_lane_has_negative_slope_and_negative_heading_err() -> None:
    """Mirror of the right-veering case: a lane veering left as it recedes
    must have both quantities negative."""
    frame = _veering_frame(x_top=450, x_bottom=750, seed=102)

    curvature_px, theta_deg, b_m, detected = process_frame(frame)
    assert detected, "test needs a frame process_frame actually detects on"
    assert abs(theta_deg) > 1.0, "theta_deg too close to zero to pin a sign"

    slope = _model_slope(curvature_px, theta_deg, b_m)
    assert slope < 0.0, f"expected a left-veering lane to have negative slope, got {slope!r}"

    result = LaneDetector().detect(frame)
    assert result.valid
    assert result.heading_err_rad < 0.0, (
        f"left-veering lane: model slope={slope!r} (negative) but "
        f"heading_err_rad={result.heading_err_rad!r} (not negative) -- signs disagree"
    )


def test_heading_err_magnitude_matches_reconstructed_slope() -> None:
    """Beyond agreeing in sign, `tan(heading_err_rad)` must approximately
    equal the reconstructed `d(cross_track)/d(distance)` -- they are the
    same physical quantity (a slope) under two different derivations, and
    should match to within the small-angle/discretisation slack of the
    finite difference, not just share a sign. D5's own measurement on the
    real detector found agreement to ~3 decimals; this test uses a looser
    tolerance to stay robust to the specific synthetic frames here.
    """
    for x_top, x_bottom, seed in ((850, 550, 101), (450, 750, 102)):
        frame = _veering_frame(x_top=x_top, x_bottom=x_bottom, seed=seed)
        curvature_px, theta_deg, b_m, detected = process_frame(frame)
        assert detected, "test needs a frame process_frame actually detects on"

        slope = _model_slope(curvature_px, theta_deg, b_m)

        result = LaneDetector().detect(frame)
        assert result.valid
        tan_heading_err = math.tan(result.heading_err_rad)

        assert math.isclose(tan_heading_err, slope, abs_tol=0.02), (
            f"tan(heading_err_rad)={tan_heading_err!r} vs reconstructed "
            f"model slope={slope!r} -- magnitudes should agree, not just signs"
        )
