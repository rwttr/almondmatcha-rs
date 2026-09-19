//! Stage 2: guide. The pluggable [`LateralController`] trait (plan §8) and
//! its only implementation, [`StaticGain`] — the bit-exact port of
//! `rover_kinematic_control_node.py`'s steering law and the parity baseline
//! every future controller is measured against (plan §10, §13.2). `Lqr` and
//! `Mpc` are named in the plan as later work and are deliberately **absent**
//! here, not stubbed — see `docs/RUST_REWRITE_PLAN.md` §8 and this crate's
//! `config` module, which rejects `control.law = "lqr"`/`"mpc"` at startup
//! with a "not implemented" error rather than silently falling back.
//!
//! # Unit trap — read this before touching a gain
//!
//! The ROS 2 law mixes degrees and radians on purpose, and the field-tuned
//! gains carry those units baked in:
//!
//! ```text
//! u_fb_deg = k_lat[deg/m]  * cross_track_m
//!          + k_head[deg/deg] * heading_err_deg
//! u_ff_deg = atan(wheelbase_m * curvature_inv_m)  -- atan() is radians,
//!            then converted to degrees, exactly like
//!            `math.degrees(math.atan(...))` in the Python source
//! u_total  = (u_fb_deg + u_ff_deg), converted to radians for this trait
//! ```
//!
//! Two sanity identities worth keeping in mind (and asserted in tests below):
//!
//! - `k_lat` in deg/m, converted to rad/m, is `181.17 * pi/180 ≈ 3.162
//!   rad/m` — exactly the `k1 = 3.162 rad/m` the Python module's docstring
//!   quotes as the raw LQR solution before the rad→deg conversion for
//!   `k_lat`. This is a good check that a refactor hasn't silently dropped
//!   or doubled a `to_radians()`/`to_degrees()`.
//! - `k_head` in deg/deg is **unit-invariant**: converting an angle to
//!   degrees, scaling by `k_head`, and converting back to radians is
//!   algebraically identical to scaling the radian value by `k_head`
//!   directly (deg→rad and rad→deg factors cancel around a *ratio* of
//!   angle to angle). Get this term wrong in the *opposite* direction —
//!   applying a rad/m-style conversion where none is needed — and the
//!   heading feedback is off by a factor of `(180/pi)^2 ≈ 3283`, not a
//!   subtle bug.
//!
//! Getting either conversion backwards does not fail loudly: it steers
//! `57.3x` (`180/pi`) too hard or too soft, which is a rover that oscillates
//! violently or barely responds — see plan §10's "Sign-convention inversion"
//! risk entry; the unit trap is its quieter sibling.
//!
//! # Sign convention
//!
//! `+` means "steer right" throughout this codebase — see `rover_msgs`'s
//! crate docs and plan §10 — the opposite of ISO 8855. Both feedback terms
//! therefore carry a **plus** sign, exactly like
//! `u_fb = (self.k_lat * b_ema) + (self.k_head * theta_ema)` in the Python
//! source. `tests::positive_cross_track_error_steers_right` and
//! `tests::positive_heading_error_steers_right` are the executable check.
//!
//! # Lookahead reconstruction (plan §2.7)
//!
//! `k_lat`/`k_head` were tuned against errors measured at the camera's 1.22 m
//! lookahead point, not at the front axle where the EKF's `RoverState` is
//! referenced. Feeding the raw `RoverState` fields to these gains would be a
//! quiet regression — the EKF removed exactly the geometry the gains assume
//! is still there. [`RoverState::at_lookahead`] reconstructs it, which is
//! why every call below goes through it rather than reading
//! `s.cross_track_m`/`s.heading_err_rad` directly.

use rover_model::VehicleParams;
use rover_msgs::RoverState;

/// A steering law. `steer` returns radians, positive = steer right (see the
/// module docs on sign convention). The actuate stage owns every guard rail
/// (saturation, slew limiting, the speed cap, stall detection) regardless of
/// which law is active — plan §8.3, "controllers propose, actuation
/// disposes" — so nothing here needs to self-limit.
pub trait LateralController {
    fn steer(&mut self, s: &RoverState, dt: f32) -> f32;
    fn reset(&mut self);
    fn name(&self) -> &'static str;
}

/// `control.static_gain` from `config/rover.toml`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StaticGainConfig {
    pub k_lat_deg_per_m: f32,
    pub k_head_deg_per_deg: f32,
}

/// The ported ROS 2 law: static-gain feedback on lookahead-referenced
/// cross-track and heading error, plus an Ackermann feedforward on the
/// curvature ahead.
///
/// Stateless by construction, matching the source: `rover_kinematic_control_node.py`
/// keeps no PID memory here — no integral, no derivative — only the EMA
/// filters upstream of it, which the EKF's own state (fed with lookahead
/// reconstruction) now stands in for. `reset` is therefore a no-op; it
/// exists so this type satisfies [`LateralController`] uniformly with future
/// stateful laws (`Lqr`, `Mpc`) that will need it.
pub struct StaticGain {
    cfg: StaticGainConfig,
    vehicle: VehicleParams,
}

impl StaticGain {
    pub fn new(cfg: StaticGainConfig, vehicle: VehicleParams) -> Self {
        Self { cfg, vehicle }
    }
}

impl LateralController for StaticGain {
    fn steer(&mut self, s: &RoverState, _dt: f32) -> f32 {
        let (cross_track_look_m, heading_look_rad) = s.at_lookahead(self.vehicle.lookahead_m);
        let heading_look_deg = heading_look_rad.to_degrees();

        // Feedback: both terms carry a PLUS sign (see module docs on sign
        // convention). Degrees throughout, matching the field-tuned gains.
        let u_fb_deg = self.cfg.k_lat_deg_per_m * cross_track_look_m
            + self.cfg.k_head_deg_per_deg * heading_look_deg;

        // Feedforward: Ackermann angle for the curvature ahead. `curvature_inv_m`
        // is not lookahead-adjusted by `at_lookahead` — it is "curvature of the
        // road ahead" in both `LaneMeasurement` and `RoverState`, not a
        // quantity referenced to a particular point along the vehicle, so
        // there is nothing to reconstruct here (unlike cross-track/heading,
        // which are measured *at* a point that moves with the lookahead
        // distance).
        let u_ff_deg = (self.vehicle.wheelbase_m * s.curvature_inv_m)
            .atan()
            .to_degrees();

        (u_fb_deg + u_ff_deg).to_radians()
    }

    fn reset(&mut self) {}

    fn name(&self) -> &'static str {
        "static_gain"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    fn vehicle() -> VehicleParams {
        VehicleParams::new(0.4875, 1.22) // plan §10 field-validated constants
    }

    fn gains() -> StaticGainConfig {
        StaticGainConfig {
            k_lat_deg_per_m: 181.17,
            k_head_deg_per_deg: 2.024,
        }
    }

    fn state(cross_track_m: f32, heading_err_rad: f32, curvature_inv_m: f32) -> RoverState {
        RoverState {
            cross_track_m,
            heading_err_rad,
            curvature_inv_m,
            speed_mps: 0.2,
            gyro_bias_radps: 0.0,
            p_diag: [0.0; 5],
            lane_age_ms: 0,
        }
    }

    #[test]
    fn positive_cross_track_error_steers_right() {
        let mut c = StaticGain::new(gains(), vehicle());
        let u = c.steer(&state(0.1, 0.0, 0.0), 0.02);
        assert!(
            u > 0.0,
            "positive cross-track error must steer right (+), got {u}"
        );
    }

    #[test]
    fn negative_cross_track_error_steers_left() {
        let mut c = StaticGain::new(gains(), vehicle());
        let u = c.steer(&state(-0.1, 0.0, 0.0), 0.02);
        assert!(
            u < 0.0,
            "negative cross-track error must steer left (-), got {u}"
        );
    }

    #[test]
    fn positive_heading_error_steers_right() {
        let mut c = StaticGain::new(gains(), vehicle());
        let u = c.steer(&state(0.0, 0.05, 0.0), 0.02);
        assert!(
            u > 0.0,
            "positive heading error must steer right (+), got {u}"
        );
    }

    #[test]
    fn zero_error_and_zero_curvature_gives_zero_steer() {
        let mut c = StaticGain::new(gains(), vehicle());
        let u = c.steer(&state(0.0, 0.0, 0.0), 0.02);
        assert!(u.abs() < 1e-6, "expected exactly zero, got {u}");
    }

    /// `k_lat = 181.17 deg/m` converted to rad/m must equal the raw LQR
    /// solution quoted in `rover_kinematic_control_node.py`'s docstring:
    /// `k1 = 3.162 rad/m`. Isolate the cross-track term by zeroing lookahead
    /// (a controller built with `lookahead_m = 0.0` reads `cross_track_m`
    /// unchanged) and heading/curvature.
    #[test]
    fn k_lat_converts_to_the_documented_3_162_rad_per_metre() {
        let mut c = StaticGain::new(gains(), VehicleParams::new(0.4875, 0.0));
        let u = c.steer(&state(1.0, 0.0, 0.0), 0.02);
        assert!(
            (u - 3.162).abs() < 1e-2,
            "expected ~3.162 rad for a 1m offset (k1 from the ROS2 docstring), got {u}"
        );
    }

    /// `k_head = 2.024 deg/deg` is unit-invariant: converting to degrees,
    /// scaling, and converting back to radians is algebraically the same as
    /// scaling the radian value directly. This is the identity that a
    /// mistaken extra (or missing) `to_radians()`/`to_degrees()` on this
    /// term alone would break.
    #[test]
    fn k_head_is_unit_invariant_between_degrees_and_radians() {
        let mut c = StaticGain::new(gains(), VehicleParams::new(0.4875, 0.0));
        let heading = 0.1_f32; // radians
        let u = c.steer(&state(0.0, heading, 0.0), 0.02);
        let expected = gains().k_head_deg_per_deg * heading;
        assert!(
            (u - expected).abs() < 1e-6,
            "expected exactly k_head * heading_rad = {expected}, got {u}"
        );
    }

    /// The feedforward term must be exactly the Ackermann angle for the
    /// curvature ahead, independent of the feedback gains. Isolated with
    /// `lookahead_m = 0.0` so nonzero curvature doesn't also perturb the
    /// lookahead-reconstructed cross-track/heading feedback terms (see
    /// `lookahead_reconstruction_changes_the_output` for that interaction).
    #[test]
    fn feedforward_matches_ackermann_atan() {
        let mut c = StaticGain::new(gains(), VehicleParams::new(0.4875, 0.0));
        let kappa = 0.05; // 1/m, a 20 m radius curve
        let u = c.steer(&state(0.0, 0.0, kappa), 0.02);
        let expected = (0.4875_f32 * kappa).atan();
        assert!(
            (u - expected).abs() < 1e-4,
            "expected feedforward-only output {expected}, got {u}"
        );
    }

    /// Lookahead reconstruction must actually be in the loop: with nonzero
    /// curvature, the lookahead-referenced cross-track and heading pick up
    /// extra terms (`l_a*e_psi + kappa*l_a^2/2` and `l_a*kappa`) beyond the
    /// raw front-axle state, so the output must differ from what a
    /// controller built with `lookahead_m = 0` would produce.
    #[test]
    fn lookahead_reconstruction_changes_the_output() {
        let s = state(0.05, 0.02, 0.1);
        let mut with_lookahead = StaticGain::new(gains(), vehicle());
        let mut without_lookahead = StaticGain::new(gains(), VehicleParams::new(0.4875, 0.0));
        let u_with = with_lookahead.steer(&s, 0.02);
        let u_without = without_lookahead.steer(&s, 0.02);
        assert!(
            (u_with - u_without).abs() > 1e-3,
            "lookahead reconstruction had no effect: {u_with} vs {u_without}"
        );
    }

    #[test]
    fn name_identifies_the_law() {
        let c = StaticGain::new(gains(), vehicle());
        assert_eq!(c.name(), "static_gain");
    }

    #[test]
    fn reset_does_not_panic_and_is_a_true_no_op() {
        let mut c = StaticGain::new(gains(), vehicle());
        let s = state(0.1, 0.05, 0.02);
        let before = c.steer(&s, 0.02);
        c.reset();
        let after = c.steer(&s, 0.02);
        assert_eq!(
            before, after,
            "StaticGain is stateless; reset must not change its output"
        );
    }

    /// Sanity bound: a large but plausible field error should not produce an
    /// absurd multi-radian command before saturation is applied (that guard
    /// rail lives in `crate::actuate`, but the raw law output for realistic
    /// inputs should stay in a sane ballpark, not the ~57x-too-hard failure
    /// mode a dropped unit conversion would produce).
    #[test]
    fn realistic_error_produces_a_bounded_raw_command() {
        let mut c = StaticGain::new(gains(), vehicle());
        // 10 cm off-line, ~3 degrees of heading error, a gentle curve.
        let u = c.steer(&state(0.10, 3.0_f32.to_radians(), 0.02), 0.02);
        assert!(
            u.abs() < PI / 2.0,
            "raw command {u} rad is implausibly large for a modest field error \
             — suspect a dropped or duplicated unit conversion"
        );
    }
}
