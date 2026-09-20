//! Synthetic trace generation. Stands in for real `runs/*.csv` field data,
//! which does not exist in this repository — see `crate::trace`'s module
//! docs. Every row's `recorded_steer_deg` is filled by
//! `crate::legacy::LegacyEmaLaw`, driven by the *same* underlying "true"
//! lane geometry the row's `LaneMeasurement`-shaped fields carry, so the
//! comparison in `crate::replay` is meaningful: both the old EMA law and the
//! new EKF-based pipeline see the same physical signal, just through
//! different filters.
//!
//! Two scenarios:
//! - [`Scenario::Steady`]: continuous, always-valid lane detection through a
//!   gentle curve — the main parity check, expected to track tightly.
//! - [`Scenario::Dropout`]: the same curve, but with an extended
//!   lane-detection dropout in the middle. **Expected to diverge** during
//!   the dropout — see the module docs on why that divergence is a known,
//!   intentional difference (plan §13.3 deviation 5) and not a bug: the old
//!   law snaps to `steer_when_lost = 0.0`, while the EKF coasts on gyro and
//!   keeps reporting a live (growing-uncertainty) estimate. `crate::replay`
//!   reports this scenario's dropout window separately from its pass/fail
//!   gate for exactly this reason.

use crate::legacy::{LegacyConfig, LegacyEmaLaw};
use crate::trace::{Trace, TraceRow};

pub const PREDICT_HZ: f32 = 100.0;
/// Camera runs slower than the 100 Hz predict loop; every `CAMERA_DECIMATION`th
/// tick carries a fresh `LaneMeasurement`, matching roughly 33 Hz — the
/// plan's "close to the real camera rate" figure (§2's estimator tests use
/// the same ratio).
const CAMERA_DECIMATION: u64 = 3;
/// Wheel encoders at 10 Hz.
const WHEEL_DECIMATION: u64 = 10;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scenario {
    Steady,
    Dropout,
}

/// Generate `duration_s` seconds of synthetic driving at [`PREDICT_HZ`].
///
/// The "true" trajectory is a simple, smooth model, not a physically
/// simulated vehicle: cross-track error decays from an initial offset
/// toward a gentle steady-state wobble, heading error tracks it loosely, and
/// curvature ramps into a mild, constant-radius bend partway through. This
/// is enough to exercise the EKF's predict/correct cycle and the controller
/// across a curve without pretending to be a validated vehicle simulator —
/// it earns its keep as a controlled input for a **regression** check
/// (`crate::legacy` vs `rover_control::guide::StaticGain`), not as a claim
/// about real-world behaviour.
pub fn generate(scenario: Scenario, duration_s: f32, legacy_cfg: LegacyConfig) -> Trace {
    let n_ticks = (duration_s * PREDICT_HZ) as u64;
    let dt_s = 1.0 / PREDICT_HZ;

    let mut legacy = LegacyEmaLaw::new(legacy_cfg);
    let mut rows = Vec::with_capacity(n_ticks as usize);

    let (dropout_start_tick, dropout_len_ticks) = dropout_window_ticks(n_ticks);

    let mut ticks_left: i32 = 0;
    let mut ticks_right: i32 = 0;
    // A speed and a wheel/gyro relationship consistent enough to be a sane
    // synthetic input, not a calibrated constant: metres_per_tick is
    // deliberately never assumed here (matching this rover's own
    // uncalibrated standing state, docs/STATUS_OPEN.md §1.1) — ticks are advanced at a
    // fixed, made-up rate purely so `WheelSensors` corrections have
    // something nonzero to act on.
    const SYNTHETIC_TICKS_PER_SEC: f32 = 200.0;
    const SPEED_MPS: f32 = 0.20;

    for i in 0..n_ticks {
        let t_s = i as f32 * dt_s;
        let t_us = (t_s * 1_000_000.0) as u64;

        let in_dropout = scenario == Scenario::Dropout
            && i >= dropout_start_tick
            && i < dropout_start_tick + dropout_len_ticks;

        // "True" trajectory: exponential decay of an initial 0.15m offset,
        // plus a mild curve from t=2s onward.
        let true_cross_track_m = 0.15 * (-t_s / 3.0).exp();
        let true_curvature_inv_m = if t_s > 2.0 { 0.03 } else { 0.0 }; // ~33m radius
        let true_heading_err_rad = -true_cross_track_m.signum() * 0.02 * (1.0 - (-t_s / 2.0).exp());

        // Gyro z consistent with the curvature and forward speed: yaw rate
        // for a bicycle model tracking this curvature at SPEED_MPS.
        let gyro_z_radps = SPEED_MPS * true_curvature_inv_m;

        let lane_valid = !in_dropout && i.is_multiple_of(CAMERA_DECIMATION);
        let wheel_valid = i.is_multiple_of(WHEEL_DECIMATION);
        if wheel_valid {
            let delta = (SYNTHETIC_TICKS_PER_SEC * WHEEL_DECIMATION as f32 / PREDICT_HZ) as i32;
            ticks_left += delta;
            ticks_right += delta;
        }

        // The legacy oracle only advances on genuinely fresh camera frames,
        // exactly like the ROS2 node only ran per `tpc_rover_nav_lane`
        // message rather than once per control tick.
        let recorded_steer_deg = if i.is_multiple_of(CAMERA_DECIMATION) {
            Some(legacy.step(
                true_heading_err_rad,
                true_cross_track_m,
                true_curvature_inv_m,
                lane_valid,
            ))
        } else {
            None // mid-decimation ticks: no new legacy output to compare against
        };

        rows.push(TraceRow {
            t_us,
            gyro_z_radps,
            lane_valid,
            cross_track_m: true_cross_track_m,
            heading_err_rad: true_heading_err_rad,
            curvature_inv_m: true_curvature_inv_m,
            wheel_valid,
            ticks_left,
            ticks_right,
            throttle: 0.2,
            recorded_steer_deg,
        });
    }

    Trace { rows }
}

/// The `(start_tick, length_ticks)` of the deliberate dropout window
/// `Scenario::Dropout` introduces, for a trace of `n_ticks` total rows: 3
/// seconds starting at the 40% mark, comfortably inside the plan §2.4
/// "5-15s of useful coasting" window — long enough to see the old and new
/// laws diverge, short enough that the EKF should not yet be lost.
///
/// Exposed publicly so `main.rs` can report that window's statistics
/// separately from the pass/fail gate (see this module's docs on why that
/// divergence is expected, not a regression) using the same row indices
/// [`generate`] used to build the trace, rather than duplicating the
/// formula.
pub fn dropout_window_ticks(n_ticks: u64) -> (u64, u64) {
    let start = (0.4 * n_ticks as f32) as u64;
    let len = (3.0 * PREDICT_HZ) as u64;
    (start, len)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steady_scenario_marks_every_camera_tick_valid() {
        let trace = generate(Scenario::Steady, 5.0, LegacyConfig::default());
        // Every tick that lands on the camera's decimation must be valid —
        // `Scenario::Steady` never introduces a dropout.
        for (i, r) in trace.rows.iter().enumerate() {
            if (i as u64).is_multiple_of(CAMERA_DECIMATION) {
                assert!(
                    r.lane_valid,
                    "row {i} should be a valid camera frame in Steady"
                );
            }
        }
    }

    #[test]
    fn dropout_scenario_has_an_invalid_stretch() {
        let trace = generate(Scenario::Dropout, 6.0, LegacyConfig::default());
        let invalid_run = trace.rows.iter().filter(|r| !r.lane_valid).count();
        assert!(
            invalid_run > 200,
            "expected a multi-second dropout, got {invalid_run} invalid rows"
        );
    }

    #[test]
    fn ticks_advance_monotonically() {
        let trace = generate(Scenario::Steady, 3.0, LegacyConfig::default());
        let mut prev = i32::MIN;
        for r in &trace.rows {
            assert!(r.ticks_left >= prev, "ticks_left must never go backward");
            prev = r.ticks_left;
        }
    }
}
