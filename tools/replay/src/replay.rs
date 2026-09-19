//! The actual replay: drives `rover_control`'s exact production
//! `Estimator` + `StaticGain` over a [`Trace`], with no network and no
//! hardware anywhere in the loop, and reports per-sample recorded-vs-computed
//! steering plus summary error statistics.

use crate::trace::Trace;
use rover_control::estimate::Estimator;
use rover_control::guide::{LateralController, StaticGain, StaticGainConfig};
use rover_estimator::EstimatorConfig;
use rover_model::VehicleParams;
use rover_msgs::{ImuSample, LaneMeasurement, WheelSensors, EKF_STATES};

/// Matches `rover-control`'s own `main.rs`: the rover starts on the line at
/// rest, so the same conservative initial covariance applies here.
pub const INITIAL_P_DIAG: [f32; EKF_STATES] = [1e-2, 1e-2, 1e-2, 1e-2, 1e-4];

pub struct OutputRow {
    pub t_us: u64,
    pub recorded_steer_deg: Option<f32>,
    pub computed_steer_deg: f32,
    /// `None` for a row with no ground truth, or one excluded by `warmup_s`.
    pub error_deg: Option<f32>,
    pub cross_track_m: f32,
    pub heading_err_rad: f32,
    pub cross_track_sigma_m: f32,
    pub lane_age_ms: u16,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stats {
    pub n_compared: usize,
    pub mean_abs_error_deg: f32,
    pub rmse_deg: f32,
    pub max_abs_error_deg: f32,
}

impl Stats {
    fn from_errors(errors: &[f32]) -> Self {
        let n = errors.len();
        if n == 0 {
            return Self {
                n_compared: 0,
                mean_abs_error_deg: f32::NAN,
                rmse_deg: f32::NAN,
                max_abs_error_deg: f32::NAN,
            };
        }
        let mean_abs_error_deg = errors.iter().map(|e| e.abs()).sum::<f32>() / n as f32;
        let rmse_deg = (errors.iter().map(|e| e * e).sum::<f32>() / n as f32).sqrt();
        let max_abs_error_deg = errors.iter().fold(0.0_f32, |acc, e| acc.max(e.abs()));
        Self {
            n_compared: n,
            mean_abs_error_deg,
            rmse_deg,
            max_abs_error_deg,
        }
    }
}

pub struct ReplayOutput {
    pub rows: Vec<OutputRow>,
    pub stats: Stats,
}

/// Drive the estimator and controller over every row of `trace`, in order.
///
/// `warmup_s` excludes the first `warmup_s` seconds of the trace (by its own
/// `t_us` clock) from `stats` — the EKF starts at zero-error regardless of
/// where the synthetic/real trajectory actually begins, so an initial
/// convergence transient is expected and not evidence of a parity problem.
/// Per-row output still includes every row, warmup or not; only `stats`
/// (and each row's `error_deg`) excludes it.
pub fn run(
    trace: &Trace,
    vehicle: VehicleParams,
    estimator_cfg: EstimatorConfig,
    static_gain_cfg: StaticGainConfig,
    warmup_s: f32,
) -> ReplayOutput {
    let mut estimator = Estimator::at_rest(vehicle, estimator_cfg, INITIAL_P_DIAG);
    let mut controller = StaticGain::new(static_gain_cfg, vehicle);

    let mut last_state = estimator.state();
    let mut rows = Vec::with_capacity(trace.rows.len());
    let mut errors = Vec::new();

    let t0 = trace.rows.first().map_or(0, |r| r.t_us);
    let warmup_us = (warmup_s.max(0.0) * 1_000_000.0) as u64;

    for row in &trace.rows {
        let imu = ImuSample {
            accel_mps2: [0.0; 3],
            gyro_radps: [0.0, 0.0, row.gyro_z_radps],
            t_us: row.t_us as u32,
        };
        if let Some(state) = estimator.predict_from_imu(&imu) {
            last_state = state;
        }

        if row.lane_valid {
            let meas = LaneMeasurement {
                curvature_inv_m: row.curvature_inv_m,
                heading_err_rad: row.heading_err_rad,
                cross_track_m: row.cross_track_m,
                valid: true,
                t_us: row.t_us as u32,
            };
            estimator.correct_lane(&meas);
            last_state = estimator.state();
        }

        if row.wheel_valid {
            let wheel = WheelSensors {
                ticks_left: row.ticks_left,
                ticks_right: row.ticks_right,
                t_us: row.t_us as u32,
            };
            estimator.correct_wheel(&wheel, row.throttle, row.gyro_z_radps);
            last_state = estimator.state();
        }

        // Nominal predict-rate dt: `StaticGain` ignores it (stateless), but
        // a real replay of a future stateful law would want the trace's
        // own tick period here rather than a hardcoded constant.
        let computed_steer_deg = controller.steer(&last_state, 0.01).to_degrees();

        let past_warmup = row.t_us.saturating_sub(t0) >= warmup_us;
        let error_deg = if past_warmup {
            row.recorded_steer_deg.map(|rec| computed_steer_deg - rec)
        } else {
            None
        };
        if let Some(e) = error_deg {
            errors.push(e);
        }

        rows.push(OutputRow {
            t_us: row.t_us,
            recorded_steer_deg: row.recorded_steer_deg,
            computed_steer_deg,
            error_deg,
            cross_track_m: last_state.cross_track_m,
            heading_err_rad: last_state.heading_err_rad,
            cross_track_sigma_m: last_state.cross_track_sigma(),
            lane_age_ms: last_state.lane_age_ms,
        });
    }

    let stats = Stats::from_errors(&errors);
    ReplayOutput { rows, stats }
}

pub fn write_output_csv(
    rows: &[OutputRow],
    path: impl AsRef<std::path::Path>,
) -> std::io::Result<()> {
    use std::io::Write;
    let mut f = std::fs::File::create(path)?;
    writeln!(
        f,
        "t_us,recorded_steer_deg,computed_steer_deg,error_deg,cross_track_m,heading_err_rad,cross_track_sigma_m,lane_age_ms"
    )?;
    for r in rows {
        writeln!(
            f,
            "{},{},{},{},{},{},{},{}",
            r.t_us,
            r.recorded_steer_deg
                .map(|v| v.to_string())
                .unwrap_or_default(),
            r.computed_steer_deg,
            r.error_deg.map(|v| v.to_string()).unwrap_or_default(),
            r.cross_track_m,
            r.heading_err_rad,
            r.cross_track_sigma_m,
            r.lane_age_ms,
        )?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::legacy::LegacyConfig;
    use crate::synthetic::{self, Scenario};

    fn vehicle() -> VehicleParams {
        VehicleParams::new(0.4875, 1.22)
    }

    fn estimator_cfg() -> EstimatorConfig {
        rover_estimator::EstimatorConfig {
            q: rover_estimator::ProcessNoise::new(1e-2, 1e-2, 1e-1, 1e-1, 1e-5),
            r_lane: rover_estimator::LaneNoise::new(2.5e-3, 2.0e-3, 1.0e-2),
            r_odom_speed: 1.0e-3,
            r_zero_rate_gyro: 1e-6,
            nis_gate: 11.34,
            zero_rate_max_ticks: 0,
            metres_per_tick: 0.0,
        }
    }

    fn static_gain_cfg() -> StaticGainConfig {
        StaticGainConfig {
            k_lat_deg_per_m: 181.17,
            k_head_deg_per_deg: 2.024,
        }
    }

    /// The harness must show close agreement on a steady, always-detected
    /// scenario: this is the "does the port actually work" check.
    #[test]
    fn steady_scenario_tracks_the_legacy_law_closely() {
        let trace = synthetic::generate(Scenario::Steady, 10.0, LegacyConfig::default());
        let out = run(&trace, vehicle(), estimator_cfg(), static_gain_cfg(), 2.0);
        assert!(out.stats.n_compared > 0, "must have compared some samples");
        assert!(
            out.stats.rmse_deg < 3.0,
            "RMSE {} deg is too large for a steady, fully-detected scenario",
            out.stats.rmse_deg
        );
    }

    /// The harness must be capable of FAILING. A grossly wrong `k_lat`
    /// (10x, simulating a dropped-or-duplicated unit conversion — see
    /// `rover_control::guide`'s module docs on the unit trap) must blow the
    /// error stats up, not quietly pass.
    #[test]
    fn a_wrong_gain_produces_a_large_error_the_harness_actually_reports() {
        let trace = synthetic::generate(Scenario::Steady, 10.0, LegacyConfig::default());
        let bad_gains = StaticGainConfig {
            k_lat_deg_per_m: static_gain_cfg().k_lat_deg_per_m * 10.0,
            ..static_gain_cfg()
        };
        let out = run(&trace, vehicle(), estimator_cfg(), bad_gains, 2.0);
        assert!(
            out.stats.rmse_deg > 20.0,
            "a 10x gain error should produce a large, visible RMSE, got {}",
            out.stats.rmse_deg
        );
    }

    /// Same failure-mode check for the unit conversion itself: comparing
    /// degrees against what would happen if `k_head` were applied to
    /// radians instead of degrees (a ~57x error, per `guide`'s module docs).
    #[test]
    fn a_dropped_degrees_conversion_on_k_head_is_caught() {
        let trace = synthetic::generate(Scenario::Steady, 10.0, LegacyConfig::default());
        let correct = static_gain_cfg();
        // Scaling k_head by (180/pi) mimics applying it as though the
        // heading term were still in radians when the formula expects
        // degrees — the exact bug the module docs warn about.
        let wrong_units = StaticGainConfig {
            k_head_deg_per_deg: correct.k_head_deg_per_deg * (180.0 / std::f32::consts::PI),
            ..correct
        };
        let good = run(&trace, vehicle(), estimator_cfg(), correct, 2.0);
        let bad = run(&trace, vehicle(), estimator_cfg(), wrong_units, 2.0);
        assert!(
            bad.stats.rmse_deg > good.stats.rmse_deg * 5.0,
            "good={} bad={}",
            good.stats.rmse_deg,
            bad.stats.rmse_deg
        );
    }

    #[test]
    fn dropout_window_shows_expected_divergence_not_a_crash() {
        let trace = synthetic::generate(Scenario::Dropout, 10.0, LegacyConfig::default());
        let out = run(&trace, vehicle(), estimator_cfg(), static_gain_cfg(), 2.0);
        // Just confirm it runs to completion and produces one output row per
        // input row, including through the dropout.
        assert_eq!(out.rows.len(), trace.rows.len());
    }

    #[test]
    fn warmup_excludes_early_rows_from_stats_but_not_from_output() {
        let trace = synthetic::generate(Scenario::Steady, 5.0, LegacyConfig::default());
        let out_no_warmup = run(&trace, vehicle(), estimator_cfg(), static_gain_cfg(), 0.0);
        let out_with_warmup = run(&trace, vehicle(), estimator_cfg(), static_gain_cfg(), 2.0);
        assert_eq!(out_no_warmup.rows.len(), out_with_warmup.rows.len());
        assert!(out_with_warmup.stats.n_compared < out_no_warmup.stats.n_compared);
    }
}
