//! Stage 1: estimate. A thin, deterministic wrapper around
//! `rover_estimator::Ekf` that turns the raw feeds (`ImuSample`,
//! `LaneMeasurement`, `WheelSensors`) into ticks of the filter.
//!
//! # Why `dt` comes from `ImuSample.t_us`, not a wall clock read here
//!
//! `rover_estimator::Ekf::predict`'s own docs are explicit: `Q` is a
//! continuous-time spectral density scaled by `dt`, specifically so that a
//! late or dropped IMU packet widens the covariance instead of being trusted
//! as if it arrived on schedule. Reading `Instant::now()` inside this module
//! would measure "how long since this function was last called", which
//! conflates scheduling jitter on the control thread with the sensor's own
//! timing. `ImuSample.t_us` is the chassis board's own free-running
//! microsecond clock, sampled at the moment the IMU was actually read, so
//! differencing consecutive values (with wrapping arithmetic, since it is a
//! 32-bit counter that wraps every ~71.6 minutes) gives the true elapsed
//! sensor time regardless of anything that happened to the packet in
//! between. `main`'s control loop calls [`Estimator::predict_from_imu`] once
//! per tick with whatever the newest `ImuSample` is; if the network thread
//! hasn't delivered a new one since the last tick, `t_us` is unchanged, the
//! delta is zero, and this is treated as "nothing to advance" rather than a
//! spurious zero-length predict.

use rover_estimator::{Ekf, EstimatorConfig};
use rover_model::VehicleParams;
use rover_msgs::{EkfDebug, ImuSample, LaneMeasurement, RoverState, WheelSensors};

/// Owns the filter and just enough bookkeeping to turn wall-clock-free wire
/// messages into well-defined `predict`/`correct` calls.
pub struct Estimator {
    ekf: Ekf,
    cfg: EstimatorConfig,
    /// `t_us` of the last `ImuSample` a predict was actually run for.
    /// `None` until the first sample arrives — there is nothing to
    /// difference against yet, mirroring `Ekf::correct_wheel_sensors`'s own
    /// "seed on the first call" idiom.
    last_imu_t_us: Option<u32>,
}

impl Estimator {
    pub fn at_rest(
        params: VehicleParams,
        cfg: EstimatorConfig,
        initial_p_diag: [f32; rover_msgs::EKF_STATES],
    ) -> Self {
        Self {
            ekf: Ekf::at_rest(params, initial_p_diag),
            cfg,
            last_imu_t_us: None,
        }
    }

    /// Predict from the newest `ImuSample`. Returns the freshly predicted
    /// state, or `None` when there was nothing new to advance from (the very
    /// first sample this `Estimator` has ever seen, seeding `t_us`; or a
    /// repeat of the same sample the previous tick already consumed).
    pub fn predict_from_imu(&mut self, sample: &ImuSample) -> Option<RoverState> {
        let gyro_z = sample.gyro_radps[2];
        let prev_t_us = self.last_imu_t_us.replace(sample.t_us)?;
        let dt_us = sample.t_us.wrapping_sub(prev_t_us);
        if dt_us == 0 {
            return None;
        }
        let dt_s = dt_us as f32 / 1_000_000.0;
        self.ekf.predict(gyro_z, dt_s, &self.cfg);
        Some(self.ekf.state())
    }

    /// Camera correction; see `Ekf::correct_camera` for the coasting
    /// semantics on `meas.valid == false`.
    pub fn correct_lane(&mut self, meas: &LaneMeasurement) -> Option<EkfDebug> {
        self.ekf.correct_camera(meas, &self.cfg)
    }

    /// Wheel-tick correction: odometry while moving, zero-rate gyro-bias
    /// update while stationary. `throttle` is the last commanded throttle
    /// (from the actuate stage), used to decide which of the two this is —
    /// see `Ekf::correct_wheel_sensors`.
    pub fn correct_wheel(&mut self, wheel: &WheelSensors, throttle: f32, gyro_z: f32) {
        self.ekf
            .correct_wheel_sensors(wheel, throttle, gyro_z, &self.cfg);
    }

    /// The current fused estimate, without advancing anything. Useful right
    /// after construction, before the first `ImuSample` has arrived.
    pub fn state(&self) -> RoverState {
        self.ekf.state()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rover_msgs::WheelSensors as Wheel;

    fn params() -> VehicleParams {
        VehicleParams::new(0.4875, 1.22)
    }

    fn cfg() -> EstimatorConfig {
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

    fn imu(t_us: u32, gyro_z: f32) -> ImuSample {
        ImuSample {
            accel_mps2: [0.0; 3],
            gyro_radps: [0.0, 0.0, gyro_z],
            t_us,
        }
    }

    #[test]
    fn first_sample_only_seeds_and_predicts_nothing() {
        let mut est = Estimator::at_rest(params(), cfg(), [1e-2; 5]);
        let before = est.state();
        let out = est.predict_from_imu(&imu(1_000, 0.5));
        assert!(out.is_none());
        assert_eq!(
            est.state(),
            before,
            "a seeding call must not advance the filter"
        );
    }

    #[test]
    fn dt_is_computed_from_t_us_delta_not_a_nominal_period() {
        let mut est = Estimator::at_rest(params(), cfg(), [1e-2; 5]);
        est.predict_from_imu(&imu(0, 0.0)); // seed

        // A gyro bias of 0 and a big, deliberately non-round dt (37ms, not
        // 10ms) drives heading error at exactly gyro*dt if dt is being
        // measured correctly rather than assumed to be 1/predict_hz.
        let gyro = 0.2_f32;
        let out = est.predict_from_imu(&imu(37_000, gyro)).unwrap();
        let expected_heading = gyro * 0.037;
        assert!(
            (out.heading_err_rad - expected_heading).abs() < 1e-5,
            "got {}, expected ~{}",
            out.heading_err_rad,
            expected_heading
        );
    }

    #[test]
    fn repeated_t_us_is_treated_as_no_new_sample() {
        let mut est = Estimator::at_rest(params(), cfg(), [1e-2; 5]);
        est.predict_from_imu(&imu(0, 0.0));
        est.predict_from_imu(&imu(10_000, 1.0)).unwrap();
        let after_first = est.state();

        // Same t_us again (e.g. the control loop ticked faster than the
        // network delivered a fresh packet): must be a no-op, not a
        // zero-dt-but-still-ran predict.
        let out = est.predict_from_imu(&imu(10_000, 1.0));
        assert!(out.is_none());
        assert_eq!(est.state(), after_first);
    }

    #[test]
    fn wrapping_t_us_still_produces_a_sane_forward_dt() {
        let mut est = Estimator::at_rest(params(), cfg(), [1e-2; 5]);
        // Seed right near the u32 wrap boundary.
        est.predict_from_imu(&imu(u32::MAX - 5_000, 0.0));
        // Next sample wraps around to a small absolute value 10ms later.
        let out = est.predict_from_imu(&imu(5_000, 0.3)).unwrap();
        let expected_heading = 0.3 * 0.010;
        assert!(
            (out.heading_err_rad - expected_heading).abs() < 1e-5,
            "wraparound dt not handled: got {}",
            out.heading_err_rad
        );
    }

    #[test]
    fn correct_lane_and_correct_wheel_delegate_without_panicking() {
        let mut est = Estimator::at_rest(params(), cfg(), [1e-2; 5]);
        est.predict_from_imu(&imu(0, 0.0));
        est.predict_from_imu(&imu(10_000, 0.0));

        let dbg = est.correct_lane(&LaneMeasurement {
            curvature_inv_m: 0.0,
            heading_err_rad: 0.01,
            cross_track_m: 0.02,
            valid: true,
            t_us: 10_000,
        });
        assert!(dbg.is_some());

        // First wheel sample only seeds; must not panic.
        est.correct_wheel(
            &Wheel {
                ticks_left: 0,
                ticks_right: 0,
                t_us: 0,
            },
            0.0,
            0.0,
        );
        est.correct_wheel(
            &Wheel {
                ticks_left: 5,
                ticks_right: 5,
                t_us: 100_000,
            },
            0.0,
            0.0,
        );
    }
}
