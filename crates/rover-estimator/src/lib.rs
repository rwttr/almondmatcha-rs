//! 5-state EKF for lane-relative pose. Plan §2.2.
//!
//! # State, referenced to the FRONT AXLE
//!
//! ```text
//! x = [ e_y      cross-track error at the front axle   (m)
//!       e_psi    heading error vs. the lane tangent    (rad)
//!       kappa    lane curvature ahead                  (1/m)
//!       v        forward speed                         (m/s)
//!       b_g    ] gyro z-axis bias                        (rad/s)
//! ```
//!
//! Index order matches [`rover_msgs::state_idx`] exactly; [`rover_model`]
//! builds the process Jacobian against the same indices, so there is exactly
//! one place a state gets reordered by mistake, and it would fail to compile
//! rather than silently swap two rows.
//!
//! # Sign convention — read this before wiring up a controller
//!
//! `cross_track_m` and `heading_err_rad` are positive when the correct
//! response is to **steer right**. This is the opposite of the textbook
//! ISO 8855 convention (`-k1*e_lat - k2*e_heading`); the field-tuned gains
//! `k_lat = 181.17 deg/m` and `k_head = 2.024 deg/deg` (plan §10) assume the
//! **plus** sign on both terms. This filter does not itself compute a
//! steering command, but every measurement it consumes and every state it
//! reports uses this convention, and getting it backwards anywhere in the
//! pipeline turns a stabilising controller into positive feedback. See
//! `rover_msgs`'s crate docs and plan §10; `tests/ekf.rs` in this crate has
//! the executable check.
//!
//! # Coasting, not holding
//!
//! When [`rover_msgs::LaneMeasurement::valid`] is `false`, [`Ekf::correct_camera`]
//! does nothing at all — no state change, no `EkfDebug`. The filter keeps
//! predicting forward on gyro (and odometry, if calibrated), so the estimate
//! and its uncertainty both keep evolving through a dropout instead of
//! freezing at the last good camera fix. `lane_age_ms` is the observable
//! signal for how long that has been going on; it resets on any *received*
//! camera measurement, whether or not the chi-square gate then accepts it —
//! a gated measurement still proves the camera is alive, just that this one
//! reading looked like an outlier. A real dropout (`valid == false`) is a
//! different failure mode from a persistently gated one (a diverged filter,
//! or sustained glare) and the two are deliberately reported through
//! different signals: `lane_age_ms` for the former, [`rover_msgs::EkfDebug`]
//! for the latter.
//!
//! # The `metres_per_tick == 0.0` trap
//!
//! `config/rover.toml`'s `drivetrain.metres_per_tick` is currently `0.0`
//! (uncalibrated, plan §2.6). [`Ekf::correct_wheel_sensors`] checks for this
//! explicitly and disables the odometry branch entirely rather than dividing
//! by it or feeding a bogus all-zero speed measurement into the filter — a
//! silent zero would look like perfectly confident evidence that the rover
//! is stationary, which is worse than no measurement at all. It logs a
//! `log::warn!` once per [`Ekf`] instance so the condition is loud without
//! spamming a 10 Hz loop, and [`config::EstimatorConfig::odometry_calibrated`]
//! lets a caller (or a test) check the same condition programmatically.

#![forbid(unsafe_code)]

pub mod config;

use nalgebra::{SMatrix, SVector};
use rover_model::{discretize, process_derivative, process_jacobian, StateMatrix, StateVector, VehicleParams};
use rover_msgs::{state_idx, EkfDebug, LaneMeasurement, RoverState, WheelSensors, EKF_STATES};

pub use config::{EstimatorConfig, LaneNoise, ProcessNoise};

/// The filter. Owns its belief (`x`, `P`) and just enough bookkeeping
/// (previous wheel sample, a one-shot warning flag) to make its updates
/// well-defined; nothing here touches a clock, a socket, or a file.
#[derive(Debug, Clone)]
pub struct Ekf {
    x: StateVector,
    p: StateMatrix,
    params: VehicleParams,
    /// Milliseconds since the last *received* camera measurement (gated or
    /// not). Saturates rather than wraps — see the module docs.
    lane_age_ms: u16,
    last_wheel: Option<WheelSensors>,
    odometry_disabled_warned: bool,
}

impl Ekf {
    /// Build a filter with an explicit initial belief. Mostly useful for
    /// tests that need to start from a known offset; a real startup should
    /// generally prefer [`Ekf::at_rest`].
    pub fn new(
        params: VehicleParams,
        initial_state: StateVector,
        initial_p_diag: [f32; EKF_STATES],
    ) -> Self {
        let mut p = StateMatrix::zeros();
        for i in 0..EKF_STATES {
            p[(i, i)] = initial_p_diag[i];
        }
        Self {
            x: initial_state,
            p,
            params,
            lane_age_ms: 0,
            last_wheel: None,
            odometry_disabled_warned: false,
        }
    }

    /// A filter initialised to zero error, zero speed, zero bias — the
    /// ordinary startup condition (the rover is placed on the line at rest).
    pub fn at_rest(params: VehicleParams, initial_p_diag: [f32; EKF_STATES]) -> Self {
        Self::new(params, StateVector::zeros(), initial_p_diag)
    }

    /// Fixed-rate predict step. Call this every tick of the estimator's
    /// 100 Hz loop (`config/rover.toml`: `estimator.predict_hz`) regardless
    /// of whether any measurement arrived — this is what fixes the ROS 2
    /// system's defect of a `dt` that was "whatever the camera did last"
    /// (plan §2.3).
    ///
    /// `gyro_radps` is the raw (bias-uncorrected) gyro z-axis reading; the
    /// process model subtracts the filter's own bias estimate internally.
    pub fn predict(&mut self, gyro_radps: f32, dt_s: f32, cfg: &EstimatorConfig) {
        // Linearise at the pre-step estimate, then advance the mean with the
        // same nonlinear model the Jacobian was taken from.
        let f_c = process_jacobian(&self.x);
        let f_d = discretize(&f_c, dt_s);

        let dx = process_derivative(&self.x, gyro_radps);
        self.x += dx * dt_s;

        // Q is a continuous-time spectral density, so it is scaled by dt.
        // The predict step is driven by IMU packet arrival over UDP, not a
        // hardware timer: dt jitters, and a dropped packet makes it several
        // times nominal. A per-tick Q would then accumulate the same process
        // noise for a 30 ms gap as for a 10 ms one, leaving the filter MORE
        // confident precisely when it has least information — the failure
        // mode that makes an estimator reject the truth once it is lost.
        self.p = f_d * self.p * f_d.transpose() + cfg.q.as_diag_matrix() * dt_s;

        // Saturating: a long dropout must read as "very stale," never wrap
        // back around to "fresh." `as u16` on a float saturates by Rust's
        // own cast semantics; `saturating_add` then protects the running
        // total against overflow across many predict ticks.
        let dt_ms = (dt_s * 1000.0).round() as u16;
        self.lane_age_ms = self.lane_age_ms.saturating_add(dt_ms);
    }

    /// Camera correction. Plan §2.2's linear measurement model under
    /// small-angle:
    ///
    /// ```text
    /// h(x)  = [ e_y + L_a*e_psi + kappa*L_a^2/2 ,  e_psi + L_a*kappa ,  kappa ]
    /// H_cam = [ 1  L_a  L_a^2/2  0  0 ]
    ///         [ 0   1     L_a    0  0 ]
    ///         [ 0   0      1     0  0 ]
    /// ```
    ///
    /// Returns `None` when `meas.valid` is `false` — the filter coasts, full
    /// stop, no `EkfDebug` emitted (see the module docs). Returns
    /// `Some(EkfDebug)` for every measurement that *was* received, whether
    /// the chi-square gate then accepted it or rejected it as an outlier.
    pub fn correct_camera(&mut self, meas: &LaneMeasurement, cfg: &EstimatorConfig) -> Option<EkfDebug> {
        if !meas.valid {
            return None;
        }
        let l_a = self.params.lookahead_m;
        let l_a2 = l_a * l_a;

        let mut h = SMatrix::<f32, 3, EKF_STATES>::zeros();
        h[(0, state_idx::CROSS_TRACK)] = 1.0;
        h[(0, state_idx::HEADING_ERR)] = l_a;
        h[(0, state_idx::CURVATURE)] = 0.5 * l_a2;
        h[(1, state_idx::HEADING_ERR)] = 1.0;
        h[(1, state_idx::CURVATURE)] = l_a;
        h[(2, state_idx::CURVATURE)] = 1.0;

        let z = SVector::<f32, 3>::new(meas.cross_track_m, meas.heading_err_rad, meas.curvature_inv_m);
        let y = z - h * self.x;

        let r = SMatrix::<f32, 3, 3>::from_diagonal(&SVector::<f32, 3>::new(
            cfg.r_lane.cross_track,
            cfg.r_lane.heading,
            cfg.r_lane.curvature,
        ));
        let s = h * self.p * h.transpose() + r;

        let Some(s_inv) = s.try_inverse() else {
            // A singular innovation covariance means the filter's own P has
            // collapsed to something degenerate. Treat it as gated rather
            // than propagating a NaN/garbage update or panicking on the
            // inverse.
            return Some(EkfDebug {
                innovation: [y[0], y[1], y[2]],
                nis: f32::INFINITY,
                gated: true,
            });
        };

        let nis = (y.transpose() * s_inv * y)[(0, 0)];
        let gated = nis > cfg.nis_gate;

        if !gated {
            // Reset only on an ACCEPTED update. `lane_age_ms` answers "how
            // long since lane information actually entered the estimate",
            // because that is the question guidance is asking when it decides
            // how much to trust the state and how fast to drive.
            //
            // A gated reading proves the camera is alive but contributes
            // nothing to the estimate. Counting it as fresh would let a
            // detector producing consistent garbage read as healthy while the
            // filter silently coasts with zero corrections. Camera liveness is
            // tracked separately, by `HealthBits::LANE_STALE` in telemetry.
            self.lane_age_ms = 0;

            let k = self.p * h.transpose() * s_inv;
            self.x += k * y;
            // Joseph form: numerically stable (stays symmetric and PSD under
            // rounding error) at the cost of a couple of extra matrix
            // multiplies the RPi 4 will not notice at 30 Hz.
            let i_kh = StateMatrix::identity() - k * h;
            self.p = i_kh * self.p * i_kh.transpose() + k * r * k.transpose();
        }

        Some(EkfDebug {
            innovation: [y[0], y[1], y[2]],
            nis,
            gated,
        })
    }

    /// Wheel-tick correction, at the 10 Hz `WheelSensors` rate.
    ///
    /// Does one of two things, depending on whether the rover is moving:
    ///
    /// - **Stationary** (`throttle` is ~zero and neither encoder ticked since
    ///   the last sample): a zero-rate update, `z = gyro_radps` against
    ///   `h(x) = b_g`. This is the *only* place gyro bias becomes observable
    ///   (plan §2.2) — it runs for free every time the rover pauses.
    /// - **Moving**: an odometry update, `z = v` from the tick delta, unless
    ///   `metres_per_tick` is `0.0` (uncalibrated), in which case it is
    ///   skipped — see the module docs on the `0.0` trap.
    ///
    /// The first call after construction only seeds `last_wheel` and returns
    /// without updating anything: there is nothing to difference yet.
    pub fn correct_wheel_sensors(
        &mut self,
        wheel: &WheelSensors,
        throttle: f32,
        gyro_radps: f32,
        cfg: &EstimatorConfig,
    ) {
        let prev = match self.last_wheel.replace(*wheel) {
            Some(prev) => prev,
            None => return,
        };

        // `t_us` is a free-running 32-bit microsecond counter that wraps
        // every ~71.6 minutes; `wrapping_sub` recovers the true forward
        // delta across a wrap as long as consecutive samples are closer
        // together than that, which is always true at 10 Hz.
        let dt_us = wheel.t_us.wrapping_sub(prev.t_us);
        if dt_us == 0 {
            return; // duplicate sample, nothing to difference
        }
        let dt_s = dt_us as f32 / 1_000_000.0;

        let delta_left = wheel.ticks_left.wrapping_sub(prev.ticks_left);
        let delta_right = wheel.ticks_right.wrapping_sub(prev.ticks_right);

        const THROTTLE_EPS: f32 = 1e-3;
        let tol = cfg.zero_rate_max_ticks;
        let quiet = throttle.abs() < THROTTLE_EPS
            && delta_left.abs() <= tol
            && delta_right.abs() <= tol;

        if quiet {
            self.scalar_update(state_idx::GYRO_BIAS, gyro_radps, cfg.r_zero_rate_gyro);
        } else {
            self.correct_odometry(delta_left, delta_right, dt_s, cfg);
        }
    }

    fn correct_odometry(&mut self, delta_left: i32, delta_right: i32, dt_s: f32, cfg: &EstimatorConfig) {
        if !cfg.odometry_calibrated() {
            if !self.odometry_disabled_warned {
                log::warn!(
                    "rover-estimator: drivetrain.metres_per_tick is 0.0 (uncalibrated, plan §2.6) \
                     — odometry update DISABLED; run the §2.6 calibration procedures before \
                     trusting speed_mps"
                );
                self.odometry_disabled_warned = true;
            }
            return;
        }
        let avg_ticks = (delta_left as f32 + delta_right as f32) * 0.5;
        let z = avg_ticks * cfg.metres_per_tick / dt_s;
        self.scalar_update(state_idx::SPEED, z, cfg.r_odom_speed);
    }

    /// Kalman update for a direct scalar observation of one state,
    /// `h(x) = x[idx]`, `H = e_idx`. Both the odometry update (`idx = v`) and
    /// the zero-rate update (`idx = b_g`) have exactly this shape, so there
    /// is one implementation instead of two hand-derived ones that could
    /// disagree.
    fn scalar_update(&mut self, idx: usize, z: f32, r: f32) {
        let y = z - self.x[idx];
        let s = self.p[(idx, idx)] + r;
        if s <= 0.0 || s.is_nan() {
            // Degenerate covariance or negative/zero noise config: skip
            // rather than divide by (near) zero.
            return;
        }
        let k = self.p.column(idx).into_owned() / s;
        self.x += k * y;

        let mut i_kh = StateMatrix::identity();
        for j in 0..EKF_STATES {
            i_kh[(j, idx)] -= k[j];
        }
        self.p = i_kh * self.p * i_kh.transpose() + (k * r) * k.transpose();
    }

    /// Whether the odometry update is currently disabled because
    /// `metres_per_tick` is uncalibrated. Exposed so tests (and, later,
    /// health reporting) can check this without scraping log output.
    pub fn odometry_disabled(&self) -> bool {
        self.odometry_disabled_warned
    }

    /// The fused estimate, in wire format, including the `P` diagonal and
    /// `lane_age_ms`.
    pub fn state(&self) -> RoverState {
        RoverState {
            cross_track_m: self.x[state_idx::CROSS_TRACK],
            heading_err_rad: self.x[state_idx::HEADING_ERR],
            curvature_inv_m: self.x[state_idx::CURVATURE],
            speed_mps: self.x[state_idx::SPEED],
            gyro_bias_radps: self.x[state_idx::GYRO_BIAS],
            p_diag: [
                self.p[(state_idx::CROSS_TRACK, state_idx::CROSS_TRACK)],
                self.p[(state_idx::HEADING_ERR, state_idx::HEADING_ERR)],
                self.p[(state_idx::CURVATURE, state_idx::CURVATURE)],
                self.p[(state_idx::SPEED, state_idx::SPEED)],
                self.p[(state_idx::GYRO_BIAS, state_idx::GYRO_BIAS)],
            ],
            lane_age_ms: self.lane_age_ms,
        }
    }
}
