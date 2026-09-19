//! Tuning knobs, mirroring `config/rover.toml`'s `[estimator]` table.
//!
//! Parsing TOML is a job for the binary that owns startup I/O (`rover-control`
//! or `tools/replay`) — this crate stays a pure library, so these are plain
//! structs the caller fills in from whatever source it likes (a parsed TOML
//! file, a hardcoded value in a bench test, a sweep in `tools/replay`).

use rover_model::StateMatrix;
use rover_msgs::state_idx;

/// Process noise, one **continuous-time spectral density per state**, in
/// units of (state unit)^2 per second.
///
/// The predict step applies `P += Q * dt`, not `P += Q`. The distinction
/// matters because predict is driven by IMU packet arrival over UDP rather
/// than a hardware timer: `dt` jitters, and a dropped packet makes it several
/// times nominal. Per-tick noise would accumulate the same uncertainty for a
/// 30 ms gap as for a 10 ms one, leaving the filter overconfident exactly when
/// it has least information.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProcessNoise {
    pub cross_track: f32,
    pub heading: f32,
    pub curvature: f32,
    pub speed: f32,
    pub gyro_bias: f32,
}

impl ProcessNoise {
    pub const fn new(
        cross_track: f32,
        heading: f32,
        curvature: f32,
        speed: f32,
        gyro_bias: f32,
    ) -> Self {
        Self {
            cross_track,
            heading,
            curvature,
            speed,
            gyro_bias,
        }
    }

    pub(crate) fn as_diag_matrix(&self) -> StateMatrix {
        let mut q = StateMatrix::zeros();
        q[(state_idx::CROSS_TRACK, state_idx::CROSS_TRACK)] = self.cross_track;
        q[(state_idx::HEADING_ERR, state_idx::HEADING_ERR)] = self.heading;
        q[(state_idx::CURVATURE, state_idx::CURVATURE)] = self.curvature;
        q[(state_idx::SPEED, state_idx::SPEED)] = self.speed;
        q[(state_idx::GYRO_BIAS, state_idx::GYRO_BIAS)] = self.gyro_bias;
        q
    }
}

/// Camera measurement noise, one variance per `LaneMeasurement` channel.
/// `config/rover.toml`: `[estimator.r]` `lane_*`. "MEASURE, do not guess" —
/// the config file's own comment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaneNoise {
    pub cross_track: f32,
    pub heading: f32,
    pub curvature: f32,
}

impl LaneNoise {
    pub const fn new(cross_track: f32, heading: f32, curvature: f32) -> Self {
        Self {
            cross_track,
            heading,
            curvature,
        }
    }
}

/// Everything the filter needs beyond the vehicle geometry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EstimatorConfig {
    pub q: ProcessNoise,
    pub r_lane: LaneNoise,
    /// `config/rover.toml`: `estimator.r.odom_speed`.
    pub r_odom_speed: f32,
    /// Gyro measurement noise used by the zero-rate update, `(rad/s)^2`.
    /// `config/rover.toml`: `estimator.r.zero_rate_gyro`.
    ///
    /// Seed from the LSM6DSV16X's datasheet angular-rate noise density
    /// converted to a variance at the sample rate, then refine against a
    /// stationary bench log — the rover sitting still for a minute gives the
    /// true figure directly, including whatever the chassis contributes.
    pub r_zero_rate_gyro: f32,
    /// Chi-square gate for the camera update, 3 DoF. `config/rover.toml`:
    /// `estimator.nis_gate` (11.34, 99% confidence).
    pub nis_gate: f32,
    /// `config/rover.toml`: `drivetrain.metres_per_tick`. **`0.0` means
    /// uncalibrated** (plan §2.6) — the odometry update must disable itself
    /// rather than divide by it or trust a bogus scale factor.
    pub metres_per_tick: f32,
    /// Largest per-sample tick delta on either wheel still counted as "not
    /// moving" for the zero-rate gyro-bias update. `config/rover.toml`:
    /// `estimator.zero_rate_max_ticks`.
    ///
    /// Exists as a knob rather than a hardcoded exact zero because a real
    /// encoder sitting still can dither by a count on electrical noise or a
    /// wheel resting exactly on an edge transition. A single stray tick must
    /// not be allowed to suppress bias estimation for a whole stop — bias
    /// observability is the thing that bounds how long the rover can coast on
    /// a lost lane. Start at 0 and raise it only if bench data shows dither.
    pub zero_rate_max_ticks: i32,
}

impl EstimatorConfig {
    /// `false` when `metres_per_tick` is the uncalibrated placeholder value
    /// (`0.0`, plan §2.6). [`crate::Ekf::correct_wheel_sensors`] uses this to
    /// disable the odometry branch instead of dividing by it.
    pub fn odometry_calibrated(&self) -> bool {
        self.metres_per_tick != 0.0
    }
}
