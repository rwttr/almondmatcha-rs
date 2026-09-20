//! Every message that crosses the bus.
//!
//! Each type is hand-written rather than macro-generated, so the byte layout
//! can be read straight off `encode`. The cost of that choice is that `encode`
//! and `decode` could drift apart; `tests/roundtrip.rs` closes it by
//! round-tripping every type and asserting the encoded length equals
//! [`Wire::WIRE_LEN`].
//!
//! # Type IDs
//!
//! Allocated by group and **permanent**. Reusing an ID for a different shape
//! makes two machines silently disagree about what they are reading.
//!
//! | Range    | Group                    |
//! |----------|--------------------------|
//! | `0x01xx` | chassis board → RPi      |
//! | `0x02xx` | RPi → chassis board      |
//! | `0x03xx` | sensors board → RPi      |
//! | `0x04xx` | GNSS                     |
//! | `0x05xx` | perception               |
//! | `0x06xx` | estimation and guidance  |
//! | `0x07xx` | mission                  |
//! | `0x08xx` | base station link        |
//! | `0x09xx` | debug                    |

use crate::codec::{DecodeError, Reader, Writer};
use crate::{check_len, Wire};

// ===========================================================================
// Enums and bit flags
// ===========================================================================

/// GNSS solution quality, in ascending order of trust.
///
/// Replaces the free-text `fix_quality` string the ROS 2 `UbloxGNSS` message
/// carried, which could not be compared or ordered without parsing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
#[repr(u8)]
pub enum FixQuality {
    #[default]
    None = 0,
    Autonomous = 1,
    Dgps = 2,
    RtkFloat = 3,
    RtkFixed = 4,
}

impl FixQuality {
    pub fn from_u8(v: u8) -> Result<Self, DecodeError> {
        match v {
            0 => Ok(Self::None),
            1 => Ok(Self::Autonomous),
            2 => Ok(Self::Dgps),
            3 => Ok(Self::RtkFloat),
            4 => Ok(Self::RtkFixed),
            _ => Err(DecodeError::BadDiscriminant {
                field: "FixQuality",
                value: v,
            }),
        }
    }

    /// True once the solution is good enough to trust course-over-ground as a
    /// heading reference (see the plan, §2.5).
    pub fn is_rtk(self) -> bool {
        matches!(self, Self::RtkFloat | Self::RtkFixed)
    }
}

/// Mission state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum MissionState {
    /// No goal loaded.
    #[default]
    Idle = 0,
    /// Goal loaded, waiting for a usable GNSS fix before moving.
    Armed = 1,
    /// Driving toward the goal.
    Running = 2,
    /// Goal reached.
    Arrived = 3,
    /// Cancelled by the operator.
    Cancelled = 4,
    /// Halted by a fault: stall, watchdog, or loss of a required sensor.
    Fault = 5,
}

impl MissionState {
    pub fn from_u8(v: u8) -> Result<Self, DecodeError> {
        match v {
            0 => Ok(Self::Idle),
            1 => Ok(Self::Armed),
            2 => Ok(Self::Running),
            3 => Ok(Self::Arrived),
            4 => Ok(Self::Cancelled),
            5 => Ok(Self::Fault),
            _ => Err(DecodeError::BadDiscriminant {
                field: "MissionState",
                value: v,
            }),
        }
    }

    /// True when the rover should be allowed to drive.
    pub fn is_driving(self) -> bool {
        matches!(self, Self::Running)
    }
}

/// Chassis board fault flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FaultBits(pub u8);

impl FaultBits {
    pub const NONE: Self = Self(0);
    /// IMU did not answer, or returned the wrong `WHO_AM_I`.
    pub const IMU_LOST: Self = Self(1 << 0);
    /// Motor driver reported a fault, or current exceeded its limit.
    pub const MOTOR_FAULT: Self = Self(1 << 1);
    /// Board came up from an independent-watchdog reset, i.e. firmware hung.
    pub const IWDG_RESET: Self = Self(1 << 2);
    /// Steering servo out of range or unresponsive.
    pub const SERVO_FAULT: Self = Self(1 << 3);

    pub fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }
    pub fn set(&mut self, other: Self) {
        self.0 |= other.0;
    }
    pub fn clear(&mut self, other: Self) {
        self.0 &= !other.0;
    }
    pub fn is_clear(self) -> bool {
        self.0 == 0
    }
}

/// System-wide health flags, aggregated by `rover-telemetry`.
///
/// One bit per feed that the rover needs and can lose independently. A stale
/// feed is not necessarily a fault — the estimator copes with a missing camera
/// for seconds — but it must be visible from the base station.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct HealthBits(pub u16);

impl HealthBits {
    pub const NONE: Self = Self(0);
    pub const CHASSIS_STALE: Self = Self(1 << 0);
    pub const SENSORS_STALE: Self = Self(1 << 1);
    pub const LANE_STALE: Self = Self(1 << 2);
    pub const RTK_STALE: Self = Self(1 << 3);
    pub const BACKUP_GNSS_STALE: Self = Self(1 << 4);
    /// Chassis board reported its command watchdog had tripped.
    pub const WATCHDOG_TRIPPED: Self = Self(1 << 5);
    /// Speed loop believes a wheel is stalled.
    pub const STALL_DETECTED: Self = Self(1 << 6);
    /// Estimator covariance exceeded its trust threshold.
    pub const ESTIMATOR_DIVERGED: Self = Self(1 << 7);
    /// A board reported a power-on self-test failure — see
    /// [`BoardDiagnostics::post_failures`]. Latched for the run: a POST
    /// failure does not heal, and clearing it when the board stops
    /// re-announcing would hide it.
    pub const BOARD_POST_FAIL: Self = Self(1 << 8);
    /// A board reported an abnormal reset cause (watchdog or brown-out), or
    /// its uptime went backwards — meaning it rebooted mid-run. Latched, for
    /// the same reason: the reboot is the event, and it is over by the time
    /// anyone reads this.
    pub const BOARD_RESET: Self = Self(1 << 9);
    /// A board's Ethernet link negotiated below 100 Mbit/s full duplex, or
    /// its PHY reported symbol errors. The link still works, which is exactly
    /// why this needs a bit of its own — nothing else will show it until it
    /// degrades into packet loss.
    pub const LINK_DEGRADED: Self = Self(1 << 10);

    pub fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }
    pub fn set(&mut self, other: Self) {
        self.0 |= other.0;
    }
    pub fn clear(&mut self, other: Self) {
        self.0 &= !other.0;
    }
    pub fn is_clear(self) -> bool {
        self.0 == 0
    }
}

// ===========================================================================
// 0x01xx — chassis board → RPi
// ===========================================================================

/// Inertial sample from the LSM6DSV16X on the chassis board.
///
/// Published at **100 Hz**, not the 10 Hz the ROS 2 firmware used: the board
/// always sampled at 100 Hz and discarded nine of every ten samples. The EKF
/// wants all of them.
///
/// Unlike the ROS 2 `ChassisIMU` message, which shipped raw sensor LSBs
/// nominally scaled by 1000 (a conversion no consumer ever actually applied),
/// these are real SI values converted on the board.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ImuSample {
    pub accel_mps2: [f32; 3],
    pub gyro_radps: [f32; 3],
    /// Board uptime in microseconds. Lets the estimator compute a true `dt`
    /// and measure end-to-end latency against its own clock.
    pub t_us: u32,
}

impl Wire for ImuSample {
    const TYPE_ID: u16 = 0x0101;
    const WIRE_LEN: usize = 28;
    const NAME: &'static str = "ImuSample";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.f32x3(self.accel_mps2);
        w.f32x3(self.gyro_radps);
        w.u32(self.t_us);
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        Ok(Self {
            accel_mps2: r.f32x3(),
            gyro_radps: r.f32x3(),
            t_us: r.u32(),
        })
    }
}

/// Magnetic field from the LIS2MDL.
///
/// Present on the X-NUCLEO-IKS4A1 shield but **optional and off by default**.
/// Hard- and soft-iron distortion from the drive motors varies with current
/// draw, so a stationary calibration does not hold under load, and the field
/// gives yaw in the *earth* frame rather than the lane-relative heading error
/// the estimator actually tracks. Intended as a gyro-bias aid only — RTK
/// course-over-ground is the better heading reference while moving. See the
/// plan, §2.5.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MagSample {
    pub field_gauss: [f32; 3],
    pub t_us: u32,
}

impl Wire for MagSample {
    const TYPE_ID: u16 = 0x0102;
    const WIRE_LEN: usize = 16;
    const NAME: &'static str = "MagSample";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.f32x3(self.field_gauss);
        w.u32(self.t_us);
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        Ok(Self {
            field_gauss: r.f32x3(),
            t_us: r.u32(),
        })
    }
}

/// Chassis board liveness and fault report.
///
/// New in the Rust system. The ROS 2 firmware had no way to tell the rover
/// that its command watchdog had tripped — or that it had a watchdog at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ChassisStatus {
    /// `ChassisCommand.seq` of the most recently applied command. The RPi
    /// compares this against what it sent to measure command loss.
    pub seq_echo: u16,
    /// True while the board is in the watchdog-tripped state: no command
    /// arrived within its timeout, throttle was ramped to zero and steering
    /// centred. Cleared only by an explicit zero-throttle command.
    pub watchdog_tripped: bool,
    pub fault: FaultBits,
    pub t_us: u32,
}

impl Wire for ChassisStatus {
    const TYPE_ID: u16 = 0x0103;
    const WIRE_LEN: usize = 8;
    const NAME: &'static str = "ChassisStatus";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.u16(self.seq_echo);
        w.bool(self.watchdog_tripped);
        w.u8(self.fault.0);
        w.u32(self.t_us);
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        Ok(Self {
            seq_echo: r.u16(),
            watchdog_tripped: r.bool(),
            fault: FaultBits(r.u8()),
            t_us: r.u32(),
        })
    }
}

// ===========================================================================
// 0x02xx — RPi → chassis board
// ===========================================================================

/// Actuation command for the chassis board, at 50 Hz.
///
/// Two signed normalised quantities. The ROS 2 `ChassisCtrl` split these into
/// four fields — `fdr_msg` (1=right/2=straight/3=left) with `ro_ctrl_msg`
/// (0.0–1.0), and `bdr_msg` (0=stop/1=fwd/2=bwd) with `spd_msg` (0–255) —
/// which put H-bridge direction pins, a firmware concern, onto the wire.
/// The firmware now derives direction from the sign.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ChassisCommand {
    /// `-1.0` full left … `+1.0` full right. Positive steers right, matching
    /// the sign convention of `heading_err_rad` and `cross_track_m`.
    pub steer: f32,
    /// `-1.0` full reverse … `+1.0` full forward. Exactly `0.0` is stop.
    pub throttle: f32,
    /// Increments per command. Echoed in [`ChassisStatus::seq_echo`], and used
    /// by the firmware watchdog to tell a fresh command from a stale one.
    pub seq: u16,
}

impl ChassisCommand {
    /// A command that stops the rover and centres the steering. Also the value
    /// the firmware applies on a watchdog trip.
    pub const STOP: Self = Self {
        steer: 0.0,
        throttle: 0.0,
        seq: 0,
    };

    /// Clamp both channels into range. Call before sending: a controller under
    /// development can and will produce values outside `[-1, 1]`, and the
    /// firmware should never be the only thing standing between a bad gain and
    /// the hardware.
    pub fn clamped(self) -> Self {
        Self {
            steer: self.steer.clamp(-1.0, 1.0),
            throttle: self.throttle.clamp(-1.0, 1.0),
            seq: self.seq,
        }
    }
}

impl Wire for ChassisCommand {
    const TYPE_ID: u16 = 0x0201;
    const WIRE_LEN: usize = 10;
    const NAME: &'static str = "ChassisCommand";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.f32(self.steer);
        w.f32(self.throttle);
        w.u16(self.seq);
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        Ok(Self {
            steer: r.f32(),
            throttle: r.f32(),
            seq: r.u16(),
        })
    }
}

// ===========================================================================
// 0x03xx — sensors board → RPi
// ===========================================================================

/// Wheel encoder counts, at 10 Hz.
///
/// Split out of the ROS 2 `ChassisSensors` message, which bundled encoders and
/// power into one 4 Hz publication even though they have different rates and
/// entirely different consumers. Encoders feed the estimator; power feeds
/// telemetry.
///
/// Counts are free-running and signed; consumers difference consecutive
/// samples. Converting to metres needs `metres_per_tick` from
/// `config/rover.toml`, which **depends on the decoding mode** — hardware
/// quadrature gives 4× the counts the ROS 2 firmware's 2× edge counting did.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WheelSensors {
    pub ticks_left: i32,
    pub ticks_right: i32,
    pub t_us: u32,
}

impl Wire for WheelSensors {
    const TYPE_ID: u16 = 0x0301;
    const WIRE_LEN: usize = 12;
    const NAME: &'static str = "WheelSensors";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.i32(self.ticks_left);
        w.i32(self.ticks_right);
        w.u32(self.t_us);
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        Ok(Self {
            ticks_left: r.i32(),
            ticks_right: r.i32(),
            t_us: r.u32(),
        })
    }
}

/// Battery bus voltage and current from the INA226, at 5 Hz.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PowerSample {
    pub bus_volts: f32,
    pub current_amps: f32,
}

impl PowerSample {
    /// Instantaneous draw. Derived rather than transmitted — the ROS 2
    /// `TelemetryRelay` carried a `power_watts` field that was just this
    /// product, recomputed and sent across the network for no reason.
    pub fn watts(&self) -> f32 {
        self.bus_volts * self.current_amps
    }
}

impl Wire for PowerSample {
    const TYPE_ID: u16 = 0x0302;
    const WIRE_LEN: usize = 8;
    const NAME: &'static str = "PowerSample";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.f32(self.bus_volts);
        w.f32(self.current_amps);
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        Ok(Self {
            bus_volts: r.f32(),
            current_amps: r.f32(),
        })
    }
}

// ===========================================================================
// 0x04xx — GNSS
// ===========================================================================

/// Which physical receiver a [`GnssFix`] came from.
///
/// **Design defect D2** (`docs/RUST_REWRITE_PLAN.md` §13.3b): a single
/// `GnssFix` type serves both the u-blox and the Spresense, so before this
/// field existed a subscriber in a different process from
/// `rover-navigation` had no way to tell them apart from the bus alone.
/// `rover-telemetry` used to guess from `FixQuality` (see the now-deleted
/// `gnss_source::classify`) — sound in one direction, but a u-blox in cold
/// start reporting `Autonomous` was indistinguishable from the backup, and
/// the RTK stream is what mission logic and the heading reference depend
/// on. Self-describing beats a heuristic: this field is set once, at the
/// point each receiver's reading is assembled (`rover-navigation`'s
/// `UbloxAssembler`/`SpresenseAssembler`), and never has to be inferred
/// again downstream.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum GnssSource {
    /// u-blox SimpleRTK2b — the correctable, primary receiver.
    #[default]
    Rtk = 0,
    /// Spresense — uncorrected backup.
    Backup = 1,
}

impl GnssSource {
    pub fn from_u8(v: u8) -> Result<Self, DecodeError> {
        match v {
            0 => Ok(Self::Rtk),
            1 => Ok(Self::Backup),
            _ => Err(DecodeError::BadDiscriminant {
                field: "GnssSource",
                value: v,
            }),
        }
    }
}

/// One GNSS solution.
///
/// A single type serves both receivers — the u-blox SimpleRTK2b and the
/// Spresense — on two separate streams. The ROS 2 system had two near-identical
/// messages (`UbloxGNSS`, `SpresenseGNSS`) whose fields had drifted apart:
/// different date/time representations, one with SNR, one with a satellite
/// count named differently. Consumers had to special-case each.
///
/// Free-text fields are gone: `fix_quality` is now an ordered enum, and the
/// separate `date` and `time` strings are one UTC millisecond count.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct GnssFix {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f32,
    pub fix: FixQuality,
    pub sats: u8,
    /// Horizontal accuracy estimate, metres (1-sigma). The ROS 2 message
    /// carried `centimeter_error`; metres keeps units consistent everywhere.
    pub h_acc_m: f32,
    pub speed_mps: f32,
    /// Course over ground, degrees from true north, `[0, 360)`.
    ///
    /// Only meaningful while moving — below roughly 0.3 m/s it is noise. With
    /// an RTK fix this is a better heading reference than the magnetometer
    /// (see the plan, §2.5), which is why it is carried explicitly rather than
    /// being differentiated out of successive positions.
    pub course_deg: f32,
    pub utc_ms: u64,
    /// Which receiver produced this reading. See [`GnssSource`].
    pub source: GnssSource,
}

impl GnssFix {
    /// Whether this fix is good enough to navigate on.
    pub fn is_usable(&self) -> bool {
        self.fix != FixQuality::None && self.sats >= 4
    }

    /// Whether `course_deg` can be trusted as a heading reference right now.
    pub fn course_is_trustworthy(&self) -> bool {
        self.fix.is_rtk() && self.speed_mps > 0.3
    }
}

impl Wire for GnssFix {
    const TYPE_ID: u16 = 0x0401;
    const WIRE_LEN: usize = 43;
    const NAME: &'static str = "GnssFix";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.f64(self.lat_deg);
        w.f64(self.lon_deg);
        w.f32(self.alt_m);
        w.u8(self.fix as u8);
        w.u8(self.sats);
        w.f32(self.h_acc_m);
        w.f32(self.speed_mps);
        w.f32(self.course_deg);
        w.u64(self.utc_ms);
        w.u8(self.source as u8);
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        Ok(Self {
            lat_deg: r.f64(),
            lon_deg: r.f64(),
            alt_m: r.f32(),
            fix: FixQuality::from_u8(r.u8())?,
            sats: r.u8(),
            h_acc_m: r.f32(),
            speed_mps: r.f32(),
            course_deg: r.f32(),
            utc_ms: r.u64(),
            source: GnssSource::from_u8(r.u8())?,
        })
    }
}

// ===========================================================================
// 0x05xx — perception
// ===========================================================================

/// Lane geometry from the Jetson, at roughly 30 Hz.
///
/// Replaces the ROS 2 `Float32MultiArray` whose four elements meant
/// `[curvature, theta, b, detected]` by position only, documented in a comment.
///
/// **All three geometry values are measured at the lookahead point**, roughly
/// 1.22 m ahead of the front axle, not at the rover. That is a property of the
/// ROI geometry, and it means `cross_track_m` is non-zero on a curve even when
/// the rover is perfectly on line. The estimator models this explicitly —
/// see the plan, §2.2 — which is what lets `cross_track_m` and
/// `curvature_inv_m` be separated rather than fighting each other in the gains.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LaneMeasurement {
    /// Lane curvature, **1/metres**.
    ///
    /// Converted at the source. The ROS 2 pipeline shipped a parabola
    /// coefficient in bird's-eye-view pixels (1/px) and left every consumer to
    /// fold in `BEV_PX_PER_M = 200.0` for itself — while `b` was converted to
    /// metres before publication. One conversion, one place.
    pub curvature_inv_m: f32,
    /// Heading error relative to the lane tangent, radians. Positive means the
    /// correct response is to steer right, matching `cross_track_m` and
    /// `steer`.
    ///
    /// This convention is *enforced*, not merely documented: the raw fit
    /// inside the detector produces the opposite sign (a structural
    /// consequence of the fit's coordinate frame, not a sensor artifact —
    /// see `docs/RUST_REWRITE_PLAN.md` §13.3b D5), and
    /// `perception/rover_perception/lane.py`'s `LaneDetector.detect`
    /// negates it before publication so this field always agrees with the
    /// convention stated here. `perception/tests/test_lane_sign_convention.py`
    /// tests that agreement directly, against the model
    /// (`RoverState::at_lookahead`, `Ekf::correct_camera`) that consumes it.
    pub heading_err_rad: f32,
    /// Lateral offset from lane centre at the lookahead point, metres.
    /// Positive means the correct response is to steer right.
    pub cross_track_m: f32,
    /// False when the detector found no usable lane this frame. The estimator
    /// skips its camera update and coasts; it does not hold the last value.
    pub valid: bool,
    pub t_us: u32,
}

impl Wire for LaneMeasurement {
    const TYPE_ID: u16 = 0x0501;
    const WIRE_LEN: usize = 17;
    const NAME: &'static str = "LaneMeasurement";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.f32(self.curvature_inv_m);
        w.f32(self.heading_err_rad);
        w.f32(self.cross_track_m);
        w.bool(self.valid);
        w.u32(self.t_us);
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        Ok(Self {
            curvature_inv_m: r.f32(),
            heading_err_rad: r.f32(),
            cross_track_m: r.f32(),
            valid: r.bool(),
            t_us: r.u32(),
        })
    }
}

// ===========================================================================
// 0x06xx — estimation and guidance
// ===========================================================================

/// Number of EKF states. Kept here so `RoverState` and the estimator cannot
/// disagree about the size of `p_diag`.
pub const EKF_STATES: usize = 5;

/// Index of each state within [`RoverState::p_diag`].
pub mod state_idx {
    pub const CROSS_TRACK: usize = 0;
    pub const HEADING_ERR: usize = 1;
    pub const CURVATURE: usize = 2;
    pub const SPEED: usize = 3;
    pub const GYRO_BIAS: usize = 4;
}

/// Fused estimate of where the rover is relative to the lane.
///
/// Produced at 100 Hz by the EKF. Unlike [`LaneMeasurement`], these values are
/// referenced to the **front axle**, not the lookahead point — the estimator
/// removes the lookahead geometry. Controllers that want lookahead-referenced
/// values (the ported static-gain law does, because its field-tuned gains
/// assume them) reconstruct them with [`Self::at_lookahead`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RoverState {
    /// Lateral offset from lane centre at the front axle, metres.
    pub cross_track_m: f32,
    /// Heading error relative to the lane tangent, radians.
    pub heading_err_rad: f32,
    /// Lane curvature ahead, 1/metres.
    pub curvature_inv_m: f32,
    /// Forward speed, metres per second. Needs `metres_per_tick` calibration —
    /// the ROS 2 system had no metric speed anywhere.
    pub speed_mps: f32,
    /// Estimated gyro z-axis bias, rad/s. Observable through the zero-rate
    /// update while stationary; bounds how long the rover can coast on a lost
    /// lane before heading drifts.
    pub gyro_bias_radps: f32,
    /// Diagonal of the covariance matrix, indexed by [`state_idx`]. Guidance
    /// slows down as this grows rather than relying on a fixed timeout.
    pub p_diag: [f32; EKF_STATES],
    /// Milliseconds since the last accepted camera update. Saturates rather
    /// than wrapping, so a long dropout reads as "very stale", not "fresh".
    pub lane_age_ms: u16,
}

impl RoverState {
    /// Reconstruct the lookahead-referenced errors the ported control law
    /// expects, `l_a` metres ahead of the front axle.
    ///
    /// Returns `(cross_track_m, heading_err_rad)`. Feeding these to the
    /// static-gain law keeps the field-derived `k_lat = 181.17 deg/m` and
    /// `k_head = 2.024 deg/deg` valid unchanged, so swapping the EMA for the
    /// EKF is not a reason to re-tune.
    pub fn at_lookahead(&self, l_a: f32) -> (f32, f32) {
        let cross = self.cross_track_m
            + l_a * self.heading_err_rad
            + 0.5 * self.curvature_inv_m * l_a * l_a;
        let heading = self.heading_err_rad + l_a * self.curvature_inv_m;
        (cross, heading)
    }

    /// Variance of the lateral position estimate, m^2.
    ///
    /// Available everywhere. `sqrt` lives in `std`, so firmware that wants to
    /// reason about uncertainty compares variances rather than pulling in a
    /// software float library for a cosmetic square root.
    pub fn cross_track_var(&self) -> f32 {
        self.p_diag[state_idx::CROSS_TRACK].max(0.0)
    }

    /// One-sigma uncertainty in lateral position, metres.
    #[cfg(feature = "std")]
    pub fn cross_track_sigma(&self) -> f32 {
        self.cross_track_var().sqrt()
    }
}

impl Wire for RoverState {
    const TYPE_ID: u16 = 0x0601;
    const WIRE_LEN: usize = 42;
    const NAME: &'static str = "RoverState";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.f32(self.cross_track_m);
        w.f32(self.heading_err_rad);
        w.f32(self.curvature_inv_m);
        w.f32(self.speed_mps);
        w.f32(self.gyro_bias_radps);
        w.f32x5(self.p_diag);
        w.u16(self.lane_age_ms);
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        Ok(Self {
            cross_track_m: r.f32(),
            heading_err_rad: r.f32(),
            curvature_inv_m: r.f32(),
            speed_mps: r.f32(),
            gyro_bias_radps: r.f32(),
            p_diag: r.f32x5(),
            lane_age_ms: r.u16(),
        })
    }
}

/// What the guidance law wants the rover to do, before safety limiting.
///
/// Kept separate from [`ChassisCommand`] on purpose: this is a request in
/// physical units, that one is a normalised actuator command. The actuation
/// task owns the conversion and every guard rail — saturation, slew limiting,
/// the speed cap, stall detection — so an experimental controller cannot reach
/// the hardware directly.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MotionSetpoint {
    /// Requested steering angle, radians. Positive steers right.
    pub steer_rad: f32,
    /// Requested forward speed, metres per second.
    pub speed_mps: f32,
}

impl Wire for MotionSetpoint {
    const TYPE_ID: u16 = 0x0602;
    const WIRE_LEN: usize = 8;
    const NAME: &'static str = "MotionSetpoint";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.f32(self.steer_rad);
        w.f32(self.speed_mps);
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        Ok(Self {
            steer_rad: r.f32(),
            speed_mps: r.f32(),
        })
    }
}

// ===========================================================================
// 0x07xx — mission
// ===========================================================================

/// A destination waypoint.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MissionGoal {
    pub lat_deg: f64,
    pub lon_deg: f64,
}

impl Wire for MissionGoal {
    const TYPE_ID: u16 = 0x0702;
    const WIRE_LEN: usize = 16;
    const NAME: &'static str = "MissionGoal";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.f64(self.lat_deg);
        w.f64(self.lon_deg);
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        Ok(Self {
            lat_deg: r.f64(),
            lon_deg: r.f64(),
        })
    }
}

/// Mission progress, produced on the rover.
///
/// This replaces the ROS 2 `DesData` action's feedback and result channels.
/// The action machinery — goal UUIDs, accept/reject, cancel handshakes, a
/// feedback watchdog on the base station — existed to deliver these few fields
/// reliably. Publishing them in the telemetry stream does the same job without
/// a session, and keeps working over a lossy one-way link.
///
/// **The rover owns mission state.** `mission_active` came from the RPi in the
/// ROS 2 system too; making that explicit means losing the base station never
/// stops or endangers the rover, which is what the LoRa future requires.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MissionStatus {
    /// True while the rover is permitted to drive. The actuation task
    /// emergency-stops on a false edge.
    pub active: bool,
    /// Great-circle distance to the goal, metres. The ROS 2 action reported
    /// kilometres in one place and metres in another; this is metres, always.
    pub distance_remaining_m: f32,
    /// `None` when no goal is loaded. On the wire this is a presence flag plus
    /// a goal that is zeroed when absent — there are no optional fields in the
    /// byte layout.
    pub target: Option<MissionGoal>,
    pub state: MissionState,
}

impl Wire for MissionStatus {
    const TYPE_ID: u16 = 0x0701;
    const WIRE_LEN: usize = 23;
    const NAME: &'static str = "MissionStatus";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.bool(self.active);
        w.f32(self.distance_remaining_m);
        w.bool(self.target.is_some());
        let goal = self.target.unwrap_or_default();
        w.f64(goal.lat_deg);
        w.f64(goal.lon_deg);
        w.u8(self.state as u8);
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        let active = r.bool();
        let distance_remaining_m = r.f32();
        let has_target = r.bool();
        let goal = MissionGoal {
            lat_deg: r.f64(),
            lon_deg: r.f64(),
        };
        Ok(Self {
            active,
            distance_remaining_m,
            target: has_target.then_some(goal),
            state: MissionState::from_u8(r.u8())?,
        })
    }
}

// ===========================================================================
// 0x08xx — base station link
// ===========================================================================

/// What the base station is asking the rover to do.
///
/// Discriminant values are part of the wire format and must not be renumbered.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Command {
    /// Do nothing. Sent as a keepalive so the base can prove the link works
    /// without changing rover state.
    #[default]
    Nop,
    /// Cap the speed the actuation task will ever request, as a percentage of
    /// full duty, `0..=100`. `0` is a full stop override.
    SetSpeedLimit(u8),
    /// Load and arm a destination.
    SetMissionGoal(MissionGoal),
    /// Abandon the current goal and stop.
    CancelMission,
    /// Immediate stop.
    ///
    /// Best-effort by nature — it crosses a network. The stop that is
    /// guaranteed is the firmware command watchdog, which needs no packet to
    /// arrive in order to fire. Never design safety around this reaching the
    /// rover.
    EStop,
    /// Explicitly release an [`Command::EStop`] latch.
    ///
    /// Added because `rover-control::actuate::SafetyGate` used to clear the
    /// E-stop latch on `CancelMission` — a judgement call the previous pass
    /// flagged as surprising: an operator pressing "cancel mission" to
    /// resume from an emergency stop is not an obviously safe reading of
    /// that word, and it meant there was no way to cancel a mission *without*
    /// also releasing the E-stop. This variant separates the two: cancelling
    /// a mission never touches the E-stop latch, and only this command does.
    ClearEStop,
}

impl Command {
    /// Largest payload any variant carries: a [`MissionGoal`].
    const PAYLOAD_LEN: usize = MissionGoal::WIRE_LEN;

    fn tag(&self) -> u8 {
        match self {
            Command::Nop => 0,
            Command::SetSpeedLimit(_) => 1,
            Command::SetMissionGoal(_) => 2,
            Command::CancelMission => 3,
            Command::EStop => 4,
            Command::ClearEStop => 5,
        }
    }
}

/// A command from the base station, carried as an idempotent datagram.
///
/// There is no request/response handshake and no connection. The base
/// retransmits the same frame at about 1 Hz until it sees `cmd_seq` echoed
/// back in [`Telemetry::last_cmd_seq`]; the rover applies any frame newer than
/// the last one it applied and ignores repeats.
///
/// This replaces the ROS 2 service and action clients. It is loss-tolerant,
/// stateless, and works unchanged over a link that cannot carry TCP — which
/// the LoRa base link cannot.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CommandFrame {
    /// Monotonic per base-station session. Wrapping is handled by comparing
    /// with wrapping arithmetic, so a long session does not break the ordering.
    pub cmd_seq: u16,
    pub body: Command,
}

impl CommandFrame {
    /// Whether this frame is newer than the last applied sequence number,
    /// tolerant of `u16` wraparound.
    ///
    /// Treats the nearer half of the sequence space as "newer", the standard
    /// approach for wrapping counters: a frame more than 32767 ahead is read as
    /// stale rather than as an enormous jump forward.
    pub fn is_newer_than(&self, last_applied: u16) -> bool {
        self.cmd_seq != last_applied && self.cmd_seq.wrapping_sub(last_applied) < 0x8000
    }
}

impl Wire for CommandFrame {
    const TYPE_ID: u16 = 0x0802;
    // cmd_seq(2) + tag(1) + fixed payload area(16)
    const WIRE_LEN: usize = 3 + Command::PAYLOAD_LEN;
    const NAME: &'static str = "CommandFrame";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.u16(self.cmd_seq);
        w.u8(self.body.tag());

        // The payload area is always written in full, whatever the variant, so
        // the frame is a constant size. Unused bytes are zero.
        match self.body {
            Command::SetSpeedLimit(pct) => {
                w.u8(pct);
                for _ in 0..Command::PAYLOAD_LEN - 1 {
                    w.u8(0);
                }
            }
            Command::SetMissionGoal(goal) => {
                w.f64(goal.lat_deg);
                w.f64(goal.lon_deg);
            }
            Command::Nop | Command::CancelMission | Command::EStop | Command::ClearEStop => {
                for _ in 0..Command::PAYLOAD_LEN {
                    w.u8(0);
                }
            }
        }
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        let cmd_seq = r.u16();
        let tag = r.u8();
        let body = match tag {
            0 => Command::Nop,
            1 => Command::SetSpeedLimit(r.u8()),
            2 => Command::SetMissionGoal(MissionGoal {
                lat_deg: r.f64(),
                lon_deg: r.f64(),
            }),
            3 => Command::CancelMission,
            4 => Command::EStop,
            5 => Command::ClearEStop,
            _ => {
                return Err(DecodeError::BadDiscriminant {
                    field: "Command",
                    value: tag,
                })
            }
        };
        Ok(Self { cmd_seq, body })
    }
}

/// Full telemetry frame for the LAN, at 5 Hz.
///
/// Composed of the same types the rest of the system uses rather than
/// re-flattening them. The ROS 2 `TelemetryRelay` restated every field of
/// every source message as a flat struct with a `*_valid` bool beside each
/// group — 40-odd fields that had to be kept in step by hand, and which had
/// already drifted: its `lane_*` fields were permanently hardcoded to
/// `false`/`0` because the bridge that would have filled them was ruled out on
/// architectural grounds and never removed.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Telemetry {
    pub seq: u32,
    pub t_us: u64,
    pub state: RoverState,
    pub mission: MissionStatus,
    pub power: PowerSample,
    /// u-blox SimpleRTK2b.
    pub rtk: GnssFix,
    /// Spresense, as a cross-check and fallback.
    pub backup: GnssFix,
    /// Most recent [`CommandFrame::cmd_seq`] the rover applied. This is the
    /// acknowledgement the base station retransmits until it sees.
    pub last_cmd_seq: u16,
    pub health: HealthBits,
}

impl Wire for Telemetry {
    const TYPE_ID: u16 = 0x0801;
    const WIRE_LEN: usize = 4
        + 8
        + RoverState::WIRE_LEN
        + MissionStatus::WIRE_LEN
        + PowerSample::WIRE_LEN
        + GnssFix::WIRE_LEN * 2
        + 2
        + 2;
    const NAME: &'static str = "Telemetry";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut n = 0;
        let mut w = Writer::new(buf);
        w.u32(self.seq);
        w.u64(self.t_us);
        n += w.len();
        n += self.state.encode(&mut buf[n..]);
        n += self.mission.encode(&mut buf[n..]);
        n += self.power.encode(&mut buf[n..]);
        n += self.rtk.encode(&mut buf[n..]);
        n += self.backup.encode(&mut buf[n..]);
        let mut w = Writer::new(&mut buf[n..]);
        w.u16(self.last_cmd_seq);
        w.u16(self.health.0);
        n + w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        let seq = r.u32();
        let t_us = r.u64();
        let mut n = r.len();

        let state = RoverState::decode(&buf[n..])?;
        n += RoverState::WIRE_LEN;
        let mission = MissionStatus::decode(&buf[n..])?;
        n += MissionStatus::WIRE_LEN;
        let power = PowerSample::decode(&buf[n..])?;
        n += PowerSample::WIRE_LEN;
        let rtk = GnssFix::decode(&buf[n..])?;
        n += GnssFix::WIRE_LEN;
        let backup = GnssFix::decode(&buf[n..])?;
        n += GnssFix::WIRE_LEN;

        let mut r = Reader::new(&buf[n..]);
        Ok(Self {
            seq,
            t_us,
            state,
            mission,
            power,
            rtk,
            backup,
            last_cmd_seq: r.u16(),
            health: HealthBits(r.u16()),
        })
    }
}

// NOTE: `TelemetryLite`, the 18-byte packed frame for a constrained LoRa link,
// is specified in `docs/RUST_REWRITE_PLAN.md` §6.3 but deliberately not
// implemented yet — the ESP32 radios are out of scope for this pass, and an
// unused type is code that rots. Add it with `LoraSerialLink`, not before.

// ===========================================================================
// 0x09xx — debug
// ===========================================================================

/// Internals of the closed-loop speed PID, at the encoder rate.
///
/// These signals were computed and discarded inside the ROS 2 controller until
/// a debug topic was added late; they turned out to be what made
/// auto-calibration and stall detection tunable. First-class from the start
/// here.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SpeedLoopDebug {
    pub measured_left_tps: f32,
    pub measured_right_tps: f32,
    pub target_tps: f32,
    /// Error as a percentage of full scale — the unit the PID operates in.
    /// Expressing it this way rather than in ticks/sec is what keeps the gains
    /// valid across a re-calibration of `max_ticks_per_sec`.
    pub error_pct: f32,
    pub pid_output_pct: f32,
}

impl Wire for SpeedLoopDebug {
    const TYPE_ID: u16 = 0x0901;
    const WIRE_LEN: usize = 20;
    const NAME: &'static str = "SpeedLoopDebug";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.f32(self.measured_left_tps);
        w.f32(self.measured_right_tps);
        w.f32(self.target_tps);
        w.f32(self.error_pct);
        w.f32(self.pid_output_pct);
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        Ok(Self {
            measured_left_tps: r.f32(),
            measured_right_tps: r.f32(),
            target_tps: r.f32(),
            error_pct: r.f32(),
            pid_output_pct: r.f32(),
        })
    }
}

/// EKF innovation for one camera update.
///
/// Logged from the first bench run. `Q` and `R` cannot be tuned without seeing
/// the innovation sequence, and retrofitting this after the fact means
/// repeating field runs to get the data.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct EkfDebug {
    /// Measurement minus prediction, in the camera's own units:
    /// `[cross_track_m, heading_err_rad, curvature_inv_m]`.
    pub innovation: [f32; 3],
    /// Normalised innovation squared. Should average near 3 (the measurement
    /// dimension) when `Q` and `R` are consistent with reality; persistently
    /// higher means the filter trusts itself too much.
    pub nis: f32,
    /// True when the chi-square gate rejected this update. A steady trickle is
    /// healthy — it is the gate catching glare and blob false positives. A
    /// sustained run of rejections means the filter has diverged from the
    /// world and is now rejecting the truth.
    pub gated: bool,
}

impl Wire for EkfDebug {
    const TYPE_ID: u16 = 0x0902;
    const WIRE_LEN: usize = 17;
    const NAME: &'static str = "EkfDebug";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.f32x3(self.innovation);
        w.f32(self.nis);
        w.bool(self.gated);
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        Ok(Self {
            innovation: r.f32x3(),
            nis: r.f32(),
            gated: r.bool(),
        })
    }
}

// ===========================================================================
// 0x0903 — board self-diagnostics
// ===========================================================================

/// Which board a [`BoardDiagnostics`] came from.
///
/// One message type serves both boards rather than two near-identical ones,
/// because every consumer (telemetry CSV, `rover-tap`, the preflight check)
/// wants to treat them uniformly — the ROS 2 system's `UbloxGNSS`/
/// `SpresenseGNSS` split is the cautionary example (see [`GnssFix`]). The
/// POST bits differ per board and are documented on [`PostBits`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum BoardId {
    #[default]
    Chassis = 0,
    Sensors = 1,
}

impl BoardId {
    pub fn from_u8(v: u8) -> Result<Self, DecodeError> {
        match v {
            0 => Ok(Self::Chassis),
            1 => Ok(Self::Sensors),
            _ => Err(DecodeError::BadDiscriminant {
                field: "BoardId",
                value: v,
            }),
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Chassis => "chassis",
            Self::Sensors => "sensors",
        }
    }
}

/// Why the board last reset.
///
/// Read once at boot from `RCC_CSR` and latched for the life of the run. This
/// is the single most diagnostic byte either board produces: a rover that
/// silently reboots mid-run looks, from the RPi's side, exactly like a brief
/// link drop — the sequence numbers restart and the feeds come back. Only the
/// reset cause distinguishes "the watchdog fired" from "somebody nudged the
/// USB cable".
///
/// [`Self::IndependentWatchdog`] in particular means the firmware hung long
/// enough for the IWDG to fire (500 ms, `[safety] iwdg_timeout_ms`) — that is
/// a firmware bug, not a field condition, and it must never be dismissed as
/// noise.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum ResetCause {
    /// Could not be determined — no flag set, or the register was already
    /// cleared. Distinct from `PowerOn` on purpose: "I do not know" is not
    /// the same claim as "this was a clean cold boot".
    #[default]
    Unknown = 0,
    /// Power-on / brown-out. The normal cold boot.
    PowerOn = 1,
    /// External reset pin — the Nucleo's black button, or the ST-LINK.
    Pin = 2,
    /// Software-requested reset (`SCB::sys_reset`).
    Software = 3,
    /// Independent watchdog fired. **Firmware hung.**
    IndependentWatchdog = 4,
    /// Window watchdog fired.
    WindowWatchdog = 5,
    /// Low-power reset (entered standby without clearing the flag).
    LowPower = 6,
    /// Brown-out reset, where the part reports it separately from power-on.
    BrownOut = 7,
}

impl ResetCause {
    pub fn from_u8(v: u8) -> Result<Self, DecodeError> {
        match v {
            0 => Ok(Self::Unknown),
            1 => Ok(Self::PowerOn),
            2 => Ok(Self::Pin),
            3 => Ok(Self::Software),
            4 => Ok(Self::IndependentWatchdog),
            5 => Ok(Self::WindowWatchdog),
            6 => Ok(Self::LowPower),
            7 => Ok(Self::BrownOut),
            _ => Err(DecodeError::BadDiscriminant {
                field: "ResetCause",
                value: v,
            }),
        }
    }

    /// True for a cause that means something went wrong, as opposed to a
    /// normal or operator-initiated boot. Drives [`HealthBits::BOARD_RESET`].
    pub fn is_abnormal(self) -> bool {
        matches!(
            self,
            Self::IndependentWatchdog | Self::WindowWatchdog | Self::BrownOut
        )
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Unknown => "unknown",
            Self::PowerOn => "power-on",
            Self::Pin => "pin",
            Self::Software => "software",
            Self::IndependentWatchdog => "IWDG",
            Self::WindowWatchdog => "WWDG",
            Self::LowPower => "low-power",
            Self::BrownOut => "brown-out",
        }
    }
}

/// Power-on self-test results, one bit per check.
///
/// Carried as a **pair** of bitfields — `run` and `pass` — because "this test
/// did not run" and "this test failed" are different facts and collapsing
/// them loses the one that matters. A board whose I²C bus is dead cannot run
/// the IMU identity check at all; reporting that as a plain failure would
/// send someone looking at the IMU instead of the bus.
///
/// **Bit meanings differ per board.** The first four are common; the rest are
/// board-specific, which is why [`BoardDiagnostics::board`] must be read
/// before interpreting them.
///
/// | Bit | Chassis | Sensors |
/// |-----|---------|---------|
/// | 0 | `CLOCK` — PLL reached the configured SYSCLK | same |
/// | 1 | `PHY_ID` — Ethernet PHY answered with the expected ID | same |
/// | 2 | `LINK` — link came up at 100 Mbps full duplex | same |
/// | 3 | `NET_BIND` — UDP socket bound | same |
/// | 4 | `IMU` — LSM6DSV16X `WHO_AM_I` == `0x70` | `POWER` — INA226 manufacturer ID |
/// | 5 | `MOTOR_PWM` — TIM1/TIM3 produced output | `ENCODER_IDLE` — both channels readable and quiet at rest |
/// | 6 | `SERVO_PWM` — TIM2 CH4 produced output | *(unused)* |
/// | 7 | `IWDG` — independent watchdog armed | same |
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PostBits(pub u16);

impl PostBits {
    pub const NONE: Self = Self(0);
    pub const CLOCK: Self = Self(1 << 0);
    pub const PHY_ID: Self = Self(1 << 1);
    pub const LINK: Self = Self(1 << 2);
    pub const NET_BIND: Self = Self(1 << 3);
    /// Chassis: IMU identity. Sensors: INA226 identity.
    pub const SENSOR_A: Self = Self(1 << 4);
    /// Chassis: motor PWM. Sensors: encoder idle check.
    pub const SENSOR_B: Self = Self(1 << 5);
    /// Chassis: servo PWM. Unused on sensors.
    pub const SENSOR_C: Self = Self(1 << 6);
    pub const IWDG: Self = Self(1 << 7);

    pub fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }
    pub fn set(&mut self, other: Self) {
        self.0 |= other.0;
    }
    pub fn is_clear(self) -> bool {
        self.0 == 0
    }
}

/// Board self-diagnostics: power-on self-test results plus the runtime facts
/// that only the board itself can see.
///
/// Published once immediately after POST completes, then at a slow heartbeat
/// rate (1 Hz) so a board that reboots mid-run re-announces itself — the
/// `reset_cause` on that second announcement is what tells the operator a
/// reboot happened at all.
///
/// # Why this exists
///
/// Before it, a failed peripheral init on either board was logged over
/// `defmt`/RTT and nowhere else. RTT needs a debugger physically attached,
/// which in a field run it is not. A rover whose IMU failed to initialise
/// would simply drive with no gyro input to the EKF and no indication
/// anywhere that anything was wrong.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BoardDiagnostics {
    pub board: BoardId,
    /// Which POST checks actually executed. See [`PostBits`].
    pub post_run: PostBits,
    /// Which of those passed. `post_run & !post_pass` is the failure set.
    pub post_pass: PostBits,
    pub reset_cause: ResetCause,
    /// PHY identity, `(ID1 << 16) | ID2`, as read over MDIO. `0` means the
    /// read was not attempted or the PHY did not answer. The LAN8742A
    /// reports `0x0007_C130` (OUI `0x0007C0`, model `0x13`, revision in the
    /// low nibble, which is why the low 4 bits vary between parts).
    pub phy_id: u32,
    /// Negotiated link speed in Mbit/s. `0` means **unknown** — the PHY read
    /// failed or has not happened yet.
    ///
    /// It does *not* mean "link down", and a consumer must not treat it as a
    /// fault. This message only ever arrives over the link it describes, so
    /// by the time anyone can read this field the link is demonstrably up;
    /// `0` can only be the board admitting it could not resolve the speed.
    /// Conflating the two would report a degraded link on every board whose
    /// first sample beat its first PHY poll.
    ///
    /// Worth reporting separately from "link up" because the silent failure
    /// on this hardware is a link that negotiates 10 Mbit/s half duplex on a
    /// marginal cable: everything reports "up", and the MAC is still
    /// configured for 100 full.
    pub link_speed_mbps: u8,
    pub link_full_duplex: bool,
    /// PHY symbol-error count, read raw from the LAN8742A's Symbol Error
    /// Counter (register `0x1A`). Non-zero means the physical layer is
    /// marginal — a bad cable, a bad connector, or interference — long
    /// before it becomes packet loss anyone notices.
    ///
    /// **Free-running, not read-to-clear.** The datasheet (Rev 1.1,
    /// 05-21-13) is explicit: *"This register is cleared on reset, but is not
    /// cleared by reading the register"*, and it rolls over at 65,536. So
    /// this is a running total since the board booted, and accumulating it
    /// across samples would multiply the true count by the sample count.
    ///
    /// **Meaningless at 10 Mbit/s**: the same datasheet notes the counter
    /// *"does not increment in 10BASE-T mode"*. A `0` here is only evidence
    /// of a clean link when [`Self::link_speed_mbps`] is 100.
    pub phy_symbol_errors: u16,
    /// Seconds since this board booted. Compare against the RPi's own uptime
    /// to spot a board that has restarted without anyone noticing.
    pub uptime_s: u32,
    /// Frames this board has dropped because a send failed. Saturating.
    pub tx_drops: u16,
}

impl BoardDiagnostics {
    /// POST checks that ran and did not pass.
    pub fn post_failures(&self) -> PostBits {
        PostBits(self.post_run.0 & !self.post_pass.0)
    }

    /// True when every check that ran also passed.
    pub fn post_ok(&self) -> bool {
        self.post_failures().is_clear()
    }
}

impl Wire for BoardDiagnostics {
    const TYPE_ID: u16 = 0x0903;
    // board(1) + post_run(2) + post_pass(2) + reset_cause(1) + phy_id(4) +
    // link_speed_mbps(1) + link_full_duplex(1) + phy_symbol_errors(2) +
    // uptime_s(4) + tx_drops(2) = 20. (Was declared 19 here before this pass
    // wired BoardDiagnostics into `[routes]` and the golden/roundtrip
    // suites -- nothing previously exercised `encode()`'s actual length
    // against this constant, so the off-by-one went uncaught.)
    const WIRE_LEN: usize = 20;
    const NAME: &'static str = "BoardDiagnostics";

    fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.u8(self.board as u8);
        w.u16(self.post_run.0);
        w.u16(self.post_pass.0);
        w.u8(self.reset_cause as u8);
        w.u32(self.phy_id);
        w.u8(self.link_speed_mbps);
        w.bool(self.link_full_duplex);
        w.u16(self.phy_symbol_errors);
        w.u32(self.uptime_s);
        w.u16(self.tx_drops);
        w.len()
    }

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, Self::WIRE_LEN)?;
        let mut r = Reader::new(buf);
        Ok(Self {
            board: BoardId::from_u8(r.u8())?,
            post_run: PostBits(r.u16()),
            post_pass: PostBits(r.u16()),
            reset_cause: ResetCause::from_u8(r.u8())?,
            phy_id: r.u32(),
            link_speed_mbps: r.u8(),
            link_full_duplex: r.bool(),
            phy_symbol_errors: r.u16(),
            uptime_s: r.u32(),
            tx_drops: r.u16(),
        })
    }
}
