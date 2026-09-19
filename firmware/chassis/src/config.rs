//! Constants transcribed from `config/rover.toml` and the ROS 2 firmware.
//!
//! # Deviation from the plan
//!
//! `config/rover.toml`'s header comment says the firmware should parse the
//! subset it needs **at compile time** (`include_str!` + a const parser), so
//! nothing is duplicated between the config file and the source. That parser
//! was not built for this pass: a `const fn` TOML subset parser (sections,
//! floats, arrays, comments, no heap) is a project in its own right, and
//! getting it subtly wrong (a float rounding differently, a stale cached
//! value) is a worse failure mode than a small, obviously-named constants
//! file that a human can diff against `config/rover.toml` by eye.
//!
//! Every constant below cites the exact `rover.toml` key or ROS 2 source line
//! it was transcribed from. If you change one in `rover.toml`, grep this file
//! for the same value and change it here too - that manual step is the
//! documented cost of not having the parser yet.
#![allow(dead_code)]

use embassy_net::Ipv4Address;

/// `[hosts] chassis` / `[ports] chassis` - this board's own address.
pub const SELF_IP: Ipv4Address = Ipv4Address::new(192, 168, 1, 2);
pub const SELF_PORT: u16 = 7002;

/// `[hosts] rpi` / `[ports] rpi` - every message this board publishes routes
/// to the RPi (`[routes]`: `ImuSample`, `ChassisStatus`, `MagSample` all list
/// only `["rpi"]`), and `ChassisCommand` arrives from the RPi's actuation
/// task, so there is exactly one peer to talk to.
pub const RPI_IP: Ipv4Address = Ipv4Address::new(192, 168, 1, 1);
pub const RPI_PORT: u16 = 7001;

/// Locally-administered MAC (U/L bit set, OUI zeroed) - there is no vendor
/// assignment to collide with on a closed LAN with four fixed hosts. The last
/// octet mirrors the board's IP host part (`.2`) purely so a packet capture
/// is legible; it has no protocol meaning.
pub const MAC_ADDR: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x02];

/// `[safety] command_timeout_ms` - plan §5.2: ten missed frames at the 50 Hz
/// command rate. This is the single most important constant in this crate:
/// the ROS 2 firmware had no equivalent at all and held its last PWM forever
/// if the RPi died.
pub const CMD_TIMEOUT_MS: u64 = 200;

/// `[safety] throttle_ramp_ms` - plan §5.2: a ramp, not a step. A step to
/// zero throttle in a loaded drivetrain is a mechanical shock; 300 ms brings
/// it down smoothly.
pub const RAMP_TIME_MS: u64 = 300;

/// Number of steps the throttle ramp is divided into. 30 steps over 300 ms is
/// 10 ms per step - fine enough that the motor driver sees a smooth
/// deceleration rather than a handful of visible jumps, coarse enough that
/// it is not needlessly spamming the PWM peripheral.
pub const RAMP_STEPS: u32 = 30;

/// `[safety] iwdg_timeout_ms` - the independent hardware watchdog. This is a
/// second, independent layer from `CMD_TIMEOUT_MS`: that one detects a dead
/// *peer* (RPi/link), this one detects a dead *firmware* (a hung task,
/// deadlocked I2C). Petted only from the control task (see `watchdog.rs`),
/// so a hang anywhere else in the program resets the board.
pub const IWDG_TIMEOUT_US: u32 = 500_000;

/// `[control] steer_max_deg` - mechanical steering limit, field-derived.
/// `ChassisCommand.steer` is normalised to `[-1, 1]`; this is what `1.0`
/// means in degrees off centre.
pub const STEER_MAX_DEG: f32 = 45.0;

/// Servo centre position, degrees, out of the driver's 0-180 deg range.
/// From `motor_control.cpp`'s `servo_center_angle = 100`. Not 90 - this
/// particular servo/horn/linkage combination was field-calibrated to that
/// value, not to a textbook midpoint.
pub const SERVO_CENTER_DEG: f32 = 100.0;

/// Servo PWM duty range, as a fraction of the 20 ms period: 5%-10% duty is a
/// 1.0-2.0 ms pulse, the standard hobby-servo range. From
/// `calculate_steering_pwm_duty()`'s `0.05 + (degree/180.0)*(0.10-0.05)`.
pub const SERVO_DUTY_MIN: f32 = 0.05;
pub const SERVO_DUTY_MAX: f32 = 0.10;

/// Servo PWM frequency: 20 ms period, from `pwm_period_us = 20000` in
/// `motor_control.cpp`.
pub const SERVO_PWM_HZ: u32 = 50;

/// Drive motor PWM frequency: `motor_right_pwm.period_us(50)` /
/// `motor_left_pwm.period_us(50)` in `apply_motor_control()` - 50 us period =
/// 20 kHz, comfortably above the audible range for the H-bridge switching.
pub const MOTOR_PWM_HZ: u32 = 20_000;

/// Below this magnitude, `steer`/`throttle` are treated as exactly zero for
/// the purposes of picking a direction (straight / stop) and for the
/// watchdog's "explicit zero-throttle" recovery test. `ChassisCommand`'s
/// fields are `f32` arriving over the network; comparing to `0.0` exactly
/// would make recovery depend on the sender producing a bit-exact zero.
pub const CMD_EPSILON: f32 = 1.0e-3;

/// `[estimator] predict_hz` intent, restated at the source: the IMU samples
/// and publishes at the same rate. Ten times the ROS 2 firmware's publish
/// rate - it always sampled at 100 Hz, it just threw away nine of every ten
/// samples before publishing. See `imu.rs`.
pub const IMU_PUBLISH_HZ: u32 = 100;

/// `ChassisStatus` publish rate. Not specified as a precise number in the
/// plan ("~5 Hz"); 5 Hz matches `Telemetry`'s rate so a base-station operator
/// sees the watchdog state at the same cadence as everything else.
pub const STATUS_PUBLISH_HZ: u32 = 5;

/// One Earth gravity, for converting `mg` (from the driver's
/// `from_fs4_to_mg`) to m/s^2.
pub const STANDARD_GRAVITY: f32 = 9.80665;
