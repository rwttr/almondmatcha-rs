//! Constants transcribed from `config/rover.toml` and the ROS 2 firmware.
//!
//! Same deviation-from-the-plan rationale as `firmware/chassis/src/config.rs`:
//! `config/rover.toml` says the firmware should parse the subset it needs at
//! compile time; that parser was not built for this pass. Every constant
//! below cites the exact `rover.toml` key or ROS 2 source line it was
//! transcribed from — if you change one in `rover.toml`, grep this file for
//! the same value and change it here too.
#![allow(dead_code)]

use embassy_net::Ipv4Address;

/// `[hosts] sensors` / `[ports] sensors` - this board's own address.
pub const SELF_IP: Ipv4Address = Ipv4Address::new(192, 168, 1, 6);
pub const SELF_PORT: u16 = 7004;

/// `[hosts] rpi` / `[ports] rpi` - `[routes]` sends `WheelSensors` and
/// `PowerSample` only to `["rpi"]`, so there is exactly one publish peer.
pub const RPI_IP: Ipv4Address = Ipv4Address::new(192, 168, 1, 1);
pub const RPI_PORT: u16 = 7001;

/// Locally-administered MAC (U/L bit set, OUI zeroed), same scheme as
/// `firmware/chassis/src/config.rs` - last octet mirrors the host part of
/// this board's IP (`.6`) purely so a packet capture is legible.
pub const MAC_ADDR: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x06];

/// `WheelSensors` publish rate. Raised from the ROS 2 firmware's 4 Hz
/// (`MAIN_LOOP_PERIOD_MS = 250`, the combined encoder+power+GNSS publish
/// loop) to 10 Hz per the plan: the estimator's odometry update runs at
/// 10 Hz, and `WheelSensors`/`PowerSample` are now separate messages with
/// separate rates instead of one struct bundling both at the slower one.
pub const ENCODER_PUBLISH_HZ: u32 = 10;

/// `PowerSample` publish rate. The ROS 2 firmware's `power_monitor_task`
/// already polled the INA226 at 5 Hz (`POWER_SAMPLE_PERIOD_MS = 200`) - only
/// the publish side changes here, from being bundled into the 4 Hz
/// `ChassisSensors` message to publishing on its own at the rate it was
/// always sampled.
pub const POWER_PUBLISH_HZ: u32 = 5;

/// Shunt resistor, Ohms. `power_monitor.h`'s `INA226_SHUNT_RESISTOR` - a
/// board-level hardware value (the physical shunt), not a device default.
pub const SHUNT_OHMS: f32 = 0.1;

/// INA226 I2C address (7-bit). `power_monitor.h`'s `INA226_I2C_ADDR` -
/// matches the crate's own `ina226::DEFAULT_ADDRESS`, restated here so the
/// hardware fact is visible in this file rather than only in a dependency.
pub const INA226_ADDR: u8 = 0x40;

/// `[safety] iwdg_timeout_ms` - independent hardware watchdog. Same value and
/// same role as `firmware/chassis/src/config.rs::IWDG_TIMEOUT_US`: catches a
/// hung *firmware* task (stuck I2C transaction, deadlocked executor), as
/// opposed to the link watchdog below, which catches a dead *peer*.
pub const IWDG_TIMEOUT_US: u32 = 500_000;

/// How often the link-watchdog task's petting tick fires. Must be
/// comfortably under `IWDG_TIMEOUT_US` (500 ms) so a hang anywhere else in
/// the firmware still resets the board — chosen at the same ~2.5x margin
/// chassis uses (its 200 ms command timeout against a 500 ms IWDG window).
pub const IWDG_PET_INTERVAL_MS: u64 = 200;

/// Plan §5.2: "if the RPi stops *consuming* (no `Telemetry` heartbeat seen
/// for 2 s), it logs and flashes the status LED." This board has no
/// actuation to cut, so this is the mirror image of the chassis board's
/// 200 ms command watchdog - much longer, because its job is to make a
/// one-way link failure *visible*, not to stop a motor before it hurts
/// something.
///
/// See `watchdog.rs`'s module doc for a real integration gap this constant
/// cannot fix by itself: `config/rover.toml`'s `[routes]` table does not
/// currently route anything at all to `sensors`.
pub const LINK_TIMEOUT_MS: u64 = 2_000;
