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

/// `[services] sensors` - this board's own address.
///
/// # Design defect D1 (`docs/RUST_REWRITE_PLAN.md` §13.3b)
///
/// `config/rover.toml` used to give one UDP port per *host*, under `[hosts]`/
/// `[ports]`, which put three separate RPi processes on the same "rpi" port —
/// only one could actually bind it. `[services]` now maps a service name
/// straight to a `host:port`, one per *process*, so the RPi side below is
/// `CONTROL_ADDR`/`TELEMETRY_ADDR` (two different ports on the same IP)
/// instead of a single `RPI_IP`/`RPI_PORT` pair.
pub const SELF_IP: Ipv4Address = Ipv4Address::new(192, 168, 1, 6);
pub const SELF_PORT: u16 = 7011;

/// `[services] control` address. `rover-control` subscribes to
/// `WheelSensors` (`[routes]`) for the speed loop and odometry.
pub const CONTROL_ADDR: (Ipv4Address, u16) = (Ipv4Address::new(192, 168, 1, 1), 7001);

/// `[services] telemetry` address. `rover-telemetry` subscribes to
/// `PowerSample` from this board (`[routes]`), and this board watches
/// *inbound* `Telemetry` from here as its own link-watchdog heartbeat — see
/// `watchdog.rs`.
pub const TELEMETRY_ADDR: (Ipv4Address, u16) = (Ipv4Address::new(192, 168, 1, 1), 7003);

/// `[base]` - the base-station PC. On the same `/24` as every other host on
/// this closed LAN (`SELF_IP`'s static config carries no gateway at all -
/// see `net.rs::init`), so it's directly reachable with no routing hop.
pub const BASE_ADDR: (Ipv4Address, u16) = (Ipv4Address::new(192, 168, 1, 10), 7030);

/// Per-message-type destination table.
///
/// Every message this board publishes has exactly one destination, so each
/// is a single address rather than a slice. Naming them by message type
/// (not by host) is what makes it obvious at the call site *why* that
/// destination was chosen, and keeps a future second destination a one-line
/// change here rather than a call-site rewrite.
///
/// # This used to send `WheelSensors` to telemetry as well, and it was wrong
///
/// The slice was justified by a comment claiming `rover-telemetry` consumed
/// wheel ticks "for its CSV and liveness bookkeeping". It does neither:
/// `crates/rover-telemetry/src/main.rs` never calls
/// `bus.subscribe::<WheelSensors>`, `csv_fmt.rs` has no wheel formatter, and
/// `HealthBits::SENSORS_STALE` is aged off `PowerSample` arrivals. So the
/// board was emitting a 10 Hz datagram that the RPi decoded and dropped,
/// while `config/rover.toml` said `WheelSensors = ["control"]` — the one
/// invariant this file rests on ("every constant cites the `rover.toml` key
/// it was transcribed from") was already broken by it.
///
/// If wheel ticks are ever wanted on the RPi — recording a calibration run
/// for plan §2.6 is the obvious reason — add `"telemetry"` to `[routes]`
/// *and* a subscriber, together. Until then a `[debug] mirror` plus
/// `rover-tap` sees them without changing firmware.
pub const WHEEL_SENSORS_DEST: (Ipv4Address, u16) = CONTROL_ADDR;
pub const POWER_SAMPLE_DEST: (Ipv4Address, u16) = TELEMETRY_ADDR;

/// `[routes] BoardDiagnostics = ["telemetry", "base"]` - the one message
/// this board publishes to *two* destinations, not one: `rover-telemetry`'s
/// health-tracking and the base-station operator both need this board's
/// self-test/reset-cause/PHY-health report, and neither should have to
/// relay it to the other. See `diag::publish_task`.
pub const BOARD_DIAGNOSTICS_DESTS: [(Ipv4Address, u16); 2] = [TELEMETRY_ADDR, BASE_ADDR];

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
/// `watchdog.rs`'s module doc has the routing this depends on:
/// `config/rover.toml`'s `[routes]` sends `Telemetry` to `["base", "sensors"]`
/// specifically so this board has a heartbeat to watch. (An earlier draft of
/// this comment described a gap where nothing routed to `"sensors"` at all;
/// that was fixed in `rover.toml`, and `watchdog.rs` was updated while this
/// sentence was not.) Still unexercised on real hardware — plan §13.4.
pub const LINK_TIMEOUT_MS: u64 = 2_000;
