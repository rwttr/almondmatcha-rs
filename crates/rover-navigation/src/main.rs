//! `rover-navigation` — RPi binary: dual GNSS + the mission state machine.
//!
//! Replaces `gnss_ublox_node.cpp`, `gnss_spresense_node.cpp` and
//! `gnss_mission_monitor_node.cpp` with one process: two serial readers on
//! their own threads feeding [`gnss::UbloxAssembler`] /
//! [`gnss::SpresenseAssembler`], and a main loop that runs [`mission::Mission`]
//! and answers [`rover_msgs::CommandFrame`]s from the base station.
//!
//! # Two known cross-cutting gaps, not fixed here
//!
//! Both are protocol/deployment questions that reach outside this crate's
//! locked dependencies (`rover-msgs`, `rover-bus`, `rover-link`) and are
//! reported rather than silently patched around, per this task's brief:
//!
//! 1. **One UDP port per host, three RPi processes.** `config/rover.toml`
//!    gives `PeerId::Rpi` exactly one port. `rover-control`, `rover-navigation`
//!    and `rover-telemetry` are three separate OS processes that each need to
//!    *receive* traffic addressed to "rpi" (`CommandFrame` here;
//!    `ImuSample`/`WheelSensors`/`LaneMeasurement` for `rover-control`;
//!    `RoverState`/`MissionStatus`/`PowerSample`/`GnssFix` for
//!    `rover-telemetry`). Only one process can actually bind that port at a
//!    time under the current schema — `rover-link::UdpLink` does not use
//!    `SO_REUSEPORT`, and unicast delivery would not fan out to multiple
//!    listeners even if it did. This binary binds `PeerId::Rpi` the same way
//!    `rover-tap` does, which is correct in isolation and under test, but it
//!    will collide (`EADDRINUSE`) with `rover-control`/`rover-telemetry` doing
//!    the same on real hardware. Resolving it needs a decision in
//!    `rover-bus`/`rover-link`/`config/rover.toml` (per-service ports, or an
//!    in-process fan-out) that is out of this crate's scope.
//! 2. **`GnssFix` cannot self-identify as "rtk" or "backup" on the bus.**
//!    `rover_msgs::GnssFix` is one `Wire` type with one `TYPE_ID` for both
//!    receivers by design (see its doc comment), so the bus's newest-wins,
//!    one-slot-per-type model cannot let a cross-process subscriber tell
//!    which receiver a given `GnssFix` came from. Within this process that
//!    is not a problem — `select_navigation_fix` sees both readings
//!    directly — but any other process subscribing to `GnssFix` over the
//!    bus (`rover-telemetry`, to fill `Telemetry::rtk`/`Telemetry::backup`)
//!    cannot reliably do so from this stream alone. See `rover-telemetry`'s
//!    `main.rs` for how it copes today, and the top-level report for why a
//!    real fix means adding a discriminant to `GnssFix` (or splitting it
//!    into two types) in `rover-msgs`.

mod config;
mod geo;
mod gnss;
mod mission;
mod nmea;
mod rtcm;
mod serial;
mod spresense_json;
mod time_utils;

use clap::Parser;
use gnss::{SpresenseAssembler, UbloxAssembler};
use mission::{select_navigation_fix, Mission};
use rover_bus::{Bus, BusConfig, CommandReceiver};
use rover_link::{PeerId, UdpLink};
use rover_msgs::{CommandFrame, GnssFix, MissionStatus};
use std::io::Read;
use std::sync::mpsc;
use std::time::Duration;

#[derive(Parser)]
#[command(about = "GNSS + mission state machine, replacing gnss_ublox_node / \
                    gnss_spresense_node / gnss_mission_monitor_node.")]
struct Args {
    /// Path to the bus + mission config file.
    #[arg(long, default_value = "config/rover.toml")]
    config: std::path::PathBuf,

    /// u-blox SimpleRTK2b serial port. Overridable because the ROS 2 node
    /// hardcoded `/dev/ttyACM0`, which is a udev enumeration order, not a
    /// stable identity — a bench setup or a second board on the same host
    /// legitimately needs a different path.
    #[arg(long, default_value = serial::UBLOX_DEFAULT_PORT)]
    ublox_port: String,

    #[arg(long, default_value_t = serial::UBLOX_DEFAULT_BAUD)]
    ublox_baud: u32,

    /// Spresense serial port. See `--ublox-port`.
    #[arg(long, default_value = serial::SPRESENSE_DEFAULT_PORT)]
    spresense_port: String,

    #[arg(long, default_value_t = serial::SPRESENSE_DEFAULT_BAUD)]
    spresense_baud: u32,
}

/// How often the main loop ticks when there is nothing to do. Fast enough
/// that mission state and command handling feel immediate; slow enough not
/// to spin the CPU on an RPi core with headroom needed elsewhere.
const TICK_INTERVAL: Duration = Duration::from_millis(50);

/// Cap on an incomplete serial line before it is dropped — see
/// `gnss.rs::LineReader`.
const MAX_LINE_LEN: usize = 4096;

fn main() {
    env_logger::init();
    let args = Args::parse();

    let bus_config = BusConfig::load(&args.config).unwrap_or_else(|e| {
        log::error!("loading {}: {e}", args.config.display());
        std::process::exit(1);
    });
    let mission_config = config::MissionConfig::load(&args.config).unwrap_or_else(|e| {
        log::error!("loading [mission] from {}: {e}", args.config.display());
        std::process::exit(1);
    });

    let bind_addr = bus_config.addr_of(PeerId::Rpi).unwrap_or_else(|| {
        log::error!(
            "no [hosts]/[ports] entry for `rpi` in {}",
            args.config.display()
        );
        std::process::exit(1);
    });
    let link =
        UdpLink::bind(PeerId::Rpi, bind_addr, bus_config.peers().clone()).unwrap_or_else(|e| {
            log::error!("binding {bind_addr}: {e}");
            std::process::exit(1);
        });
    let mut bus = Bus::new(link, bus_config);

    let (ublox_tx, ublox_rx) = mpsc::channel::<GnssFix>();
    let (spresense_tx, spresense_rx) = mpsc::channel::<GnssFix>();

    spawn_reader(
        "ublox",
        args.ublox_port.clone(),
        args.ublox_baud,
        UbloxAssembler::new(),
        ublox_tx,
    );
    spawn_reader(
        "spresense",
        args.spresense_port.clone(),
        args.spresense_baud,
        SpresenseAssembler::new(),
        spresense_tx,
    );

    let mut mission = Mission::new(&mission_config);
    let mut cmd_receiver = CommandReceiver::new();
    let mut rtk_fix = GnssFix::default();
    let mut backup_fix = GnssFix::default();

    log::info!(
        "rover-navigation ready: ublox={} spresense={} mission_state={:?}",
        args.ublox_port,
        args.spresense_port,
        mission.state()
    );

    loop {
        bus.poll();

        for fix in ublox_rx.try_iter() {
            rtk_fix = fix;
            let _ = bus.publish(&rtk_fix);
        }
        for fix in spresense_rx.try_iter() {
            backup_fix = fix;
            let _ = bus.publish(&backup_fix);
        }

        if let Some(frame) = bus.latest::<CommandFrame>() {
            if let Some(cmd) = cmd_receiver.apply(frame) {
                log::info!(
                    "applying command: {cmd:?} (mission state was {:?}, goal was {:?})",
                    mission.state(),
                    mission.goal()
                );
                mission.apply_command(cmd);
            }
        }

        let position = select_navigation_fix(&rtk_fix, &backup_fix);
        let status: MissionStatus = mission.update(position);
        let _ = bus.publish(&status);

        std::thread::sleep(TICK_INTERVAL);
    }
}

/// Spawn a thread that owns one serial port for its whole life, feeding
/// complete lines to `assembler` and sending every resulting fix down `tx`.
///
/// A port that fails to open (no hardware attached — the common case on a
/// dev machine) logs once and retries on a backoff, rather than exiting the
/// whole process: the other GNSS source and the mission state machine should
/// keep working even if one receiver is unplugged.
fn spawn_reader<A>(
    name: &'static str,
    path: String,
    baud: u32,
    mut assembler: A,
    tx: mpsc::Sender<GnssFix>,
) where
    A: GnssAssembler + Send + 'static,
{
    std::thread::spawn(move || {
        let mut reader = gnss::LineReader::new(MAX_LINE_LEN);
        let mut buf = [0u8; 512];
        loop {
            let mut port = match serial::open(&path, baud) {
                Ok(p) => p,
                Err(e) => {
                    log::warn!("{name}: opening {path}: {e} — retrying in 5s");
                    std::thread::sleep(Duration::from_secs(5));
                    continue;
                }
            };
            log::info!("{name}: reading {path} at {baud} baud");
            loop {
                match port.read(&mut buf) {
                    Ok(0) => continue,
                    Ok(n) => {
                        for line in reader.feed(&buf[..n]) {
                            if let Some(fix) = assembler.ingest(&line) {
                                if tx.send(fix).is_err() {
                                    return; // main thread gone
                                }
                            }
                        }
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
                    Err(e) => {
                        log::warn!("{name}: read error on {path}: {e} — reopening");
                        break;
                    }
                }
            }
        }
    });
}

/// Common interface over the two assemblers, so `spawn_reader` is written
/// once instead of twice.
trait GnssAssembler {
    fn ingest(&mut self, line: &str) -> Option<GnssFix>;
}
impl GnssAssembler for UbloxAssembler {
    fn ingest(&mut self, line: &str) -> Option<GnssFix> {
        UbloxAssembler::ingest(self, line)
    }
}
impl GnssAssembler for SpresenseAssembler {
    fn ingest(&mut self, line: &str) -> Option<GnssFix> {
        SpresenseAssembler::ingest(self, line)
    }
}
