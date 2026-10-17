//! `rover-navigation` — RPi binary: dual GNSS + the mission state machine.
//!
//! Replaces `gnss_ublox_node.cpp`, `gnss_spresense_node.cpp` and
//! `gnss_mission_monitor_node.cpp` with one process: two serial readers on
//! their own threads feeding [`gnss::UbloxAssembler`] /
//! [`gnss::SpresenseAssembler`], and a main loop that runs [`mission::Mission`]
//! and answers [`rover_msgs::CommandFrame`]s from the base station.
//!
//! # Two cross-cutting gaps that used to live here, now fixed
//!
//! `docs/RUST_REWRITE_PLAN.md` §13.3b, D1 and D2 — recorded here because this
//! file's own history flagged both:
//!
//! 1. **One UDP port per host, three RPi processes.** `config/rover.toml`
//!    used to give `PeerId::Rpi` exactly one port, and `rover-control`,
//!    `rover-navigation` and `rover-telemetry` all needed to bind it —
//!    correct in isolation and under test, but an `EADDRINUSE` collision on
//!    real hardware. Fixed by making `PeerId` a service identity: `[hosts]`/
//!    `[ports]` collapsed into one `[services]` table with a full `host:port`
//!    per process, so this binary now binds its own `PeerId::Navigation`
//!    address. See `rover-link::PeerId`'s doc comment.
//! 2. **`GnssFix` could not self-identify as "rtk" or "backup" on the bus.**
//!    `rover_msgs::GnssFix` was one `Wire` type with one `TYPE_ID` for both
//!    receivers, so a cross-process subscriber (`rover-telemetry`, filling
//!    `Telemetry::rtk`/`Telemetry::backup`) could not tell which receiver a
//!    given reading came from. Fixed by `GnssFix::source`: this file's
//!    `UbloxAssembler`/`SpresenseAssembler` (`gnss.rs`) now set it directly
//!    at the point each reading is assembled — see `rover_msgs::GnssSource`.

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
use mission::{select_fresh_navigation_fix, Mission, GNSS_FIX_MAX_AGE_S};
use rover_bus::{Bus, BusConfig, CommandReceiver};
use rover_link::{PeerId, UdpLink};
use rover_msgs::{CommandFrame, GnssFix, MissionStatus};
use std::io::Read;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::mpsc;
use std::time::{Duration, Instant};

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

    let bind_addr = bus_config.addr_of(PeerId::Navigation).unwrap_or_else(|| {
        log::error!(
            "no [services] entry for `navigation` in {}",
            args.config.display()
        );
        std::process::exit(1);
    });
    let link = UdpLink::bind(PeerId::Navigation, bind_addr, bus_config.peers().clone())
        .unwrap_or_else(|e| {
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
    // Receive time of each stored fix. A fix is only as good as it is recent:
    // without this the last fix was held forever (see `GNSS_FIX_MAX_AGE_S`).
    let mut rtk_rx_at: Option<Instant> = None;
    let mut backup_rx_at: Option<Instant> = None;
    let start = Instant::now();

    // `subscribe`, not `latest`: `latest` is newest-wins, so two `CommandFrame`s
    // landing between ticks (an EStop followed by a Nop retransmit, say) would
    // lose the first. The callback runs inside `poll`, once per frame, in
    // order; the channel just hands them to this loop (same shape as
    // rover-control's network thread).
    let (cmd_tx, cmd_rx) = mpsc::channel::<CommandFrame>();
    bus.subscribe::<CommandFrame>(move |frame| {
        let _ = cmd_tx.send(frame);
    });

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
            rtk_rx_at = Some(Instant::now());
            let _ = bus.publish(&rtk_fix);
        }
        for fix in spresense_rx.try_iter() {
            backup_fix = fix;
            backup_rx_at = Some(Instant::now());
            let _ = bus.publish(&backup_fix);
        }

        for frame in cmd_rx.try_iter() {
            if let Some(cmd) = cmd_receiver.apply(frame) {
                log::info!(
                    "applying command: {cmd:?} (mission state was {:?}, goal was {:?})",
                    mission.state(),
                    mission.goal()
                );
                mission.apply_command(cmd);
            }
        }

        let now = Instant::now();
        let age_s = |t: Option<Instant>| t.map(|t| now.duration_since(t).as_secs_f64());
        let position = select_fresh_navigation_fix(
            &rtk_fix,
            age_s(rtk_rx_at),
            &backup_fix,
            age_s(backup_rx_at),
            GNSS_FIX_MAX_AGE_S,
        );
        let status: MissionStatus =
            mission.update_at(position, now.duration_since(start).as_secs_f64());
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
                        // A panic while parsing one line of line-noise used to
                        // end this thread for good: no more fixes, the last
                        // one held forever. Parse under `catch_unwind` so one
                        // bad line costs one line.
                        let lines = match guarded(name, "line framing", || reader.feed(&buf[..n])) {
                            Some(lines) => lines,
                            None => {
                                reader = gnss::LineReader::new(MAX_LINE_LEN);
                                continue;
                            }
                        };
                        for line in lines {
                            if let Some(Some(fix)) =
                                guarded(name, "sentence parsing", || assembler.ingest(&line))
                            {
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

/// Run `f`, turning a panic into a logged `None` so a reader thread survives
/// a malformed line.
fn guarded<T>(name: &str, what: &str, f: impl FnOnce() -> T) -> Option<T> {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(v) => Some(v),
        Err(_) => {
            log::error!("{name}: panic during {what} — dropping that input and carrying on");
            None
        }
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn guarded_survives_a_panic_and_the_next_call_still_works() {
        let r: Option<i32> = guarded("t", "boom", || panic!("bad line"));
        assert_eq!(r, None);
        assert_eq!(guarded("t", "fine", || 7), Some(7));
    }

    /// End to end through the real assembler: a lossy-UTF8 line that used to
    /// panic inside `verify_and_strip` is now just dropped, and a good line
    /// after it still yields a fix.
    #[test]
    fn ublox_assembler_survives_non_ascii_noise_then_parses_a_good_line() {
        let mut a = UbloxAssembler::new();
        assert!(guarded("u", "parse", || a.ingest("$GPGGA,1*\u{FFFD}\u{FFFD}")).is_some());
        let body = "GPGGA,123456.00,1342.2030,N,10031.3912,E,1,08,0.9,10.0,M,0.0,M,,";
        let cs = body.bytes().fold(0u8, |acc, b| acc ^ b);
        let fix = a.ingest(&format!("${body}*{cs:02X}"));
        assert!(fix.is_some());
    }
}
