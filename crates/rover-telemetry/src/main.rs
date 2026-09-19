//! `rover-telemetry` — RPi binary: per-topic CSV logging + the 5 Hz
//! `Telemetry` feed to the base station.
//!
//! Replaces `rover_monitoring_node.cpp` (the full-fidelity CSV logger) and
//! `mission_monitoring_node_rpi.cpp` (the Domain 5 -> Domain 4 relay) with
//! **one** process rather than two.
//!
//! # Why one process where ROS 2 used two, and why that does not undo
//! `HANDOFF_field_run_verification.md`'s ruling
//!
//! The ROS 2 split existed for two reasons, per the hand-off doc: (1) keep
//! the relay lean as "the planned future home for a low-bitrate LPWAN
//! telemetry link", and (2) make sure the *relay* role specifically never
//! grows local-storage responsibilities that could stall it. Reason (1) does
//! not apply to this rewrite — `rover-telemetry`'s job already includes
//! picking `Telemetry` vs. the (future) `TelemetryLite` per plan §6.3, so
//! constrained-link logic already lives here regardless of whether CSV
//! logging does too. Reason (2) is the one that must survive, and it does:
//! every CSV write goes through [`csv_writer::CsvLogger`]'s background
//! thread and channel (see its module doc comment), so a slow SD card can
//! never delay the 5 Hz `Telemetry` publish this process is also
//! responsible for. **What must not come back** — and does not — is a
//! second bridge of raw `LaneMeasurement` data onto the RPi/base side: this
//! binary never subscribes to `LaneMeasurement` at all. Lane staleness for
//! `HealthBits::LANE_STALE` comes from `RoverState::lane_age_ms`, which the
//! estimator already computes — see `health.rs`.
//!
//! # Two cross-cutting gaps that used to live here, now fixed
//!
//! `docs/RUST_REWRITE_PLAN.md` §13.3b, D1 and D2:
//!
//! 1. **One UDP port per host, three RPi processes.** This process now binds
//!    `PeerId::Telemetry`'s own address from `[services]`, not a `"rpi"`
//!    entry shared with `rover-control`/`rover-navigation` — see
//!    `rover-link::PeerId`'s doc comment for the fix (D1).
//! 2. **`GnssFix` could not self-identify as rtk/backup on the wire.** Fixed
//!    by `GnssFix::source` (D2): this process reads it directly instead of
//!    guessing from `FixQuality` (the old `gnss_source::classify` heuristic,
//!    now deleted).
//!
//! # A remaining gap
//!
//! **`last_cmd_seq` is computed independently, not shared via IPC.** This
//! process runs its own `rover_bus::CommandReceiver`, fed by whatever
//! `CommandFrame`s it itself receives (routed to `telemetry` alongside
//! `control` and `navigation` — see `config/rover.toml`). `CommandReceiver::
//! apply` is a pure function of the sequence of frames seen so far, so as
//! long as every process observes the same frames, they all agree on
//! `last_applied` without talking to each other — no shared state needed for
//! *this* value specifically.

mod config;
mod csv_fmt;
mod csv_writer;
mod health;
mod runs;

use csv_writer::CsvLogger;
use health::stall::StallDetector;
use health::{compute_health, FeedAges};
use rover_bus::{Bus, BusConfig, CommandReceiver};
use rover_link::{PeerId, UdpLink};
use rover_msgs::{
    ChassisStatus, CommandFrame, GnssFix, GnssSource, MissionStatus, PowerSample, RoverState,
    SpeedLoopDebug, Telemetry,
};
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use clap::Parser;

#[derive(Parser)]
#[command(about = "Per-topic CSV logging + 5 Hz Telemetry feed, replacing \
                    rover_monitoring_node / mission_monitoring_node_rpi.")]
struct Args {
    #[arg(long, default_value = "config/rover.toml")]
    config: std::path::PathBuf,

    /// Root directory under which `run_NNN_<stamp>/` is created — see
    /// `runs.rs`. Overridable for bench testing away from the real `runs/`
    /// tree; `$ROVER_RUN_DIR`, if set, still takes priority (see
    /// `RunDir::resolve`).
    #[arg(long, default_value = "runs")]
    runs_dir: std::path::PathBuf,
}

/// `Telemetry` publish rate to the base — plan §3.2.
const TELEMETRY_INTERVAL: Duration = Duration::from_millis(200);
/// Bus polling interval. Faster than the telemetry rate so per-topic CSVs
/// (some native rates up to 100 Hz, e.g. `RoverState`) are not artificially
/// downsampled by this loop.
const POLL_INTERVAL: Duration = Duration::from_millis(5);

fn now_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

/// Everything the subscription callbacks update and the aggregation loop
/// reads, behind one `RefCell` — this process is single-threaded apart from
/// the CSV writer threads (which only ever receive already-formatted rows
/// over a channel, never touch this state), so a `RefCell` is enough; there
/// is no data race to design around.
#[derive(Default)]
struct SharedState {
    rover_state: RoverState,
    mission: MissionStatus,
    power: PowerSample,
    chassis_status: Option<ChassisStatus>,
    rtk_fix: GnssFix,
    backup_fix: GnssFix,
    last_cmd_seq: u16,
    stall_detected: bool,

    chassis_seen: Option<Instant>,
    power_seen: Option<Instant>,
    rtk_seen: Option<Instant>,
    backup_seen: Option<Instant>,
}

fn main() {
    env_logger::init();
    let args = Args::parse();

    let bus_config = BusConfig::load(&args.config).unwrap_or_else(|e| {
        log::error!("loading {}: {e}", args.config.display());
        std::process::exit(1);
    });
    let telemetry_config = config::TelemetryConfig::load(&args.config).unwrap_or_else(|e| {
        log::error!(
            "loading telemetry config from {}: {e}",
            args.config.display()
        );
        std::process::exit(1);
    });

    let bind_addr = bus_config.addr_of(PeerId::Telemetry).unwrap_or_else(|| {
        log::error!(
            "no [services] entry for `telemetry` in {}",
            args.config.display()
        );
        std::process::exit(1);
    });
    let link = UdpLink::bind(PeerId::Telemetry, bind_addr, bus_config.peers().clone())
        .unwrap_or_else(|e| {
            log::error!("binding {bind_addr}: {e}");
            std::process::exit(1);
        });
    let mut bus = Bus::new(link, bus_config);

    std::fs::create_dir_all(&args.runs_dir).ok();
    let run_dir = Arc::new(runs::RunDir::resolve(&args.runs_dir));
    log::info!("logging to {}", run_dir.path().display());

    let rover_state_log = CsvLogger::spawn(
        run_dir.clone(),
        "rover_state.csv",
        csv_fmt::ROVER_STATE_HEADER,
    );
    let mission_log = CsvLogger::spawn(
        run_dir.clone(),
        "mission_status.csv",
        csv_fmt::MISSION_STATUS_HEADER,
    );
    let power_log = CsvLogger::spawn(run_dir.clone(), "power.csv", csv_fmt::POWER_HEADER);
    let rtk_log = CsvLogger::spawn(run_dir.clone(), "rtk_gnss.csv", csv_fmt::GNSS_HEADER);
    let backup_log = CsvLogger::spawn(run_dir.clone(), "backup_gnss.csv", csv_fmt::GNSS_HEADER);
    let chassis_status_log = CsvLogger::spawn(
        run_dir.clone(),
        "chassis_status.csv",
        csv_fmt::CHASSIS_STATUS_HEADER,
    );
    let speed_loop_log = CsvLogger::spawn(
        run_dir.clone(),
        "speed_loop_debug.csv",
        csv_fmt::SPEED_LOOP_DEBUG_HEADER,
    );

    let state = Rc::new(RefCell::new(SharedState::default()));
    let stall_detector = Rc::new(RefCell::new(StallDetector::new(telemetry_config.stall)));
    let mut cmd_receiver = CommandReceiver::new();

    {
        let state = state.clone();
        bus.subscribe::<RoverState>(move |s| {
            state.borrow_mut().rover_state = s;
            rover_state_log.log(csv_fmt::format_rover_state_row(now_us(), &s));
        });
    }
    {
        let state = state.clone();
        bus.subscribe::<MissionStatus>(move |m| {
            state.borrow_mut().mission = m;
            mission_log.log(csv_fmt::format_mission_status_row(now_us(), &m));
        });
    }
    {
        let state = state.clone();
        bus.subscribe::<PowerSample>(move |p| {
            let mut s = state.borrow_mut();
            s.power = p;
            s.power_seen = Some(Instant::now());
            power_log.log(csv_fmt::format_power_row(now_us(), &p));
        });
    }
    {
        let state = state.clone();
        bus.subscribe::<ChassisStatus>(move |c| {
            let mut s = state.borrow_mut();
            s.chassis_status = Some(c);
            s.chassis_seen = Some(Instant::now());
            chassis_status_log.log(csv_fmt::format_chassis_status_row(now_us(), &c));
        });
    }
    {
        let state = state.clone();
        bus.subscribe::<GnssFix>(move |fix| {
            let mut s = state.borrow_mut();
            // Self-describing since D2 (plan §13.3b) — no more guessing from
            // `FixQuality`. See `rover_msgs::GnssSource`'s doc comment.
            match fix.source {
                GnssSource::Rtk => {
                    s.rtk_fix = fix;
                    s.rtk_seen = Some(Instant::now());
                    rtk_log.log(csv_fmt::format_gnss_row(now_us(), &fix));
                }
                GnssSource::Backup => {
                    s.backup_fix = fix;
                    s.backup_seen = Some(Instant::now());
                    backup_log.log(csv_fmt::format_gnss_row(now_us(), &fix));
                }
            }
        });
    }
    {
        let state = state.clone();
        let stall_detector = stall_detector.clone();
        bus.subscribe::<SpeedLoopDebug>(move |d| {
            speed_loop_log.log(csv_fmt::format_speed_loop_debug_row(now_us(), &d));
            let now_s = now_us() as f64 / 1_000_000.0;
            let stalled = stall_detector.borrow_mut().observe(&d, now_s);
            state.borrow_mut().stall_detected = stalled;
        });
    }

    log::info!("rover-telemetry ready, binding {bind_addr}");

    let mut next_publish = Instant::now();
    let mut seq: u32 = 0;

    loop {
        bus.poll();

        if let Some(frame) = bus.latest::<CommandFrame>() {
            // See gap 3 in the module doc comment: applied purely for its
            // sequence-number bookkeeping, not to act on the command.
            let _ = cmd_receiver.apply(frame);
        }
        state.borrow_mut().last_cmd_seq = cmd_receiver.last_applied();

        let now = Instant::now();
        if now >= next_publish {
            next_publish = now + TELEMETRY_INTERVAL;
            seq = seq.wrapping_add(1);

            let s = state.borrow();
            let ages = FeedAges {
                chassis_ms: s
                    .chassis_seen
                    .map(|t| now.duration_since(t).as_millis() as u64),
                sensors_ms: s
                    .power_seen
                    .map(|t| now.duration_since(t).as_millis() as u64),
                rtk_ms: s.rtk_seen.map(|t| now.duration_since(t).as_millis() as u64),
                backup_gnss_ms: s
                    .backup_seen
                    .map(|t| now.duration_since(t).as_millis() as u64),
            };
            let health = compute_health(
                &ages,
                s.rover_state.lane_age_ms,
                telemetry_config.lane_stale_ms,
                s.chassis_status.as_ref(),
                s.stall_detected,
                &s.rover_state,
            );

            let telemetry = Telemetry {
                seq,
                t_us: now_us(),
                state: s.rover_state,
                mission: s.mission,
                power: s.power,
                rtk: s.rtk_fix,
                backup: s.backup_fix,
                last_cmd_seq: s.last_cmd_seq,
                health,
            };
            drop(s);
            let _ = bus.publish(&telemetry);
        }

        std::thread::sleep(POLL_INTERVAL);
    }
}
