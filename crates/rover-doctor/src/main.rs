//! `rover-doctor` — base PC binary: preflight GO/NO-GO check.
//!
//! Run once on the base PC before driving, in place of `ground-station`
//! (both bind the `base` service, so they cannot run at the same time —
//! that is fine, this tool is meant to run *before* the operator starts
//! `ground-station` for the session, not alongside it). It listens for
//! about ten seconds, then prints one line per check and exits `0` for GO
//! or non-zero for NO-GO, so a launch script can gate on it directly:
//!
//! ```text
//! rover-doctor --config config/rover.toml || exit 1
//! ```
//!
//! # Why a listen window, not a one-shot poll
//!
//! Every feed this tool judges is a UDP broadcast on its own schedule —
//! `BoardDiagnostics` at 1 Hz, `Telemetry` at 5 Hz (`config/rover.toml`,
//! `rover_msgs::types`). A single `Bus::poll()` call would race those
//! schedules and report `NOT SEEN` on nothing more than bad luck. Ten
//! seconds is comfortably more than one full period of the slowest feed,
//! long enough to make a `NOT SEEN` verdict mean what it says: nothing
//! arrived, not "nothing arrived yet".
//!
//! # Never report GO on missing data
//!
//! All of the actual judgement is in `verdict.rs`, which is deliberately
//! free of sockets, clocks, and sleeps so it can be tested as a pure
//! function. This file only owns the parts that cannot be pure: binding the
//! link, running the listen window, and printing the result.

mod config;
mod verdict;

use clap::Parser;
use config::DoctorConfigError;
use rover_bus::{Bus, BusConfig};
use rover_link::{PeerId, UdpLink};
use rover_msgs::{BoardDiagnostics, BoardId, Telemetry};
use std::cell::RefCell;
use std::rc::Rc;
use std::time::{Duration, Instant};
use verdict::{evaluate, DoctorConfig, Observations, Verdict};

#[derive(Parser)]
#[command(about = "Preflight GO/NO-GO check: run on the base PC before driving.")]
struct Args {
    #[arg(long, default_value = "config/rover.toml")]
    config: std::path::PathBuf,

    /// How long to listen for feeds before judging. Ten seconds comfortably
    /// covers more than one period of every feed this tool checks, including
    /// `BoardDiagnostics`' 1 Hz heartbeat -- see the module doc comment.
    #[arg(long, default_value_t = 10)]
    listen_secs: u64,
}

const POLL_INTERVAL: Duration = Duration::from_millis(20);

fn main() {
    env_logger::init();
    let args = Args::parse();

    let bus_config = BusConfig::load(&args.config).unwrap_or_else(|e| {
        log::error!("loading {}: {e}", args.config.display());
        std::process::exit(2);
    });
    let doctor_config = DoctorConfig::load(&args.config).unwrap_or_else(|e: DoctorConfigError| {
        log::error!(
            "loading drivetrain/estimator config from {}: {e}",
            args.config.display()
        );
        std::process::exit(2);
    });

    let bind_addr = bus_config.addr_of(PeerId::Base).unwrap_or_else(|| {
        log::error!(
            "no [services] entry for `base` in {}",
            args.config.display()
        );
        std::process::exit(2);
    });
    let link =
        UdpLink::bind(PeerId::Base, bind_addr, bus_config.peers().clone()).unwrap_or_else(|e| {
            log::error!("binding {bind_addr}: {e}");
            std::process::exit(2);
        });
    let mut bus = Bus::new(link, bus_config);

    // `BoardDiagnostics` shares one TYPE_ID for both boards (see its own
    // doc comment), so `Bus`'s single newest-wins slot cannot distinguish
    // them -- exactly the reason `rover-telemetry::health::BoardHealth`
    // exists. Track each board's last sample in its own `Rc<RefCell<_>>`
    // slot instead.
    let chassis_diag: Rc<RefCell<Option<BoardDiagnostics>>> = Rc::new(RefCell::new(None));
    let sensors_diag: Rc<RefCell<Option<BoardDiagnostics>>> = Rc::new(RefCell::new(None));
    {
        let chassis_diag = chassis_diag.clone();
        let sensors_diag = sensors_diag.clone();
        bus.subscribe::<BoardDiagnostics>(move |d| match d.board {
            BoardId::Chassis => *chassis_diag.borrow_mut() = Some(d),
            BoardId::Sensors => *sensors_diag.borrow_mut() = Some(d),
        });
    }

    println!(
        "rover-doctor: listening on {bind_addr} for {}s (board diagnostics + telemetry)...",
        args.listen_secs
    );

    let deadline = Instant::now() + Duration::from_secs(args.listen_secs);
    while Instant::now() < deadline {
        bus.poll();
        std::thread::sleep(POLL_INTERVAL);
    }

    let obs = Observations {
        chassis_diag: *chassis_diag.borrow(),
        sensors_diag: *sensors_diag.borrow(),
        telemetry: bus.latest::<Telemetry>(),
    };

    let results = evaluate(&obs, &doctor_config);

    println!();
    let mut all_go = true;
    for r in &results {
        if !r.verdict.is_go() {
            all_go = false;
        }
        println!("[{:<8}] {} -- {}", label(r.verdict), r.name, r.detail);
    }

    println!();
    if all_go {
        println!("GO: every check passed.");
        std::process::exit(0);
    } else {
        println!(
            "NO-GO: see the checks above. NOT SEEN means a feed never arrived -- check \
                   that the process is running and the cable is connected; NO-GO means a fault \
                   was actually observed."
        );
        std::process::exit(1);
    }
}

fn label(v: Verdict) -> &'static str {
    match v {
        Verdict::Go => "GO",
        Verdict::NoGo => "NO-GO",
        Verdict::NotSeen => "NOT SEEN",
    }
}
