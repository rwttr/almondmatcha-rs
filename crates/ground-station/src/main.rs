//! `ground-station` — base PC binary: send commands, show live telemetry,
//! log received frames to CSV.
//!
//! Replaces `mission_command_node.cpp` (goal/speed-limit/watchdog action
//! client) and `mission_monitoring_node_pc.cpp` (telemetry display) with one
//! process, matching plan §1.2's single `ground-station` entry. The ROS 2
//! split existed because actions and services needed a client node and a
//! display needed a subscriber node; `rover-bus`'s idempotent
//! `CommandSender`/`Bus::subscribe` need neither, so there is no longer a
//! structural reason to keep them apart.
//!
//! # E-stop: read this before wiring this up to anything real
//!
//! `Command::EStop` crosses a UDP link with no delivery guarantee. This
//! binary retransmits it once a second until `Telemetry::{last_cmd_session,
//! last_cmd_seq}` acknowledges it (`rover_bus::CommandSender`, already
//! written and tested — see its doc comment), which is the best a network
//! command can do. An EStop pre-empts: it discards every queued command and
//! replaces the one in flight (each is logged at warn level), and nothing
//! issued afterwards can displace it. It is
//! **not** the rover's safety mechanism. The guaranteed stop is the
//! firmware command watchdog (plan §5.2): 200 ms with no `ChassisCommand`
//! trips it, steering centres, throttle ramps to zero over 300 ms, and it
//! needs no packet from this binary to fire. `display.rs::ESTOP_DISCLAIMER`
//! says this on every redraw, not just once at startup, because the moment
//! an operator reaches for E-stop is the moment they are least likely to
//! recall a start-up banner.
//!
//! # What is not tested here
//!
//! Nothing in this file is covered by `cargo test` — it needs a real
//! terminal, a real socket and a live rover to exercise meaningfully. Every
//! piece of *logic* it calls (`link::classify`, `command_parse::parse_command`,
//! `display::render_dashboard` and friends, `csv_log::format_row`) is pure
//! and unit-tested without any of those. See `docs/RUST_REWRITE_PLAN.md` §13.4 for what
//! specifically needs a bench/field run to confirm end-to-end.

mod command_parse;
mod csv_log;
mod display;
mod link;

use clap::Parser;
use rover_bus::{Bus, BusConfig, CommandSender};
use rover_link::{PeerId, UdpLink};
use rover_msgs::Telemetry;
use std::cell::RefCell;
use std::io::BufRead;
use std::rc::Rc;
use std::sync::mpsc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

#[derive(Parser)]
#[command(about = "Send commands, show live telemetry, log to CSV -- replacing \
                    mission_command_node / mission_monitoring_node_pc.")]
struct Args {
    #[arg(long, default_value = "config/rover.toml")]
    config: std::path::PathBuf,

    /// Root directory under which `run_NNN_<stamp>/` is created — see
    /// `csv_log.rs` and `rover-runs`. Received telemetry is logged inside it
    /// as `ground_station_telemetry.csv`, created lazily on first write.
    #[arg(long, default_value = "runs")]
    runs_dir: std::path::PathBuf,
}

const REDRAW_INTERVAL: Duration = Duration::from_millis(200);
const POLL_INTERVAL: Duration = Duration::from_millis(20);

fn now_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

fn main() {
    env_logger::init();
    let args = Args::parse();

    let bus_config = BusConfig::load(&args.config).unwrap_or_else(|e| {
        log::error!("loading {}: {e}", args.config.display());
        std::process::exit(1);
    });
    let bind_addr = bus_config.addr_of(PeerId::Base).unwrap_or_else(|| {
        log::error!(
            "no [services] entry for `base` in {}",
            args.config.display()
        );
        std::process::exit(1);
    });
    let link_socket = UdpLink::bind(PeerId::Base, bind_addr, bus_config.peers().clone())
        .unwrap_or_else(|e| {
            log::error!("binding {bind_addr}: {e}");
            std::process::exit(1);
        });
    let mut bus = Bus::new(link_socket, bus_config);

    // Nothing here creates a directory eagerly -- see `csv_log.rs`'s doc
    // comment. `create_dir_all` on the *root* only ensures the scan
    // `RunDir::resolve` does has somewhere to look; it does not create (or
    // consume a run number for) the run directory itself.
    std::fs::create_dir_all(&args.runs_dir).ok();
    let run_dir = rover_runs::RunDir::resolve(&args.runs_dir);
    let run_dir_path = run_dir.path().display().to_string();
    let csv_logger = csv_log::CsvLogger::spawn(run_dir);
    let mut sender = CommandSender::default();
    log::info!("command session {:#06x}", sender.session());

    let latest_telemetry: Rc<RefCell<Telemetry>> = Rc::new(RefCell::new(Telemetry::default()));
    let last_telemetry_at: Rc<RefCell<Option<Instant>>> = Rc::new(RefCell::new(None));

    {
        let latest_telemetry = latest_telemetry.clone();
        let last_telemetry_at = last_telemetry_at.clone();
        bus.subscribe::<Telemetry>(move |t| {
            csv_logger.log(csv_log::format_row(now_us(), &t));
            *latest_telemetry.borrow_mut() = t;
            *last_telemetry_at.borrow_mut() = Some(Instant::now());
        });
    }

    // Stdin is read on its own thread and fed to the main loop over a
    // channel: a blocking `read_line` cannot share a thread with the
    // poll/redraw loop, and this is the plain-terminal equivalent of a raw
    // keyboard-event stream (see command_parse.rs's doc comment on why this
    // is words-plus-Enter rather than single keystrokes).
    let (cmd_tx, cmd_rx) = mpsc::channel::<String>();
    std::thread::spawn(move || {
        let stdin = std::io::stdin();
        for line in stdin.lock().lines() {
            match line {
                Ok(l) => {
                    if cmd_tx.send(l).is_err() {
                        return;
                    }
                }
                Err(_) => return,
            }
        }
    });

    log::info!("ground-station ready, bound {bind_addr}, logging under {run_dir_path}");
    println!("{}", command_parse::command_hint());

    let mut next_redraw = Instant::now();
    // Anything the operator needs to read goes through this, not `println!`:
    // the 200 ms full-screen redraw would erase a printed line almost at once.
    let mut notice = display::Notice::default();

    let mut publish_limit = csv_log::RateLimit::with_interval(Duration::from_secs(5));

    loop {
        bus.poll();

        while let Ok(line) = cmd_rx.try_recv() {
            let mut msgs: Vec<String> = Vec::new();
            match command_parse::parse_command(&line) {
                Ok(cmd) => {
                    let is_estop = matches!(cmd, rover_msgs::Command::EStop);
                    // An EStop drops whatever was queued or in flight; say so
                    // loudly, once per command, so the operator knows what
                    // will NOT be sent (and, for the in-flight one, may
                    // already have been applied).
                    for d in sender.set(cmd) {
                        let msg = match d.cmd_seq {
                            Some(seq) => format!(
                                "E-STOP discarded in-flight command {:?} (cmd_seq={seq}, \
                                 unacknowledged -- the rover may already have applied it)",
                                d.body
                            ),
                            None => format!("E-STOP discarded queued command {:?}", d.body),
                        };
                        log::warn!("{msg}");
                        msgs.push(msg);
                    }
                    if is_estop {
                        // The disclaimer itself is on every redraw already.
                        msgs.push(match sender.in_flight() {
                            Some((seq, _)) => format!("Sending E-STOP (cmd_seq={seq})."),
                            None => "Sending E-STOP.".to_string(),
                        });
                    } else if sender.queue_len() > 0 {
                        msgs.push(format!(
                            "Queued behind the unacknowledged command ({} waiting).",
                            sender.queue_len()
                        ));
                    }
                }
                Err(e) => msgs.push(format!("command error: {e}")),
            }
            if !msgs.is_empty() {
                notice.set(msgs.join("\n>> "), Instant::now());
            }
        }

        if let Some(frame) = sender.poll(Instant::now()) {
            // A send error (no route, socket down) used to vanish, leaving
            // the operator staring at "pending ack" for a command that never
            // left the PC. Log it (rate-limited: poll() retries, so this can
            // repeat every cycle) and put it on the dashboard too.
            if let Err(e) = bus.publish(&frame) {
                let now = Instant::now();
                if let Some(n) = publish_limit.check(now) {
                    log::warn!("command send failed: {e} (+{n} similar failures suppressed)");
                }
                notice.set(format!("command send FAILED: {e}"), now);
            }
        }
        {
            let t = latest_telemetry.borrow();
            sender.on_telemetry(t.last_cmd_session, t.last_cmd_seq);
        }

        let now = Instant::now();
        if now >= next_redraw {
            next_redraw = now + REDRAW_INTERVAL;
            let age_ms = last_telemetry_at
                .borrow()
                .map(|t| now.duration_since(t).as_millis() as u64);
            let status = link::classify(age_ms);
            let t = latest_telemetry.borrow();
            let command = display::CommandStatus {
                in_flight: sender.in_flight(),
                queue_len: sender.queue_len(),
                session: sender.session(),
                // Only an echo under OUR session is an ack of ours; right
                // after a restart the rover still echoes the old session's
                // seq (or 0 = no consensus).
                last_acked_seq: (t.last_cmd_session == sender.session()).then_some(t.last_cmd_seq),
            };
            print!(
                "{}",
                display::render_dashboard(
                    status,
                    &t.rtk,
                    &t.backup,
                    &t.state,
                    &t.mission,
                    &t.power,
                    t.health,
                    &command,
                    notice.visible(now),
                )
            );
        }

        std::thread::sleep(POLL_INTERVAL);
    }
}
