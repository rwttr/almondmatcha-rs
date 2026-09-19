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
//! binary retransmits it once a second until `Telemetry::last_cmd_seq`
//! echoes back (`rover_bus::CommandSender`, already written and tested —
//! see its doc comment), which is the best a network command can do. It is
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
//! and unit-tested without any of those. See the top-level report for what
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

    /// Where received telemetry is logged. See `csv_log.rs` for why this
    /// exists despite the ROS 2 base station never having had one.
    #[arg(long, default_value = "ground_station_telemetry.csv")]
    log_file: std::path::PathBuf,
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
            "no [hosts]/[ports] entry for `base` in {}",
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

    let csv_logger = csv_log::CsvLogger::spawn(&args.log_file);
    let mut sender = CommandSender::default();

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

    log::info!(
        "ground-station ready, bound {bind_addr}, logging to {}",
        args.log_file.display()
    );
    println!("commands: estop | clearestop | cancel | goal <lat> <lon> | speed <pct> | nop");

    let mut next_redraw = Instant::now();

    loop {
        bus.poll();

        while let Ok(line) = cmd_rx.try_recv() {
            match command_parse::parse_command(&line) {
                Ok(cmd) => {
                    let is_estop = matches!(cmd, rover_msgs::Command::EStop);
                    sender.set(cmd);
                    if is_estop {
                        // Printed immediately, not just on the next redraw --
                        // an operator who just pressed Enter on "estop"
                        // should see the disclaimer without waiting for the
                        // next 200 ms tick.
                        println!("\nSending E-STOP (cmd_seq={}).", sender.cmd_seq());
                        println!("{}", display::ESTOP_DISCLAIMER);
                    }
                }
                Err(e) => println!("\ncommand error: {e}"),
            }
        }

        if let Some(frame) = sender.poll(Instant::now()) {
            let _ = bus.publish(&frame);
        }
        sender.on_telemetry(latest_telemetry.borrow().last_cmd_seq);

        let now = Instant::now();
        if now >= next_redraw {
            next_redraw = now + REDRAW_INTERVAL;
            let age_ms = last_telemetry_at
                .borrow()
                .map(|t| now.duration_since(t).as_millis() as u64);
            let status = link::classify(age_ms);
            let t = latest_telemetry.borrow();
            let pending_seq = (!sender.is_acked()).then(|| sender.cmd_seq());
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
                    pending_seq,
                    t.last_cmd_seq,
                )
            );
        }

        std::thread::sleep(POLL_INTERVAL);
    }
}
