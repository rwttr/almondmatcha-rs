//! `rover-tap` — a debug CLI that replaces `ros2 topic echo`.
//!
//! There is no `ros2 bag`, no topic list, no `rqt` here: this bus has no
//! discovery (plan §0), so there is nothing to introspect except by binding a
//! socket where a real host would and watching what arrives. That is exactly
//! what this does, "as" whichever host's traffic you want to see.
//!
//! Two modes:
//! - default: pretty-print every decoded message, one line per frame.
//! - `--hz`: a live per-type rate and packet-loss table instead — the
//!   question that actually matters standing next to a rover in a field.

#![forbid(unsafe_code)]

mod registry;
mod stats;

use clap::Parser;
use rover_bus::{BusConfig, ConfigError};
use rover_link::{Link, LinkError, PeerId, UdpLink};
use std::collections::HashMap;
use std::fmt;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

#[derive(Parser)]
#[command(about = "Debug sniffer for the rover bus, replacing `ros2 topic echo`.")]
struct Args {
    /// Path to the bus config file.
    #[arg(long, default_value = "config/rover.toml")]
    config: PathBuf,

    /// Which host's traffic to observe. Routing is unicast fan-out, not
    /// multicast (plan §4.1), so this tool only sees what is actually
    /// addressed to that host — pick the one that receives what you care
    /// about. Most types route to "rpi".
    #[arg(long = "as", default_value = "rpi")]
    as_host: String,

    /// Only show one message type, by its wire name (e.g. `ImuSample`).
    #[arg(long = "type")]
    type_filter: Option<String>,

    /// Show a per-type rate/loss table instead of every message.
    #[arg(long)]
    hz: bool,
}

fn main() -> ExitCode {
    let args = Args::parse();
    match run(args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("rover-tap: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(args: Args) -> Result<(), TapError> {
    let config = BusConfig::load(&args.config).map_err(TapError::Config)?;

    let self_id =
        PeerId::parse(&args.as_host).ok_or_else(|| TapError::UnknownHost(args.as_host.clone()))?;
    let port = config
        .addr_of(self_id)
        .ok_or(TapError::NoAddress(self_id))?
        .port();

    // Bind on every interface rather than the exact address `rover.toml`
    // gives that host: `rover-tap` is a laptop-run debug tool, not the real
    // process for that host, and it has no reason to require running on the
    // same machine that owns that IP. Only the *port* — and the peer
    // addresses below, checked against each datagram's source — need to
    // match the config for anything to be recognised.
    let bind_addr = SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), port);

    let mut peers = HashMap::new();
    for peer in PeerId::ALL {
        if peer != self_id {
            if let Some(addr) = config.addr_of(peer) {
                peers.insert(peer, addr);
            }
        }
    }

    let link = UdpLink::bind(self_id, bind_addr, peers).map_err(TapError::Link)?;

    eprintln!(
        "rover-tap: listening as `{self_id}` on port {port}{}",
        args.type_filter
            .as_deref()
            .map(|t| format!(", filtering to `{t}`"))
            .unwrap_or_default()
    );

    if args.hz {
        run_hz(link, args.type_filter.as_deref())
    } else {
        run_echo(link, args.type_filter.as_deref())
    }
}

/// Print every matching message as it arrives, with a wall-clock timestamp.
fn run_echo(mut link: UdpLink, filter: Option<&str>) -> Result<(), TapError> {
    loop {
        match link.recv() {
            Some((peer, frame)) => {
                let Some(decoded) = registry::decode(frame.header.type_id, frame.body) else {
                    continue;
                };
                if filter.is_some_and(|f| f != decoded.name) {
                    continue;
                }
                println!(
                    "{} {peer:<8} seq={:<6} {} {}",
                    timestamp(),
                    frame.header.seq,
                    decoded.name,
                    decoded.rendered
                );
            }
            None => std::thread::sleep(Duration::from_millis(2)),
        }
    }
}

/// Print a per-type rate/loss table once a second instead of every message.
fn run_hz(mut link: UdpLink, filter: Option<&str>) -> Result<(), TapError> {
    let mut tracker = stats::RateTracker::new();
    let report_every = Duration::from_secs(1);

    loop {
        while let Some((_peer, frame)) = link.recv() {
            let Some(decoded) = registry::decode(frame.header.type_id, frame.body) else {
                continue;
            };
            if filter.is_some_and(|f| f != decoded.name) {
                continue;
            }
            tracker.record(frame.header.type_id, decoded.name, frame.header.seq);
        }

        if tracker.elapsed() >= report_every {
            let lines = tracker.report();
            println!("--- {} ---", timestamp());
            if lines.is_empty() {
                println!("  (nothing received)");
            }
            for line in lines {
                println!("  {line}");
            }
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn timestamp() -> String {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    format!("[{:>10}.{:03}]", now.as_secs(), now.subsec_millis())
}

#[derive(Debug)]
enum TapError {
    Config(ConfigError),
    UnknownHost(String),
    NoAddress(PeerId),
    Link(LinkError),
}

impl fmt::Display for TapError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TapError::Config(e) => write!(f, "loading config: {e}"),
            TapError::UnknownHost(h) => write!(f, "`{h}` is not a known host (see PeerId::ALL)"),
            TapError::NoAddress(p) => write!(f, "no address configured for `{p}` in [hosts]"),
            TapError::Link(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for TapError {}
