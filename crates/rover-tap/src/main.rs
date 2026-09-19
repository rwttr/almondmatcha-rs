//! `rover-tap` — a debug CLI that replaces `ros2 topic echo`.
//!
//! There is no `ros2 bag`, no topic list, no `rqt` here: this bus has no
//! discovery (plan §0), so there is nothing to introspect except by binding a
//! socket where a real service would and watching what arrives. That is
//! exactly what this does, "as" whichever service's traffic you want to see
//! — see `rover_link::PeerId`'s doc comment for why that is a service
//! (`control`, `navigation`, ...) rather than a host since design defect D1.
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

    /// Which service's traffic to observe (e.g. "control", "telemetry",
    /// "chassis" — see `config/rover.toml`'s `[services]`). Routing is
    /// unicast fan-out, not multicast (plan §4.1), so this tool only sees
    /// what is actually addressed to that service — pick the one that
    /// receives what you care about; check `[routes]` for which service a
    /// type goes to. Ignored if `--mirror` is given.
    #[arg(long = "as", default_value = "control")]
    as_host: String,

    /// Bind to the debug firehose mirror instead of a named service — the
    /// only way to see the *entire* bus at once, including `ChassisCommand`
    /// and `Telemetry`, which no single `--as <service>` can (see `--as`'s
    /// help).
    /// Requires `[debug] mirror` to be set in the config to a real address,
    /// normally this machine's own — that is what tells every publisher to
    /// send a copy here.
    #[arg(long)]
    mirror: bool,

    /// Only show one message type, by its wire name (e.g. `ImuSample`).
    #[arg(long = "type")]
    type_filter: Option<String>,

    /// Show a per-type rate/loss table instead of every message.
    #[arg(long)]
    hz: bool,
}

/// Where to bind and which senders to recognise, resolved from either
/// `--as <service>` or `--mirror`.
struct Listen {
    bind_addr: SocketAddr,
    peers: HashMap<PeerId, SocketAddr>,
    /// What to print in the startup banner.
    label: String,
    /// `UdpLink::bind` needs *some* `PeerId` to label itself with, but a
    /// mirror listener doesn't stand in for any of the real services — it
    /// is an extra observer, not a peer anything sends *to* by name. Nothing
    /// in this binary reads `UdpLink::self_id()` back, so an arbitrary value
    /// here is inert; `label` above is what actually gets shown.
    link_self_id: PeerId,
}

fn plan_listen(config: &BusConfig, args: &Args) -> Result<Listen, TapError> {
    if args.mirror {
        let mirror_addr = config.mirror().ok_or(TapError::MirrorNotConfigured)?;
        let mut peers = HashMap::new();
        for peer in PeerId::ALL {
            if let Some(addr) = config.addr_of(peer) {
                peers.insert(peer, addr);
            }
        }
        return Ok(Listen {
            bind_addr: SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), mirror_addr.port()),
            peers,
            label: "the debug mirror (the whole bus)".to_string(),
            link_self_id: PeerId::Base,
        });
    }

    let self_id =
        PeerId::parse(&args.as_host).ok_or_else(|| TapError::UnknownHost(args.as_host.clone()))?;
    let port = config
        .addr_of(self_id)
        .ok_or(TapError::NoAddress(self_id))?
        .port();

    // Bind on every interface rather than the exact address `rover.toml`
    // gives that service: `rover-tap` is a laptop-run debug tool, not the
    // real process for that service, and it has no reason to require
    // running on the same machine that owns that IP. Only the *port* — and
    // the peer addresses below, checked against each datagram's source —
    // need to match the config for anything to be recognised.
    let bind_addr = SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), port);

    let mut peers = HashMap::new();
    for peer in PeerId::ALL {
        if peer != self_id {
            if let Some(addr) = config.addr_of(peer) {
                peers.insert(peer, addr);
            }
        }
    }

    Ok(Listen {
        bind_addr,
        peers,
        label: format!("`{self_id}`"),
        link_self_id: self_id,
    })
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
    let listen = plan_listen(&config, &args)?;

    let link = UdpLink::bind(listen.link_self_id, listen.bind_addr, listen.peers)
        .map_err(TapError::Link)?;

    eprintln!(
        "rover-tap: listening as {} on port {}{}",
        listen.label,
        listen.bind_addr.port(),
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
    MirrorNotConfigured,
}

impl fmt::Display for TapError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TapError::Config(e) => write!(f, "loading config: {e}"),
            TapError::UnknownHost(h) => write!(f, "`{h}` is not a known service (see PeerId::ALL)"),
            TapError::NoAddress(p) => write!(f, "no address configured for `{p}` in [services]"),
            TapError::Link(e) => write!(f, "{e}"),
            TapError::MirrorNotConfigured => write!(
                f,
                "--mirror was given but [debug] mirror is empty in the config; \
                 set it to this machine's address (e.g. \"192.168.1.100:7099\") first"
            ),
        }
    }
}

impl std::error::Error for TapError {}
