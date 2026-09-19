//! Loading `config/rover.toml`'s bus-relevant sections: `[hosts]`, `[ports]`
//! and `[routes]`.
//!
//! This is the only place those three tables get parsed. Everything else in
//! `rover.toml` (drivetrain calibration, control gains, estimator noise, ...)
//! belongs to other crates, so it is deserialized into an untyped
//! [`toml::Value`] here and ignored — adding a section to the config file for
//! `rover-control` should never require a change in this crate.

use rover_link::PeerId;
use std::collections::HashMap;
use std::fmt;
use std::net::{AddrParseError, IpAddr, SocketAddr};
use std::path::Path;

/// Bus-relevant configuration, resolved into the types the rest of this
/// crate works with: real [`SocketAddr`]s per [`PeerId`], and routes keyed by
/// a message's [`rover_msgs::Wire::NAME`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BusConfig {
    peers: HashMap<PeerId, SocketAddr>,
    // Keyed by `Wire::NAME` rather than `Wire::TYPE_ID`: the plan (§4.1) and
    // `rover.toml` both spell routes by name, and resolving by name means
    // `Bus::route_for::<T>()` needs nothing but `T` — no separate registry of
    // every type's ID has to be kept in step with this file.
    routes: HashMap<String, Vec<PeerId>>,
    // Debug firehose mirror (`[debug] mirror`). `None` when unset — the
    // field default, and the only state that costs `Bus::publish` anything
    // at runtime: see its doc comment.
    mirror: Option<SocketAddr>,
}

impl BusConfig {
    /// Load and resolve `config/rover.toml` (or any file with the same
    /// `[hosts]` / `[ports]` / `[routes]` shape).
    pub fn load(path: impl AsRef<Path>) -> Result<Self, ConfigError> {
        let text = std::fs::read_to_string(path.as_ref())
            .map_err(|e| ConfigError::Io(path.as_ref().display().to_string(), e))?;
        Self::parse(&text)
    }

    /// Parse from an already-read string. Split out from [`BusConfig::load`]
    /// so tests can exercise malformed configs without touching the
    /// filesystem.
    pub fn parse(toml_text: &str) -> Result<Self, ConfigError> {
        let raw: RawConfig = toml::from_str(toml_text).map_err(ConfigError::Toml)?;

        let mut peers = HashMap::new();
        for (name, ip_str) in &raw.hosts {
            let peer = PeerId::parse(name).ok_or_else(|| ConfigError::UnknownHost(name.clone()))?;
            let ip: IpAddr = ip_str
                .parse()
                .map_err(|e| ConfigError::BadAddress(name.clone(), e))?;
            let port = *raw
                .ports
                .get(name)
                .ok_or_else(|| ConfigError::MissingPort(name.clone()))?;
            peers.insert(peer, SocketAddr::new(ip, port));
        }

        let mut routes = HashMap::new();
        for (msg_name, hosts) in raw.routes {
            let mut dests = Vec::with_capacity(hosts.len());
            for host in hosts {
                dests.push(
                    PeerId::parse(&host).ok_or_else(|| ConfigError::UnknownHost(host.clone()))?,
                );
            }
            routes.insert(msg_name, dests);
        }

        // Empty (the field default, and what a bare `[debug]` section with no
        // `mirror` key also deserializes to) means disabled. Anything else
        // must parse as a real address — a typo'd mirror address that
        // silently disables debugging is worse than a startup failure, since
        // the whole point is to have it available when a field problem shows
        // up.
        let mirror = match raw.debug.mirror.trim() {
            "" => None,
            addr => Some(
                addr.parse::<SocketAddr>()
                    .map_err(|e| ConfigError::BadMirrorAddress(addr.to_string(), e))?,
            ),
        };

        Ok(Self {
            peers,
            routes,
            mirror,
        })
    }

    /// The resolved address of a peer, if `[hosts]`/`[ports]` named it.
    pub fn addr_of(&self, peer: PeerId) -> Option<SocketAddr> {
        self.peers.get(&peer).copied()
    }

    /// Every configured peer and its address, for building a [`Link`]'s peer
    /// table.
    ///
    /// [`Link`]: rover_link::Link
    pub fn peers(&self) -> &HashMap<PeerId, SocketAddr> {
        &self.peers
    }

    /// Destinations configured for a message type, by its [`Wire::NAME`].
    ///
    /// An empty slice — rather than an error — for a type with no `[routes]`
    /// entry: a partial config (a bench rig with only some hosts wired up) is
    /// a normal thing to run with, not a broken one.
    ///
    /// [`Wire::NAME`]: rover_msgs::Wire::NAME
    pub fn route_for<T: rover_msgs::Wire>(&self) -> &[PeerId] {
        self.routes.get(T::NAME).map(Vec::as_slice).unwrap_or(&[])
    }

    /// The debug firehose mirror address (`[debug] mirror`), if configured.
    ///
    /// `Bus::publish` sends an extra, best-effort copy of every frame here on
    /// top of its normal route — see its doc comment. `None` (an absent or
    /// empty `mirror` key) is the zero-cost default field runs must keep.
    pub fn mirror(&self) -> Option<SocketAddr> {
        self.mirror
    }
}

/// Shape of the TOML file, before host/port names are resolved into
/// [`PeerId`]s and validated against each other. Any other table in
/// `rover.toml` (`[drivetrain]`, `[control]`, ...) is simply absent from this
/// struct and `serde` leaves it alone.
#[derive(Debug, serde::Deserialize)]
struct RawConfig {
    hosts: HashMap<String, String>,
    ports: HashMap<String, u16>,
    #[serde(default)]
    routes: HashMap<String, Vec<String>>,
    #[serde(default)]
    debug: RawDebug,
}

/// `[debug]` is entirely optional — a config with no such section at all
/// (every fixture and test config before this feature existed) must still
/// parse, with mirroring disabled.
#[derive(Debug, Default, serde::Deserialize)]
struct RawDebug {
    #[serde(default)]
    mirror: String,
}

/// Something was wrong with a bus config file.
#[derive(Debug)]
pub enum ConfigError {
    /// Could not read the file at all. Carries the path so the message is
    /// useful without the caller having to repeat it.
    Io(String, std::io::Error),
    /// Malformed TOML, or missing/mistyped one of the required tables.
    Toml(toml::de::Error),
    /// A `[hosts]` or `[routes]` entry named a host that is not one of
    /// [`PeerId::ALL`].
    UnknownHost(String),
    /// A `[hosts]` entry's address did not parse as an IP.
    BadAddress(String, AddrParseError),
    /// A host appears in `[hosts]` but has no matching entry in `[ports]`.
    MissingPort(String),
    /// `[debug] mirror` was non-empty but did not parse as `ip:port`.
    BadMirrorAddress(String, AddrParseError),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConfigError::Io(path, e) => write!(f, "reading `{path}`: {e}"),
            ConfigError::Toml(e) => write!(f, "parsing config: {e}"),
            ConfigError::UnknownHost(name) => {
                write!(f, "`{name}` is not a known host (see PeerId::ALL)")
            }
            ConfigError::BadAddress(name, e) => {
                write!(f, "host `{name}` has an invalid address: {e}")
            }
            ConfigError::MissingPort(name) => {
                write!(f, "host `{name}` is in [hosts] but missing from [ports]")
            }
            ConfigError::BadMirrorAddress(addr, e) => {
                write!(f, "[debug] mirror = \"{addr}\" is not a valid address: {e}")
            }
        }
    }
}

impl std::error::Error for ConfigError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            ConfigError::Io(_, e) => Some(e),
            ConfigError::Toml(e) => Some(e),
            ConfigError::BadAddress(_, e) => Some(e),
            ConfigError::BadMirrorAddress(_, e) => Some(e),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rover_msgs::{ChassisCommand, ImuSample, Telemetry};

    const SAMPLE: &str = r#"
        [hosts]
        rpi     = "192.168.1.1"
        chassis = "192.168.1.2"
        base    = "192.168.1.10"

        [ports]
        rpi     = 7001
        chassis = 7002
        base    = 7005

        [routes]
        ImuSample      = ["rpi"]
        ChassisCommand = ["chassis"]
        Telemetry      = ["base"]

        # A section that belongs to another crate entirely. Must not break
        # parsing here.
        [drivetrain]
        wheel_diameter_m = 0.125
        decoding         = "quadrature_4x"
    "#;

    #[test]
    fn resolves_hosts_to_socket_addrs() {
        let cfg = BusConfig::parse(SAMPLE).unwrap();
        assert_eq!(
            cfg.addr_of(PeerId::Rpi),
            Some("192.168.1.1:7001".parse().unwrap())
        );
        assert_eq!(
            cfg.addr_of(PeerId::Base),
            Some("192.168.1.10:7005".parse().unwrap())
        );
        assert_eq!(cfg.addr_of(PeerId::Jetson), None);
    }

    #[test]
    fn resolves_routes_by_wire_name() {
        let cfg = BusConfig::parse(SAMPLE).unwrap();
        assert_eq!(cfg.route_for::<ImuSample>(), &[PeerId::Rpi]);
        assert_eq!(cfg.route_for::<ChassisCommand>(), &[PeerId::Chassis]);
        // Asserted against SAMPLE, this module's own fixture — deliberately
        // not against config/rover.toml, which `loads_the_real_repo_config`
        // covers. Keeping them separate means a routing change in the shipped
        // config fails exactly one test, and that test names the real file.
        assert_eq!(cfg.route_for::<Telemetry>(), &[PeerId::Base]);
    }

    #[test]
    fn unrouted_type_resolves_to_no_destinations() {
        let cfg = BusConfig::parse(SAMPLE).unwrap();
        // GnssFix has no [routes] entry in SAMPLE.
        assert!(cfg.route_for::<rover_msgs::GnssFix>().is_empty());
    }

    #[test]
    fn unknown_host_in_hosts_table_is_rejected() {
        let bad = SAMPLE.replace("rpi     = \"192.168.1.1\"", "rover    = \"192.168.1.1\"");
        let err = BusConfig::parse(&bad).unwrap_err();
        assert!(matches!(err, ConfigError::UnknownHost(h) if h == "rover"));
    }

    #[test]
    fn unknown_host_in_routes_is_rejected() {
        let bad = SAMPLE.replace(
            "ImuSample      = [\"rpi\"]",
            "ImuSample      = [\"groundstation\"]",
        );
        let err = BusConfig::parse(&bad).unwrap_err();
        assert!(matches!(err, ConfigError::UnknownHost(h) if h == "groundstation"));
    }

    #[test]
    fn host_missing_from_ports_is_rejected() {
        let bad = SAMPLE.replace("rpi     = 7001\n", "");
        let err = BusConfig::parse(&bad).unwrap_err();
        assert!(matches!(err, ConfigError::MissingPort(h) if h == "rpi"));
    }

    #[test]
    fn bad_ip_address_is_rejected() {
        // Quoted, so this matches only rpi's address and not the
        // `192.168.1.10` prefix shared with base's.
        let bad = SAMPLE.replace("\"192.168.1.1\"", "\"not-an-ip\"");
        let err = BusConfig::parse(&bad).unwrap_err();
        assert!(matches!(err, ConfigError::BadAddress(h, _) if h == "rpi"));
    }

    #[test]
    fn mirror_absent_section_is_none() {
        // SAMPLE has no [debug] section at all.
        let cfg = BusConfig::parse(SAMPLE).unwrap();
        assert_eq!(cfg.mirror(), None);
    }

    #[test]
    fn mirror_empty_string_is_none() {
        let with_debug = format!("{SAMPLE}\n[debug]\nmirror = \"\"\n");
        let cfg = BusConfig::parse(&with_debug).unwrap();
        assert_eq!(cfg.mirror(), None);
    }

    #[test]
    fn mirror_present_resolves_to_socket_addr() {
        let with_debug = format!("{SAMPLE}\n[debug]\nmirror = \"192.168.1.100:7099\"\n");
        let cfg = BusConfig::parse(&with_debug).unwrap();
        assert_eq!(cfg.mirror(), Some("192.168.1.100:7099".parse().unwrap()));
    }

    #[test]
    fn mirror_malformed_address_is_a_hard_error_not_a_silent_none() {
        let with_debug = format!("{SAMPLE}\n[debug]\nmirror = \"not-an-address\"\n");
        let err = BusConfig::parse(&with_debug).unwrap_err();
        assert!(matches!(err, ConfigError::BadMirrorAddress(a, _) if a == "not-an-address"));
    }

    #[test]
    fn loads_the_real_repo_config() {
        // The actual file this crate ships against. If this ever fails, the
        // config file and this parser have drifted apart.
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../config/rover.toml");
        let cfg = BusConfig::load(path).expect("config/rover.toml must parse");
        assert_eq!(
            cfg.addr_of(PeerId::Rpi),
            Some("192.168.1.1:7001".parse().unwrap())
        );
        assert_eq!(cfg.route_for::<ImuSample>(), &[PeerId::Rpi]);
        // Telemetry also goes to the sensors board: it publishes only, so it
        // has no command stream to time out on, and uses Telemetry as its
        // liveness heartbeat (plan §5.2 mirror watchdog).
        assert_eq!(
            cfg.route_for::<Telemetry>(),
            &[PeerId::Base, PeerId::Sensors]
        );
        // Shipped default is `mirror = ""` — disabled.
        assert_eq!(cfg.mirror(), None);
    }
}
