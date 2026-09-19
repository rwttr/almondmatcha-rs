//! The named services on the rover network.
//!
//! **Design defect D1** (`docs/RUST_REWRITE_PLAN.md` §13.3b): this used to be
//! five *hosts* (`Rpi`, `Chassis`, `Jetson`, `Sensors`, `Base`), one UDP port
//! each, per `config/rover.toml`'s old `[hosts]`/`[ports]` tables. But the RPi
//! runs **three** separate receiving processes — `rover-control`,
//! `rover-navigation`, `rover-telemetry` — that all need to bind `PeerId::Rpi`.
//! On real hardware the second and third to start die with `EADDRINUSE`, and
//! even if `UdpLink` used `SO_REUSEPORT` that load-balances one socket's
//! traffic across listeners rather than duplicating it to all of them, which
//! is not what this bus needs. The bug was a modelling error: the bus routes
//! to *endpoints*, and an endpoint is a process, not a machine.
//!
//! **Fix — `PeerId` names a service (a process), not a host.** `config/
//! rover.toml`'s `[hosts]`/`[ports]` collapse into one `[services]` table
//! mapping a service name straight to a `host:port` socket address, and
//! `[routes]` targets service names. A single physical machine can (and does,
//! for the RPi) host several services, each with its own port — that is what
//! makes running `rover-control`, `rover-navigation` and `rover-telemetry` as
//! three OS processes actually work.
use std::fmt;
use std::str::FromStr;

/// A named endpoint on the rover network — one bus-visible process.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PeerId {
    /// `rover-control`, on the RPi.
    Control,
    /// `rover-navigation`, on the RPi.
    Navigation,
    /// `rover-telemetry`, on the RPi.
    Telemetry,
    /// The chassis board firmware.
    Chassis,
    /// The sensors board firmware.
    Sensors,
    /// The Jetson perception process.
    Perception,
    /// `ground-station`, on the base PC.
    Base,
}

impl PeerId {
    /// All seven, in the order `config/rover.toml`'s `[services]` table
    /// lists them.
    pub const ALL: [PeerId; 7] = [
        PeerId::Control,
        PeerId::Navigation,
        PeerId::Telemetry,
        PeerId::Chassis,
        PeerId::Sensors,
        PeerId::Perception,
        PeerId::Base,
    ];

    /// The spelling used as a key in `config/rover.toml`'s `[services]` /
    /// `[routes]` tables, and by this type's `Display` impl.
    pub const fn as_str(self) -> &'static str {
        match self {
            PeerId::Control => "control",
            PeerId::Navigation => "navigation",
            PeerId::Telemetry => "telemetry",
            PeerId::Chassis => "chassis",
            PeerId::Sensors => "sensors",
            PeerId::Perception => "perception",
            PeerId::Base => "base",
        }
    }

    /// Parse a `config/rover.toml` service name. Case-sensitive on purpose:
    /// the config file is the one source of truth for spelling, and silently
    /// accepting `"Control"` would just move a typo further from where it
    /// starts.
    pub fn parse(name: &str) -> Option<PeerId> {
        Self::ALL.into_iter().find(|p| p.as_str() == name)
    }
}

impl fmt::Display for PeerId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl FromStr for PeerId {
    type Err = UnknownPeerName;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Self::parse(s).ok_or_else(|| UnknownPeerName(s.to_string()))
    }
}

/// A service name that matches none of [`PeerId::ALL`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownPeerName(pub String);

impl fmt::Display for UnknownPeerName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "unknown service name `{}`", self.0)
    }
}

impl std::error::Error for UnknownPeerName {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_every_name() {
        for peer in PeerId::ALL {
            assert_eq!(PeerId::parse(peer.as_str()), Some(peer));
            assert_eq!(peer.as_str().parse::<PeerId>(), Ok(peer));
        }
    }

    #[test]
    fn rejects_unknown_and_wrong_case() {
        assert_eq!(
            PeerId::parse("rpi"),
            None,
            "the old host-based name is gone"
        );
        assert_eq!(PeerId::parse("Control"), None);
        assert!("nonsense".parse::<PeerId>().is_err());
    }
}
