//! The five hosts on the rover network.
//!
//! `docs/RUST_REWRITE_PLAN.md` §1.2 and `config/rover.toml`'s `[hosts]` table
//! name exactly these five. A closed enum rather than a `String` or a raw
//! `SocketAddr`, because the set is small, fixed at build time, and shared by
//! every binary in the workspace: a typo in a host name becomes a compile
//! error here instead of a misrouted UDP packet in a field.

use std::fmt;
use std::str::FromStr;

/// A named endpoint on the rover network.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PeerId {
    Rpi,
    Chassis,
    Jetson,
    Sensors,
    Base,
}

impl PeerId {
    /// All five, in the order `config/rover.toml` lists them.
    pub const ALL: [PeerId; 5] = [
        PeerId::Rpi,
        PeerId::Chassis,
        PeerId::Jetson,
        PeerId::Sensors,
        PeerId::Base,
    ];

    /// The spelling used as a key in `config/rover.toml`'s `[hosts]` /
    /// `[ports]` tables, and by this type's `Display` impl.
    pub const fn as_str(self) -> &'static str {
        match self {
            PeerId::Rpi => "rpi",
            PeerId::Chassis => "chassis",
            PeerId::Jetson => "jetson",
            PeerId::Sensors => "sensors",
            PeerId::Base => "base",
        }
    }

    /// Parse a `config/rover.toml` host name. Case-sensitive on purpose: the
    /// config file is the one source of truth for spelling, and silently
    /// accepting `"RPi"` would just move a typo further from where it starts.
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

/// A host name that matches none of [`PeerId::ALL`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownPeerName(pub String);

impl fmt::Display for UnknownPeerName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "unknown host name `{}`", self.0)
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
        assert_eq!(PeerId::parse("groundstation"), None);
        assert_eq!(PeerId::parse("RPi"), None);
        assert!("nonsense".parse::<PeerId>().is_err());
    }
}
