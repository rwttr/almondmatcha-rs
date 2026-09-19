//! Link layer: how a frame gets from one host to another.
//!
//! `rover-bus` is written against the [`Link`] trait, not against UDP
//! directly, because the base station will not always be on the LAN.
//! `docs/RUST_REWRITE_PLAN.md` §6 sets the end state: Ethernet today,
//! `LoraSerialLink` over a USB-CDC ESP32 radio once the base leaves range.
//! That implementation is deliberately **not** written in this crate — see
//! [`LinkClass`] for what it will need from this trait when it arrives — but
//! the trait is shaped so adding it is a new `impl`, not a redesign.
//!
//! # Why not TCP
//!
//! The command protocol (`rover-bus`, plan §4.2) depends on every link being
//! a bare, connectionless datagram pipe: send whatever you have, whenever you
//! have it, and let the receiver decide what is current. A LoRa link is
//! half-duplex and can be one-way in practice, which TCP's connection state
//! cannot survive. `Link` has no notion of a connection anywhere in it, on
//! purpose — see `docs/RUST_REWRITE_PLAN.md` §4.2 and §6.

#![forbid(unsafe_code)]

mod peer;
mod udp;

pub use peer::{PeerId, UnknownPeerName};
pub use udp::{UdpLink, LAN_MTU};

use rover_msgs::Frame;
use std::fmt;
use std::io;

/// Largest single frame this crate's links will carry.
///
/// Matches [`rover_msgs::frame::MAX_FRAME_LEN`] rather than restating a
/// number: every `Link` impl moves whole frames and never fragments, so this
/// is the one receive-buffer size any implementation needs.
pub const MAX_FRAME_LEN: usize = rover_msgs::frame::MAX_FRAME_LEN;

/// A datagram transport between rover hosts.
///
/// One instance owns one local endpoint (a bound socket, eventually a serial
/// port to a radio) and a fixed table of how to reach its peers. There is no
/// connection setup and no per-peer session state beyond an address, matching
/// the plan's "datagrams over a pluggable link layer" decision (§0).
pub trait Link {
    /// Send one already-encoded frame to `dest`.
    ///
    /// Best-effort: a dropped or reordered frame is normal operation for this
    /// bus (plan §4.1), not a failure this call needs to recover from.
    fn send(&mut self, dest: PeerId, frame: &[u8]) -> Result<(), LinkError>;

    /// Return the next received frame and who sent it, if one is waiting.
    ///
    /// Never blocks: a control loop calls this once per tick and moves on
    /// whether or not anything was there. `None` means "nothing waiting right
    /// now", not an error.
    ///
    /// The returned [`Frame`] borrows an internal buffer owned by the link,
    /// so a caller decodes it (or copies out the fields it wants) before
    /// calling `recv` again — this is what lets an implementation avoid
    /// allocating for every packet.
    fn recv(&mut self) -> Option<(PeerId, Frame<'_>)>;

    /// Largest frame this link can carry in one piece.
    fn mtu(&self) -> usize;

    /// What kind of link this is, so a caller like `rover-telemetry` can pick
    /// a wire format sized for it — `Telemetry` on a LAN, the constrained
    /// `TelemetryLite` on a narrowband one (plan §6.3). Nothing consumes this
    /// yet; `TelemetryLite` is not implemented until `LoraSerialLink` exists
    /// (see `rover_msgs::types` for why), but the trait carries the
    /// information now so that decision does not require touching every
    /// existing `Link` impl later.
    fn class(&self) -> LinkClass;
}

/// What kind of link this is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LinkClass {
    /// Effectively unlimited bandwidth and duty cycle: Ethernet, today's only
    /// transport.
    Lan,
    /// A narrowband or duty-cycle-limited radio link (LoRa). `bps` is the raw
    /// link rate; `duty_pct` is the fraction of time it may legally transmit,
    /// which in most sub-GHz ISM bands is a regulatory ceiling, not a design
    /// choice — see plan §6.3 for the numbers this rover's LoRa radios would
    /// actually see.
    Constrained { bps: u32, duty_pct: f32 },
}

/// Something went wrong moving a frame.
#[derive(Debug)]
pub enum LinkError {
    /// The underlying transport failed (socket error, serial error, ...).
    Io(io::Error),
    /// `send` was asked for a peer this link has no address for.
    UnknownPeer(PeerId),
    /// A frame larger than [`Link::mtu`] was handed to `send`.
    FrameTooLarge { len: usize, mtu: usize },
}

impl fmt::Display for LinkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LinkError::Io(e) => write!(f, "link I/O error: {e}"),
            LinkError::UnknownPeer(p) => write!(f, "no address configured for peer `{p}`"),
            LinkError::FrameTooLarge { len, mtu } => {
                write!(f, "frame of {len} bytes exceeds link MTU of {mtu}")
            }
        }
    }
}

impl std::error::Error for LinkError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            LinkError::Io(e) => Some(e),
            _ => None,
        }
    }
}
