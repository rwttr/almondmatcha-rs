//! `UdpLink`: the [`Link`] impl for the LAN, today's only transport.
//!
//! Non-blocking by construction — the trait's contract is that `recv` never
//! blocks a control loop, and a socket left in blocking mode cannot honour
//! that. There is no background thread and no internal queue: `recv` drains
//! exactly what the kernel already has buffered, once, and returns.

use crate::{Link, LinkClass, LinkError, PeerId, MAX_FRAME_LEN};
use rover_msgs::Frame;
use std::collections::HashMap;
use std::io;
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};

/// Ethernet MTU minus IPv4 and UDP headers (1500 − 20 − 8).
///
/// Every message in `rover-msgs` is well under this — `Telemetry`, the
/// largest, is under 200 bytes — so this only matters as a sanity check:
/// `send` rejects an oversized frame with a typed error instead of letting
/// the kernel silently fragment or drop it.
pub const LAN_MTU: usize = 1472;

/// A UDP socket bound for one host, with a fixed table of where to find the
/// others.
pub struct UdpLink {
    socket: UdpSocket,
    self_id: PeerId,
    peers: HashMap<PeerId, SocketAddr>,
    // Owned receive buffer, reused across calls. This is what lets `recv`
    // hand back a `Frame<'_>` borrowing the decoded bytes without allocating
    // per packet — see the `Link` trait docs on the borrow.
    buf: [u8; MAX_FRAME_LEN],
}

impl UdpLink {
    /// Bind the local endpoint for `self_id` and register where every other
    /// peer can be reached.
    ///
    /// `bind_addr` is normally `self_id`'s own address and port from
    /// `config/rover.toml`. Tests bind to `127.0.0.1:0` (an ephemeral port,
    /// chosen by the OS) and read the real address back with
    /// [`UdpLink::local_addr`], so a test run can never collide with a real
    /// deployment or another test running at the same time.
    pub fn bind(
        self_id: PeerId,
        bind_addr: impl ToSocketAddrs,
        peers: HashMap<PeerId, SocketAddr>,
    ) -> Result<Self, LinkError> {
        let socket = UdpSocket::bind(bind_addr).map_err(LinkError::Io)?;
        socket.set_nonblocking(true).map_err(LinkError::Io)?;
        Ok(Self {
            socket,
            self_id,
            peers,
            buf: [0u8; MAX_FRAME_LEN],
        })
    }

    /// The address the OS actually bound this link to. Needed by anything
    /// that bound to an ephemeral port and must tell its peer what it turned
    /// out to be.
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    /// Which host this link speaks for.
    pub fn self_id(&self) -> PeerId {
        self.self_id
    }

    /// Add or replace a peer's address.
    ///
    /// Kept separate from [`UdpLink::bind`] rather than required up front, so
    /// two links in a test can be bound first — to learn their ephemeral
    /// ports — and then pointed at each other.
    pub fn set_peer(&mut self, peer: PeerId, addr: SocketAddr) {
        self.peers.insert(peer, addr);
    }

    /// Look up which configured peer a datagram's source address belongs to.
    ///
    /// A linear scan over five entries; a `HashMap<SocketAddr, PeerId>` would
    /// be the "correct" reverse index for a bigger table, but at this size it
    /// would just be a second collection to keep in sync with `peers`.
    fn peer_of(&self, addr: SocketAddr) -> Option<PeerId> {
        self.peers
            .iter()
            .find(|&(_, &a)| a == addr)
            .map(|(&p, _)| p)
    }

    /// Shared by [`Link::send`] and [`Link::send_to_addr`]: the MTU check and
    /// the actual socket write are identical either way, only how the
    /// destination address was obtained differs.
    fn send_raw(&mut self, addr: SocketAddr, frame: &[u8]) -> Result<(), LinkError> {
        if frame.len() > self.mtu() {
            return Err(LinkError::FrameTooLarge {
                len: frame.len(),
                mtu: self.mtu(),
            });
        }
        self.socket.send_to(frame, addr).map_err(LinkError::Io)?;
        Ok(())
    }

    /// Read one datagram from a recognised peer into `self.buf`, retrying
    /// past anything from an unrecognised sender. Returns the sender and how
    /// many bytes of `self.buf` it wrote — deliberately not a `Frame`, so
    /// this can be an ordinary `&mut self` call with no borrowed return; see
    /// [`UdpLink::recv`] for why that split matters.
    fn recv_one(&mut self) -> Option<(PeerId, usize)> {
        loop {
            let (n, from) = match self.socket.recv_from(&mut self.buf) {
                Ok(v) => v,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => return None,
                // Any other I/O error on a connectionless UDP socket (e.g. an
                // ICMP port-unreachable bounced back from a peer that isn't
                // listening yet) is transient and indistinguishable from
                // "nothing arrived" as far as a caller polling every tick is
                // concerned, so it costs one skipped poll rather than a
                // panic or a variant every caller has to plumb through.
                Err(_) => return None,
            };
            // Datagrams from anyone not in the peer table are dropped rather
            // than surfaced: on a shared LAN segment stray traffic is normal,
            // and this bus has no notion of an unauthenticated sender.
            if let Some(peer) = self.peer_of(from) {
                return Some((peer, n));
            }
        }
    }
}

impl Link for UdpLink {
    fn send(&mut self, dest: PeerId, frame: &[u8]) -> Result<(), LinkError> {
        let addr = *self.peers.get(&dest).ok_or(LinkError::UnknownPeer(dest))?;
        self.send_raw(addr, frame)
    }

    fn recv(&mut self) -> Option<(PeerId, Frame<'_>)> {
        // Split into a phase that mutates `self.buf` (`recv_one`, an ordinary
        // `&mut self` call with no borrowed return) and a phase that only
        // reads it (`Frame::parse`, below). Folding both into one loop that
        // also returns data borrowed from `self.buf` defeats NLL: the
        // returned `Frame`'s lifetime is tied to this whole `&mut self`, so
        // the borrow checker cannot tell that each iteration's mutable
        // reborrow for `recv_from` has already ended by the time a later
        // iteration reads the buffer back. Two steps sidesteps the question
        // instead of fighting it.
        let (peer, n) = self.recv_one()?;
        Frame::parse(&self.buf[..n]).ok().map(|frame| (peer, frame))
    }

    fn mtu(&self) -> usize {
        LAN_MTU
    }

    fn class(&self) -> LinkClass {
        LinkClass::Lan
    }

    fn send_to_addr(&mut self, addr: SocketAddr, frame: &[u8]) -> Result<(), LinkError> {
        self.send_raw(addr, frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rover_msgs::frame::encode_frame;
    use rover_msgs::{ImuSample, Wire};

    fn linked_pair() -> (UdpLink, UdpLink) {
        let mut a = UdpLink::bind(PeerId::Rpi, "127.0.0.1:0", HashMap::new()).unwrap();
        let mut b = UdpLink::bind(PeerId::Chassis, "127.0.0.1:0", HashMap::new()).unwrap();
        let a_addr = a.local_addr().unwrap();
        let b_addr = b.local_addr().unwrap();
        a.set_peer(PeerId::Chassis, b_addr);
        b.set_peer(PeerId::Rpi, a_addr);
        (a, b)
    }

    #[test]
    fn send_recv_round_trips_a_frame() {
        let (mut a, mut b) = linked_pair();

        let msg = ImuSample {
            accel_mps2: [1.0, 2.0, 3.0],
            gyro_radps: [0.1, 0.2, 0.3],
            t_us: 42,
        };
        let mut buf = [0u8; rover_msgs::frame::MAX_FRAME_LEN];
        let n = encode_frame(&msg, 7, &mut buf);
        a.send(PeerId::Chassis, &buf[..n]).unwrap();

        // The datagram is delivered asynchronously by the OS; give it a
        // moment rather than asserting on the very first poll.
        let mut received = None;
        for _ in 0..200 {
            if let Some((peer, frame)) = b.recv() {
                received = Some((
                    peer,
                    frame.header.seq,
                    ImuSample::decode(frame.body).unwrap(),
                ));
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        let (peer, seq, decoded) = received.expect("frame never arrived");
        assert_eq!(peer, PeerId::Rpi);
        assert_eq!(seq, 7);
        assert_eq!(decoded, msg);
    }

    #[test]
    fn recv_is_none_when_nothing_sent() {
        let (_a, mut b) = linked_pair();
        assert!(b.recv().is_none());
    }

    #[test]
    fn send_to_unknown_peer_is_a_typed_error() {
        let mut a = UdpLink::bind(PeerId::Rpi, "127.0.0.1:0", HashMap::new()).unwrap();
        let err = a.send(PeerId::Base, &[0u8; 4]).unwrap_err();
        assert!(matches!(err, LinkError::UnknownPeer(PeerId::Base)));
    }

    #[test]
    fn oversized_frame_is_rejected_before_the_socket_sees_it() {
        let (mut a, _b) = linked_pair();
        let big = vec![0u8; LAN_MTU + 1];
        let err = a.send(PeerId::Chassis, &big).unwrap_err();
        assert!(matches!(err, LinkError::FrameTooLarge { .. }));
    }

    #[test]
    fn send_to_addr_reaches_a_raw_address_outside_the_peer_table() {
        // The debug mirror's whole point: a destination `send` cannot reach
        // because it was never registered as a `PeerId`.
        let mut a = UdpLink::bind(PeerId::Rpi, "127.0.0.1:0", HashMap::new()).unwrap();
        let mut mirror = UdpLink::bind(PeerId::Base, "127.0.0.1:0", HashMap::new()).unwrap();
        let mirror_addr = mirror.local_addr().unwrap();
        // The mirror still has to recognise `a` as a sender to accept its
        // datagram — that part of the peer table isn't bypassed, only the
        // *destination* lookup on the sending side is.
        mirror.set_peer(PeerId::Rpi, a.local_addr().unwrap());

        let mut buf = [0u8; rover_msgs::frame::MAX_FRAME_LEN];
        let n = encode_frame(
            &ImuSample {
                accel_mps2: [0.0; 3],
                gyro_radps: [0.0; 3],
                t_us: 0,
            },
            0,
            &mut buf,
        );
        a.send_to_addr(mirror_addr, &buf[..n]).unwrap();

        let mut received = false;
        for _ in 0..200 {
            if mirror.recv().is_some() {
                received = true;
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert!(received, "frame sent via send_to_addr never arrived");
    }

    #[test]
    fn send_to_addr_oversized_frame_is_rejected_the_same_way_as_send() {
        let mut a = UdpLink::bind(PeerId::Rpi, "127.0.0.1:0", HashMap::new()).unwrap();
        let big = vec![0u8; LAN_MTU + 1];
        let err = a
            .send_to_addr("127.0.0.1:9".parse().unwrap(), &big)
            .unwrap_err();
        assert!(matches!(err, LinkError::FrameTooLarge { .. }));
    }
}
