//! Publish/subscribe and the command protocol, over any [`rover_link::Link`].
//!
//! `docs/RUST_REWRITE_PLAN.md` §4.1: a stream is unicast fan-out from a
//! static routing table, newest-wins on the receiving end, with no
//! retransmit and no ordering guarantee. That is a deliberate simplification
//! over the DDS/mros2 system it replaces — there is no discovery, no QoS
//! profile to pick, and no queue that can grow unbounded while a subscriber
//! falls behind, because there is no queue at all: each message type has
//! exactly one slot, and a new arrival overwrites it.
//!
//! This crate is generic over [`Link`], not tied to UDP, so the base station
//! moving from Ethernet to a LoRa serial link (plan §6) is a different `Link`
//! impl passed to [`Bus::new`], not a rewrite of anything in here.

#![forbid(unsafe_code)]

pub mod command;
mod config;

pub use command::{CommandReceiver, CommandSender, DEFAULT_RETRANSMIT_INTERVAL};
pub use config::{BusConfig, ConfigError};

use rover_link::{Link, LinkError};
use rover_msgs::frame::{encode_frame, MAX_FRAME_LEN};
use rover_msgs::Wire;
use std::collections::HashMap;
use std::fmt;

/// The most recent frame seen for one message type: its sequence number and
/// still-encoded body. Kept encoded rather than as `Box<dyn Any>` so storing
/// it never needs to know the concrete type — only [`Bus::latest`] and
/// [`Bus::subscribe`], which do know `T`, ever decode it.
struct Slot {
    seq: u16,
    body: Vec<u8>,
}

/// One subscriber's decode-and-call step, boxed so `Bus` can hold callbacks
/// for arbitrarily many message types without becoming generic over all of
/// them at once.
type Callback = Box<dyn FnMut(&[u8])>;

/// Publish/subscribe over one [`Link`].
///
/// Owns a monotonic per-message-type sequence counter for publishing, a
/// newest-wins slot per message type seen on `recv`, and any callbacks
/// registered with [`Bus::subscribe`]. There is one `Bus` per process, wired
/// to that process's `Link`.
pub struct Bus<L: Link> {
    link: L,
    config: BusConfig,
    send_seq: HashMap<u16, u16>,
    latest: HashMap<u16, Slot>,
    callbacks: HashMap<u16, Vec<Callback>>,
    // Count, not propagate: see `publish`'s doc comment on the mirror send.
    // `u64` so a field run with the mirror pointed at a laptop that's been
    // switched off for the whole run still can't wrap this around.
    mirror_failures: u64,
}

impl<L: Link> Bus<L> {
    pub fn new(link: L, config: BusConfig) -> Self {
        Self {
            link,
            config,
            send_seq: HashMap::new(),
            latest: HashMap::new(),
            callbacks: HashMap::new(),
            mirror_failures: 0,
        }
    }

    /// Encode `msg` and send it to every host `config/rover.toml` routes its
    /// type to, plus the debug firehose mirror if `[debug] mirror` is set.
    ///
    /// A per-`TYPE_ID` sequence counter is what lets a receiver — or
    /// `rover-tap --hz` — see packet loss as gaps, per plan §4.1. Fan-out is
    /// best-effort per destination: a send failure to one host does not stop
    /// the others from receiving it, since an unreachable peer is exactly the
    /// kind of thing this loss-tolerant protocol is meant to shrug off.
    ///
    /// **The returned `Err`, if any, names only the first destination that
    /// failed — it is not a complete report.** Every configured destination
    /// is still attempted regardless of earlier failures; a caller that needs
    /// to know about every failed destination, not just that at least one
    /// did, cannot get that from this return value and would need per-`Link`
    /// instrumentation instead.
    ///
    /// The mirror send (see [`BusConfig::mirror`]) is different in kind, not
    /// just another destination: it is a debugging convenience with a
    /// deliberately absent operator on the other end most of the time, so its
    /// failure is expected, routine, and must never surface as this call
    /// failing — a laptop that has been closed or walked out of range cannot
    /// be allowed to affect the control path. Its failures are only counted;
    /// see [`Bus::mirror_failures`].
    pub fn publish<T: Wire>(&mut self, msg: &T) -> Result<(), LinkError> {
        let seq_slot = self.send_seq.entry(T::TYPE_ID).or_insert(0);
        let seq = *seq_slot;
        *seq_slot = seq_slot.wrapping_add(1);

        let mut buf = [0u8; MAX_FRAME_LEN];
        let n = encode_frame(msg, seq, &mut buf);

        let mut first_err = None;
        for &dest in self.config.route_for::<T>() {
            if let Err(e) = self.link.send(dest, &buf[..n]) {
                first_err.get_or_insert(e);
            }
        }

        if let Some(mirror_addr) = self.config.mirror() {
            if self.link.send_to_addr(mirror_addr, &buf[..n]).is_err() {
                self.mirror_failures = self.mirror_failures.wrapping_add(1);
            }
        }

        match first_err {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    /// How many mirror sends have failed since this `Bus` was created —
    /// including every publish while `[debug] mirror` points at an address
    /// nothing is listening on, which is the expected state outside an
    /// active debugging session. Purely a diagnostic; nothing in this crate
    /// acts on it.
    pub fn mirror_failures(&self) -> u64 {
        self.mirror_failures
    }

    /// Drain every frame currently waiting on the link, updating the
    /// newest-wins slot for its type and running any callback registered for
    /// it. Call once per control-loop tick; never blocks, because
    /// [`Link::recv`] never blocks.
    pub fn poll(&mut self) {
        while let Some((_peer, frame)) = self.link.recv() {
            let type_id = frame.header.type_id;
            if let Some(cbs) = self.callbacks.get_mut(&type_id) {
                for cb in cbs.iter_mut() {
                    cb(frame.body);
                }
            }
            self.latest.insert(
                type_id,
                Slot {
                    seq: frame.header.seq,
                    body: frame.body.to_vec(),
                },
            );
        }
    }

    /// The most recently received `T`, decoded, or `None` if nothing of that
    /// type has arrived yet (or the last one failed to decode).
    ///
    /// This is the "give me the latest `ImuSample`" side of the API: fine for
    /// a control loop that wants this tick's best estimate and does not care
    /// whether it missed intermediate samples, which is exactly the EKF's
    /// contract (plan §2.3) — it predicts at a fixed rate and corrects
    /// whenever a measurement shows up.
    pub fn latest<T: Wire>(&self) -> Option<T> {
        self.latest
            .get(&T::TYPE_ID)
            .and_then(|slot| T::decode(&slot.body).ok())
    }

    /// The `seq` carried by the most recently received `T`, if any — the
    /// building block `rover-tap --hz` uses to turn gaps into a loss count.
    pub fn latest_seq<T: Wire>(&self) -> Option<u16> {
        self.latest.get(&T::TYPE_ID).map(|slot| slot.seq)
    }

    /// Run `f` on every `T` received from now on, in the order `poll`
    /// receives them.
    ///
    /// This is the "call me on every `LaneMeasurement`" side of the API, for
    /// a caller that cannot afford to miss one even under load — logging, or
    /// applying a [`command::CommandReceiver`] to every `CommandFrame`. There
    /// is still no queue: `f` runs synchronously inside `poll`, once per
    /// frame that arrives before the next `poll` call, so a slow callback
    /// slows the loop that calls `poll` rather than piling up work.
    pub fn subscribe<T: Wire>(&mut self, mut f: impl FnMut(T) + 'static) {
        self.callbacks
            .entry(T::TYPE_ID)
            .or_default()
            .push(Box::new(move |body: &[u8]| {
                if let Ok(msg) = T::decode(body) {
                    f(msg);
                }
            }));
    }

    /// The config this bus was built with, for callers that need routing or
    /// address information directly (e.g. `rover-tap` picking which host to
    /// listen as).
    pub fn config(&self) -> &BusConfig {
        &self.config
    }

    /// The underlying link, e.g. for `local_addr()` in a test.
    pub fn link(&self) -> &L {
        &self.link
    }
}

impl<L: Link> fmt::Debug for Bus<L> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Bus")
            .field("known_types_seen", &self.latest.len())
            .field("subscribed_types", &self.callbacks.len())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rover_link::{PeerId, UdpLink};
    use rover_msgs::ImuSample;
    use std::net::UdpSocket;
    use std::time::Duration;

    fn config_with_mirror(mirror: Option<&str>) -> BusConfig {
        let mirror_line = mirror
            .map(|a| format!("[debug]\nmirror = \"{a}\"\n"))
            .unwrap_or_default();
        let text = format!(
            r#"
            [services]
            control = "127.0.0.1:7001"

            [routes]
            ImuSample = ["control"]

            {mirror_line}
            "#
        );
        BusConfig::parse(&text).unwrap()
    }

    fn imu() -> ImuSample {
        ImuSample {
            accel_mps2: [1.0, 2.0, 3.0],
            gyro_radps: [0.0, 0.0, 0.0],
            t_us: 1,
        }
    }

    fn recv_with_retries(sock: &UdpSocket) -> Option<Vec<u8>> {
        let mut buf = [0u8; 256];
        for _ in 0..200 {
            match sock.recv(&mut buf) {
                Ok(n) => return Some(buf[..n].to_vec()),
                Err(_) => std::thread::sleep(Duration::from_millis(5)),
            }
        }
        None
    }

    #[test]
    fn publish_reaches_both_the_route_and_the_mirror() {
        // A bare socket standing in for rpi (the normal route destination):
        // recv() on a real socket, not another Bus, so this test only
        // exercises Bus::publish's fan-out, not a second Bus's receive path.
        let route_sock = UdpSocket::bind("127.0.0.1:0").unwrap();
        route_sock.set_nonblocking(false).unwrap();
        route_sock
            .set_read_timeout(Some(Duration::from_millis(500)))
            .unwrap();
        let mirror_sock = UdpSocket::bind("127.0.0.1:0").unwrap();
        mirror_sock
            .set_read_timeout(Some(Duration::from_millis(500)))
            .unwrap();

        let config = config_with_mirror(Some(&mirror_sock.local_addr().unwrap().to_string()));
        let mut peers = HashMap::new();
        peers.insert(PeerId::Control, route_sock.local_addr().unwrap());
        let link = UdpLink::bind(PeerId::Base, "127.0.0.1:0", peers).unwrap();
        let mut bus = Bus::new(link, config);

        bus.publish(&imu()).unwrap();

        assert!(
            recv_with_retries(&route_sock).is_some(),
            "normal route never received the frame"
        );
        assert!(
            recv_with_retries(&mirror_sock).is_some(),
            "mirror never received the frame"
        );
        assert_eq!(bus.mirror_failures(), 0);
    }

    #[test]
    fn publish_succeeds_even_when_the_mirror_is_unreachable() {
        let route_sock = UdpSocket::bind("127.0.0.1:0").unwrap();
        route_sock
            .set_read_timeout(Some(Duration::from_millis(500)))
            .unwrap();

        // Bind and immediately drop a socket to get a port nothing is
        // listening on, standing in for "the debugging laptop isn't here".
        let dead_addr = {
            let s = UdpSocket::bind("127.0.0.1:0").unwrap();
            s.local_addr().unwrap()
        };

        let config = config_with_mirror(Some(&dead_addr.to_string()));
        let mut peers = HashMap::new();
        peers.insert(PeerId::Control, route_sock.local_addr().unwrap());
        let link = UdpLink::bind(PeerId::Base, "127.0.0.1:0", peers).unwrap();
        let mut bus = Bus::new(link, config);

        // UDP has no delivery confirmation, so a send to a dead local port
        // does not actually surface as a `send_to_addr` error on most
        // platforms — the point of this test is the *contract*: whatever
        // happens on the wire, `publish`'s `Ok` here is not contingent on
        // the mirror at all. Combined with `Link::send_to_addr`'s own
        // default-`Unsupported` test in rover-link, this is what makes the
        // "must never affect the return value" requirement concrete rather
        // than just documented.
        assert!(bus.publish(&imu()).is_ok());
        assert!(
            recv_with_retries(&route_sock).is_some(),
            "the real route must still get the frame even though the mirror is dead"
        );
    }

    #[test]
    fn publish_is_unaffected_by_a_link_that_cannot_mirror_at_all() {
        // A `Link` whose `send_to_addr` uses the trait default
        // (`LinkError::Unsupported`) — standing in for a future
        // `LoraSerialLink`. This is the case the trait default exists for:
        // `publish` must swallow it exactly like a network failure.
        struct NoMirrorLink {
            sent: Vec<u8>,
        }
        impl Link for NoMirrorLink {
            fn send(&mut self, _dest: PeerId, frame: &[u8]) -> Result<(), LinkError> {
                self.sent = frame.to_vec();
                Ok(())
            }
            fn recv(&mut self) -> Option<(PeerId, rover_msgs::Frame<'_>)> {
                None
            }
            fn mtu(&self) -> usize {
                256
            }
            fn class(&self) -> rover_link::LinkClass {
                rover_link::LinkClass::Lan
            }
            // send_to_addr intentionally not overridden.
        }

        let config = config_with_mirror(Some("127.0.0.1:9"));
        let mut bus = Bus::new(NoMirrorLink { sent: Vec::new() }, config);

        assert!(bus.publish(&imu()).is_ok());
        assert_eq!(bus.mirror_failures(), 1);
        assert!(!bus.link().sent.is_empty(), "the real route still ran");
    }

    #[test]
    fn no_mirror_configured_means_no_mirror_send_is_attempted() {
        struct CountingLink {
            mirror_calls: u32,
        }
        impl Link for CountingLink {
            fn send(&mut self, _dest: PeerId, _frame: &[u8]) -> Result<(), LinkError> {
                Ok(())
            }
            fn recv(&mut self) -> Option<(PeerId, rover_msgs::Frame<'_>)> {
                None
            }
            fn mtu(&self) -> usize {
                256
            }
            fn class(&self) -> rover_link::LinkClass {
                rover_link::LinkClass::Lan
            }
            fn send_to_addr(
                &mut self,
                _addr: std::net::SocketAddr,
                _frame: &[u8],
            ) -> Result<(), LinkError> {
                self.mirror_calls += 1;
                Ok(())
            }
        }

        let config = config_with_mirror(None);
        let mut bus = Bus::new(CountingLink { mirror_calls: 0 }, config);
        bus.publish(&imu()).unwrap();
        assert_eq!(bus.link().mirror_calls, 0);
    }
}
