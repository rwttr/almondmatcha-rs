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
}

impl<L: Link> Bus<L> {
    pub fn new(link: L, config: BusConfig) -> Self {
        Self {
            link,
            config,
            send_seq: HashMap::new(),
            latest: HashMap::new(),
            callbacks: HashMap::new(),
        }
    }

    /// Encode `msg` and send it to every host `config/rover.toml` routes its
    /// type to.
    ///
    /// A per-`TYPE_ID` sequence counter is what lets a receiver — or
    /// `rover-tap --hz` — see packet loss as gaps, per plan §4.1. Fan-out is
    /// best-effort per destination: a send failure to one host does not stop
    /// the others from receiving it, since an unreachable peer is exactly the
    /// kind of thing this loss-tolerant protocol is meant to shrug off. If
    /// any destination failed, this returns that failure after every send
    /// has been attempted.
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
        match first_err {
            Some(e) => Err(e),
            None => Ok(()),
        }
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
