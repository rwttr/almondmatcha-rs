//! The base↔rover command protocol: idempotent datagrams with an
//! end-to-end acknowledgement.
//!
//! `docs/RUST_REWRITE_PLAN.md` §4.2 replaces what used to be a TCP
//! request/response (`SetSpeedLimit`, mission goals) with something that
//! survives a link with no return path guaranteed on any given try: the base
//! keeps resending the same [`CommandFrame`] at about 1 Hz until it sees it
//! acknowledged, and each rover consumer applies a frame that is new by
//! `(session, cmd_seq)` and ignores the rest. Neither side keeps a
//! connection; either side can restart without the other noticing anything
//! beyond a gap in retransmits — a restarted base picks a new random
//! `session` (see [`CommandFrame`]), which is what makes its low sequence
//! numbers count as new instead of stale.
//!
//! The acknowledgement path is deliberately NOT the receiver that applied the
//! command's own echo: a [`CommandFrame`] fans out as three unicast datagrams
//! (control, navigation, telemetry), so each consumer that acts on a command
//! publishes a [`rover_msgs::CommandAck`] of what *it* applied, and
//! `rover-telemetry` folds both into `Telemetry::{last_cmd_session,
//! last_cmd_seq}` only when both are fresh and agree. The base therefore stops
//! retransmitting only once every consumer has the command.
//!
//! [`CommandSender`] and [`CommandReceiver`] below are deliberately not
//! wired to a [`crate::Bus`] or a [`rover_link::Link`] directly. A sender only
//! needs to be told "what time is it" and "here is what got acknowledged"; a
//! receiver only needs "here is a frame that arrived". That keeps both
//! trivially unit-testable without a socket, and leaves the actual
//! publish/subscribe call at the boundary the caller already controls (see
//! the integration test in `tests/`).

use rover_msgs::{seq_is_newer, Command, CommandAck, CommandFrame};
use std::collections::hash_map::RandomState;
use std::collections::VecDeque;
use std::hash::{BuildHasher, Hasher};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// Default retransmit interval: "~1 Hz" per plan §4.2.
pub const DEFAULT_RETRANSMIT_INTERVAL: Duration = Duration::from_secs(1);

/// A fresh random, non-zero session id for one ground-station process.
///
/// No `rand` crate: std's `RandomState` is seeded from the OS RNG per
/// instance, and the wall-clock nanoseconds plus process id are mixed in as a
/// second, independent source so a platform with a weak `RandomState` still
/// differs between two quick restarts. Zero is reserved ("none") and mapped
/// to 1. Collision with the previous run's session is 1 in 65535 — a missed
/// restart then degrades to the old stale-seq behaviour for that one run,
/// which the operator sees as a command that never acks.
pub fn random_session() -> u16 {
    let mut h = RandomState::new().build_hasher();
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    h.write_u128(nanos);
    h.write_u32(std::process::id());
    let v = h.finish();
    // Fold all 64 bits so no hashed byte is discarded.
    let folded = (v ^ (v >> 16) ^ (v >> 32) ^ (v >> 48)) as u16;
    if folded == 0 {
        1
    } else {
        folded
    }
}

/// A command [`CommandSender::set`] dropped without it ever being acked, so
/// the caller can tell the operator.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Discarded {
    /// `Some(cmd_seq)` if it had already been sent (it was the in-flight
    /// frame and the rover may even have applied it), `None` if it was still
    /// queued and never left the station.
    pub cmd_seq: Option<u16>,
    pub body: Command,
}

/// Base-station side: a FIFO of commands with one frame in flight.
///
/// One command is retransmitted at a time until acknowledged, then the next
/// queued one is sent; a command issued while another is unacked queues
/// behind it instead of silently replacing it (it used to — so "estop, then
/// anything before the ack" lost the EStop). Order is preserved: a
/// `SetMissionGoal` followed by `CancelMission` reaches the rover in that
/// order.
///
/// **EStop pre-empts.** Issuing [`Command::EStop`] discards every queued
/// command and replaces the in-flight one, and nothing but another EStop's
/// arrival may displace an in-flight EStop — anything issued while one is in
/// flight queues behind it. Every discarded command is returned from
/// [`CommandSender::set`] so the caller can warn the operator.
///
/// `cmd_seq` is assigned when a command goes in flight, not when it is
/// issued, so the sequence numbers on the wire are dense. It starts at 0 and
/// the first command is 1; unlike before, 0 is not special (the receiver's
/// "nothing applied" state is an `Option`, and the ack's is `session == 0`).
pub struct CommandSender {
    session: u16,
    /// `cmd_seq` of the most recent frame put in flight (0 before any).
    cmd_seq: u16,
    in_flight: Option<CommandFrame>,
    queue: VecDeque<Command>,
    interval: Duration,
    last_sent: Option<Instant>,
}

impl CommandSender {
    /// A sender with a fresh [`random_session`].
    pub fn new(interval: Duration) -> Self {
        Self::with_session(interval, random_session())
    }

    /// A sender with a chosen session, for tests.
    ///
    /// # Panics
    /// If `session == 0`, which is reserved for "none" on the wire.
    pub fn with_session(interval: Duration, session: u16) -> Self {
        assert_ne!(session, 0, "session 0 is reserved for \"none\"");
        Self {
            session,
            cmd_seq: 0,
            in_flight: None,
            queue: VecDeque::new(),
            interval,
            last_sent: None,
        }
    }

    /// Issue `body`. Returns the commands this dropped (always empty unless
    /// `body` is an EStop that pre-empted something) — log each one.
    ///
    /// Always results in a fresh `cmd_seq` eventually, even if `body` equals
    /// the one in flight, so a repeated "cancel" click still sends.
    pub fn set(&mut self, body: Command) -> Vec<Discarded> {
        let mut discarded = Vec::new();
        if body == Command::EStop {
            // Queued commands first, in issue order, then the in-flight one.
            discarded.extend(self.queue.drain(..).map(|body| Discarded {
                cmd_seq: None,
                body,
            }));
            match self.in_flight {
                // An EStop already out there is exactly what the operator is
                // asking for again; a second one would only delay the ack.
                Some(f) if f.body == Command::EStop => return discarded,
                Some(f) => discarded.push(Discarded {
                    cmd_seq: Some(f.cmd_seq),
                    body: f.body,
                }),
                None => {}
            }
            self.in_flight = None;
            self.queue.push_back(body);
            self.promote();
            return discarded;
        }
        self.queue.push_back(body);
        if self.in_flight.is_none() {
            self.promote();
        }
        discarded
    }

    /// Move the head of the queue into flight with a fresh `cmd_seq`.
    fn promote(&mut self) {
        let Some(body) = self.queue.pop_front() else {
            return;
        };
        self.cmd_seq = self.cmd_seq.wrapping_add(1);
        self.in_flight = Some(CommandFrame {
            session: self.session,
            cmd_seq: self.cmd_seq,
            body,
        });
        self.last_sent = None; // send on the very next poll, don't wait out the interval
    }

    /// Call once per base-station tick. Returns the frame to put on the wire
    /// if one is due — either never sent, or the retransmit interval has
    /// elapsed — and `None` once everything has been acknowledged.
    pub fn poll(&mut self, now: Instant) -> Option<CommandFrame> {
        let frame = self.in_flight?;
        let due = match self.last_sent {
            None => true,
            Some(t) => now.duration_since(t) >= self.interval,
        };
        due.then(|| {
            self.last_sent = Some(now);
            frame
        })
    }

    /// Feed the `(last_cmd_session, last_cmd_seq)` pair from a received
    /// `Telemetry`. The in-flight command is acknowledged when the session is
    /// ours and the seq is wrap-aware newer-than-or-equal to its own (the
    /// rover reports the *older* of its consumers' seqs, so a later command's
    /// ack also covers this one); a leftover echo from another session, or
    /// from before this command, never matches. On ack the next queued
    /// command goes in flight and is sent on the next poll.
    pub fn on_telemetry(&mut self, last_cmd_session: u16, last_cmd_seq: u16) {
        let Some(f) = self.in_flight else {
            return;
        };
        if last_cmd_session == self.session
            && (last_cmd_seq == f.cmd_seq || seq_is_newer(last_cmd_seq, f.cmd_seq))
        {
            self.in_flight = None;
            self.promote();
        }
    }

    /// True when nothing is in flight — i.e. the last command sent has been
    /// acknowledged (and nothing is queued, since a queued command is
    /// promoted the moment the in-flight one is acked).
    pub fn is_acked(&self) -> bool {
        self.in_flight.is_none()
    }

    /// The frame currently being retransmitted, as `(cmd_seq, body)`.
    pub fn in_flight(&self) -> Option<(u16, Command)> {
        self.in_flight.map(|f| (f.cmd_seq, f.body))
    }

    /// Commands waiting behind the in-flight one.
    pub fn queue_len(&self) -> usize {
        self.queue.len()
    }

    /// This process's session id, stamped on every frame.
    pub fn session(&self) -> u16 {
        self.session
    }

    /// The `cmd_seq` of the most recent frame put in flight, for logging
    /// (0 if none yet).
    pub fn cmd_seq(&self) -> u16 {
        self.cmd_seq
    }
}

impl Default for CommandSender {
    fn default() -> Self {
        Self::new(DEFAULT_RETRANSMIT_INTERVAL)
    }
}

/// Rover side: applies a frame that is new and ignores the rest.
///
/// A frame is new when nothing has been applied yet, OR its `session` differs
/// from the last applied one (a restarted base is a new ordering — see
/// [`CommandFrame`]), OR it is in the same session with a wrap-aware newer
/// `cmd_seq`. Frames with `session == 0` are ignored: 0 is reserved for
/// "none" and no sender emits it, so one is corruption or a pre-`session`
/// build. "Nothing applied" is an `Option`, so no `cmd_seq` value is special.
///
/// Known limit: a late duplicate of an *old* session's frame arriving after a
/// new session has applied would count as a session change and be applied.
/// On a LAN with one ground station that does not happen; two stations running
/// at once would fight, which is an operator error this protocol does not try
/// to arbitrate.
#[derive(Debug, Default)]
pub struct CommandReceiver {
    last_applied: Option<(u16, u16)>,
}

impl CommandReceiver {
    pub fn new() -> Self {
        Self::default()
    }

    /// Apply `frame` if new (see the type docs). Returns the command body
    /// when applied, `None` for a stale, duplicate or session-0 frame.
    pub fn apply(&mut self, frame: CommandFrame) -> Option<Command> {
        if frame.session == 0 {
            return None;
        }
        let new = match self.last_applied {
            None => true,
            Some((session, _)) if session != frame.session => true,
            Some((_, seq)) => frame.is_newer_than(seq),
        };
        if new {
            self.last_applied = Some((frame.session, frame.cmd_seq));
            Some(frame.body)
        } else {
            None
        }
    }

    /// `(session, cmd_seq)` of the last applied frame, `None` before any.
    pub fn last_applied(&self) -> Option<(u16, u16)> {
        self.last_applied
    }

    /// What to publish as this consumer's [`CommandAck`]: `session == 0`
    /// when nothing has been applied.
    pub fn ack(&self) -> CommandAck {
        match self.last_applied {
            Some((session, cmd_seq)) => CommandAck { session, cmd_seq },
            None => CommandAck::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rover_msgs::MissionGoal;

    const S: u16 = 0x1234;

    fn frame(session: u16, cmd_seq: u16, body: Command) -> CommandFrame {
        CommandFrame {
            session,
            cmd_seq,
            body,
        }
    }

    fn sender() -> CommandSender {
        CommandSender::with_session(Duration::from_millis(100), S)
    }

    #[test]
    fn receiver_applies_the_first_command() {
        let mut rx = CommandReceiver::new();
        assert_eq!(rx.ack(), CommandAck::default());
        assert_eq!(rx.apply(frame(S, 1, Command::EStop)), Some(Command::EStop));
        assert_eq!(rx.last_applied(), Some((S, 1)));
        assert_eq!(
            rx.ack(),
            CommandAck {
                session: S,
                cmd_seq: 1
            }
        );
    }

    #[test]
    fn receiver_applies_cmd_seq_zero_as_the_first_command() {
        // 0 is no longer a sentinel: "nothing applied" is an Option.
        let mut rx = CommandReceiver::new();
        assert_eq!(rx.apply(frame(S, 0, Command::Nop)), Some(Command::Nop));
    }

    #[test]
    fn receiver_ignores_exact_duplicates() {
        let mut rx = CommandReceiver::new();
        let f = frame(S, 5, Command::SetSpeedLimit(40));
        assert_eq!(rx.apply(f), Some(Command::SetSpeedLimit(40)));
        // The base is still retransmitting the same frame at 1 Hz; every
        // repeat after the first must be a no-op.
        assert_eq!(rx.apply(f), None);
        assert_eq!(rx.apply(f), None);
        assert_eq!(rx.last_applied(), Some((S, 5)));
    }

    #[test]
    fn receiver_ignores_stale_out_of_order_frames() {
        let mut rx = CommandReceiver::new();
        rx.apply(frame(S, 10, Command::CancelMission));
        assert_eq!(rx.apply(frame(S, 7, Command::EStop)), None);
        assert_eq!(rx.last_applied(), Some((S, 10)));
    }

    /// Regression for bug A: a restarted ground station starts at cmd_seq 1
    /// while the receiver remembers L = 500. Before `session`, 1..=L were all
    /// "stale" — EStop included — and command L got a false ack.
    #[test]
    fn receiver_applies_low_seq_from_a_new_session() {
        let mut rx = CommandReceiver::new();
        rx.apply(frame(S, 500, Command::Nop));
        assert_eq!(
            rx.apply(frame(S, 1, Command::EStop)),
            None,
            "same session: stale"
        );
        assert_eq!(
            rx.apply(frame(S + 1, 1, Command::EStop)),
            Some(Command::EStop),
            "new session must apply even though 1 < 500"
        );
        assert_eq!(rx.last_applied(), Some((S + 1, 1)));
        // ...and ordering is now relative to the new session.
        assert_eq!(
            rx.apply(frame(S + 1, 2, Command::ClearEStop)),
            Some(Command::ClearEStop)
        );
    }

    #[test]
    fn receiver_ignores_session_zero() {
        let mut rx = CommandReceiver::new();
        assert_eq!(rx.apply(frame(0, 1, Command::EStop)), None);
        assert_eq!(rx.last_applied(), None);
    }

    #[test]
    fn receiver_handles_u16_wraparound() {
        let mut rx = CommandReceiver::new();

        // Climb in hops that each count as "newer"; a leap of more than half
        // the sequence space reads as stale.
        rx.apply(frame(S, 32_767, Command::Nop));
        rx.apply(frame(S, u16::MAX - 1, Command::Nop));
        assert_eq!(rx.last_applied(), Some((S, u16::MAX - 1)));

        // One past the max wraps to 0, still "newer" than MAX - 1.
        assert_eq!(
            rx.apply(frame(S, 0, Command::SetSpeedLimit(10))),
            Some(Command::SetSpeedLimit(10))
        );
        assert_eq!(
            rx.apply(frame(S, 1, Command::SetSpeedLimit(20))),
            Some(Command::SetSpeedLimit(20))
        );
        // Something from before the wrap arriving late is stale.
        assert_eq!(rx.apply(frame(S, u16::MAX - 2, Command::EStop)), None);
        assert_eq!(rx.last_applied(), Some((S, 1)));
    }

    #[test]
    fn random_session_is_never_zero_and_varies() {
        let sessions: Vec<u16> = (0..64).map(|_| random_session()).collect();
        assert!(sessions.iter().all(|&s| s != 0));
        assert!(
            sessions.iter().any(|&s| s != sessions[0]),
            "64 draws all identical: {sessions:?}"
        );
    }

    #[test]
    fn sender_stamps_its_session_on_frames() {
        let mut tx = sender();
        tx.set(Command::Nop);
        assert_eq!(tx.poll(Instant::now()).unwrap().session, S);
        assert_eq!(tx.session(), S);
    }

    #[test]
    fn sender_sends_immediately_then_waits_for_the_interval() {
        let mut tx = sender();
        let t0 = Instant::now();
        tx.set(Command::EStop);

        let first = tx.poll(t0).expect("must send immediately after set()");
        assert_eq!(first.cmd_seq, 1);
        assert!(tx.poll(t0 + Duration::from_millis(50)).is_none());
        let second = tx.poll(t0 + Duration::from_millis(150)).unwrap();
        assert_eq!(second, first);
    }

    #[test]
    fn sender_stops_once_acked() {
        let mut tx = sender();
        let t0 = Instant::now();
        tx.set(Command::SetMissionGoal(MissionGoal {
            lat_deg: 7.0,
            lon_deg: 100.0,
        }));
        let sent = tx.poll(t0).unwrap();
        assert!(!tx.is_acked());

        tx.on_telemetry(S, sent.cmd_seq);
        assert!(tx.is_acked());
        assert!(tx.poll(t0 + Duration::from_secs(10)).is_none());
    }

    #[test]
    fn sender_ignores_an_ack_for_another_session_or_an_older_seq() {
        let mut tx = sender();
        tx.set(Command::Nop);
        tx.poll(Instant::now());
        tx.on_telemetry(S, 1);
        tx.set(Command::Nop); // seq 2 in flight
                              // Same seq number but a previous run's session: must not ack.
        tx.on_telemetry(S + 1, 2);
        // Our session but an older seq (leftover echo of command 1).
        tx.on_telemetry(S, 1);
        // No ack reported at all.
        tx.on_telemetry(0, 0);
        assert!(!tx.is_acked());
        assert_eq!(tx.in_flight(), Some((2, Command::Nop)));
    }

    #[test]
    fn sender_ack_accepts_a_newer_seq() {
        let mut tx = sender();
        tx.set(Command::Nop);
        tx.on_telemetry(S, 9);
        assert!(tx.is_acked());
    }

    /// Regression for bug C: a second command used to replace an unacked one.
    #[test]
    fn sender_queues_behind_the_in_flight_command_in_order() {
        let mut tx = sender();
        let t0 = Instant::now();
        assert!(tx.set(Command::SetSpeedLimit(30)).is_empty());
        assert!(tx.set(Command::CancelMission).is_empty());
        assert!(tx.set(Command::Nop).is_empty());
        assert_eq!(tx.queue_len(), 2);
        assert_eq!(tx.in_flight(), Some((1, Command::SetSpeedLimit(30))));
        assert_eq!(tx.poll(t0).unwrap().body, Command::SetSpeedLimit(30));

        // Ack of #1 promotes #2, sent on the very next poll.
        tx.on_telemetry(S, 1);
        assert_eq!(tx.queue_len(), 1);
        let f = tx.poll(t0).expect("promoted command sent immediately");
        assert_eq!((f.cmd_seq, f.body), (2, Command::CancelMission));

        tx.on_telemetry(S, 2);
        assert_eq!(tx.poll(t0).unwrap().body, Command::Nop);
        tx.on_telemetry(S, 3);
        assert!(tx.is_acked());
        assert_eq!(tx.queue_len(), 0);
    }

    #[test]
    fn estop_preempts_queue_and_in_flight_and_reports_what_it_dropped() {
        let mut tx = sender();
        tx.set(Command::SetSpeedLimit(30)); // in flight, seq 1
        tx.set(Command::CancelMission); // queued
        tx.set(Command::Nop); // queued
        let dropped = tx.set(Command::EStop);

        assert_eq!(
            dropped,
            vec![
                Discarded {
                    cmd_seq: None,
                    body: Command::CancelMission
                },
                Discarded {
                    cmd_seq: None,
                    body: Command::Nop
                },
                Discarded {
                    cmd_seq: Some(1),
                    body: Command::SetSpeedLimit(30)
                },
            ]
        );
        assert_eq!(tx.in_flight(), Some((2, Command::EStop)));
        assert_eq!(tx.queue_len(), 0);
        // And it goes out at once.
        assert_eq!(tx.poll(Instant::now()).unwrap().body, Command::EStop);
    }

    /// The headline of bug C: "estop, then anything before the ack" must not
    /// lose the EStop.
    #[test]
    fn nothing_replaces_an_in_flight_estop() {
        let mut tx = sender();
        tx.set(Command::EStop);
        assert!(tx
            .set(Command::SetMissionGoal(MissionGoal {
                lat_deg: 1.0,
                lon_deg: 2.0
            }))
            .is_empty());
        assert!(tx.set(Command::ClearEStop).is_empty());
        assert_eq!(tx.in_flight(), Some((1, Command::EStop)));
        assert_eq!(tx.queue_len(), 2);

        // A second EStop is redundant: it drops the queued commands but the
        // in-flight EStop stays put (same seq, retransmit clock untouched).
        let dropped = tx.set(Command::EStop);
        assert_eq!(dropped.len(), 2);
        assert_eq!(tx.in_flight(), Some((1, Command::EStop)));
        assert_eq!(tx.queue_len(), 0);
    }

    #[test]
    fn full_handshake_between_sender_and_receiver() {
        let mut tx = sender();
        let mut rx = CommandReceiver::new();
        let mut t = Instant::now();

        tx.set(Command::SetSpeedLimit(50));
        let mut applied = None;
        for _ in 0..3 {
            if let Some(f) = tx.poll(t) {
                if let Some(body) = rx.apply(f) {
                    applied = Some(body);
                }
            }
            t += Duration::from_millis(200);
        }
        assert_eq!(applied, Some(Command::SetSpeedLimit(50)));

        let ack = rx.ack();
        tx.on_telemetry(ack.session, ack.cmd_seq);
        assert!(tx.is_acked());
        assert!(tx.poll(t + Duration::from_secs(1)).is_none());
    }

    /// Regression for bug A end to end: restart the station (new sender,
    /// new session) against a receiver that remembers a high seq.
    #[test]
    fn restarted_sender_is_obeyed_and_not_falsely_acked() {
        let mut rx = CommandReceiver::new();
        let mut old = CommandSender::with_session(Duration::from_millis(10), 100);
        for _ in 0..50 {
            old.set(Command::Nop);
            let f = old.poll(Instant::now()).unwrap();
            rx.apply(f);
            old.on_telemetry(rx.ack().session, rx.ack().cmd_seq);
        }
        let mut new = CommandSender::with_session(Duration::from_millis(10), 200);
        new.set(Command::EStop);
        // Telemetry still echoes the old session's seq 50: not our ack.
        new.on_telemetry(rx.ack().session, rx.ack().cmd_seq);
        assert!(!new.is_acked());
        let f = new.poll(Instant::now()).unwrap();
        assert_eq!(rx.apply(f), Some(Command::EStop));
        new.on_telemetry(rx.ack().session, rx.ack().cmd_seq);
        assert!(new.is_acked());
    }

    /// `cmd_seq` wraps through 0 like any other value, for the life of the
    /// process; 0 is no longer reserved.
    #[test]
    fn cmd_seq_wraps_through_zero() {
        let mut tx = sender();
        for _ in 0..u16::MAX {
            tx.set(Command::Nop);
            let (seq, _) = tx.in_flight().unwrap();
            tx.on_telemetry(S, seq);
        }
        assert_eq!(tx.cmd_seq(), u16::MAX);
        tx.set(Command::Nop);
        assert_eq!(tx.cmd_seq(), 0);
        tx.on_telemetry(S, 0);
        tx.set(Command::Nop);
        assert_eq!(tx.cmd_seq(), 1);
    }
}
