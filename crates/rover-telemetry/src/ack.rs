//! Folding the two consumers' [`CommandAck`]s into the one acknowledgement
//! `Telemetry` carries.
//!
//! A `CommandFrame` reaches the rover as separate datagrams to
//! `rover-control` and `rover-navigation`. This process used to run its own
//! `CommandReceiver` on a third copy and echo *that*, so the base stopped
//! retransmitting as soon as telemetry got its copy — even if control (and
//! the safety gate behind it) never did. Now each consumer publishes what *it*
//! applied, and the base is told a command landed only when both say so.
//!
//! Pure and clock-free (callers pass `now`) so every rule below is unit
//! tested without sockets.

use rover_link::PeerId;
use rover_msgs::{seq_is_newer, CommandAck};
use std::time::{Duration, Instant};

/// An ack older than this counts as missing. Consumers repeat their ack at
/// 2 Hz, so 2 s tolerates three consecutive lost datagrams; beyond that the
/// consumer is presumed dead or unreachable and the base must keep
/// retransmitting rather than be told a stale "applied".
pub const ACK_FRESH_FOR: Duration = Duration::from_secs(2);

/// Latest [`CommandAck`] per consumer, with its receive time.
#[derive(Debug, Default)]
pub struct AckTracker {
    control: Option<(CommandAck, Instant)>,
    navigation: Option<(CommandAck, Instant)>,
}

impl AckTracker {
    /// Record an ack from `peer`. Acks from any service other than control
    /// and navigation are ignored: only those two apply commands.
    pub fn observe(&mut self, peer: PeerId, ack: CommandAck, now: Instant) {
        match peer {
            PeerId::Control => self.control = Some((ack, now)),
            PeerId::Navigation => self.navigation = Some((ack, now)),
            _ => {}
        }
    }

    /// The `(last_cmd_session, last_cmd_seq)` to put in `Telemetry`.
    ///
    /// Both consumers must have a fresh ack, with the same non-zero session;
    /// the seq is the OLDER of the two (wrap-aware), i.e. the newest command
    /// both are known to have applied. Anything else is `(0, 0)` — "no
    /// acknowledgement", which the base can never mistake for its own session.
    pub fn merged(&self, now: Instant) -> (u16, u16) {
        let fresh = |slot: &Option<(CommandAck, Instant)>| {
            slot.filter(|&(_, t)| now.saturating_duration_since(t) <= ACK_FRESH_FOR)
                .map(|(a, _)| a)
        };
        let (Some(c), Some(n)) = (fresh(&self.control), fresh(&self.navigation)) else {
            return (0, 0);
        };
        if c.session == 0 || c.session != n.session {
            return (0, 0);
        }
        let seq = if seq_is_newer(c.cmd_seq, n.cmd_seq) {
            n.cmd_seq
        } else {
            c.cmd_seq
        };
        (c.session, seq)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ack(session: u16, cmd_seq: u16) -> CommandAck {
        CommandAck { session, cmd_seq }
    }

    fn both(c: CommandAck, n: CommandAck) -> (AckTracker, Instant) {
        let t0 = Instant::now();
        let mut t = AckTracker::default();
        t.observe(PeerId::Control, c, t0);
        t.observe(PeerId::Navigation, n, t0);
        (t, t0)
    }

    /// Bug B end to end, with the real sender and receivers: the EStop's
    /// control copy is lost (control dead), navigation and telemetry saw
    /// theirs. The base must keep retransmitting until control has applied it.
    #[test]
    fn estop_is_not_acked_until_control_has_applied_it() {
        use rover_bus::{CommandReceiver, CommandSender};
        use rover_msgs::Command;

        let mut tx = CommandSender::with_session(Duration::from_millis(10), 0x4242);
        let mut control = CommandReceiver::new();
        let mut nav = CommandReceiver::new();
        let mut acks = AckTracker::default();
        let t0 = Instant::now();

        tx.set(Command::EStop);
        let frame = tx.poll(t0).unwrap();

        // Navigation applies and acks; control never got its copy.
        assert_eq!(nav.apply(frame), Some(Command::EStop));
        acks.observe(PeerId::Navigation, nav.ack(), t0);
        // Control had applied nothing and acks "none" (session 0).
        acks.observe(PeerId::Control, control.ack(), t0);
        let (session, seq) = acks.merged(t0);
        tx.on_telemetry(session, seq);
        assert!(!tx.is_acked(), "EStop acked without reaching control");

        // Retransmit reaches control.
        let retransmit = tx.poll(t0 + Duration::from_millis(20)).unwrap();
        assert_eq!(control.apply(retransmit), Some(Command::EStop));
        acks.observe(PeerId::Control, control.ack(), t0);
        let (session, seq) = acks.merged(t0);
        tx.on_telemetry(session, seq);
        assert!(tx.is_acked());
    }

    #[test]
    fn nothing_received_reports_no_ack() {
        assert_eq!(AckTracker::default().merged(Instant::now()), (0, 0));
    }

    #[test]
    fn both_fresh_and_equal_reports_the_ack() {
        let (t, t0) = both(ack(7, 12), ack(7, 12));
        assert_eq!(t.merged(t0), (7, 12));
    }

    /// Regression for bug B: with control missing (dead, or its copy lost),
    /// the base must NOT be told its command landed.
    #[test]
    fn one_consumer_missing_reports_no_ack() {
        let t0 = Instant::now();
        let mut t = AckTracker::default();
        t.observe(PeerId::Navigation, ack(7, 12), t0);
        assert_eq!(t.merged(t0), (0, 0));
        let mut t = AckTracker::default();
        t.observe(PeerId::Control, ack(7, 12), t0);
        assert_eq!(t.merged(t0), (0, 0));
    }

    #[test]
    fn reports_the_older_seq_of_the_two() {
        let (t, t0) = both(ack(7, 12), ack(7, 11));
        assert_eq!(t.merged(t0), (7, 11));
        let (t, t0) = both(ack(7, 11), ack(7, 12));
        assert_eq!(t.merged(t0), (7, 11));
    }

    #[test]
    fn older_seq_is_wrap_aware() {
        let (t, t0) = both(ack(7, 1), ack(7, u16::MAX));
        assert_eq!(t.merged(t0), (7, u16::MAX));
    }

    #[test]
    fn different_sessions_report_no_ack() {
        let (t, t0) = both(ack(7, 12), ack(8, 1));
        assert_eq!(t.merged(t0), (0, 0));
    }

    #[test]
    fn session_zero_means_nothing_applied() {
        let (t, t0) = both(ack(0, 0), ack(0, 0));
        assert_eq!(t.merged(t0), (0, 0));
        let (t, t0) = both(ack(7, 3), ack(0, 0));
        assert_eq!(t.merged(t0), (0, 0));
    }

    #[test]
    fn an_ack_older_than_two_seconds_counts_as_missing() {
        let (t, t0) = both(ack(7, 12), ack(7, 12));
        assert_eq!(
            t.merged(t0 + ACK_FRESH_FOR),
            (7, 12),
            "exactly at the limit"
        );
        assert_eq!(
            t.merged(t0 + ACK_FRESH_FOR + Duration::from_millis(1)),
            (0, 0)
        );
    }

    #[test]
    fn only_one_consumer_refreshing_still_expires() {
        let (mut t, t0) = both(ack(7, 12), ack(7, 12));
        let later = t0 + Duration::from_secs(5);
        t.observe(PeerId::Control, ack(7, 12), later);
        assert_eq!(t.merged(later), (0, 0), "navigation's ack is 5 s old");
    }

    #[test]
    fn acks_from_other_services_are_ignored() {
        let t0 = Instant::now();
        let mut t = AckTracker::default();
        t.observe(PeerId::Base, ack(7, 12), t0);
        t.observe(PeerId::Perception, ack(7, 12), t0);
        assert_eq!(t.merged(t0), (0, 0));
    }
}
