//! The base↔rover command protocol: idempotent datagrams with sequence echo.
//!
//! `docs/RUST_REWRITE_PLAN.md` §4.2 replaces what used to be a TCP
//! request/response (`SetSpeedLimit`, mission goals) with something that
//! survives a link with no return path guaranteed on any given try: the base
//! keeps resending the same [`CommandFrame`] at about 1 Hz until it sees its
//! `cmd_seq` echoed back in `Telemetry::last_cmd_seq`, and the rover applies
//! whichever frame is newest by that sequence number and ignores the rest.
//! Neither side keeps a connection; either side can restart without the
//! other noticing anything beyond a gap in retransmits.
//!
//! [`CommandSender`] and [`CommandReceiver`] below are deliberately not
//! wired to a [`crate::Bus`] or a [`rover_link::Link`] directly. A sender only
//! needs to be told "what time is it" and "here is what got echoed back"; a
//! receiver only needs "here is a frame that arrived". That keeps both
//! trivially unit-testable without a socket, and leaves the actual
//! publish/subscribe call at the boundary the caller already controls (see
//! the integration test in `tests/`).

use rover_msgs::{Command, CommandFrame};
use std::time::{Duration, Instant};

/// Default retransmit interval: "~1 Hz" per plan §4.2.
pub const DEFAULT_RETRANSMIT_INTERVAL: Duration = Duration::from_secs(1);

/// Base-station side: owns the current command and decides when to
/// (re)transmit it.
///
/// `cmd_seq` starts at `0` and the first real command is `1`. Zero is kept
/// permanently unusable as a sequence number so it can double as "no command
/// has ever been applied yet" on the receiver side without a separate
/// `Option` — see [`CommandReceiver`].
pub struct CommandSender {
    cmd_seq: u16,
    pending: Option<CommandFrame>,
    interval: Duration,
    last_sent: Option<Instant>,
}

impl CommandSender {
    pub fn new(interval: Duration) -> Self {
        Self {
            cmd_seq: 0,
            pending: None,
            interval,
            last_sent: None,
        }
    }

    /// Queue `body` as the command to send, replacing any command still
    /// unacknowledged. Always assigns a fresh, higher `cmd_seq` — even if
    /// `body` is identical to the pending one — so a repeated "cancel" click
    /// from an operator still restarts the 1 Hz retransmit clock.
    pub fn set(&mut self, body: Command) {
        // The receiver's "nothing applied yet" state is the bare u16 `0`
        // (see `CommandReceiver`), not an `Option` — chosen to mirror what
        // actually crosses the wire, where there is no room for one. That
        // makes "this sender never emits 0" load-bearing: if it did, a
        // freshly restarted receiver would treat that command as a duplicate
        // of its own startup state and silently drop it.
        //
        // So 0 is skipped on wraparound rather than merely asserted against.
        // An assertion that can fire during correct operation is not an
        // invariant, it is a delayed panic: `cmd_seq` wraps once every 65536
        // commands, and a soak test or a long-lived ground station in a debug
        // build would eventually hit it for no reason. Skipping the value
        // makes the invariant true for the life of the process, which is what
        // lets the debug_assert below mean something.
        self.cmd_seq = match self.cmd_seq.wrapping_add(1) {
            0 => 1,
            n => n,
        };
        debug_assert_ne!(
            self.cmd_seq, 0,
            "CommandSender must never emit cmd_seq == 0; it is CommandReceiver's \
             sentinel for \"nothing applied yet\""
        );
        self.pending = Some(CommandFrame {
            cmd_seq: self.cmd_seq,
            body,
        });
        self.last_sent = None; // send on the very next poll, don't wait out the interval
    }

    /// Call once per base-station tick. Returns the frame to put on the wire
    /// if one is due — either never sent, or the retransmit interval has
    /// elapsed — and `None` once the pending command has been acknowledged or
    /// there was never one to send.
    pub fn poll(&mut self, now: Instant) -> Option<CommandFrame> {
        let frame = self.pending?;
        let due = match self.last_sent {
            None => true,
            Some(t) => now.duration_since(t) >= self.interval,
        };
        due.then(|| {
            self.last_sent = Some(now);
            frame
        })
    }

    /// Feed the `last_cmd_seq` echoed in a received `Telemetry` frame. Clears
    /// the pending command once it matches what was last sent, which is what
    /// stops the retransmits.
    pub fn on_telemetry(&mut self, last_cmd_seq: u16) {
        if self.pending.is_some_and(|f| f.cmd_seq == last_cmd_seq) {
            self.pending = None;
        }
    }

    /// True once the pending command (if any) has been acknowledged.
    pub fn is_acked(&self) -> bool {
        self.pending.is_none()
    }

    /// The `cmd_seq` of the most recently queued command, for logging.
    pub fn cmd_seq(&self) -> u16 {
        self.cmd_seq
    }
}

impl Default for CommandSender {
    fn default() -> Self {
        Self::new(DEFAULT_RETRANSMIT_INTERVAL)
    }
}

/// Rover side: applies whichever frame is newest and ignores the rest.
///
/// `last_applied` starts at `0`, which is never a `cmd_seq` [`CommandSender`]
/// actually sends (see its docs) — so the very first real command, whatever
/// its sequence number turns out to be, is always newer than the starting
/// state and gets applied.
#[derive(Debug, Default)]
pub struct CommandReceiver {
    last_applied: u16,
}

impl CommandReceiver {
    pub fn new() -> Self {
        Self::default()
    }

    /// Apply `frame` if it is newer than the last one applied
    /// ([`CommandFrame::is_newer_than`], which already handles `u16`
    /// wraparound — this does not reimplement that comparison). Returns the
    /// command body when applied, `None` for a stale or duplicate frame.
    pub fn apply(&mut self, frame: CommandFrame) -> Option<Command> {
        if frame.is_newer_than(self.last_applied) {
            self.last_applied = frame.cmd_seq;
            Some(frame.body)
        } else {
            None
        }
    }

    /// The sequence number to echo back in `Telemetry::last_cmd_seq`.
    pub fn last_applied(&self) -> u16 {
        self.last_applied
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rover_msgs::MissionGoal;

    #[test]
    fn receiver_applies_the_first_command() {
        let mut rx = CommandReceiver::new();
        let frame = CommandFrame {
            cmd_seq: 1,
            body: Command::EStop,
        };
        assert_eq!(rx.apply(frame), Some(Command::EStop));
        assert_eq!(rx.last_applied(), 1);
    }

    #[test]
    fn receiver_ignores_exact_duplicates() {
        let mut rx = CommandReceiver::new();
        let frame = CommandFrame {
            cmd_seq: 5,
            body: Command::SetSpeedLimit(40),
        };
        assert_eq!(rx.apply(frame), Some(Command::SetSpeedLimit(40)));
        // The base is still retransmitting the same frame at 1 Hz; every
        // repeat after the first must be a no-op.
        assert_eq!(rx.apply(frame), None);
        assert_eq!(rx.apply(frame), None);
        assert_eq!(rx.last_applied(), 5);
    }

    #[test]
    fn receiver_ignores_stale_out_of_order_frames() {
        let mut rx = CommandReceiver::new();
        rx.apply(CommandFrame {
            cmd_seq: 10,
            body: Command::CancelMission,
        });
        // A frame from before the one already applied arrives late.
        let stale = CommandFrame {
            cmd_seq: 7,
            body: Command::EStop,
        };
        assert_eq!(rx.apply(stale), None);
        assert_eq!(rx.last_applied(), 10);
    }

    #[test]
    fn receiver_handles_u16_wraparound() {
        let mut rx = CommandReceiver::new();

        // Walk up toward the top of the range in hops small enough to each
        // count as "newer" (`is_newer_than` treats a jump of more than half
        // the sequence space as stale, so this can't be reached in one leap
        // from the `0` starting state) — the same way a long-running base
        // station session would climb there one command at a time.
        rx.apply(CommandFrame {
            cmd_seq: 32_767,
            body: Command::Nop,
        });
        rx.apply(CommandFrame {
            cmd_seq: u16::MAX - 1,
            body: Command::Nop,
        });
        assert_eq!(rx.last_applied(), u16::MAX - 1);

        // One past the max wraps to 0. That is still "newer" than MAX - 1.
        let wrapped = CommandFrame {
            cmd_seq: 0,
            body: Command::SetSpeedLimit(10),
        };
        assert_eq!(rx.apply(wrapped), Some(Command::SetSpeedLimit(10)));
        assert_eq!(rx.last_applied(), 0);

        // And 1 is newer than the wrapped 0.
        let next = CommandFrame {
            cmd_seq: 1,
            body: Command::SetSpeedLimit(20),
        };
        assert_eq!(rx.apply(next), Some(Command::SetSpeedLimit(20)));

        // But something from "before the wrap" arriving late must not be
        // treated as an enormous jump forward.
        let late = CommandFrame {
            cmd_seq: u16::MAX - 2,
            body: Command::EStop,
        };
        assert_eq!(rx.apply(late), None);
        assert_eq!(rx.last_applied(), 1);
    }

    #[test]
    fn sender_sends_immediately_then_waits_for_the_interval() {
        let mut tx = CommandSender::new(Duration::from_millis(100));
        let t0 = Instant::now();
        tx.set(Command::EStop);

        let first = tx.poll(t0).expect("must send immediately after set()");
        assert_eq!(first.cmd_seq, 1);

        // Too soon: no retransmit yet.
        assert!(tx.poll(t0 + Duration::from_millis(50)).is_none());

        // Interval elapsed: retransmit the same frame.
        let second = tx.poll(t0 + Duration::from_millis(150)).unwrap();
        assert_eq!(second, first);
    }

    #[test]
    fn sender_stops_once_acked() {
        let mut tx = CommandSender::new(Duration::from_millis(10));
        let t0 = Instant::now();
        tx.set(Command::SetMissionGoal(MissionGoal {
            lat_deg: 7.0,
            lon_deg: 100.0,
        }));
        let sent = tx.poll(t0).unwrap();
        assert!(!tx.is_acked());

        tx.on_telemetry(sent.cmd_seq);
        assert!(tx.is_acked());
        assert!(tx.poll(t0 + Duration::from_secs(10)).is_none());
    }

    #[test]
    fn sender_ignores_an_ack_for_a_different_seq() {
        let mut tx = CommandSender::new(Duration::from_millis(10));
        tx.set(Command::EStop);
        tx.on_telemetry(999); // echo of some earlier, unrelated command
        assert!(!tx.is_acked());
    }

    #[test]
    fn full_handshake_between_sender_and_receiver() {
        let mut tx = CommandSender::new(Duration::from_millis(10));
        let mut rx = CommandReceiver::new();
        let mut t = Instant::now();

        tx.set(Command::SetSpeedLimit(50));

        // The base retransmits a few times; the rover applies the first one
        // it sees and ignores the identical repeats that follow.
        let mut applied = None;
        for _ in 0..3 {
            if let Some(frame) = tx.poll(t) {
                if let Some(body) = rx.apply(frame) {
                    applied = Some(body);
                }
            }
            t += Duration::from_millis(20);
        }
        assert_eq!(applied, Some(Command::SetSpeedLimit(50)));

        // Rover starts echoing last_applied; base sees it and stops.
        tx.on_telemetry(rx.last_applied());
        assert!(tx.is_acked());
        assert!(tx.poll(t + Duration::from_secs(1)).is_none());
    }
}

#[cfg(test)]
mod wraparound_tests {
    use super::*;

    /// `cmd_seq` must skip 0 when it wraps, for the life of the process.
    ///
    /// 0 is the receiver's "nothing applied yet" sentinel. A sender that
    /// emitted it after 65536 commands would have that command silently
    /// dropped by any receiver still sitting at startup state — a failure
    /// that would only ever show up in a long field session, which is the
    /// worst possible place to discover it.
    #[test]
    fn cmd_seq_skips_zero_on_wraparound() {
        let mut s = CommandSender::new(core::time::Duration::from_secs(1));

        // Walk right up to the wrap point.
        for _ in 0..u16::MAX {
            s.set(Command::Nop);
        }
        assert_eq!(s.cmd_seq, u16::MAX, "should be at the wrap boundary");

        // The next one must land on 1, not 0.
        s.set(Command::Nop);
        assert_eq!(s.cmd_seq, 1, "wraparound must skip the reserved sentinel 0");
    }
}
