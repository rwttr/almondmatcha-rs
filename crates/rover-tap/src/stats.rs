//! Per-(sender, type) rate and loss tracking for `rover-tap --hz`.
//!
//! Loss is derived from gaps in [`rover_msgs::FrameHeader::seq`], which
//! `Bus::publish` increments by exactly one per send of a given type (see
//! `rover-bus`). A gap of more than one between consecutive frames of the
//! same type *from the same sender* means something in between never arrived — this is the only
//! packet-loss signal this bus has, by design (plan §4.1: no acks, no
//! retransmit on the stream side), and it is the number that actually
//! matters when a field link is behaving badly.

use rover_link::PeerId;
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// One stream: a message type from one sender. `seq` counts per (type,
/// sender) (`Bus::publish`), so two senders of the same type (both boards
/// send `BoardDiagnostics`) are two independent counters — keyed on type
/// alone they interleave and read as constant loss and "restarts".
type StreamKey = (PeerId, u16);

/// A backward `seq` jump smaller than this is a late (reordered) frame; a
/// forward jump this large or larger — and a backward one at least this
/// large — is read as the sender having restarted its counter.
const RESTART_THRESHOLD: u16 = 1000;

/// Counters for one stream, accumulated since the last time they were
/// printed and reset.
#[derive(Default)]
struct Counters {
    name: &'static str,
    frames: u32,
    lost: u32,
    reordered: u32,
}

/// Tracks every stream seen since the last report.
pub struct RateTracker {
    per_stream: HashMap<StreamKey, Counters>,
    last_seq: HashMap<StreamKey, u16>,
    window_start: Instant,
}

impl RateTracker {
    pub fn new() -> Self {
        Self {
            per_stream: HashMap::new(),
            last_seq: HashMap::new(),
            window_start: Instant::now(),
        }
    }

    /// Record one received frame.
    pub fn record(&mut self, peer: PeerId, type_id: u16, name: &'static str, seq: u16) {
        let key = (peer, type_id);
        let entry = self.per_stream.entry(key).or_insert_with(|| Counters {
            name,
            ..Counters::default()
        });
        entry.frames += 1;

        if let Some(&prev) = self.last_seq.get(&key) {
            // Forward distance from `prev` to `seq`, wrapping-aware.
            //  - 1: normal spacing. 2..THRESHOLD: that many minus one lost.
            //  - 0: a duplicate; nothing to count.
            //  - A *small backward* step (1..THRESHOLD behind `prev`) is a
            //    late or reordered frame, counted separately. It must not
            //    become the new reference (the next in-order frame would
            //    then look like a gap and count phantom losses), and since
            //    it did arrive, it un-counts the loss its gap was charged.
            //  - Anything else large is a restarted sender (`rover-telemetry`
            //    bounced): neither loss nor reorder, and it becomes the new
            //    reference. A naive `seq - prev - 1` would instead claim
            //    tens of thousands of lost frames.
            let forward = seq.wrapping_sub(prev);
            let backward = prev.wrapping_sub(seq);
            if forward > 1 && forward < RESTART_THRESHOLD {
                entry.lost += u32::from(forward - 1);
            } else if forward != 0 && backward < RESTART_THRESHOLD {
                entry.reordered += 1;
                entry.lost = entry.lost.saturating_sub(1);
                return;
            }
        }
        self.last_seq.insert(key, seq);
    }

    /// How long the current reporting window has been open.
    pub fn elapsed(&self) -> Duration {
        self.window_start.elapsed()
    }

    /// Render one report line per type seen this window and start a new one.
    /// Types with zero frames this window (nothing arrived) are dropped
    /// rather than printed at 0 Hz forever, so a link that has gone
    /// completely silent stops cluttering the table instead of the most
    /// useful signal — silence — being buried in it.
    pub fn report(&mut self) -> Vec<String> {
        let secs = self.elapsed().as_secs_f64().max(0.001);
        let mut lines: Vec<((&'static str, PeerId), String)> = self
            .per_stream
            .iter()
            .filter(|(_, c)| c.frames > 0)
            .map(|(&(peer, _), c)| {
                let hz = f64::from(c.frames) / secs;
                (
                    (c.name, peer),
                    format!(
                        "{:<16} {:<10} {:>7.1} Hz   lost {:>4}   reordered {:>3}",
                        c.name,
                        peer.as_str(),
                        hz,
                        c.lost,
                        c.reordered
                    ),
                )
            })
            .collect();
        lines.sort_by(|a, b| a.0.cmp(&b.0));

        self.per_stream.clear();
        self.window_start = Instant::now();
        lines.into_iter().map(|(_, line)| line).collect()
    }
}

impl Default for RateTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const P: PeerId = PeerId::Control;

    #[test]
    fn no_gap_means_no_loss() {
        let mut t = RateTracker::new();
        t.record(P, 1, "ImuSample", 10);
        t.record(P, 1, "ImuSample", 11);
        t.record(P, 1, "ImuSample", 12);
        let lines = t.report();
        assert_eq!(lines.len(), 1);
        assert!(lines[0].contains("lost    0"), "{}", lines[0]);
        assert!(lines[0].contains("reordered   0"), "{}", lines[0]);
    }

    #[test]
    fn gap_counts_the_missing_frames() {
        let mut t = RateTracker::new();
        t.record(P, 1, "ImuSample", 10);
        t.record(P, 1, "ImuSample", 14); // 11, 12, 13 never arrived
        let lines = t.report();
        assert!(lines[0].contains("lost    3"), "{}", lines[0]);
    }

    #[test]
    fn seq_wraparound_is_not_counted_as_loss() {
        let mut t = RateTracker::new();
        t.record(P, 1, "ImuSample", u16::MAX - 1);
        t.record(P, 1, "ImuSample", 1); // wraps: MAX-1, MAX, 0, 1 -> 2 missed
        let lines = t.report();
        assert!(lines[0].contains("lost    2"), "{}", lines[0]);
    }

    #[test]
    fn a_restarted_sender_is_not_reported_as_a_mass_loss() {
        let mut t = RateTracker::new();
        t.record(P, 1, "ImuSample", 5000);
        t.record(P, 1, "ImuSample", 0); // process restart, not 65535 - 5000 losses
        t.record(P, 1, "ImuSample", 1);
        let lines = t.report();
        assert!(lines[0].contains("lost    0"), "{}", lines[0]);
        assert!(lines[0].contains("reordered   0"), "{}", lines[0]);
    }

    /// Regression for bug E: one late frame (10, 12, 11, 13) used to be read
    /// as a counter restart, re-anchoring on 11 so that 13 then charged a
    /// phantom loss.
    #[test]
    fn a_single_late_frame_is_reordered_not_a_restart_or_a_loss() {
        let mut t = RateTracker::new();
        for seq in [10, 12, 11, 13] {
            t.record(P, 1, "ImuSample", seq);
        }
        let lines = t.report();
        assert!(lines[0].contains("lost    0"), "{}", lines[0]);
        assert!(lines[0].contains("reordered   1"), "{}", lines[0]);
    }

    #[test]
    fn a_late_frame_across_the_wrap_is_reordered() {
        let mut t = RateTracker::new();
        for seq in [u16::MAX, 1, 0, 2] {
            t.record(P, 1, "ImuSample", seq);
        }
        let lines = t.report();
        assert!(lines[0].contains("lost    0"), "{}", lines[0]);
        assert!(lines[0].contains("reordered   1"), "{}", lines[0]);
    }

    /// Regression for bug E: stats keyed on type alone. Two senders of the
    /// same type each have their own `seq`; interleaved, they used to read as
    /// perpetual loss/restarts.
    #[test]
    fn two_senders_of_one_type_are_tracked_separately() {
        let mut t = RateTracker::new();
        // Chassis at seq 100.., sensors at seq 5.., interleaved, no loss.
        for i in 0..5u16 {
            t.record(PeerId::Chassis, 0x0903, "BoardDiagnostics", 100 + i);
            t.record(PeerId::Sensors, 0x0903, "BoardDiagnostics", 5 + i);
        }
        let lines = t.report();
        assert_eq!(lines.len(), 2, "one line per (peer, type): {lines:?}");
        for l in &lines {
            assert!(l.contains("lost    0"), "{l}");
            assert!(l.contains("reordered   0"), "{l}");
        }
        assert!(lines.iter().any(|l| l.contains("chassis")));
        assert!(lines.iter().any(|l| l.contains("sensors")));
    }

    #[test]
    fn silent_types_drop_out_of_the_next_report() {
        let mut t = RateTracker::new();
        t.record(P, 1, "ImuSample", 1);
        assert_eq!(t.report().len(), 1);
        // Nothing recorded in the second window.
        assert_eq!(t.report().len(), 0);
    }
}
