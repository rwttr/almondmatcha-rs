//! Per-type rate and loss tracking for `rover-tap --hz`.
//!
//! Loss is derived from gaps in [`rover_msgs::FrameHeader::seq`], which
//! `Bus::publish` increments by exactly one per send of a given type (see
//! `rover-bus`). A gap of more than one between consecutive frames of the
//! same type means something in between never arrived — this is the only
//! packet-loss signal this bus has, by design (plan §4.1: no acks, no
//! retransmit on the stream side), and it is the number that actually
//! matters when a field link is behaving badly.

use std::collections::HashMap;
use std::time::{Duration, Instant};

/// Counters for one message type, accumulated since the last time they were
/// printed and reset.
#[derive(Default)]
struct Counters {
    name: &'static str,
    frames: u32,
    lost: u32,
}

/// Tracks every type seen since the last report.
pub struct RateTracker {
    per_type: HashMap<u16, Counters>,
    last_seq: HashMap<u16, u16>,
    window_start: Instant,
}

impl RateTracker {
    pub fn new() -> Self {
        Self {
            per_type: HashMap::new(),
            last_seq: HashMap::new(),
            window_start: Instant::now(),
        }
    }

    /// Record one received frame.
    pub fn record(&mut self, type_id: u16, name: &'static str, seq: u16) {
        let entry = self.per_type.entry(type_id).or_insert_with(|| Counters {
            name,
            frames: 0,
            lost: 0,
        });
        entry.frames += 1;

        if let Some(&prev) = self.last_seq.get(&type_id) {
            // Forward distance from `prev` to `seq`, wrapping-aware. A small
            // positive distance is normal spacing (1) or loss (>1). A large
            // one is read as the sender having restarted its counter — e.g.
            // `rover-telemetry` was bounced — rather than as tens of
            // thousands of lost frames, which is what a naive
            // `seq - prev - 1` would otherwise claim.
            let forward = seq.wrapping_sub(prev);
            const RESTART_THRESHOLD: u16 = 1000;
            if forward > 1 && forward < RESTART_THRESHOLD {
                entry.lost += u32::from(forward - 1);
            }
        }
        self.last_seq.insert(type_id, seq);
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
        let mut lines: Vec<(String, String)> = self
            .per_type
            .iter()
            .filter(|(_, c)| c.frames > 0)
            .map(|(_, c)| {
                let hz = f64::from(c.frames) / secs;
                (
                    c.name.to_string(),
                    format!("{:<16} {:>7.1} Hz   lost {:>4}", c.name, hz, c.lost),
                )
            })
            .collect();
        lines.sort_by(|a, b| a.0.cmp(&b.0));

        self.per_type.clear();
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

    #[test]
    fn no_gap_means_no_loss() {
        let mut t = RateTracker::new();
        t.record(1, "ImuSample", 10);
        t.record(1, "ImuSample", 11);
        t.record(1, "ImuSample", 12);
        let lines = t.report();
        assert_eq!(lines.len(), 1);
        assert!(lines[0].contains("lost    0"), "{}", lines[0]);
    }

    #[test]
    fn gap_counts_the_missing_frames() {
        let mut t = RateTracker::new();
        t.record(1, "ImuSample", 10);
        t.record(1, "ImuSample", 14); // 11, 12, 13 never arrived
        let lines = t.report();
        assert!(lines[0].contains("lost    3"), "{}", lines[0]);
    }

    #[test]
    fn seq_wraparound_is_not_counted_as_loss() {
        let mut t = RateTracker::new();
        t.record(1, "ImuSample", u16::MAX - 1);
        t.record(1, "ImuSample", 1); // wraps: MAX-1, MAX, 0, 1 -> 2 missed
        let lines = t.report();
        assert!(lines[0].contains("lost    2"), "{}", lines[0]);
    }

    #[test]
    fn a_restarted_sender_is_not_reported_as_a_mass_loss() {
        let mut t = RateTracker::new();
        t.record(1, "ImuSample", 5000);
        t.record(1, "ImuSample", 0); // process restart, not 65535 - 5000 losses
        let lines = t.report();
        assert!(lines[0].contains("lost    0"), "{}", lines[0]);
    }

    #[test]
    fn silent_types_drop_out_of_the_next_report() {
        let mut t = RateTracker::new();
        t.record(1, "ImuSample", 1);
        assert_eq!(t.report().len(), 1);
        // Nothing recorded in the second window.
        assert_eq!(t.report().len(), 0);
    }
}
