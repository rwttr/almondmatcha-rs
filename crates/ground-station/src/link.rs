//! Deciding whether the link to the rover is up, from the age of the last
//! `Telemetry` frame received.
//!
//! Pure and clock-free by construction: [`classify`] takes an already
//! computed age rather than reading a clock itself, so a test can assert on
//! the boundary exactly instead of racing a real timer.

/// How stale the last `Telemetry` frame must be before the link is
/// considered lost. `Telemetry` publishes at 5 Hz (200 ms period, plan
/// §3.2); five missed frames is a full second with nothing at all, which is
/// long enough that this is a real link problem, not one unlucky UDP drop.
pub const LINK_LOSS_TIMEOUT_MS: u64 = 1000;

/// Whether the base considers itself connected to the rover right now.
///
/// This is a display/UI concept only. It must never be read as a safety
/// signal in either direction: a `Live` link does not make an `EStop`
/// delivery guaranteed, and a `Lost` link does not mean the rover has
/// stopped — plan §6.2 is explicit that losing this link must never stop or
/// endanger the rover, which is still driving its mission with zero uplink.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkStatus {
    /// `age_ms` since the last `Telemetry` frame arrived.
    Live { age_ms: u64 },
    /// `age_ms` since the last frame, or `None` if nothing has ever
    /// arrived this session (reported as `u64::MAX` so callers do not need
    /// a second case to handle "never" vs. "a very long time ago").
    Lost { age_ms: u64 },
}

/// Classify link health from the age of the last received `Telemetry`
/// frame. `None` means nothing has been received at all this session.
pub fn classify(age_ms: Option<u64>) -> LinkStatus {
    match age_ms {
        None => LinkStatus::Lost { age_ms: u64::MAX },
        Some(ms) if ms > LINK_LOSS_TIMEOUT_MS => LinkStatus::Lost { age_ms: ms },
        Some(ms) => LinkStatus::Live { age_ms: ms },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nothing_received_yet_is_lost() {
        assert_eq!(classify(None), LinkStatus::Lost { age_ms: u64::MAX });
    }

    #[test]
    fn fresh_frame_is_live() {
        assert_eq!(classify(Some(50)), LinkStatus::Live { age_ms: 50 });
    }

    #[test]
    fn exactly_at_the_timeout_is_still_live() {
        assert_eq!(
            classify(Some(LINK_LOSS_TIMEOUT_MS)),
            LinkStatus::Live {
                age_ms: LINK_LOSS_TIMEOUT_MS
            }
        );
    }

    #[test]
    fn one_ms_past_the_timeout_is_lost() {
        assert_eq!(
            classify(Some(LINK_LOSS_TIMEOUT_MS + 1)),
            LinkStatus::Lost {
                age_ms: LINK_LOSS_TIMEOUT_MS + 1
            }
        );
    }
}
