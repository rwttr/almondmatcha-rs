//! Telling the RTK receiver's readings apart from the backup receiver's.
//!
//! **This module exists because of a real gap in the wire protocol, not a
//! stylistic choice — see the top-level report.** `rover_msgs::GnssFix` is
//! one `Wire` type, with one `TYPE_ID`, used for both the u-blox and the
//! Spresense "on two separate streams" (its own doc comment). `rover-bus`'s
//! newest-wins, one-slot-per-type model has no way for a subscriber in a
//! *different process* to know which physical receiver a given `GnssFix`
//! came from — there is no source tag anywhere on the wire. Within
//! `rover-navigation` this is not a problem (it reads both serial ports
//! directly and never confuses them); it only bites a downstream subscriber
//! like this crate, which needs `Telemetry::rtk` and `Telemetry::backup` to
//! be two distinct values.
//!
//! [`classify`] is a best-effort heuristic, not a real fix: a proper fix
//! needs a `source` field (or two distinct types) added to `rover_msgs`,
//! which is out of this crate's scope. The heuristic is sound in one
//! direction and documented as unsound in the other — see below.

use rover_msgs::{FixQuality, GnssFix};

/// Which physical receiver a [`GnssFix`] most likely came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GnssSource {
    Rtk,
    Backup,
}

/// Classify a `GnssFix` by its fix quality.
///
/// **Sound in one direction:** `rover-navigation`'s `SpresenseAssembler`
/// (the backup receiver's own assembler) can only ever produce
/// [`FixQuality::None`] or [`FixQuality::Autonomous`] — the Spresense's JSON
/// output carries a bare `fix: bool` and nothing else, so there is no way
/// for a backup reading to be misclassified as [`GnssSource::Rtk`]: a
/// `Dgps`/`RtkFloat`/`RtkFixed` reading is *always* from the u-blox.
///
/// **Unsound in the other direction:** the u-blox itself reports
/// `Autonomous` (or `None`) before it has achieved a corrected fix — at
/// cold start, or after losing corrections — and at that moment this
/// function cannot tell it apart from a backup reading. Such a reading is
/// classified as [`GnssSource::Backup`], which is the conservative choice:
/// it never risks mislabelling a *good* RTK fix, only mislabelling the
/// u-blox during the window where it has nothing better than the backup
/// receiver to offer anyway.
pub fn classify(fix: &GnssFix) -> GnssSource {
    match fix.fix {
        FixQuality::Dgps | FixQuality::RtkFloat | FixQuality::RtkFixed => GnssSource::Rtk,
        FixQuality::None | FixQuality::Autonomous => GnssSource::Backup,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fix_with_quality(q: FixQuality) -> GnssFix {
        GnssFix {
            fix: q,
            ..Default::default()
        }
    }

    #[test]
    fn dgps_and_better_classify_as_rtk() {
        assert_eq!(
            classify(&fix_with_quality(FixQuality::Dgps)),
            GnssSource::Rtk
        );
        assert_eq!(
            classify(&fix_with_quality(FixQuality::RtkFloat)),
            GnssSource::Rtk
        );
        assert_eq!(
            classify(&fix_with_quality(FixQuality::RtkFixed)),
            GnssSource::Rtk
        );
    }

    #[test]
    fn none_and_autonomous_classify_as_backup() {
        assert_eq!(
            classify(&fix_with_quality(FixQuality::None)),
            GnssSource::Backup
        );
        assert_eq!(
            classify(&fix_with_quality(FixQuality::Autonomous)),
            GnssSource::Backup
        );
    }
}
