//! Turning an NMEA/JSON date+time into Unix milliseconds.
//!
//! Neither GNSS source hands us a ready-made timestamp. NMEA GGA carries only
//! a time-of-day (`hhmmss.ss`) with the date in a *different* sentence (RMC's
//! `ddmmyy`); the Spresense's JSON blob carries a `"YYYY-MM-DD HH:MM:SS"`
//! string with no timezone marker at all. The ROS 2 originals never combined
//! these into a single instant — they stored date and time as separate
//! display strings and left it at that (see `gnss_ublox_node.cpp`'s
//! `GNSSData::date`/`time` and `gnss_spresense_node.cpp`'s raw passthrough).
//! `GnssFix::utc_ms` needs one number, so this module does the combining the
//! originals never had to.
//!
//! Both sources are assumed UTC (u-blox and the Spresense's GNSS chipset both
//! report UTC on the wire; there is no timezone field to get wrong).

/// Days from the civil epoch (1970-01-01) to `(y, m, d)`, using Howard
/// Hinnant's `days_from_civil` algorithm. Proleptic Gregorian, valid for any
/// year `i32` can hold — far more range than GNSS will ever report, but the
/// algorithm is no simpler restricted to a smaller range.
fn days_from_civil(y: i64, m: u32, d: u32) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400; // [0, 399]
    let mp = (m as i64 + 9) % 12; // [0, 11], Mar=0 .. Feb=11
    let doy = (153 * mp + 2) / 5 + d as i64 - 1; // [0, 365]
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy; // [0, 146096]
    era * 146097 + doe - 719468
}

/// Combine a civil UTC date and time-of-day into Unix milliseconds.
///
/// Returns `None` for an out-of-range component (`month` outside `1..=12`,
/// `hour` outside `0..24`, etc.) rather than panicking or silently wrapping —
/// a malformed sentence should fail visibly, not produce a plausible-looking
/// wrong timestamp.
#[allow(clippy::too_many_arguments)]
pub fn civil_to_unix_ms(
    year: i32,
    month: u32,
    day: u32,
    hour: u32,
    minute: u32,
    second: f64,
) -> Option<u64> {
    if !(1..=12).contains(&month) || !(1..=31).contains(&day) {
        return None;
    }
    if hour >= 24 || minute >= 60 || !(0.0..60.0).contains(&second) {
        return None;
    }
    let days = days_from_civil(year as i64, month, day);
    if days < 0 {
        // GNSS did not exist before 1970; a negative day count means the
        // sentence's date field was garbage.
        return None;
    }
    let secs_of_day = hour as f64 * 3600.0 + minute as f64 * 60.0 + second;
    let ms = (days as f64) * 86_400_000.0 + secs_of_day * 1000.0;
    if ms < 0.0 {
        return None;
    }
    Some(ms.round() as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn epoch_is_zero() {
        assert_eq!(civil_to_unix_ms(1970, 1, 1, 0, 0, 0.0), Some(0));
    }

    #[test]
    fn known_instant() {
        // 2024-01-15 12:30:45 UTC = 1705321845 s = 1705321845000 ms.
        // Cross-checked against `date -u -d @1705321845`.
        assert_eq!(
            civil_to_unix_ms(2024, 1, 15, 12, 30, 45.0),
            Some(1_705_321_845_000)
        );
    }

    #[test]
    fn fractional_seconds_round_to_the_nearest_millisecond() {
        let ms = civil_to_unix_ms(2024, 1, 15, 12, 30, 45.123).unwrap();
        assert_eq!(ms, 1_705_321_845_123);
    }

    #[test]
    fn rejects_out_of_range_components() {
        assert_eq!(civil_to_unix_ms(2024, 13, 1, 0, 0, 0.0), None); // bad month
        assert_eq!(civil_to_unix_ms(2024, 1, 32, 0, 0, 0.0), None); // bad day
        assert_eq!(civil_to_unix_ms(2024, 1, 1, 24, 0, 0.0), None); // bad hour
        assert_eq!(civil_to_unix_ms(2024, 1, 1, 0, 60, 0.0), None); // bad minute
        assert_eq!(civil_to_unix_ms(2024, 1, 1, 0, 0, 60.0), None); // bad second
    }

    #[test]
    fn rejects_pre_epoch_dates() {
        assert_eq!(civil_to_unix_ms(1969, 12, 31, 23, 59, 59.0), None);
    }

    #[test]
    fn leap_day_is_valid() {
        // 2024 is a leap year; this must not be rejected.
        assert!(civil_to_unix_ms(2024, 2, 29, 0, 0, 0.0).is_some());
    }
}
