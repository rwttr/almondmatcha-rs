//! NMEA 0183 sentence parsing for the u-blox SimpleRTK2b.
//!
//! Ports `gnss_ublox_node.cpp`'s `parseGGA`/`parseRMC`, but as pure functions
//! over a `&str` rather than methods that mutate a node's member fields —
//! that is what makes this testable without a serial port. Two behaviours
//! are deliberately **not** carried over; both are noted where they matter
//! below:
//!
//! 1. **Checksum validation is added.** The ROS 2 node parsed whatever
//!    arrived between `$` and the line ending with no checksum check at all.
//!    A rover's serial line is exactly the kind of noisy environment NMEA's
//!    checksum exists for, and accepting a torn or corrupted sentence as a
//!    real GNSS reading is worse than dropping it.
//! 2. **RMC's `A`/`V` status is honoured.** The ROS 2 node parsed speed and
//!    course out of an RMC sentence unconditionally, even when the receiver
//!    had marked the fix invalid (`V`). This parser still parses the
//!    sentence — `status` is reported — but callers (see `gnss.rs`) should
//!    not treat `speed_knots`/`course_deg` as meaningful unless
//!    `status == 'A'`.
//!
//! GSA/GSV are not parsed here (unlike the ROS 2 node): `rover_msgs::GnssFix`
//! has no SNR field to feed from GSV, and horizontal accuracy is derived from
//! GGA's own HDOP field (index 8) instead of a separate GSA sentence — one
//! less sentence type to track for the same information.

use rover_msgs::FixQuality;

/// A sentence failed to parse.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NmeaError {
    /// The `$...*hh` checksum did not match, or no checksum was present.
    BadChecksum,
    /// Fewer comma-separated fields than this sentence type requires.
    TooFewFields { need: usize, got: usize },
    /// A field that must be numeric was not.
    BadNumber,
    /// The sentence's talker+type prefix did not match what was requested.
    WrongSentenceType,
}

/// Strip a leading `$`, verify the trailing `*hh` checksum, and return the
/// sentence body (talker/type through the last data field, comma-separated,
/// checksum and `*` removed).
///
/// The checksum is the XOR of every byte between `$` and `*`, written as two
/// uppercase hex digits — standard NMEA 0183. Rejecting a bad or absent
/// checksum here means every parser below can assume the bytes it sees came
/// off the wire intact.
fn verify_and_strip(sentence: &str) -> Result<&str, NmeaError> {
    let body = sentence.strip_prefix('$').unwrap_or(sentence);
    let star = body.rfind('*').ok_or(NmeaError::BadChecksum)?;
    let (data, tail) = body.split_at(star);
    let hex = &tail[1..]; // skip '*'
    if hex.len() < 2 {
        return Err(NmeaError::BadChecksum);
    }
    let claimed = u8::from_str_radix(&hex[..2], 16).map_err(|_| NmeaError::BadChecksum)?;
    let computed = data.bytes().fold(0u8, |acc, b| acc ^ b);
    if claimed != computed {
        return Err(NmeaError::BadChecksum);
    }
    Ok(data)
}

/// Map a GGA fix-quality digit onto the ordered [`FixQuality`] enum.
///
/// | NMEA value | Meaning | Mapped to | Why |
/// |---|---|---|---|
/// | 0 | no fix | `None` | — |
/// | 1 | GPS (SPS) | `Autonomous` | — |
/// | 2 | DGPS | `Dgps` | — |
/// | 3 | PPS | `Autonomous` | still a real satellite fix, just PPS-disciplined; no closer category exists |
/// | 4 | RTK fixed | `RtkFixed` | — |
/// | 5 | RTK float | `RtkFloat` | — |
/// | 6 | Estimated (dead reckoning) | `None` | not satellite-derived; treating it as usable would let the mission state machine navigate on an unbounded-drift estimate |
/// | 7 | Manual input | `None` | not a live fix |
/// | 8 | Simulation | `None` | not a real fix — must never be mistaken for one in the field |
/// | 9 | WAAS/SBAS (non-standard, some receivers) | `Dgps` | closest accuracy class |
/// | anything else | unrecognised | `None` | fail safe rather than guess |
///
/// The ROS 2 original (`getFixQuality` in reverse: it mapped an *int* to a
/// free-text string) only handled `{0,1,2,4,5}` and fell through to
/// `"Unknown"` for everything else, including `3`, which is a real fix. This
/// mapping is deliberately more complete; see the module doc comment.
pub fn map_fix_quality(raw: u8) -> FixQuality {
    match raw {
        0 => FixQuality::None,
        1 => FixQuality::Autonomous,
        2 => FixQuality::Dgps,
        3 => FixQuality::Autonomous,
        4 => FixQuality::RtkFixed,
        5 => FixQuality::RtkFloat,
        6 => FixQuality::None,
        7 => FixQuality::None,
        8 => FixQuality::None,
        9 => FixQuality::Dgps,
        _ => FixQuality::None,
    }
}

/// Parsed `GGA` sentence: position, fix quality, satellite count, altitude.
///
/// `lat_deg`/`lon_deg` are `None` when the receiver has not sent a position
/// yet (empty fields before first fix) — ported from `gnss_ublox_node.cpp`'s
/// comment "lat/lon may be empty before fix — keep last known value"; the
/// *keeping* is the assembler's job (`gnss.rs`), this type just reports what
/// was actually in the sentence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gga {
    pub hour: u32,
    pub minute: u32,
    pub second: f64,
    pub lat_deg: Option<f64>,
    pub lon_deg: Option<f64>,
    pub fix: FixQuality,
    pub sats: u8,
    /// Horizontal accuracy estimate, metres. `hdop * 5.0`, capped at
    /// `99.99` — a rough heuristic, not a UERE-based accuracy model, carried
    /// over from `gnss_ublox_node.cpp`'s `parseGSA` (`hdop * 500.0` in
    /// centimetres; divided by 100 here for metres) but read from GGA's own
    /// HDOP field (index 8) instead of a separate GSA sentence.
    pub h_acc_m: f32,
    pub alt_m: f32,
}

fn parse_hhmmss(field: &str) -> Result<(u32, u32, f64), NmeaError> {
    if field.len() < 6 {
        return Err(NmeaError::BadNumber);
    }
    let hour: u32 = field[0..2].parse().map_err(|_| NmeaError::BadNumber)?;
    let minute: u32 = field[2..4].parse().map_err(|_| NmeaError::BadNumber)?;
    let second: f64 = field[4..].parse().map_err(|_| NmeaError::BadNumber)?;
    Ok((hour, minute, second))
}

/// Parse `ddmm.mmmm` (latitude) or `dddmm.mmmm` (longitude) plus a
/// hemisphere letter into signed decimal degrees.
///
/// The last two integer digits are always the whole-minutes part — that is
/// true regardless of whether degrees has two digits (latitude, `0..=90`) or
/// three (longitude, `0..=180`), so the split is always "divide by 100",
/// with no separate latitude/longitude case needed. (`gnss_ublox_node.cpp`'s
/// `parseLatitude`/`parseLongitude` were two near-identical functions doing
/// exactly this; one function covers both here.)
fn parse_lat_lon(value: &str, hemisphere: &str) -> Result<f64, NmeaError> {
    let raw: f64 = value.parse().map_err(|_| NmeaError::BadNumber)?;
    let degrees = (raw / 100.0).trunc();
    let minutes = raw - degrees * 100.0;
    let mut decimal = degrees + minutes / 60.0;
    match hemisphere {
        "S" | "W" => decimal = -decimal,
        "N" | "E" => {}
        _ => return Err(NmeaError::BadNumber),
    }
    Ok(decimal)
}

/// Parse a `$..GGA` sentence (checksum required — see [`verify_and_strip`]).
pub fn parse_gga(sentence: &str) -> Result<Gga, NmeaError> {
    let body = verify_and_strip(sentence)?;
    let tokens: Vec<&str> = body.split(',').collect();
    if !tokens[0].ends_with("GGA") {
        return Err(NmeaError::WrongSentenceType);
    }
    // Through altitude (index 9) is the minimum useful GGA sentence.
    if tokens.len() < 10 {
        return Err(NmeaError::TooFewFields {
            need: 10,
            got: tokens.len(),
        });
    }

    let (hour, minute, second) = if tokens[1].is_empty() {
        (0, 0, 0.0)
    } else {
        parse_hhmmss(tokens[1])?
    };

    let lat_deg = if tokens[2].is_empty() || tokens[3].is_empty() {
        None
    } else {
        Some(parse_lat_lon(tokens[2], tokens[3])?)
    };
    let lon_deg = if tokens[4].is_empty() || tokens[5].is_empty() {
        None
    } else {
        Some(parse_lat_lon(tokens[4], tokens[5])?)
    };

    let fix = if tokens[6].is_empty() {
        FixQuality::None
    } else {
        map_fix_quality(tokens[6].parse().map_err(|_| NmeaError::BadNumber)?)
    };

    let sats: u8 = if tokens[7].is_empty() {
        0
    } else {
        tokens[7].parse().map_err(|_| NmeaError::BadNumber)?
    };

    let hdop: f32 = if tokens[8].is_empty() {
        0.0
    } else {
        tokens[8].parse().map_err(|_| NmeaError::BadNumber)?
    };
    let h_acc_m = (hdop * 5.0).min(99.99);

    let alt_m: f32 = if tokens[9].is_empty() {
        0.0
    } else {
        tokens[9].parse().map_err(|_| NmeaError::BadNumber)?
    };

    Ok(Gga {
        hour,
        minute,
        second,
        lat_deg,
        lon_deg,
        fix,
        sats,
        h_acc_m,
        alt_m,
    })
}

/// Parsed `RMC` sentence: date, status, speed and course over ground.
///
/// `status` is `'A'` (valid) or `'V'` (warning — no fix). Callers must check
/// it before trusting `speed_knots`/`course_deg`; see the module doc comment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rmc {
    pub hour: u32,
    pub minute: u32,
    pub second: f64,
    pub status: char,
    pub speed_knots: f32,
    /// Course over ground, true, degrees. `0.0` (not `None`) when the
    /// receiver left the field empty — typically while stationary, when
    /// `GnssFix::course_is_trustworthy` will be `false` anyway because speed
    /// is below its 0.3 m/s threshold.
    pub course_deg: f32,
    pub year: i32,
    pub month: u32,
    pub day: u32,
}

/// Parse a `$..RMC` sentence (checksum required).
pub fn parse_rmc(sentence: &str) -> Result<Rmc, NmeaError> {
    let body = verify_and_strip(sentence)?;
    let tokens: Vec<&str> = body.split(',').collect();
    if !tokens[0].ends_with("RMC") {
        return Err(NmeaError::WrongSentenceType);
    }
    // Through the date field (index 9) is the minimum useful RMC sentence.
    if tokens.len() < 10 {
        return Err(NmeaError::TooFewFields {
            need: 10,
            got: tokens.len(),
        });
    }

    let (hour, minute, second) = if tokens[1].is_empty() {
        (0, 0, 0.0)
    } else {
        parse_hhmmss(tokens[1])?
    };

    let status = tokens[2].chars().next().unwrap_or('V');

    let speed_knots: f32 = if tokens[7].is_empty() {
        0.0
    } else {
        tokens[7].parse().map_err(|_| NmeaError::BadNumber)?
    };
    let course_deg: f32 = if tokens[8].is_empty() {
        0.0
    } else {
        tokens[8].parse().map_err(|_| NmeaError::BadNumber)?
    };

    let date = tokens[9];
    if date.len() < 6 {
        return Err(NmeaError::BadNumber);
    }
    let day: u32 = date[0..2].parse().map_err(|_| NmeaError::BadNumber)?;
    let month: u32 = date[2..4].parse().map_err(|_| NmeaError::BadNumber)?;
    // NMEA's `ddmmyy` carries no century. `gnss_ublox_node.cpp::parseDate`
    // always assumed 20xx (`"20" + date_str.substr(4, 2)`); this rover will
    // never see a pre-2000 fix, so the same assumption is kept rather than
    // adding a century-pivot heuristic NMEA itself has no way to confirm.
    let year: i32 = 2000
        + date[4..6]
            .parse::<i32>()
            .map_err(|_| NmeaError::BadNumber)?;

    Ok(Rmc {
        hour,
        minute,
        second,
        status,
        speed_knots,
        course_deg,
        year,
        month,
        day,
    })
}

/// Knots to metres per second. Same constant `gnss_ublox_node.cpp` used.
pub const KNOTS_TO_MPS: f32 = 0.514444;

#[cfg(test)]
mod tests {
    use super::*;

    /// Append a correct NMEA checksum to a sentence body (no leading `$`,
    /// no `*hh`), for building test fixtures without hand-computing XORs.
    fn with_checksum(body: &str) -> String {
        let cs = body.bytes().fold(0u8, |acc, b| acc ^ b);
        format!("${body}*{cs:02X}")
    }

    // ---- checksum ---------------------------------------------------

    #[test]
    fn rejects_bad_checksum() {
        let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*FF";
        assert_eq!(parse_gga(sentence), Err(NmeaError::BadChecksum));
    }

    #[test]
    fn rejects_missing_checksum() {
        let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
        assert_eq!(parse_gga(sentence), Err(NmeaError::BadChecksum));
    }

    #[test]
    fn accepts_correct_checksum() {
        let body = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
        let sentence = with_checksum(body);
        assert!(parse_gga(&sentence).is_ok());
    }

    // ---- GGA ----------------------------------------------------------

    #[test]
    fn parses_a_full_gga_fix() {
        let body = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
        let sentence = with_checksum(body);
        let gga = parse_gga(&sentence).unwrap();
        assert_eq!(gga.hour, 12);
        assert_eq!(gga.minute, 35);
        assert!((gga.second - 19.0).abs() < 1e-9);
        assert!((gga.lat_deg.unwrap() - 48.1173).abs() < 1e-4);
        assert!((gga.lon_deg.unwrap() - 11.516667).abs() < 1e-4);
        assert_eq!(gga.fix, FixQuality::Autonomous);
        assert_eq!(gga.sats, 8);
        assert!((gga.alt_m - 545.4).abs() < 1e-6);
    }

    #[test]
    fn southern_and_western_hemispheres_are_negative() {
        let body = "GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,";
        let sentence = with_checksum(body);
        let gga = parse_gga(&sentence).unwrap();
        assert!(gga.lat_deg.unwrap() < 0.0);
        assert!(gga.lon_deg.unwrap() < 0.0);
    }

    #[test]
    fn missing_position_before_first_fix_is_none() {
        let body = "GPGGA,123519,,,,,0,00,99.99,,M,,M,,";
        let sentence = with_checksum(body);
        let gga = parse_gga(&sentence).unwrap();
        assert_eq!(gga.lat_deg, None);
        assert_eq!(gga.lon_deg, None);
        assert_eq!(gga.fix, FixQuality::None);
    }

    #[test]
    fn every_fix_quality_digit_maps_correctly() {
        let cases = [
            (0, FixQuality::None),
            (1, FixQuality::Autonomous),
            (2, FixQuality::Dgps),
            (3, FixQuality::Autonomous),
            (4, FixQuality::RtkFixed),
            (5, FixQuality::RtkFloat),
            (6, FixQuality::None),
            (7, FixQuality::None),
            (8, FixQuality::None),
            (9, FixQuality::Dgps),
            (42, FixQuality::None),
        ];
        for (raw, expected) in cases {
            assert_eq!(map_fix_quality(raw), expected, "raw quality {raw}");
        }
    }

    #[test]
    fn rtk_fixed_end_to_end_through_a_full_sentence() {
        let body = "GNGGA,123519,4807.038,N,01131.000,E,4,12,0.6,545.4,M,46.9,M,1.0,0000";
        let sentence = with_checksum(body);
        let gga = parse_gga(&sentence).unwrap();
        assert_eq!(gga.fix, FixQuality::RtkFixed);
    }

    #[test]
    fn rtk_float_end_to_end_through_a_full_sentence() {
        let body = "GNGGA,123519,4807.038,N,01131.000,E,5,12,0.7,545.4,M,46.9,M,1.0,0000";
        let sentence = with_checksum(body);
        let gga = parse_gga(&sentence).unwrap();
        assert_eq!(gga.fix, FixQuality::RtkFloat);
    }

    #[test]
    fn hdop_becomes_a_metres_accuracy_estimate() {
        let body = "GPGGA,123519,4807.038,N,01131.000,E,1,08,2.0,545.4,M,46.9,M,,";
        let sentence = with_checksum(body);
        let gga = parse_gga(&sentence).unwrap();
        assert!((gga.h_acc_m - 10.0).abs() < 1e-6); // hdop 2.0 * 5.0
    }

    #[test]
    fn absurd_hdop_is_capped() {
        let body = "GPGGA,123519,4807.038,N,01131.000,E,1,08,99.99,545.4,M,46.9,M,,";
        let sentence = with_checksum(body);
        let gga = parse_gga(&sentence).unwrap();
        assert!(gga.h_acc_m <= 99.99);
    }

    #[test]
    fn too_few_fields_is_rejected() {
        let body = "GPGGA,123519,4807.038,N";
        let sentence = with_checksum(body);
        assert!(matches!(
            parse_gga(&sentence),
            Err(NmeaError::TooFewFields { .. })
        ));
    }

    #[test]
    fn wrong_sentence_type_is_rejected_by_parse_gga() {
        let body = "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230324,003.1,W";
        let sentence = with_checksum(body);
        assert_eq!(parse_gga(&sentence), Err(NmeaError::WrongSentenceType));
    }

    // ---- RMC ----------------------------------------------------------

    #[test]
    fn parses_a_full_rmc_sentence() {
        let body = "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230324,003.1,W";
        let sentence = with_checksum(body);
        let rmc = parse_rmc(&sentence).unwrap();
        assert_eq!(rmc.status, 'A');
        assert!((rmc.speed_knots - 22.4).abs() < 1e-6);
        assert!((rmc.course_deg - 84.4).abs() < 1e-6);
        assert_eq!((rmc.year, rmc.month, rmc.day), (2024, 3, 23));
    }

    #[test]
    fn void_status_is_reported_not_hidden() {
        let body = "GPRMC,123519,V,,,,,,,230324,,";
        let sentence = with_checksum(body);
        let rmc = parse_rmc(&sentence).unwrap();
        assert_eq!(rmc.status, 'V');
    }

    #[test]
    fn knots_to_mps_constant_matches_the_ros2_original() {
        assert!((KNOTS_TO_MPS - 0.514444).abs() < 1e-6);
    }
}
