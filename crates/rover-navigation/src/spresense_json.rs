//! Parsing the Sony Spresense's GNSS output.
//!
//! Unlike the u-blox, the Spresense does **not** speak NMEA on this link —
//! `gnss_spresense_node.cpp` parses each line as a flat JSON object with
//! `jsoncpp`:
//!
//! ```json
//! {"time":"2025-01-04 14:30:52","numSatellites":7,"fix":true,
//!  "latitude":13.736717,"longitude":100.523186,"altitude":12.4}
//! ```
//!
//! This is a fixed, flat, single-level schema — never nested, never an array
//! — so a full JSON library is more machinery than the problem needs. This
//! hand-rolled extractor mirrors `rover-msgs`' own preference for small,
//! obviously-correct hand-written parsers over pulling in a dependency for a
//! shape that never changes. It is *not* a general JSON parser: it looks for
//! `"key":value` by substring search and stops at the next unescaped comma or
//! brace, which is sufficient for (and only for) this exact message shape.
//!
//! **What the ROS 2 original didn't give us:** no fix-quality concept beyond
//! a bare `fix: bool` (so this stream can only ever produce
//! [`rover_msgs::FixQuality::None`] or `Autonomous` — there is no DGPS/RTK
//! information here, which is exactly why it is wired up as `Telemetry`'s
//! `backup` field, not `rtk`), no speed, no course, and no horizontal
//! accuracy estimate. See `gnss.rs` for how the missing fields are filled in.

/// A dedicated field is treated as a JSON string (its value is unwrapped from
/// its surrounding quotes); anything else is a bare token up to the next
/// delimiter.
fn find_field<'a>(json: &'a str, key: &str) -> Option<&'a str> {
    let pat = format!("\"{key}\"");
    let key_pos = json.find(&pat)?;
    let after_key = &json[key_pos + pat.len()..];
    let colon = after_key.find(':')?;
    let val = after_key[colon + 1..].trim_start();

    if let Some(rest) = val.strip_prefix('"') {
        let end = rest.find('"')?;
        Some(&rest[..end])
    } else {
        let end = val.find([',', '}']).unwrap_or(val.len());
        Some(val[..end].trim())
    }
}

/// A field required by [`parse_line`] was missing or did not parse as the
/// expected type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpresenseParseError {
    MissingField(&'static str),
    BadValue(&'static str),
}

/// One reading from the Spresense.
#[derive(Debug, Clone, PartialEq)]
pub struct SpresenseFix {
    pub fix: bool,
    pub num_satellites: u8,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f32,
    /// `"YYYY-MM-DD HH:MM:SS"` as reported, unparsed — see
    /// [`crate::time_utils`] for turning this into `utc_ms`. Kept as a string
    /// here because a malformed timestamp should not fail the whole reading:
    /// position and fix status are worth having even if the clock string is
    /// garbled.
    pub time_str: String,
}

/// Parse one line of the Spresense's JSON output.
///
/// Returns an error only for the fields navigation actually depends on
/// (`fix`, `latitude`, `longitude`); `numSatellites`, `altitude` and `time`
/// degrade to `0`/empty rather than failing the whole reading, since a rover
/// with a valid fix but a missing altitude field is still worth reporting.
pub fn parse_line(line: &str) -> Result<SpresenseFix, SpresenseParseError> {
    let fix_str = find_field(line, "fix").ok_or(SpresenseParseError::MissingField("fix"))?;
    let fix = match fix_str {
        "true" => true,
        "false" => false,
        _ => return Err(SpresenseParseError::BadValue("fix")),
    };

    let lat_deg: f64 = find_field(line, "latitude")
        .ok_or(SpresenseParseError::MissingField("latitude"))?
        .parse()
        .map_err(|_| SpresenseParseError::BadValue("latitude"))?;
    let lon_deg: f64 = find_field(line, "longitude")
        .ok_or(SpresenseParseError::MissingField("longitude"))?
        .parse()
        .map_err(|_| SpresenseParseError::BadValue("longitude"))?;

    let num_satellites = find_field(line, "numSatellites")
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let alt_m = find_field(line, "altitude")
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.0);
    let time_str = find_field(line, "time").unwrap_or_default().to_string();

    Ok(SpresenseFix {
        fix,
        num_satellites,
        lat_deg,
        lon_deg,
        alt_m,
        time_str,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_full_reading() {
        let line = r#"{"time":"2025-01-04 14:30:52","numSatellites":7,"fix":true,"latitude":13.736717,"longitude":100.523186,"altitude":12.4}"#;
        let fix = parse_line(line).unwrap();
        assert!(fix.fix);
        assert_eq!(fix.num_satellites, 7);
        assert!((fix.lat_deg - 13.736717).abs() < 1e-9);
        assert!((fix.lon_deg - 100.523186).abs() < 1e-9);
        assert!((fix.alt_m - 12.4).abs() < 1e-6);
        assert_eq!(fix.time_str, "2025-01-04 14:30:52");
    }

    #[test]
    fn no_fix_reading() {
        let line = r#"{"time":"2025-01-04 14:30:52","numSatellites":0,"fix":false,"latitude":0.0,"longitude":0.0,"altitude":0.0}"#;
        let fix = parse_line(line).unwrap();
        assert!(!fix.fix);
    }

    #[test]
    fn field_order_does_not_matter() {
        let line = r#"{"fix":true,"latitude":1.0,"longitude":2.0,"numSatellites":5,"altitude":3.0,"time":"x"}"#;
        let fix = parse_line(line).unwrap();
        assert!(fix.fix);
        assert!((fix.lat_deg - 1.0).abs() < 1e-9);
    }

    #[test]
    fn missing_required_field_is_an_error() {
        let line = r#"{"numSatellites":7,"latitude":13.7,"longitude":100.5}"#;
        assert_eq!(
            parse_line(line),
            Err(SpresenseParseError::MissingField("fix"))
        );
    }

    #[test]
    fn missing_optional_fields_degrade_to_defaults() {
        let line = r#"{"fix":true,"latitude":13.7,"longitude":100.5}"#;
        let fix = parse_line(line).unwrap();
        assert_eq!(fix.num_satellites, 0);
        assert_eq!(fix.alt_m, 0.0);
        assert_eq!(fix.time_str, "");
    }

    #[test]
    fn garbage_input_is_an_error_not_a_panic() {
        let result = parse_line("not json at all");
        assert!(result.is_err());
    }

    #[test]
    fn bad_fix_value_is_an_error() {
        let line = r#"{"fix":"maybe","latitude":1.0,"longitude":2.0}"#;
        assert_eq!(parse_line(line), Err(SpresenseParseError::BadValue("fix")));
    }
}
