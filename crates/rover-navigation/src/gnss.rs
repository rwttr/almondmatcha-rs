//! Turning a stream of serial bytes into [`GnssFix`]es.
//!
//! Two concerns kept deliberately separate:
//!
//! - [`LineReader`]: byte-buffer bookkeeping, generic over either source.
//!   Ported from `gnss_ublox_node.cpp`'s `readSerialData`, which accumulated
//!   into a `std::string` and clamped it at 4096 bytes so a run of binary
//!   UBX frames (which never contain the ASCII line terminators this reader
//!   looks for) could not grow the buffer forever. Both GNSS sources on this
//!   rover are configured for NMEA/JSON text output, not binary UBX, but the
//!   clamp costs nothing and closes the same failure mode if that ever
//!   changes.
//! - [`UbloxAssembler`] / [`SpresenseAssembler`]: turning complete lines into
//!   [`GnssFix`]es, one per source, using `nmea.rs` / `spresense_json.rs`.

use crate::nmea;
use crate::spresense_json;
use crate::time_utils::civil_to_unix_ms;
use rover_msgs::{FixQuality, GnssFix};

/// Horizontal accuracy placeholder for the Spresense.
///
/// The Spresense's JSON output (`gnss_spresense_node.cpp`) carries no
/// accuracy estimate of any kind — no HDOP, no error figure, nothing. `5.0`
/// metres is a documented placeholder for "uncorrected consumer-grade GPS,
/// typical case", not a measurement. Anything reading `backup.h_acc_m` should
/// treat it as approximate by construction, not as reported by the receiver.
pub const SPRESENSE_H_ACC_PLACEHOLDER_M: f32 = 5.0;

/// Accumulates bytes into complete lines, split on `\n` (with an optional
/// preceding `\r` trimmed), same as both ROS 2 serial readers did with `\r\n`
/// / `\n` scanning.
pub struct LineReader {
    buf: String,
    max_len: usize,
}

impl LineReader {
    /// `max_len` bounds how large an incomplete, delimiter-free buffer is
    /// allowed to grow before it is dropped — see the module doc comment.
    pub fn new(max_len: usize) -> Self {
        Self {
            buf: String::new(),
            max_len,
        }
    }

    /// Feed newly read bytes (decoded lossily — a single torn UTF-8
    /// multi-byte sequence at a chunk boundary must not wedge the whole
    /// reader) and return every line completed by them, in order.
    pub fn feed(&mut self, bytes: &[u8]) -> Vec<String> {
        self.buf.push_str(&String::from_utf8_lossy(bytes));

        if self.buf.len() > self.max_len {
            self.buf.clear();
            return Vec::new();
        }

        let mut lines = Vec::new();
        while let Some(pos) = self.buf.find('\n') {
            let line = self.buf[..pos].trim_end_matches('\r').to_string();
            self.buf.drain(..=pos);
            if !line.is_empty() {
                lines.push(line);
            }
        }
        lines
    }
}

/// Assembles u-blox NMEA sentences (GGA + RMC) into [`GnssFix`]es.
///
/// A fix is emitted on every accepted GGA sentence, mirroring
/// `gnss_ublox_node.cpp`'s `publishData()`, which was likewise called only
/// from `parseGGA`. RMC sentences update the internal date/speed/course but
/// never emit on their own — GGA is the "this is a complete positional
/// reading" signal in this protocol, same as the original.
pub struct UbloxAssembler {
    fix: GnssFix,
    date: Option<(i32, u32, u32)>,
}

impl Default for UbloxAssembler {
    fn default() -> Self {
        Self::new()
    }
}

impl UbloxAssembler {
    pub fn new() -> Self {
        Self {
            fix: GnssFix::default(),
            date: None,
        }
    }

    /// Feed one complete line (CR/LF already stripped by [`LineReader`]).
    /// Non-`$`-prefixed lines and sentences that fail to parse (bad
    /// checksum, wrong type, garbled fields) are silently skipped, same as
    /// the ROS 2 original's `processNMEASentence` dispatch.
    pub fn ingest(&mut self, line: &str) -> Option<GnssFix> {
        if !line.starts_with('$') {
            return None;
        }

        if let Ok(rmc) = nmea::parse_rmc(line) {
            self.date = Some((rmc.year, rmc.month, rmc.day));
            // Only a valid ('A') fix carries a trustworthy speed/course —
            // see nmea.rs's module doc comment for why this check was added.
            if rmc.status == 'A' {
                self.fix.speed_mps = rmc.speed_knots * nmea::KNOTS_TO_MPS;
                self.fix.course_deg = rmc.course_deg;
            }
            return None;
        }

        if let Ok(gga) = nmea::parse_gga(line) {
            if let Some(lat) = gga.lat_deg {
                self.fix.lat_deg = lat;
            }
            if let Some(lon) = gga.lon_deg {
                self.fix.lon_deg = lon;
            }
            self.fix.alt_m = gga.alt_m;
            self.fix.fix = gga.fix;
            self.fix.sats = gga.sats;
            self.fix.h_acc_m = gga.h_acc_m;
            self.fix.utc_ms = self
                .date
                .and_then(|(y, m, d)| civil_to_unix_ms(y, m, d, gga.hour, gga.minute, gga.second))
                .unwrap_or(0);
            return Some(self.fix);
        }

        None
    }
}

/// Parse a Spresense `"YYYY-MM-DD HH:MM:SS"` timestamp into Unix
/// milliseconds. Returns `None` for anything that does not match — see
/// [`SpresenseAssembler::ingest`] for why that degrades rather than fails.
fn parse_spresense_datetime(s: &str) -> Option<u64> {
    let (date, time) = s.split_once(' ')?;
    let mut d = date.splitn(3, '-');
    let year: i32 = d.next()?.parse().ok()?;
    let month: u32 = d.next()?.parse().ok()?;
    let day: u32 = d.next()?.parse().ok()?;

    let mut t = time.splitn(3, ':');
    let hour: u32 = t.next()?.parse().ok()?;
    let minute: u32 = t.next()?.parse().ok()?;
    let second: f64 = t.next()?.parse().ok()?;

    civil_to_unix_ms(year, month, day, hour, minute, second)
}

/// Assembles Spresense JSON readings into [`GnssFix`]es.
///
/// Every line is a complete, self-contained reading (unlike u-blox's
/// multi-sentence assembly), so every successfully parsed line emits a fix.
/// `speed_mps` and `course_deg` are always `0.0` — the Spresense JSON output
/// carries neither, unlike u-blox's RMC sentence — and `fix` can only ever be
/// `None` or `Autonomous`, since the source is a bare boolean with no
/// DGPS/RTK concept.
#[derive(Default)]
pub struct SpresenseAssembler;

impl SpresenseAssembler {
    pub fn new() -> Self {
        Self
    }

    pub fn ingest(&mut self, line: &str) -> Option<GnssFix> {
        let parsed = spresense_json::parse_line(line).ok()?;
        Some(GnssFix {
            lat_deg: parsed.lat_deg,
            lon_deg: parsed.lon_deg,
            alt_m: parsed.alt_m,
            fix: if parsed.fix {
                FixQuality::Autonomous
            } else {
                FixQuality::None
            },
            sats: parsed.num_satellites,
            h_acc_m: SPRESENSE_H_ACC_PLACEHOLDER_M,
            speed_mps: 0.0,
            course_deg: 0.0,
            utc_ms: parse_spresense_datetime(&parsed.time_str).unwrap_or(0),
        })
    }
}

#[cfg(test)]
mod line_reader_tests {
    use super::*;

    #[test]
    fn splits_multiple_complete_lines_in_one_chunk() {
        let mut r = LineReader::new(4096);
        let lines = r.feed(b"line one\nline two\n");
        assert_eq!(lines, vec!["line one", "line two"]);
    }

    #[test]
    fn handles_a_line_split_across_two_reads() {
        let mut r = LineReader::new(4096);
        assert!(r.feed(b"partial li").is_empty());
        let lines = r.feed(b"ne\n");
        assert_eq!(lines, vec!["partial line"]);
    }

    #[test]
    fn strips_carriage_return() {
        let mut r = LineReader::new(4096);
        let lines = r.feed(b"line\r\n");
        assert_eq!(lines, vec!["line"]);
    }

    #[test]
    fn empty_lines_are_dropped() {
        let mut r = LineReader::new(4096);
        let lines = r.feed(b"\n\nreal line\n");
        assert_eq!(lines, vec!["real line"]);
    }

    #[test]
    fn overlong_buffer_with_no_delimiter_is_dropped_not_grown_forever() {
        let mut r = LineReader::new(16);
        let lines = r.feed(b"this line is definitely longer than sixteen bytes");
        assert!(lines.is_empty());
        // The buffer was cleared, so a subsequent well-formed line still works.
        let lines = r.feed(b"ok\n");
        assert_eq!(lines, vec!["ok"]);
    }
}

#[cfg(test)]
mod ublox_assembler_tests {
    use super::*;

    fn with_checksum(body: &str) -> String {
        let cs = body.bytes().fold(0u8, |acc, b| acc ^ b);
        format!("${body}*{cs:02X}")
    }

    #[test]
    fn gga_alone_emits_a_fix_with_no_date() {
        let mut a = UbloxAssembler::new();
        let gga = with_checksum("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
        let fix = a.ingest(&gga).unwrap();
        assert_eq!(fix.fix, FixQuality::Autonomous);
        assert_eq!(fix.utc_ms, 0, "no RMC seen yet, so no date to combine");
    }

    #[test]
    fn rmc_then_gga_produces_a_combined_timestamp() {
        let mut a = UbloxAssembler::new();
        let rmc = with_checksum("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
        assert!(a.ingest(&rmc).is_none(), "RMC alone must not emit a fix");

        let gga = with_checksum("GPGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,,");
        let fix = a.ingest(&gga).unwrap();
        assert_eq!(fix.fix, FixQuality::RtkFixed);
        assert!(fix.utc_ms > 0, "date from RMC + time from GGA must combine");
        assert!((fix.speed_mps - 22.4 * nmea::KNOTS_TO_MPS).abs() < 1e-3);
        assert!((fix.course_deg - 84.4).abs() < 1e-6);
    }

    #[test]
    fn void_rmc_does_not_pollute_speed_or_course() {
        let mut a = UbloxAssembler::new();
        // A void RMC with a nonzero-looking speed/course must not be trusted.
        let rmc = with_checksum("GPRMC,123519,V,,,,,099.9,111.1,230394,,");
        a.ingest(&rmc);
        let gga = with_checksum("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
        let fix = a.ingest(&gga).unwrap();
        assert_eq!(fix.speed_mps, 0.0);
        assert_eq!(fix.course_deg, 0.0);
    }

    #[test]
    fn position_is_held_across_a_gga_with_no_position() {
        let mut a = UbloxAssembler::new();
        let first = with_checksum("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
        let fix1 = a.ingest(&first).unwrap();
        assert!(fix1.lat_deg != 0.0);

        // A later GGA that momentarily lost its position fields.
        let second = with_checksum("GPGGA,123520,,,,,0,00,99.99,,M,,M,,");
        let fix2 = a.ingest(&second).unwrap();
        assert_eq!(
            fix2.lat_deg, fix1.lat_deg,
            "position should be held, not zeroed"
        );
        assert_eq!(
            fix2.fix,
            FixQuality::None,
            "but fix quality reflects the new sentence"
        );
    }

    #[test]
    fn non_dollar_lines_are_ignored() {
        let mut a = UbloxAssembler::new();
        assert!(a.ingest("not a sentence").is_none());
    }

    #[test]
    fn corrupted_sentence_is_ignored_not_panicking() {
        let mut a = UbloxAssembler::new();
        assert!(a.ingest("$GPGGA,garbage*00").is_none());
    }
}

#[cfg(test)]
mod spresense_assembler_tests {
    use super::*;

    #[test]
    fn a_fix_line_produces_autonomous_quality() {
        let mut a = SpresenseAssembler::new();
        let line = r#"{"time":"2025-01-04 14:30:52","numSatellites":7,"fix":true,"latitude":13.736717,"longitude":100.523186,"altitude":12.4}"#;
        let fix = a.ingest(line).unwrap();
        assert_eq!(fix.fix, FixQuality::Autonomous);
        assert_eq!(fix.sats, 7);
        assert_eq!(fix.speed_mps, 0.0);
        assert_eq!(fix.course_deg, 0.0);
        assert!(fix.utc_ms > 0);
    }

    #[test]
    fn a_no_fix_line_produces_none_quality() {
        let mut a = SpresenseAssembler::new();
        let line = r#"{"time":"2025-01-04 14:30:52","numSatellites":0,"fix":false,"latitude":0.0,"longitude":0.0,"altitude":0.0}"#;
        let fix = a.ingest(line).unwrap();
        assert_eq!(fix.fix, FixQuality::None);
    }

    #[test]
    fn garbage_line_yields_no_fix() {
        let mut a = SpresenseAssembler::new();
        assert!(a.ingest("not json").is_none());
    }

    #[test]
    fn bad_timestamp_degrades_to_zero_rather_than_dropping_the_fix() {
        let mut a = SpresenseAssembler::new();
        let line = r#"{"time":"garbled","numSatellites":5,"fix":true,"latitude":1.0,"longitude":2.0,"altitude":0.0}"#;
        let fix = a.ingest(line).unwrap();
        assert_eq!(fix.utc_ms, 0);
        assert!((fix.lat_deg - 1.0).abs() < 1e-9);
    }
}
