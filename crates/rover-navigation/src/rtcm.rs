//! RTCM correction injection — **out of scope for this pass**.
//!
//! Plan §6.4 describes the eventual path: the 433 MHz ESP32 radio's USB CDC
//! port carries framed RTCM3 bytes from the base station, and
//! `rover-navigation` reads them and writes them straight to the u-blox's
//! serial port — "a byte pump with framing — no parsing required." The LoRa
//! radios themselves are deferred (plan §9 step 14; see
//! `docs/RUST_REWRITE_PLAN.md` §13.2), so there is nothing to read *from*
//! yet. This function is the seam that work plugs into: it exists, is typed,
//! and is tested, so wiring up the radio later is "call this" rather than
//! "first figure out what this should look like."

use std::io::{self, Write};

/// Write RTCM3 correction bytes straight to the u-blox's serial port.
///
/// No parsing, no framing, no validation of the RTCM content — the u-blox
/// receiver does that itself. This function's entire job is to exist as a
/// stable, typed hand-off point between "bytes arrived from the base link"
/// and "bytes reached the receiver".
///
/// Nothing calls this yet — see the module doc comment — so it is allowed to
/// be dead code rather than removed: deleting a documented, tested seam
/// because it has no caller yet would just mean re-deriving the exact same
/// signature when the 433 MHz link is finally wired up.
#[allow(dead_code)]
pub fn inject_rtcm(ublox_port: &mut dyn Write, rtcm_bytes: &[u8]) -> io::Result<()> {
    ublox_port.write_all(rtcm_bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bytes_pass_through_unchanged() {
        let mut sink: Vec<u8> = Vec::new();
        let rtcm = [0xD3, 0x00, 0x13, 0xAA, 0xBB, 0xCC];
        inject_rtcm(&mut sink, &rtcm).unwrap();
        assert_eq!(sink, rtcm);
    }

    #[test]
    fn empty_input_is_a_no_op() {
        let mut sink: Vec<u8> = Vec::new();
        inject_rtcm(&mut sink, &[]).unwrap();
        assert!(sink.is_empty());
    }
}
