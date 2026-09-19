//! Opening the two GNSS serial ports.
//!
//! This is the one place `serialport` is called directly — everything else
//! in this crate works against a plain `std::io::Read`/`Write`, which
//! `Box<dyn serialport::SerialPort>` satisfies, so `gnss.rs`'s assemblers and
//! `LineReader` need not know a serial port is involved at all. The ROS 2
//! originals hand-rolled `open()`/`tcsetattr` for this (see
//! `gnss_ublox_node.cpp::configureSerialPort`); a maintained crate exists and
//! is not worth re-deriving.

use std::time::Duration;

/// u-blox SimpleRTK2b default port and baud, from `gnss_ublox_node.cpp`.
pub const UBLOX_DEFAULT_PORT: &str = "/dev/ttyACM0";
pub const UBLOX_DEFAULT_BAUD: u32 = 460_800;

/// Spresense default port and baud, from `gnss_spresense_node.cpp`.
pub const SPRESENSE_DEFAULT_PORT: &str = "/dev/ttyUSB0";
pub const SPRESENSE_DEFAULT_BAUD: u32 = 115_200;

/// Read timeout for both ports. Short enough that a reader thread notices a
/// shutdown/reconfiguration promptly; long enough not to spin the CPU on an
/// idle line.
const READ_TIMEOUT: Duration = Duration::from_millis(200);

/// Open a serial port for reading NMEA/JSON lines (and, for the u-blox port,
/// later writing RTCM corrections — see `rtcm.rs`).
pub fn open(path: &str, baud: u32) -> Result<Box<dyn serialport::SerialPort>, serialport::Error> {
    serialport::new(path, baud).timeout(READ_TIMEOUT).open()
}
