//! Datagram framing.
//!
//! Every datagram on the bus is one frame: a 4-byte header then one message
//! body. There is no fragmentation, no batching and no continuation — the
//! largest message is well under any link's MTU, and keeping one datagram to
//! one message means a lost packet loses exactly one sample of one signal.
//!
//! ```text
//! +----------+----------+---------------------------+
//! | type_id  |   seq    | body (WIRE_LEN bytes)     |
//! |  u16 LE  |  u16 LE  | fixed layout, LE          |
//! +----------+----------+---------------------------+
//! ```
//!
//! `seq` increments per (type, sender). It is not used for reassembly or
//! retransmission — streams are newest-wins — but it makes packet loss
//! measurable, which matters when diagnosing a link in a field.

use crate::codec::{DecodeError, Reader, Writer};
use crate::{check_len, Wire};

/// Size of the frame header in bytes.
pub const FRAME_HEADER_LEN: usize = 4;

/// Largest body this protocol will carry. Every `Wire::WIRE_LEN` is asserted
/// against this in the test suite, so exceeding it is caught at `cargo test`
/// rather than by a truncated datagram in the field.
pub const MAX_BODY_LEN: usize = 256;

/// Largest complete frame.
pub const MAX_FRAME_LEN: usize = FRAME_HEADER_LEN + MAX_BODY_LEN;

/// Parsed frame header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameHeader {
    pub type_id: u16,
    pub seq: u16,
}

impl FrameHeader {
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        let mut w = Writer::new(buf);
        w.u16(self.type_id);
        w.u16(self.seq);
        w.len()
    }

    pub fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        check_len(buf, FRAME_HEADER_LEN)?;
        let mut r = Reader::new(buf);
        Ok(Self {
            type_id: r.u16(),
            seq: r.u16(),
        })
    }
}

/// A header plus the undecoded body bytes.
///
/// Receivers parse the header, dispatch on `type_id`, and only then decode the
/// body into a concrete type — so an unknown or unwanted message costs a
/// 4-byte parse and nothing more.
#[derive(Debug, Clone, Copy)]
pub struct Frame<'a> {
    pub header: FrameHeader,
    pub body: &'a [u8],
}

impl<'a> Frame<'a> {
    /// Split a received datagram into header and body.
    pub fn parse(datagram: &'a [u8]) -> Result<Self, DecodeError> {
        let header = FrameHeader::decode(datagram)?;
        Ok(Self {
            header,
            body: &datagram[FRAME_HEADER_LEN..],
        })
    }

    /// Decode the body as `T`, if this frame carries one.
    ///
    /// Returns `None` when the frame is some other type — the common case on a
    /// shared socket, and not an error.
    pub fn decode_as<T: Wire>(&self) -> Option<Result<T, DecodeError>> {
        if self.header.type_id == T::TYPE_ID {
            Some(T::decode(self.body))
        } else {
            None
        }
    }
}

/// Write a complete frame (header + body) into `buf`, returning its length.
///
/// # Panics
/// If `buf` is shorter than `FRAME_HEADER_LEN + T::WIRE_LEN`.
pub fn encode_frame<T: Wire>(msg: &T, seq: u16, buf: &mut [u8]) -> usize {
    let header = FrameHeader {
        type_id: T::TYPE_ID,
        seq,
    };
    let n = header.encode(buf);
    n + msg.encode(&mut buf[n..])
}
