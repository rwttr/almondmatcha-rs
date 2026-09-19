//! Wire types shared by every rover node and both STM32 boards.
//!
//! # Contract
//!
//! - Fixed-width little-endian, no padding, no alignment, fields in declaration
//!   order. Every type has a constant [`Wire::WIRE_LEN`].
//! - SI units throughout, with the unit in the field name wherever it could be
//!   read two ways (`cross_track_m`, `speed_mps`, `gyro_radps`).
//! - No variable-length data anywhere. No strings, no vectors, no options on
//!   the wire — presence is always an explicit flag or a sentinel.
//! - Every type carries a stable [`Wire::TYPE_ID`]. IDs are permanent: reuse
//!   one for a different shape and two machines will silently disagree.
//!
//! The Python perception node mirrors these layouts with `struct.Struct`, and
//! `testdata/` holds golden byte fixtures that both sides assert against. That
//! pairing is the only thing preventing silent cross-language drift, so treat a
//! fixture change as a breaking protocol change.
//!
//! # Sign conventions
//!
//! Inherited from the ROS 2 system and **deliberately not normalised to
//! ISO 8855**, because the field-tuned gains (`k_lat = 181.17`,
//! `k_head = 2.024`) depend on them:
//!
//! - `heading_err_rad`, `cross_track_m` and `steer` are all positive when the
//!   correct response is *steer right*.
//! - That is the opposite of the textbook `-k1*e_lat - k2*e_heading` form, so
//!   both feedback terms carry a **plus** sign in the control law.
//!
//! Invert this and the controller becomes positive feedback. See
//! `docs/RUST_REWRITE_PLAN.md` §10.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

pub mod codec;
pub mod frame;
pub mod types;

pub use codec::{DecodeError, Reader, Writer};
pub use frame::{encode_frame, Frame, FrameHeader, FRAME_HEADER_LEN};
pub use types::*;

/// A fixed-layout message that can cross the wire.
///
/// Implementors are plain data: `Copy`, no allocation, no lifetimes. That keeps
/// the same types usable in `no_std` firmware and in host binaries without a
/// second representation.
pub trait Wire: Sized + Copy {
    /// Stable identifier carried in every frame header. Never reused.
    const TYPE_ID: u16;

    /// Exact encoded length in bytes. Constant, by construction.
    const WIRE_LEN: usize;

    /// Human-readable name, for `rover-tap` output and CSV headers.
    const NAME: &'static str;

    /// Write `self` into `buf`.
    ///
    /// # Panics
    /// If `buf.len() < Self::WIRE_LEN`. Call sites size the buffer from that
    /// same constant, so this is a bug rather than a runtime condition.
    fn encode(&self, buf: &mut [u8]) -> usize;

    /// Read a value from the front of `buf`, ignoring trailing bytes.
    fn decode(buf: &[u8]) -> Result<Self, DecodeError>;

    /// Encode into a fresh array. Convenience for tests and one-shot sends.
    #[cfg(feature = "std")]
    fn to_vec(&self) -> std::vec::Vec<u8> {
        let mut v = std::vec![0u8; Self::WIRE_LEN];
        self.encode(&mut v);
        v
    }
}

/// Guards the `decode` length check so every implementation does it identically.
#[inline]
pub(crate) fn check_len(buf: &[u8], need: usize) -> Result<(), DecodeError> {
    if buf.len() < need {
        Err(DecodeError::TooShort {
            need,
            got: buf.len(),
        })
    } else {
        Ok(())
    }
}
