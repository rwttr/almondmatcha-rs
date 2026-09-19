//! Little-endian scalar codec.
//!
//! Every field on the wire is fixed-width little-endian with no padding and no
//! alignment. Reading a message definition tells you its byte layout exactly.
//!
//! These helpers are deliberately dull. The wire format is the contract between
//! four machines written in three languages; it should be possible to verify by
//! eye against a hex dump.

/// A message failed to decode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// Buffer was shorter than the type's fixed wire length.
    TooShort { need: usize, got: usize },
    /// A field held a value outside its defined set (bad enum discriminant).
    BadDiscriminant { field: &'static str, value: u8 },
}

impl core::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            DecodeError::TooShort { need, got } => {
                write!(f, "buffer too short: need {need} bytes, got {got}")
            }
            DecodeError::BadDiscriminant { field, value } => {
                write!(f, "invalid discriminant {value} for field `{field}`")
            }
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for DecodeError {}

/// Sequential little-endian writer over a caller-supplied buffer.
///
/// Panics on overflow rather than returning an error: every call site writes a
/// type whose `WIRE_LEN` is a compile-time constant into a buffer sized by that
/// same constant, so an overflow is a bug in this crate, not a runtime
/// condition a caller could handle.
pub struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> Writer<'a> {
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Bytes written so far.
    pub fn len(&self) -> usize {
        self.pos
    }

    pub fn is_empty(&self) -> bool {
        self.pos == 0
    }

    fn put(&mut self, bytes: &[u8]) {
        self.buf[self.pos..self.pos + bytes.len()].copy_from_slice(bytes);
        self.pos += bytes.len();
    }

    pub fn u8(&mut self, v: u8) {
        self.put(&[v]);
    }
    pub fn i8(&mut self, v: i8) {
        self.put(&v.to_le_bytes());
    }
    pub fn u16(&mut self, v: u16) {
        self.put(&v.to_le_bytes());
    }
    pub fn i16(&mut self, v: i16) {
        self.put(&v.to_le_bytes());
    }
    pub fn u32(&mut self, v: u32) {
        self.put(&v.to_le_bytes());
    }
    pub fn i32(&mut self, v: i32) {
        self.put(&v.to_le_bytes());
    }
    pub fn u64(&mut self, v: u64) {
        self.put(&v.to_le_bytes());
    }
    pub fn f32(&mut self, v: f32) {
        self.put(&v.to_le_bytes());
    }
    pub fn f64(&mut self, v: f64) {
        self.put(&v.to_le_bytes());
    }
    pub fn bool(&mut self, v: bool) {
        self.u8(v as u8);
    }
    pub fn f32x3(&mut self, v: [f32; 3]) {
        for x in v {
            self.f32(x);
        }
    }
    pub fn f32x5(&mut self, v: [f32; 5]) {
        for x in v {
            self.f32(x);
        }
    }
}

/// Sequential little-endian reader.
///
/// Length is checked once, up front, by [`Wire::decode`]; the accessors then
/// read without further bounds checks against a buffer already proven long
/// enough.
pub struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    pub fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Bytes consumed so far.
    pub fn len(&self) -> usize {
        self.pos
    }

    pub fn is_empty(&self) -> bool {
        self.pos == 0
    }

    fn take<const N: usize>(&mut self) -> [u8; N] {
        let mut out = [0u8; N];
        out.copy_from_slice(&self.buf[self.pos..self.pos + N]);
        self.pos += N;
        out
    }

    pub fn u8(&mut self) -> u8 {
        self.take::<1>()[0]
    }
    pub fn i8(&mut self) -> i8 {
        i8::from_le_bytes(self.take())
    }
    pub fn u16(&mut self) -> u16 {
        u16::from_le_bytes(self.take())
    }
    pub fn i16(&mut self) -> i16 {
        i16::from_le_bytes(self.take())
    }
    pub fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take())
    }
    pub fn i32(&mut self) -> i32 {
        i32::from_le_bytes(self.take())
    }
    pub fn u64(&mut self) -> u64 {
        u64::from_le_bytes(self.take())
    }
    pub fn f32(&mut self) -> f32 {
        f32::from_le_bytes(self.take())
    }
    pub fn f64(&mut self) -> f64 {
        f64::from_le_bytes(self.take())
    }

    /// Any non-zero byte reads as `true`, so a peer that writes `0xFF` for true
    /// interoperates. Only `0` is false.
    pub fn bool(&mut self) -> bool {
        self.u8() != 0
    }

    pub fn f32x3(&mut self) -> [f32; 3] {
        [self.f32(), self.f32(), self.f32()]
    }
    pub fn f32x5(&mut self) -> [f32; 5] {
        [self.f32(), self.f32(), self.f32(), self.f32(), self.f32()]
    }
}
