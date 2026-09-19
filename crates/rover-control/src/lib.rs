//! Library half of `rover-control`: estimate (EKF) -> guide (pluggable law)
//! -> actuate (guard rails) — everything in `main.rs` except the network
//! wiring and the thread that owns it.
//!
//! Split out as a library specifically so `tools/replay` (plan §12's gate:
//! "nothing may drive a motor until this passes") can drive the *exact*
//! code that ships in production — the same `Estimator`, the same
//! `StaticGain`, the same `Actuator` — rather than a second reimplementation
//! that could quietly drift from it. A replay harness that tests a
//! lookalike instead of the real pipeline would be worthless for exactly
//! the reason plan §10 cares about port fidelity in the first place.

#![forbid(unsafe_code)]

pub mod actuate;
pub mod config;
pub mod estimate;
pub mod guide;
