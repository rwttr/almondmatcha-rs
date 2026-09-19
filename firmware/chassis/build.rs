//! Linker glue.
//!
//! No hand-written `memory.x` here: `embassy-stm32`'s own `memory-x` feature
//! (enabled in Cargo.toml) generates the FLASH/RAM layout for the exact
//! `stm32f767zi` part from ST's own peripheral data at build time and puts it
//! on the linker search path itself. Hand-writing one risks silently
//! mismatching the real 2 MiB flash / 512 KiB RAM split the day someone
//! changes the chip feature.
//!
//! `link.x` (from `cortex-m-rt`) and `defmt.x` (from `defmt-rtt`) still need
//! to be pulled onto the linker command line explicitly, same as every
//! Embassy example.
fn main() {
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
