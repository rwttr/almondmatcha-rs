//! Linker glue. Identical to `firmware/chassis/build.rs` - same chip, same
//! reasoning: no hand-written `memory.x`, `embassy-stm32`'s `memory-x`
//! feature generates the FLASH/RAM layout for `stm32f767zi` at build time.
//!
//! `link.x` (from `cortex-m-rt`) and `defmt.x` (from `defmt-rtt`) still need
//! to be pulled onto the linker command line explicitly, same as every
//! Embassy example.
fn main() {
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
