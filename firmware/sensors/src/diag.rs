//! Board self-diagnostics: reset cause, power-on self-test, and the 1 Hz
//! [`BoardDiagnostics`] publisher.
//!
//! Deliberately not shared with `firmware/chassis::diag`: these are two
//! separate crates by design (see the root `Cargo.toml`'s module comment),
//! and the board-specific POST bits (`PostBits::SENSOR_A`/`SENSOR_B` mean
//! something different on each board - see `PostBits`' doc comment table)
//! already guarantee the two copies diverge. The reset-cause logic below
//! happens to be identical today because both boards are the same
//! STM32F767ZI part; that is a coincidence of the current hardware, not an
//! invariant either copy should be written to depend on the other for.
//!
//! # Why this exists
//!
//! Before this module, a failed peripheral init was a `defmt` log line and
//! nothing else - invisible without a debugger physically attached. See
//! [`BoardDiagnostics`]' own doc comment for the full rationale; this module
//! is what fills in every field of that struct on the sensors board.
//!
//! # A POST failure never blocks boot
//!
//! Every check below is recorded, never enforced. `main` calls
//! [`Post::record`] once per row of [`PostBits`]' table as each check
//! becomes possible during bring-up, and boot continues regardless of the
//! result - exactly the shape `main.rs` already uses for a failed INA226
//! init (log it, keep going, `WheelSensors` keeps flowing regardless). A
//! rover that refuses to start because its power monitor didn't answer its
//! identity read is worse than one that drives without power telemetry.
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::Stack;
use embassy_stm32::pac::RCC;
use embassy_time::{Duration, Instant, Timer};
use rover_msgs::{encode_frame, BoardDiagnostics, BoardId, PostBits, ResetCause, Wire};

use crate::config::BOARD_DIAGNOSTICS_DESTS;

/// Read `RCC_CSR`, decode it into a [`ResetCause`], and clear its flags.
///
/// # Call this before `embassy_stm32::init`
///
/// This must run as the very first statement in `main`, before anything
/// else - including `embassy_stm32::init` - touches the RCC block. There is
/// exactly one read of this register in the life of the program: once
/// something clears `RCC_CSR` (this function does, at the end, via `RMVF`),
/// or once another reset happens, the cause of *this* boot is gone for
/// good. Reading it does not require `embassy_stm32::init` to have run -
/// RCC is live and memory-mapped from the moment the core comes out of
/// reset, config or no config.
pub fn read_reset_cause() -> ResetCause {
    let csr = RCC.csr().read();

    // Priority order is deliberate, not alphabetical or bit-position order.
    // A single physical reset event routinely sets more than one flag at
    // once, and the flag checked first here is the one that wins.
    //
    // IWDG/WWDG/software/low-power are checked before pin/power-on/brown-out
    // because they are the rarer, more diagnostic causes: if the
    // independent watchdog fired, that is the fact that matters, even if
    // PADRSTF also happens to be set alongside it.
    //
    // POWER-ON IS CHECKED BEFORE BROWN-OUT, AND HERE IS WHY THAT MATTERS:
    // on the STM32F767, a normal cold boot (plugging in power) sets BOTH
    // PORRSTF and BORRSTF - the brown-out detector trips transiently while
    // the supply rail is still ramping up, on *every* power-on, not only on
    // a genuine sagging-supply brown-out. Checking BORRSTF first would
    // report `BrownOut` on every single cold boot. That is worse than a
    // cosmetic mislabel: `ResetCause::is_abnormal()` counts `BrownOut` as a
    // fault, and a fault flag that fires on every boot is exactly the false
    // alarm that teaches people to ignore the real one. Checking PORRSTF
    // first means an ordinary cold boot is correctly reported as `PowerOn`.
    //
    // The consequence, stated plainly: a genuine brown-out (the supply
    // actually sagging mid-run, as opposed to the power-up transient) sets
    // the exact same two flags a cold boot does, and nothing in this
    // register lets software tell the two apart. `BrownOut` is therefore,
    // in practice, never emitted by this function on this part - `PowerOn`
    // wins every time `BORRSTF` is set, because `BORRSTF` is *always* set
    // alongside `PORRSTF` on this line and this ordering checks `PORRSTF`
    // first. This is a hardware limitation of the F767's reset-flag
    // design, not a bug in the ordering below - there is no bit anywhere in
    // `RCC_CSR` that means "brown-out and not power-on."
    let cause = if csr.wdgrstf() {
        ResetCause::IndependentWatchdog
    } else if csr.wwdgrstf() {
        ResetCause::WindowWatchdog
    } else if csr.sftrstf() {
        ResetCause::Software
    } else if csr.lpwrrstf() {
        ResetCause::LowPower
    } else if csr.padrstf() {
        ResetCause::Pin
    } else if csr.porrstf() {
        ResetCause::PowerOn
    } else if csr.borrstf() {
        ResetCause::BrownOut
    } else {
        ResetCause::Unknown
    };

    // Clear every RCC_CSR reset-cause flag so a future reset's cause is
    // never confused with this one's.
    RCC.csr().modify(|w| w.set_rmvf(true));

    cause
}

/// Accumulates [`PostBits`] results across boot.
///
/// Unlike a hardware BIST, these checks cannot all run at once: the PHY
/// isn't identifiable until the network is up, the encoders don't exist
/// until that section of `main` runs, and so on. `main` calls
/// [`Self::record`] once per row of `PostBits`' table as each check becomes
/// possible, then hands the finished `run`/`pass` pair to [`publish_task`].
#[derive(Debug, Clone, Copy, Default)]
pub struct Post {
    pub run: PostBits,
    pub pass: PostBits,
}

impl Post {
    pub fn new() -> Self {
        Self::default()
    }

    /// Record that `bit`'s check executed, and whether it passed.
    pub fn record(&mut self, bit: PostBits, passed: bool) {
        self.run.set(bit);
        if passed {
            self.pass.set(bit);
        }
    }
}

/// `PostBits::CLOCK` - did SYSCLK actually switch to the PLL output
/// `clock_config()` asked for?
///
/// `embassy_stm32::init()` already busy-waits on this exact condition
/// internally (`RCC_CFGR.SWS` matching the requested source) before it
/// returns, so by the time `main` can call this, the answer can only be
/// `true` - if the PLL had failed to lock, `init()` itself would still be
/// spinning and this function would never be reached. The check is kept
/// anyway rather than assumed, on the same principle as every other bit
/// here: a POST bit backed by "well, we got this far" is a weaker claim
/// than one backed by an actual register read, even when the two happen to
/// always agree on this particular chip and driver version.
pub fn check_clock() -> bool {
    use embassy_stm32::rcc::Sysclk;
    RCC.cfgr().read().sws() == Sysclk::PLL1_P
}

/// `PostBits::PHY_ID` - did the LAN8742A answer its identity registers with
/// the expected value?
///
/// `phy_id` is `(ID1 << 16) | ID2` as read over MDIO by
/// [`crate::net::Lan8742a`]. The low nibble of ID2 is a part revision that
/// varies between parts (datasheet: PHY Identifier 2, bits [3:0]), so it is
/// masked off before comparing - see `BoardDiagnostics::phy_id`'s doc
/// comment for the same caveat on the wire.
pub fn check_phy_id(phy_id: u32) -> bool {
    phy_id != 0 && (phy_id & 0xFFFF_FFF0) == 0x0007_C130
}

/// `PostBits::NET_BIND` - can this board actually bind a UDP socket?
///
/// This does not reuse `net::make_rx_socket`'s real socket: that call
/// already `unwrap!`s on failure (see its call site in `main`), so by the
/// time `main` could ask "did it work?" the answer would always be "yes, or
/// we already panicked" - not a real test. This binds and immediately drops
/// a throwaway socket of its own instead, on a local (not `'static`) buffer
/// pair, so it can observe and report a bind failure without panicking and
/// without permanently reserving a slot in `net::SOCKET_COUNT` - the socket
/// is freed back to the stack's pool the moment it's dropped, before any of
/// the board's real sockets are created.
pub fn check_net_bind(stack: Stack<'static>) -> bool {
    let mut rx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_buf = [0u8; 8];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut tx_buf = [0u8; 8];
    let mut sock = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    sock.bind(0).is_ok()
}

/// Publish [`BoardDiagnostics`] once immediately (as soon as POST is done
/// and this task is spawned), then once per second thereafter.
///
/// # Never delays anything on the safety-relevant path
///
/// This task shares nothing with `watchdog::run`, which owns the sole IWDG
/// pet site on this board: no socket, no GPIO, no mutex either could block
/// on. `reset_cause`/`post_run`/`post_pass` are copied in by value at spawn
/// time (all three are `Copy`), and [`crate::net::phy_status`] is a handful
/// of lock-free atomic loads. A slow or failed UDP send here is entirely
/// local to this task's own socket and cannot propagate backpressure into
/// the link watchdog's pet ticker.
#[embassy_executor::task]
pub async fn publish_task(
    stack: Stack<'static>,
    board: BoardId,
    reset_cause: ResetCause,
    post_run: PostBits,
    post_pass: PostBits,
) -> ! {
    let socket = crate::tx_socket!(stack);
    let mut seq: u16 = 0;
    let mut buf = [0u8; rover_msgs::FRAME_HEADER_LEN + BoardDiagnostics::WIRE_LEN];
    let mut tx_drops: u16 = 0;

    loop {
        let phy = crate::net::phy_status();
        let diag = BoardDiagnostics {
            board,
            post_run,
            post_pass,
            reset_cause,
            phy_id: phy.phy_id,
            link_speed_mbps: phy.link_speed_mbps,
            link_full_duplex: phy.link_full_duplex,
            phy_symbol_errors: phy.phy_symbol_errors,
            uptime_s: Instant::now().as_secs() as u32,
            tx_drops,
        };

        let n = encode_frame(&diag, seq, &mut buf);
        seq = seq.wrapping_add(1);

        // `config/rover.toml` routes `BoardDiagnostics = ["telemetry",
        // "base"]` - two destinations, unlike every other message this
        // board publishes. A failed send to either one counts against the
        // same saturating counter; see `BoardDiagnostics::tx_drops`' doc
        // comment and `config::BOARD_DIAGNOSTICS_DESTS`.
        for &dest in &BOARD_DIAGNOSTICS_DESTS {
            if socket.send_to(&buf[..n], dest).await.is_err() {
                tx_drops = tx_drops.saturating_add(1);
            }
        }

        Timer::after(Duration::from_secs(1)).await;
    }
}

// Compile-time proof the shared tx_socket! buffer can hold this message.
const _: () = assert!(BoardDiagnostics::WIRE_LEN + rover_msgs::FRAME_HEADER_LEN <= 128);
