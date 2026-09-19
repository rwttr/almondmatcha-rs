//! Wheel quadrature encoders.
//!
//! # ⚠️ Decoding mode changed: every tick count DOUBLES vs. the ROS 2 firmware
//!
//! The ROS 2 firmware (`encoder_control.cpp`) attached interrupts to channel A
//! only (rise *and* fall), and read channel B purely as a direction input —
//! **2x decoding**: two counts per quadrature cycle. This firmware decodes
//! *both* channels on *every* edge — **4x decoding**: four counts per cycle.
//!
//! `config/rover.toml`'s `metres_per_tick` is currently `0.0` (unmeasured).
//! **Whoever measures it must do so against THIS firmware**, in this decoding
//! mode. A calibration run against the old 2x firmware and applied here will
//! read exactly double the true speed. `config/rover.toml` already records
//! `decoding = "quadrature_4x"` for this reason — keep it in sync with
//! whatever this file actually does.
//!
//! # ⚠️ This is NOT hardware timer encoder mode, and here is why
//!
//! The plan (§5.4) calls for STM32 general-purpose timer encoder mode:
//! zero CPU cost, no missed counts under load, because the timer's own
//! counter register free-runs off the two input channels in hardware. That
//! requires channel A and channel B of *one* encoder to be routed to the
//! *same* timer instance (as `TI1`/`TI2`).
//!
//! Checked against `stm32-metapac`'s pin-to-alternate-function tables for
//! `STM32F767ZI` (the authoritative source ST's own data is generated from —
//! not a datasheet skim), pin by pin:
//!
//! | Pin | Only timer-capable alternate function |
//! |---|---|
//! | `PA_15` (encoder A, ch A) | `TIM2_CH1` (AF1) |
//! | `PB_5`  (encoder A, ch B) | `TIM3_CH2` (AF2) |
//! | `PB_3`  (encoder B, ch A) | `TIM2_CH2` (AF1) |
//! | `PB_4`  (encoder B, ch B) | `TIM3_CH1` (AF2) |
//!
//! **Encoder A's two channels sit on TIM2 and TIM3. Encoder B's two channels
//! also sit on TIM2 and TIM3 — the other way round.** Neither encoder's
//! physical A/B pair shares a single timer, on any alternate function either
//! pin supports (verified exhaustively against every peripheral each pin can
//! be muxed to, not just the timer ones — no other shared option exists
//! either). Curiously, `{PA_15, PB_3}` *would* be a valid TIM2 CH1/CH2 pair,
//! and `{PB_4, PB_5}` a valid TIM3 CH1/CH2 pair — but that pairs encoder A's
//! channel A with encoder B's channel A, and encoder A/B's respective channel
//! Bs with each other. That is not a rewiring this firmware can perform; it
//! is a fact about how the shield/harness was wired for GPIO+interrupt
//! reading in the first place, and it forecloses hardware quadrature timer
//! mode on this board **without a hardware harness change**.
//!
//! This is exactly the situation the rewrite plan asked to be surfaced rather
//! than silently worked around. It is surfaced here, in the top-level task
//! report, and by the fact that this module is called `encoders`, not
//! `encoder_timers`.
//!
//! # What this firmware does instead: software 4x decode over EXTI
//!
//! Both channels of both encoders are wired as [`embassy_stm32::exti::ExtiInput`]
//! — GPIO edge interrupts, same underlying hardware mechanism the ROS 2
//! firmware used, but on *both* channels (4x) instead of one (2x), and
//! decoded through the standard quadrature state-transition table (below)
//! rather than the ROS 2 code's simpler "read the other channel's level on
//! this edge" scheme. The table additionally rejects a same-state "edge" (a
//! glitch or a duplicate wake) and a diagonal two-bit jump (a missed edge —
//! direction is genuinely ambiguous, so it is counted as zero rather than
//! guessed) instead of always attributing every edge to a direction.
//!
//! This is not zero-CPU the way a timer peripheral would be, but each of the
//! four EXTI lines used (`EXTI3`, `EXTI4`, `EXTI5`, `EXTI15`) is a distinct
//! line number, so there is no line-sharing contention between them, and the
//! decode work per edge is a handful of instructions. It is the best option
//! available on this pin assignment, not a silent downgrade to "good enough."
//!
//! # Sign convention — needs bench verification
//!
//! The ROS 2 handlers carried a comment ("sign inverted to match physical
//! mounting") establishing which rotation direction counts up. This firmware
//! defines its own forward/reverse convention from the quadrature state
//! sequence (`00 -> 01 -> 11 -> 10 -> 00` counts up) with no way to check it
//! against the physical mounting from source alone. **Bench-verify**: spin
//! each wheel forward by hand and confirm `ticks_left`/`ticks_right`
//! increase; if a wheel counts backwards, swap that encoder's two channel
//! pins in [`init`]'s argument order (cheaper than patching the table).
use core::sync::atomic::{AtomicI32, Ordering};

use embassy_futures::select::{select, Either};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;
use embassy_stm32::peripherals::{EXTI15, EXTI3, EXTI4, EXTI5, PA15, PB3, PB4, PB5};
use embassy_stm32::{bind_interrupts, Peri};
use embassy_time::{Duration, Instant, Ticker};
use rover_msgs::{encode_frame, Wire, WheelSensors};

use crate::config::{ENCODER_PUBLISH_HZ, WHEEL_SENSORS_DESTS};

// EXTI5 and EXTI15 each cover a *pin number* shared across all GPIO ports
// (e.g. PA5/PB5/PC5.../PA15/PB15... all arbitrate the same EXTI line), which
// is why the interrupt vectors are named by line, not by port. `EXTI3`/
// `EXTI4` have their own dedicated vectors; `EXTI5` shares `EXTI9_5` and
// `EXTI15` shares `EXTI15_10` with other lines in their group — none of
// which are used elsewhere on this board, so there is no contention, but the
// vector *names* below are the shared ones, not `EXTI5`/`EXTI15`.
bind_interrupts!(pub struct Irqs {
    EXTI3 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI3>;
    EXTI4 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI4>;
    EXTI9_5 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI9_5>;
    EXTI15_10 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI15_10>;
});

/// Free-running signed tick counters. Each has exactly one writer (its
/// encoder's counting task) and one reader (the publish task below), so a
/// plain atomic is enough — no multi-field invariant to protect with a mutex.
///
/// `encoder_a` in the ROS 2 firmware is the **left** wheel
/// (`app.cpp`: `mt_lf_encode_msg = enc_A`); `encoder_b` is the **right**
/// wheel (`mt_rt_encode_msg = enc_B`).
static TICKS_LEFT: AtomicI32 = AtomicI32::new(0);
static TICKS_RIGHT: AtomicI32 = AtomicI32::new(0);

/// Standard 4x quadrature transition table, indexed by
/// `(old_state << 2) | new_state` where each 2-bit state is `(a << 1) | b`.
///
/// `+1` for a valid single forward step, `-1` for a valid single reverse
/// step, `0` for "no change" (a duplicate wake with no actual level change)
/// and for a diagonal two-bit jump (both channels appear to have changed
/// between samples — a missed edge, not a determinable direction; counting
/// it as zero is safer than guessing, since a wrong guess in one direction
/// and then the other cancels distance but not the false confidence in the
/// intermediate readings).
#[rustfmt::skip]
const QUAD_DELTA: [i8; 16] = [
    0,  1, -1,  0,
   -1,  0,  0,  1,
    1,  0,  0, -1,
    0, -1,  1,  0,
];

#[inline]
fn state(a: bool, b: bool) -> u8 {
    ((a as u8) << 1) | (b as u8)
}

/// One encoder's two GPIO channels plus decode state.
struct QuadChannel {
    ch_a: ExtiInput<'static, embassy_stm32::mode::Async>,
    ch_b: ExtiInput<'static, embassy_stm32::mode::Async>,
    last_state: u8,
}

impl QuadChannel {
    fn new(ch_a: ExtiInput<'static, embassy_stm32::mode::Async>, ch_b: ExtiInput<'static, embassy_stm32::mode::Async>) -> Self {
        let last_state = state(ch_a.is_high(), ch_b.is_high());
        Self { ch_a, ch_b, last_state }
    }

    /// Run forever: wait for an edge on either channel, decode the
    /// transition, and accumulate it into `counter`.
    ///
    /// Waiting on both channels with `select` (rather than, say, only ever
    /// watching channel A) is what makes this 4x rather than 2x — every edge
    /// on either wire produces a decode, not just edges on one of them.
    async fn run(mut self, counter: &'static AtomicI32) -> ! {
        loop {
            match select(self.ch_a.wait_for_any_edge(), self.ch_b.wait_for_any_edge()).await {
                Either::First(()) | Either::Second(()) => {}
            }
            let new_state = state(self.ch_a.is_high(), self.ch_b.is_high());
            let idx = ((self.last_state as usize) << 2) | new_state as usize;
            self.last_state = new_state;
            let delta = QUAD_DELTA[idx] as i32;
            if delta != 0 {
                counter.fetch_add(delta, Ordering::Relaxed);
            }
        }
    }
}

#[embassy_executor::task]
async fn left_task(ch: QuadChannel) -> ! {
    ch.run(&TICKS_LEFT).await
}

#[embassy_executor::task]
async fn right_task(ch: QuadChannel) -> ! {
    ch.run(&TICKS_RIGHT).await
}

/// Owns the two encoder counting tasks. Returned by [`init`] so `main` can
/// spawn both from one place, matching how `firmware/chassis` hands spawnable
/// futures back to its own `main`.
pub struct Encoders {
    left: QuadChannel,
    right: QuadChannel,
}

impl Encoders {
    pub fn spawn(self, spawner: embassy_executor::Spawner) {
        spawner.spawn(defmt::unwrap!(left_task(self.left)));
        spawner.spawn(defmt::unwrap!(right_task(self.right)));
    }
}

/// Wire up both encoders' four GPIO lines as EXTI inputs.
///
/// Pin/timer pairing per encoder, from `encoder_control.h`:
/// left = channel A on `PA_15`, channel B on `PB_5`;
/// right = channel A on `PB_3`, channel B on `PB_4`.
/// See the module doc comment for why these are read as GPIO+EXTI rather
/// than through a timer's hardware encoder mode.
#[allow(clippy::too_many_arguments)]
pub fn init(
    exti3: Peri<'static, EXTI3>,
    pb3: Peri<'static, PB3>,
    exti4: Peri<'static, EXTI4>,
    pb4: Peri<'static, PB4>,
    exti5: Peri<'static, EXTI5>,
    pb5: Peri<'static, PB5>,
    exti15: Peri<'static, EXTI15>,
    pa15: Peri<'static, PA15>,
) -> Encoders {
    // No pull resistors: these are driven push-pull outputs from the encoder
    // modules themselves (matching the ROS 2 firmware, which used
    // `InterruptIn` at its default floating configuration).
    let left_ch_a = ExtiInput::new(pa15, exti15, Pull::None, Irqs);
    let left_ch_b = ExtiInput::new(pb5, exti5, Pull::None, Irqs);
    let right_ch_a = ExtiInput::new(pb3, exti3, Pull::None, Irqs);
    let right_ch_b = ExtiInput::new(pb4, exti4, Pull::None, Irqs);

    Encoders {
        left: QuadChannel::new(left_ch_a, left_ch_b),
        right: QuadChannel::new(right_ch_a, right_ch_b),
    }
}

/// Publish [`WheelSensors`] at [`ENCODER_PUBLISH_HZ`] (10 Hz, up from the
/// ROS 2 firmware's 4 Hz combined `ChassisSensors` publish).
#[embassy_executor::task]
pub async fn publish_task(stack: embassy_net::Stack<'static>) -> ! {
    let socket = crate::tx_socket!(stack);
    let mut seq: u16 = 0;
    let mut buf = [0u8; rover_msgs::frame::FRAME_HEADER_LEN + WheelSensors::WIRE_LEN];

    // `Ticker`, not `Timer::after`: the latter measures from the moment it
    // is awaited, so per-iteration work (a UDP send) accumulates into the
    // period. `t_us` below is a real clock reading for the same reason
    // `firmware/chassis::imu` takes one instead of a counter advanced by the
    // nominal period — a synthetic timestamp would report perfectly periodic
    // sampling no matter what actually happened, defeating the estimator's
    // `Q*dt` jitter handling.
    let mut ticker = Ticker::every(Duration::from_hz(ENCODER_PUBLISH_HZ as u64));

    loop {
        ticker.next().await;

        let sample = WheelSensors {
            ticks_left: TICKS_LEFT.load(Ordering::Relaxed),
            ticks_right: TICKS_RIGHT.load(Ordering::Relaxed),
            // Real clock reading, not a counter advanced by the nominal
            // period — see this task's own comment above the `Ticker`.
            t_us: Instant::now().as_micros() as u32,
        };

        let n = encode_frame(&sample, seq, &mut buf);
        seq = seq.wrapping_add(1);
        // Two destinations: control (speed loop / odometry) and telemetry
        // (CSV + liveness). See `config::WHEEL_SENSORS_DESTS`.
        for &dest in WHEEL_SENSORS_DESTS.iter() {
            let _ = socket.send_to(&buf[..n], dest).await;
        }
    }
}

// Compile-time proof the shared tx_socket! buffer can hold this message.
const _: () = assert!(WheelSensors::WIRE_LEN + rover_msgs::FRAME_HEADER_LEN <= 128);
