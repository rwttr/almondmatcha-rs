//! Command watchdog - plan §5.2.
//!
//! > In the current [ROS 2] firmware, `motor_control_task` acts only when
//! > `command_updated` is set. If no command arrives it does nothing - the
//! > last applied PWM is held indefinitely. If the RPi dies, the link drops,
//! > or `rover-control` crashes mid-drive, the rover keeps driving at its
//! > last throttle until something physically stops it.
//!
//! This module is the fix. Behaviour, exactly per the plan's table:
//!
//! | Parameter | Value | Why |
//! |---|---|---|
//! | `CMD_TIMEOUT` | 200 ms | ten missed frames at the 50 Hz command rate |
//! | `RAMP_TIME` | 300 ms | throttle to zero on a ramp, not a step - a step into a loaded drivetrain is a mechanical shock |
//! | steering on trip | centre | a latched steering angle turns a runaway into a circle |
//! | recovery | automatic, only after an explicit zero-throttle command | stops a flapping link from producing lurch-stop-lurch |
//! | observability | `ChassisStatus.watchdog_tripped` | today the rover cannot tell you this happened at all |
//!
//! # One deliberate deviation from the plan's code sketch
//!
//! The sketch's trip branch reads `ramp_throttle_to_zero(RAMP_TIME).await;
//! centre_steering();` - steering centred *after* the ramp finishes. Read
//! literally that leaves the servo at whatever angle it last had for the
//! full 300 ms while the vehicle is still coasting to a stop, which cuts
//! directly against the stated rationale ("a latched steering angle turns a
//! runaway into a circle") for exactly the window that rationale is about.
//! This implementation centres the servo **immediately** when the trip is
//! detected, then ramps the throttle - same three behaviours the table
//! promises, in the order that actually delivers the safety property named
//! for the steering-centre rule.
//!
//! # The independent watchdog lives here too
//!
//! Per the plan: "Enable IWDG at ~500 ms and pet it only from the control
//! task, so a hang in that task resets the board." This task **is** the
//! control task, and it is the only place `IndependentWatchdog::pet()` is
//! called in this crate. Its `select` always resolves at least once every
//! `CMD_TIMEOUT` (200 ms), comfortably inside the 500 ms IWDG window, so a
//! hang anywhere else in the firmware (a stuck I2C transaction in the IMU
//! task, for instance) still resets the board even though this task itself
//! is healthy.
use defmt::warn;
use embassy_futures::select::{select, Either};
use embassy_stm32::peripherals::IWDG;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU8, Ordering};
use rover_msgs::ChassisCommand;

use crate::config::{CMD_EPSILON, CMD_TIMEOUT_MS, RAMP_STEPS, RAMP_TIME_MS};
use crate::motor::Motors;

/// The one mailbox from the UDP receive task to the control task. A
/// `Signal`, not a `Channel`: the bus is newest-wins by design (plan §4.1),
/// so a command superseded before the control task gets to it should be
/// dropped, not queued.
pub static CMD_SIGNAL: Signal<CriticalSectionRawMutex, ChassisCommand> = Signal::new();

/// Watchdog/status state the periodic `ChassisStatus` publisher (in
/// `main.rs`) reads. Plain atomics rather than a `Mutex`: three independent
/// fields, each written by exactly one task (this one) and read by exactly
/// one other, so there is no multi-field invariant to protect.
pub struct SharedStatus {
    pub seq_echo: AtomicU16,
    pub watchdog_tripped: AtomicBool,
    pub fault: AtomicU8,
}

impl SharedStatus {
    pub const fn new() -> Self {
        Self {
            seq_echo: AtomicU16::new(0),
            watchdog_tripped: AtomicBool::new(false),
            fault: AtomicU8::new(0),
        }
    }
}

pub static STATUS: SharedStatus = SharedStatus::new();

/// Runs forever. Owns the motors, the command mailbox, and the IWDG.
pub async fn run(mut motors: Motors<'static>, mut iwdg: IndependentWatchdog<'static, IWDG>) -> ! {
    let cmd_timeout = Duration::from_millis(CMD_TIMEOUT_MS);
    let mut tripped = false;
    // Last throttle actually applied to the drivetrain, signed. The ramp
    // needs this to decelerate *through* zero along the direction the
    // vehicle was already moving, not to snap the H-bridge direction pins
    // to "forward" the instant the ramp starts (which is what a ramp that
    // only ever counted down from a positive magnitude would do to a
    // vehicle that was last commanded to reverse).
    let mut last_throttle: f32 = 0.0;

    iwdg.unleash();

    loop {
        match select(CMD_SIGNAL.wait(), Timer::after(cmd_timeout)).await {
            Either::First(cmd) => {
                if !tripped {
                    motors.apply(cmd.steer, cmd.throttle);
                    last_throttle = cmd.throttle;
                    STATUS.seq_echo.store(cmd.seq, Ordering::Relaxed);
                } else if cmd.throttle.abs() < CMD_EPSILON {
                    // Explicit zero-throttle command: re-arm. Note this
                    // command's own steer/throttle are *not* applied - only
                    // the trip is cleared, so the packet that re-arms the
                    // board can never itself be the one that moves it. The
                    // next command applies normally on the following
                    // iteration.
                    tripped = false;
                }
                // else: still tripped and this command carries non-zero
                // throttle - ignored entirely, by design (plan: "recovery
                // ... only after an explicit zero-throttle command").
            }
            Either::Second(_) => {
                if !tripped {
                    warn!("watchdog: no ChassisCommand for {}ms, tripping", CMD_TIMEOUT_MS);
                }
                tripped = true;
                // Centre first (see module doc comment for why this order
                // differs from the plan's sketch), then ramp throttle down
                // over RAMP_TIME so the drivetrain decelerates smoothly,
                // along whichever direction it was already moving, rather
                // than stepping to zero.
                motors.set_steering(0.0);
                ramp_throttle_to_zero(&mut motors, &mut iwdg, last_throttle).await;
                last_throttle = 0.0;
            }
        }

        STATUS.watchdog_tripped.store(tripped, Ordering::Relaxed);
        // The only place this crate pets the IWDG - see module doc comment.
        iwdg.pet();
    }
}

/// Linearly ramp throttle from `from` (whatever the drivetrain was last
/// commanded, signed) to zero, over `RAMP_TIME_MS`, in `RAMP_STEPS` steps.
/// Petting the IWDG partway through matters here: on a real trip this
/// coroutine runs for the entire 300 ms ramp without yielding to the outer
/// `select`, and 300 ms is more than half the 500 ms IWDG window.
async fn ramp_throttle_to_zero(motors: &mut Motors<'static>, iwdg: &mut IndependentWatchdog<'static, IWDG>, from: f32) {
    let step_delay = Duration::from_millis(RAMP_TIME_MS / RAMP_STEPS as u64);
    for step in 0..RAMP_STEPS {
        let remaining = 1.0 - (step as f32 / RAMP_STEPS as f32);
        motors.set_drive(from * remaining);
        iwdg.pet();
        Timer::after(step_delay).await;
    }
    motors.set_drive(0.0);
}
