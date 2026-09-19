//! Mirror link watchdog, and the IWDG pet site — plan §5.2.
//!
//! > Sensors board: it publishes only, so it has no actuation to cut. It gets
//! > the mirror-image watchdog instead — if the RPi stops *consuming* (no
//! > `Telemetry` heartbeat seen for 2 s), it logs and flashes the status LED.
//! > Cheap, and it makes a one-way link failure visible at the board rather
//! > than silent.
//!
//! Unlike `firmware/chassis`'s command watchdog, there is nothing to ramp to
//! zero or centre — this board has no actuator. Its only job is to make the
//! failure *visible*: a defmt log line, and a status LED that switches from
//! solid off to a fast blink. The point (per the plan) is that someone
//! standing next to the rover can see the link is down without a laptop.
//!
//! # ⚠️ Known integration gap: `config/rover.toml` routes nothing to `sensors`
//!
//! This task binds `config::SELF_PORT` (7004, per `[ports] sensors`) and
//! watches for inbound `Telemetry` frames, exactly as specified. But
//! `config/rover.toml`'s `[routes]` table is:
//!
//! ```text
//! Telemetry = ["base"]
//! ```
//!
//! — and no entry routes *anything* to `"sensors"`. Nothing in the current
//! configuration ever addresses a datagram to this board. That table lives in
//! `config/rover.toml`, outside `firmware/sensors/` and outside this task's
//! scope, so it is not changed here.
//!
//! The mechanism below is complete and matches the plan's specification: bind,
//! wait with a 2 s timeout, log and blink on timeout, go quiet again the
//! moment a real `Telemetry` frame arrives. What it cannot yet do, absent a
//! routing change (e.g. `Telemetry = ["base", "sensors"]`, or a lighter
//! dedicated heartbeat type) or a debug `[debug] mirror` pointed at this
//! board, is ever see that frame arrive on real hardware. Until that routing
//! gap is closed, this watchdog will read as permanently tripped in the
//! field — which is at least an honest, visible failure rather than a
//! silently-wrong one, but it is not yet exercised end-to-end. Flagged here
//! and in the top-level task report.
//!
//! # The IWDG pet site
//!
//! Per the plan: "Enable IWDG at ~500 ms and pet it only from the control
//! task, so a hang in that task resets the board." This board has no single
//! "control task" the way chassis's motor-owning watchdog is one — so this
//! task, the only one on this board with a safety-relevant duty, takes that
//! role. It pets on a fixed [`IWDG_PET_INTERVAL_MS`] tick that runs
//! independently of whether any UDP traffic arrives at all, so a hang in the
//! encoder or power tasks (a stuck I2C transaction, a deadlocked executor)
//! still resets the board even though this task itself would otherwise keep
//! running.
use defmt::warn;
use embassy_futures::select::{select, Either};
use embassy_net::udp::UdpSocket;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{IWDG, PB0};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::Peri;
use embassy_time::{Duration, Instant, Ticker};
use rover_msgs::{Frame, Telemetry, Wire};

use crate::config::{IWDG_PET_INTERVAL_MS, LINK_TIMEOUT_MS};

/// Runs forever. Owns the status LED, the inbound socket, and the IWDG — the
/// only place `IndependentWatchdog::pet()` is called in this crate.
pub async fn run(socket: UdpSocket<'static>, mut iwdg: IndependentWatchdog<'static, IWDG>, led_pin: Peri<'static, PB0>) -> ! {
    // LD1 on the Nucleo-144 (green). Doesn't collide with I2C1 (PB8/PB9), the
    // encoder EXTI pins (PA15/PB3/PB4/PB5), or any Ethernet RMII pin.
    let mut led = Output::new(led_pin, Level::Low, Speed::Low);

    let mut pet_ticker = Ticker::every(Duration::from_millis(IWDG_PET_INTERVAL_MS));
    let link_timeout = Duration::from_millis(LINK_TIMEOUT_MS);
    let mut last_seen = Instant::now();
    let mut tripped = false;
    let mut buf = [0u8; rover_msgs::frame::MAX_FRAME_LEN];

    iwdg.unleash();

    loop {
        match select(socket.recv_from(&mut buf), pet_ticker.next()).await {
            Either::First(Ok((n, _meta))) => {
                if let Ok(frame) = Frame::parse(&buf[..n]) {
                    if frame.header.type_id == Telemetry::TYPE_ID {
                        last_seen = Instant::now();
                        if tripped {
                            tripped = false;
                            led.set_low();
                        }
                    }
                    // Any other type_id addressed to this port: not currently
                    // possible per `[routes]` (see module doc), but if it
                    // ever is, it's not evidence of the RPi consuming this
                    // board's telemetry, so it doesn't reset the timer.
                }
            }
            Either::First(Err(_)) => {
                // Oversized or malformed datagram. Not link evidence either
                // way; keep serving.
            }
            Either::Second(()) => {
                // The only pet site in this crate. Runs every
                // IWDG_PET_INTERVAL_MS regardless of link state, which is
                // what makes it safe to pet from here — see module doc.
                iwdg.pet();

                if !tripped && last_seen.elapsed() >= link_timeout {
                    tripped = true;
                    warn!("link watchdog: no Telemetry seen for {}ms", LINK_TIMEOUT_MS);
                }
                if tripped {
                    led.toggle();
                }
            }
        }
    }
}
