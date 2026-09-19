//! Sensors board firmware — NUCLEO-F767ZI at 192.168.1.6.
//!
//! Replaces `mros2-mbed-sensors-gnss`. As with `firmware/chassis`, nothing of
//! the mROS 2 / embeddedRTPS stack survives.
//!
//! # Tasks
//!
//! | Task | Rate | Job |
//! |---|---|---|
//! | [`encoders::left_task`] / [`right_task`] | per edge | software 4x quadrature decode, see `encoders.rs` |
//! | [`encoders::publish_task`] | 10 Hz | publishes [`WheelSensors`] |
//! | [`power::publish_task`] | 5 Hz | reads the INA226, publishes [`PowerSample`] |
//! | [`watchdog::run`] | event + 200 ms pet tick | mirror link watchdog + the one IWDG pet site |
//! | `net_task` | — | drives the Ethernet interface (spawned inside [`net::init`]) |
//!
//! # Deleted: the GNSS reader
//!
//! The ROS 2 firmware ran `gnss_reader_task` on USART6 with a 4 KB stack,
//! reading NMEA sentences from the SimpleRTK2b into `sensor_data.nmea_sentence`
//! — a buffer that was **never published**. It appeared only in a heartbeat
//! `printf` every 2 seconds, while the RPi reads the same u-blox receiver
//! directly on `/dev/ttyACM0`. ~260 lines plus a thread serving a debug print,
//! on the most memory-constrained node in the system. It is gone, per the
//! plan §5.3 — not ported, not stubbed.
//!
//! # Startup order
//!
//! Network first (so a sensor init failure could in principle be reported —
//! though unlike chassis this board has no fault-bit wire type, see below),
//! then encoders and power, then the link watchdog last, because it owns the
//! IWDG: nothing pets the watchdog until every other task is already spawned
//! and running, so a hang during any earlier init step is caught by the IWDG
//! rather than left to spin forever with the watchdog already ticking down.
//!
//! # No `SensorsStatus` wire type
//!
//! Unlike chassis's `ChassisStatus`, there is no fault-report message for
//! this board in `rover-msgs` (out of scope to add — see the task brief: no
//! changes to `rover-msgs`). A failed INA226 init is logged over defmt and
//! the board simply stops publishing `PowerSample`; `WheelSensors` keeps
//! flowing regardless, since the two sensors are independent. The RPi-side
//! `HealthBits::SENSORS_STALE` bit is the intended way to observe this
//! absence, once `rover-telemetry` tracks per-message liveness.
#![no_std]
#![no_main]

mod config;
mod encoders;
mod net;
mod power;
mod watchdog;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_stm32::wdg::IndependentWatchdog;
use {defmt_rtt as _, panic_probe as _};

use crate::config::IWDG_TIMEOUT_US;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(net::clock_config());

    info!("sensors-fw starting: {} MHz sysclk, ip=192.168.1.6", 216);

    // --- Network -----------------------------------------------------------
    let stack = net::init(
        spawner, p.ETH, p.ETH_SMA, p.RNG, p.PA1, p.PA2, p.PA7, p.PC1, p.PC4, p.PC5, p.PB13, p.PG11,
        p.PG13,
    );
    net::wait_up(stack).await;

    // --- Encoders ------------------------------------------------------------
    // Software 4x quadrature over EXTI, NOT hardware timer encoder mode — see
    // encoders.rs's module doc for why the latter is physically unavailable
    // on this pin assignment.
    let enc = encoders::init(p.EXTI3, p.PB3, p.EXTI4, p.PB4, p.EXTI5, p.PB5, p.EXTI15, p.PA15);
    enc.spawn(spawner);
    spawner.spawn(defmt::unwrap!(encoders::publish_task(stack)));

    // --- Power monitor -------------------------------------------------------
    match power::init(p.I2C1, p.PB8, p.PB9) {
        Some(dev) => {
            spawner.spawn(defmt::unwrap!(power::publish_task(dev, stack)));
        }
        None => {
            // Not fatal, and there is no fault-bit wire type on this board to
            // report it with (see the module doc) — WheelSensors is
            // independent of the power rail and keeps flowing.
            warn!("power: INA226 init failed — continuing without PowerSample");
        }
    }

    // --- Link watchdog + IWDG -------------------------------------------------
    let socket = net::make_rx_socket(stack);
    let iwdg = IndependentWatchdog::new(p.IWDG, IWDG_TIMEOUT_US);

    info!("sensors-fw ready: encoders + power live, link watchdog {} ms", config::LINK_TIMEOUT_MS);
    watchdog::run(socket, iwdg, p.PB0).await
}
