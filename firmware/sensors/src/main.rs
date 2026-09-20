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
//! | [`diag::publish_task`] | 1 Hz | publishes [`rover_msgs::BoardDiagnostics`]: POST results, reset cause, PHY health |
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
//! `HealthBits::SENSORS_STALE` bit is how the RPi observes this absence:
//! `rover-telemetry` ages inbound `PowerSample` and raises the bit after
//! 500 ms (`crates/rover-telemetry/src/health.rs`'s `SENSORS_STALE_MS`).
#![no_std]
#![no_main]

mod config;
mod diag;
mod encoders;
mod net;
mod power;
mod watchdog;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_stm32::wdg::IndependentWatchdog;
use rover_msgs::{BoardId, PostBits};
use {defmt_rtt as _, panic_probe as _};

use crate::config::IWDG_TIMEOUT_US;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Must be the very first statement — see `diag::read_reset_cause`'s doc
    // comment for why nothing, not even `embassy_stm32::init`, may run
    // before this reads and latches `RCC_CSR`.
    let reset_cause = diag::read_reset_cause();

    let p = embassy_stm32::init(net::clock_config());

    info!("sensors-fw starting: {} MHz sysclk, ip=192.168.1.6", 216);
    info!("sensors-fw: reset cause = {}", reset_cause.name());

    let mut post = diag::Post::new();
    post.record(PostBits::CLOCK, diag::check_clock());

    // --- Network -----------------------------------------------------------
    let stack = net::init(
        spawner, p.ETH, p.ETH_SMA, p.RNG, p.PA1, p.PA2, p.PA7, p.PC1, p.PC4, p.PC5, p.PB13, p.PG11,
        p.PG13,
    );
    net::wait_up(stack).await;

    // By the time `wait_config_up` resolves, `Lan8742a::poll_link` has
    // already run at least once with the link reported up (see
    // `net::Lan8742a`'s doc comment and `embassy-net`'s own static-config
    // application, which is gated on link state) — so the PHY address scan
    // and register reads it does inline have already happened.
    let phy = net::phy_status();
    post.record(PostBits::PHY_ID, diag::check_phy_id(phy.phy_id));
    post.record(
        PostBits::LINK,
        phy.link_speed_mbps == 100 && phy.link_full_duplex,
    );
    post.record(PostBits::NET_BIND, diag::check_net_bind(stack));

    // --- Encoders ------------------------------------------------------------
    // Software 4x quadrature over EXTI, NOT hardware timer encoder mode — see
    // encoders.rs's module doc for why the latter is physically unavailable
    // on this pin assignment.
    let enc = encoders::init(
        p.EXTI3, p.PB3, p.EXTI4, p.PB4, p.EXTI5, p.PB5, p.EXTI15, p.PA15,
    );
    enc.spawn(spawner);
    // Runs before `publish_task` is spawned, and before anything else that
    // could move the wheels — see `encoders::post_idle_check`'s doc comment
    // for exactly what this does and does not prove.
    post.record(PostBits::SENSOR_B, encoders::post_idle_check().await);
    spawner.spawn(defmt::unwrap!(encoders::publish_task(stack)));

    // --- Power monitor -------------------------------------------------------
    let (power_dev, power_id_ok) = power::init(p.I2C1, p.PB8, p.PB9);
    post.record(PostBits::SENSOR_A, power_id_ok);
    match power_dev {
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
    // `IndependentWatchdog::unleash()` (called at the top of `watchdog::run`,
    // below) is a single unconditional register write with no failure mode
    // the STM32F7 exposes back to software — there is no IWDG status
    // register that reports "started", only PVU/RVU/WVU busy flags for the
    // separate prescaler/reload-write handshake. So this bit means "the
    // enable sequence will unconditionally be issued", not an independently
    // verified hardware confirmation that the counter is running - recorded
    // here, before the call, because `watchdog::run` never returns and this
    // is the last point at which anything else in `main` still executes.
    post.record(PostBits::IWDG, true);

    spawner.spawn(defmt::unwrap!(diag::publish_task(
        stack,
        BoardId::Sensors,
        reset_cause,
        post.run,
        post.pass,
    )));

    info!(
        "sensors-fw ready: encoders + power live, link watchdog {} ms",
        config::LINK_TIMEOUT_MS
    );
    watchdog::run(socket, iwdg, p.PB0).await
}
