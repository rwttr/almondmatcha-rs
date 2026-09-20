//! Chassis board firmware — NUCLEO-F767ZI at 192.168.1.2.
//!
//! Replaces `mros2-mbed-chassis-dynamics`. Nothing of the mROS 2 /
//! embeddedRTPS stack survives: no DDS, no discovery, no lwIP, no mbed OS,
//! no CMake, no Docker. This is `cargo build` and `probe-rs run`.
//!
//! # Tasks
//!
//! | Task | Rate | Job |
//! |---|---|---|
//! | [`watchdog::run`] | event + 200 ms timeout | owns the motors, applies commands, trips safe, pets the IWDG |
//! | [`imu::task`] | 100 Hz | reads the LSM6DSV16X, publishes [`ImuSample`] |
//! | [`rx_task`] | as they arrive | decodes [`ChassisCommand`] datagrams into the command mailbox |
//! | [`status_task`] | 5 Hz | publishes [`ChassisStatus`] so the watchdog is observable |
//! | [`diag::publish_task`] | 1 Hz | publishes [`rover_msgs::BoardDiagnostics`]: POST results, reset cause, PHY health |
//! | `net_task` | — | drives the Ethernet interface (spawned inside [`net::init`]) |
//!
//! # Why the watchdog task owns the motors outright
//!
//! Nothing else in this image can touch a PWM channel. The motors are moved
//! into [`watchdog::run`] and never come back, so there is no code path —
//! present or future — by which a command reaches the drivetrain without
//! passing the timeout check first. The ROS 2 firmware's failure was
//! structural rather than a missing `if`: any task could drive the motors,
//! and none was responsible for noticing that commands had stopped.
//!
//! # Startup order
//!
//! Network first, then sensors, then actuation. The board must be able to
//! report a failed IMU init over UDP, which means the stack has to be up
//! before the thing that might fail. The drivetrain comes last and starts
//! tripped: the watchdog's first `select` cannot resolve before either a
//! real command arrives or the 200 ms timeout fires, so an unattended board
//! sits with zero throttle rather than whatever the pins powered up as.

#![no_std]
#![no_main]

mod config;
mod diag;
mod imu;
mod motor;
mod net;
mod watchdog;

use core::sync::atomic::Ordering;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_net::udp::UdpSocket;
use embassy_net::Stack;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_time::{Duration, Timer};
use rover_msgs::{
    frame::encode_frame, BoardId, ChassisCommand, ChassisStatus, FaultBits, Frame, PostBits, Wire,
};

use {defmt_rtt as _, panic_probe as _};

use crate::config::{CHASSIS_STATUS_DEST, IWDG_TIMEOUT_US, STATUS_PUBLISH_HZ};
use crate::watchdog::{CMD_SIGNAL, STATUS};

/// Marks the IMU as absent in [`ChassisStatus::fault`].
///
/// Set once at boot rather than polled: an LSM6DSV16X that fails its
/// `WHO_AM_I` is not coming back without a power cycle, and a board that
/// silently publishes no [`ImuSample`] is indistinguishable, from the RPi's
/// side, from a board that has stopped entirely.
fn mark_imu_lost() {
    STATUS
        .fault
        .fetch_or(FaultBits::IMU_LOST.0, Ordering::Relaxed);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Must be the very first statement — see `diag::read_reset_cause`'s doc
    // comment for why nothing, not even `embassy_stm32::init`, may run
    // before this reads and latches `RCC_CSR`.
    let reset_cause = diag::read_reset_cause();

    let p = embassy_stm32::init(net::clock_config());

    info!("chassis-fw starting: {} MHz sysclk, ip=192.168.1.2", 216);
    info!("chassis-fw: reset cause = {}", reset_cause.name());

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

    // --- Sensors -----------------------------------------------------------
    match imu::init(p.I2C1, p.PB8, p.PB9) {
        Some(sensor) => {
            post.record(PostBits::SENSOR_A, true);
            spawner.spawn(defmt::unwrap!(imu::task(sensor, stack)));
        }
        None => {
            // Not fatal. A chassis board that cannot read its IMU can still
            // drive and still stop safely, and the rover is better off being
            // steerable with a degraded estimator than dead in a field. The
            // fault bit tells the RPi to stop trusting gyro-based coasting.
            warn!("IMU init failed — continuing without it, FaultBits::IMU_LOST set");
            mark_imu_lost();
            post.record(PostBits::SENSOR_A, false);
        }
    }

    // --- Command receive + status --------------------------------------------
    let socket = net::make_rx_socket(stack);
    spawner.spawn(defmt::unwrap!(rx_task(socket)));
    spawner.spawn(defmt::unwrap!(status_task(stack)));

    // --- Actuation ----------------------------------------------------------
    let mut motors = motor::Motors::new(
        p.TIM2, p.PA3, p.TIM3, p.PA6, p.TIM1, p.PE11, p.PF12, p.PD15, p.PF13, p.PE9,
    );
    let motor_post = motors.post_check();
    post.record(PostBits::SENSOR_B, motor_post.motor_pwm_ok);
    post.record(PostBits::SENSOR_C, motor_post.servo_pwm_ok);

    // `IndependentWatchdog` is clocked from the LSI, which keeps running
    // through a hung task, a spinning interrupt, or a deadlocked bus — which
    // is exactly the point. It is unleashed inside `watchdog::run` and petted
    // only there.
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
        BoardId::Chassis,
        reset_cause,
        post.run,
        post.pass,
    )));

    info!(
        "chassis-fw ready: motors armed, watchdog {} ms",
        config::CMD_TIMEOUT_MS
    );
    watchdog::run(motors, iwdg).await
}

/// Decode inbound [`ChassisCommand`] datagrams into the command mailbox.
///
/// Deliberately does no validation beyond decoding: clamping, safety limits
/// and the timeout all live in [`watchdog::run`], which is the single place
/// allowed to touch the motors. A receive task that could also veto commands
/// would be a second, quieter safety policy to keep in step with the first.
#[embassy_executor::task]
async fn rx_task(socket: UdpSocket<'static>) -> ! {
    let mut buf = [0u8; 256];
    loop {
        let Ok((n, _meta)) = socket.recv_from(&mut buf).await else {
            // embassy-net surfaces an oversized datagram as an error rather
            // than truncating. Nothing this board receives is anywhere near
            // the buffer size, so this means something is wrong upstream —
            // log it and keep serving rather than tearing the task down.
            warn!("rx: recv_from failed");
            continue;
        };

        let Ok(frame) = Frame::parse(&buf[..n]) else {
            warn!("rx: runt datagram, {} bytes", n);
            continue;
        };

        match frame.decode_as::<ChassisCommand>() {
            Some(Ok(cmd)) => {
                // Clamp at the boundary: a controller under development will
                // produce out-of-range values, and the firmware must not be
                // the last thing standing between a bad gain and the gearbox.
                CMD_SIGNAL.signal(cmd.clamped());
            }
            Some(Err(e)) => warn!("rx: malformed ChassisCommand: {}", defmt::Debug2Format(&e)),
            // Some other message type addressed to this port. Not an error —
            // ignore it and carry on.
            None => {}
        }
    }
}

/// Publish [`ChassisStatus`] at 5 Hz.
///
/// This exists because the ROS 2 firmware had no way to say "my command
/// watchdog tripped" — or that it had one. A trip in the field left no trace
/// beyond the rover stopping, which looks identical to a dozen other faults.
#[embassy_executor::task]
async fn status_task(stack: Stack<'static>) -> ! {
    let socket = tx_socket!(stack);
    let period = Duration::from_hz(STATUS_PUBLISH_HZ as u64);
    let mut seq: u16 = 0;
    let mut buf = [0u8; 64];

    loop {
        let status = ChassisStatus {
            seq_echo: STATUS.seq_echo.load(Ordering::Relaxed),
            watchdog_tripped: STATUS.watchdog_tripped.load(Ordering::Relaxed),
            fault: FaultBits(STATUS.fault.load(Ordering::Relaxed)),
            t_us: embassy_time::Instant::now().as_micros() as u32,
        };

        let n = encode_frame(&status, seq, &mut buf);
        seq = seq.wrapping_add(1);

        if socket
            .send_to(&buf[..n], CHASSIS_STATUS_DEST)
            .await
            .is_err()
        {
            // A full transmit buffer or an unreachable peer. Telemetry is
            // best-effort by design — the next frame is 200 ms away and
            // carries the same state, so dropping this one costs nothing.
            warn!("status: send failed");
        }

        Timer::after(period).await;
    }
}

/// Compile-time proof that the receive buffer can hold anything addressed to
/// this board. Cheaper than discovering it as a truncated datagram at dusk.
const _: () = assert!(ChassisCommand::WIRE_LEN + rover_msgs::FRAME_HEADER_LEN <= 256);
