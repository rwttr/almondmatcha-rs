//! LSM6DSV16X IMU: sampling, unit conversion, and the 100 Hz publish task.
//!
//! # Rate: 100 Hz, not 10 Hz
//!
//! The ROS 2 firmware's `imu_reader_task` polled the sensor every 10 ms (100
//! Hz) but only published every tenth sample (`IMU_PUBLISH_INTERVAL = 10`),
//! i.e. 10 Hz on the wire. The plan calls that out directly: the EKF wants
//! every sample, and the board was already producing them for free. This
//! task publishes every sample it reads.
//!
//! # Units: real SI on the wire, not raw LSBs
//!
//! The ROS 2 `ChassisIMU` message shipped `int32` fields holding raw sensor
//! LSBs "nominally scaled by 1000" - a conversion that, per the plan, no
//! consumer ever actually applied. `ImuSample.accel_mps2`/`gyro_radps` are
//! real m/s^2 and rad/s, converted here using the driver's own
//! `from_fs4_to_mg`/`from_fs500_to_mdps` sensitivity constants for the
//! ±4 g / ±500 dps full-scale ranges below.
//!
//! # I2C bus and address
//!
//! Pins `PB8` (SCL) / `PB9` (SDA) at 400 kHz - the mbed target's generic
//! `I2C_SDA`/`I2C_SCL` names for `NUCLEO_F767ZI` resolve to these pins (the
//! Arduino-header D14/D15 I2C bus the X-NUCLEO-IKS4A1 shield plugs into).
//! This is the standard Nucleo-144 Arduino I2C bus, not something specific
//! to this rover, but it has not been confirmed against the physical board
//! as part of this change - flagging alongside the PHY as something to
//! check on the bench. Address `I2cAddH` (0x6B, SA0 pulled high) matches the
//! X-NUCLEO-IKS4A1's default strapping.
use defmt::{info, warn};
use embassy_stm32::i2c::I2c;
use embassy_stm32::i2c::Master;
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{I2C1, PB8, PB9};
use embassy_stm32::time::khz;
use embassy_stm32::Peri;
use embassy_time::{Delay, Duration, Instant, Ticker};
// NOTE: v1.0.0 keeps the register enums (`Odr`, `XlFullScale`, ...) behind
// `prelude`, not at the crate root, and `Lsm6dsv16x` takes two type
// parameters rather than three — the `MemBank` parameter arrived in v2.x.
// See the version note in Cargo.toml for why this crate is pinned to v1.
use lsm6dsv16x_rs::prelude::*;
use lsm6dsv16x_rs::{from_fs4_to_mg, from_fs500_to_mdps, I2CAddress, Lsm6dsv16x};
use rover_msgs::{ImuSample, Wire};
use st_mems_bus::i2c::I2cBus;

use crate::config::{IMU_PUBLISH_HZ, STANDARD_GRAVITY};

/// mg (milli-g) -> m/s^2.
fn mg_to_mps2(mg: f32) -> f32 {
    mg * 1.0e-3 * STANDARD_GRAVITY
}

/// mdps (milli-degrees/s) -> rad/s.
fn mdps_to_radps(mdps: f32) -> f32 {
    mdps * 1.0e-3 * core::f32::consts::PI / 180.0
}

// embassy-stm32 0.6's `I2c` carries a master-mode parameter alongside its
// mode, hence `Master` here; the driver wraps it in st-mems-bus's `I2cBus`.
pub type Imu = Lsm6dsv16x<I2cBus<I2c<'static, Blocking, Master>>, Delay>;

/// Bring up the I2C bus and the sensor: reset, wait for it to come back,
/// enable block-data-update (so a read can never straddle a register update
/// mid-conversion), and set the ODR/full-scale from the plan's §5.1 sketch.
pub fn init(
    i2c1: Peri<'static, I2C1>,
    scl: Peri<'static, PB8>,
    sda: Peri<'static, PB9>,
) -> Option<Imu> {
    let mut cfg = embassy_stm32::i2c::Config::default();
    cfg.frequency = khz(400);
    let i2c = I2c::new_blocking(i2c1, scl, sda, cfg);

    let mut imu = Lsm6dsv16x::new_i2c(i2c, I2CAddress::I2cAddH, Delay);

    let id = imu.device_id_get().ok()?;
    if id != lsm6dsv16x_rs::ID {
        warn!("imu: unexpected WHO_AM_I 0x{:x}", id);
        return None;
    }

    imu.reset_set(Reset::RestoreCtrlRegs).ok()?;
    // Reset is a register write the sensor services in the background;
    // there is no interrupt for "done", so poll `reset_get` per the vendor
    // driver's own example. This blocks the caller (board bring-up, before
    // the executor's other tasks are spawned), which is fine here.
    loop {
        match imu.reset_get() {
            Ok(Reset::Ready) => break,
            Ok(_) => continue,
            Err(_) => return None,
        }
    }

    imu.block_data_update_set(1).ok()?;
    imu.xl_data_rate_set(Odr::_120hz).ok()?;
    imu.gy_data_rate_set(Odr::_120hz).ok()?;
    imu.xl_full_scale_set(XlFullScale::_4g).ok()?;
    imu.gy_full_scale_set(GyFullScale::_500dps).ok()?;

    info!("imu: LSM6DSV16X id=0x{:x} ready", id);
    Some(imu)
}

/// Publish `ImuSample` at `IMU_PUBLISH_HZ` for as long as the sensor keeps
/// answering. A read failure is logged and skipped rather than treated as
/// fatal - a single missed I2C transaction should not take the whole board
/// down, only cost one sample.
#[embassy_executor::task]
pub async fn task(mut imu: Imu, stack: embassy_net::Stack<'static>) -> ! {
    let socket = crate::tx_socket!(stack);
    let mut seq: u16 = 0;
    let mut buf = [0u8; rover_msgs::frame::FRAME_HEADER_LEN + ImuSample::WIRE_LEN];

    // `Ticker`, not `Timer::after`: the latter measures from the moment it is
    // awaited, so every iteration's work (an I2C round trip, a UDP send)
    // accumulates into the period and the task slowly falls behind 100 Hz.
    // A ticker keeps a fixed cadence and drops a tick if one is genuinely
    // missed, which is what the estimator's dt handling expects.
    let mut ticker = Ticker::every(Duration::from_hz(IMU_PUBLISH_HZ as u64));

    loop {
        ticker.next().await;

        let (accel_raw, gyro_raw) = match (imu.acceleration_raw_get(), imu.angular_rate_raw_get()) {
            (Ok(a), Ok(g)) => (a, g),
            _ => {
                warn!("imu: read failed, skipping sample");
                continue;
            }
        };

        let sample = ImuSample {
            accel_mps2: [
                mg_to_mps2(from_fs4_to_mg(accel_raw[0])),
                mg_to_mps2(from_fs4_to_mg(accel_raw[1])),
                mg_to_mps2(from_fs4_to_mg(accel_raw[2])),
            ],
            gyro_radps: [
                mdps_to_radps(from_fs500_to_mdps(gyro_raw[0])),
                mdps_to_radps(from_fs500_to_mdps(gyro_raw[1])),
                mdps_to_radps(from_fs500_to_mdps(gyro_raw[2])),
            ],
            // A real clock reading, taken as close to the sensor read as
            // practical — NOT a counter advanced by the nominal period.
            //
            // A synthetic timestamp would report perfectly periodic sampling
            // no matter what actually happened: a retried I2C transaction, a
            // skipped sample, a ticker that lost a tick. The estimator scales
            // its process noise by the real dt precisely so that a late
            // sample widens the covariance instead of being silently trusted
            // as if it were on time, and it can only do that if this number
            // tells the truth.
            //
            // Wraps every ~71.6 minutes at microsecond resolution; consumers
            // difference consecutive values with wrapping arithmetic, which
            // is correct as long as samples are closer together than that.
            t_us: Instant::now().as_micros() as u32,
        };

        let n = rover_msgs::encode_frame(&sample, seq, &mut buf);
        seq = seq.wrapping_add(1);
        let _ = socket
            .send_to(&buf[..n], crate::config::IMU_SAMPLE_DEST)
            .await;
    }
}
