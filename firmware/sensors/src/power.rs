//! INA226 power monitor over I2C, and the 5 Hz publish task.
//!
//! # I2C bus, address, and clock — unchanged from `power_monitor.cpp`
//!
//! Pins `PB8` (SCL) / `PB9` (SDA) at 400 kHz, address `0x40` — identical
//! wiring and identical crystal to `power_monitor.h`'s documented hardware
//! (`I2C_SDA`/`I2C_SCL` = PB9/PB8, `INA226_I2C_ADDR = 0x40`). Same caveat as
//! `firmware/chassis/src/imu.rs`'s I2C1 note: this is the standard Nucleo-144
//! Arduino I2C bus, not something rover-specific, but it has not been
//! confirmed against the physical board as part of this change.
//!
//! # Conversion: shunt Ohm's law, not the driver's calibration register
//!
//! The `ina226` crate's `callibrate()`/`current_amps()` path needs an
//! expected max current to size its internal fixed-point calibration
//! register — a number this port has no measured or datasheet value for.
//! Instead this module reads `shunt_voltage_microvolts()` and
//! `bus_voltage_millivolts()` directly and computes
//! `current = v_shunt / r_shunt` in software, exactly the formula
//! `power_monitor_read_current()` used. This is a literal port of the
//! existing, field-proven conversion rather than a switch to an unconfigured
//! internal register whose default calibration this board has never used.
use defmt::warn;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{I2C1, PB8, PB9};
use embassy_stm32::time::khz;
use embassy_stm32::Peri;
use embassy_time::{Duration, Ticker};
use ina226::{Config, INA226, AVG, MODE, VBUSCT, VSHCT};
use rover_msgs::{encode_frame, PowerSample, Wire};

use crate::config::{INA226_ADDR, POWER_PUBLISH_HZ, RPI_IP, RPI_PORT, SHUNT_OHMS};

/// `embassy_stm32::i2c::I2c` implements `embedded_hal::i2c::I2c` directly
/// (unlike the LSM6DSV16X driver on the chassis board, the INA226 crate needs
/// no vendor bus-wrapper type), so this alias is just the concrete I2C type.
pub type Power = INA226<I2c<'static, Blocking, Master>>;

/// Bring up I2C1 at 400 kHz and put the INA226 into continuous shunt+bus
/// conversion mode explicitly.
///
/// Explicit rather than relying on the power-on-reset default: the POR
/// default (`0x4127`) already decodes to continuous shunt+bus mode with 1x
/// averaging, so this is close to a no-op on a freshly powered device — but
/// a device left mid-triggered-conversion by a previous debug session (the
/// crate exposes `set_configuration` for exactly that kind of manual probing)
/// should not silently keep reading stale data forever after a firmware
/// restart that didn't power-cycle the sensor.
pub fn init(i2c1: Peri<'static, I2C1>, scl: Peri<'static, PB8>, sda: Peri<'static, PB9>) -> Option<Power> {
    let mut cfg = embassy_stm32::i2c::Config::default();
    cfg.frequency = khz(400);
    let i2c = I2c::new_blocking(i2c1, scl, sda, cfg);

    let mut dev = INA226::new(i2c, INA226_ADDR);

    let config = Config {
        avg: AVG::_1,
        vbusct: VBUSCT::_1100us,
        vshct: VSHCT::_1100us,
        mode: MODE::ShuntBusVoltageContinuous,
    };
    if dev.set_configuration(&config).is_err() {
        warn!("power: INA226 configuration write failed");
        return None;
    }

    Some(dev)
}

/// Publish [`PowerSample`] at [`POWER_PUBLISH_HZ`] (5 Hz) — the rate the
/// ROS 2 firmware always sampled the INA226 at, now published on its own
/// instead of being bundled into the 4 Hz `ChassisSensors` message.
#[embassy_executor::task]
pub async fn publish_task(mut power: Power, stack: embassy_net::Stack<'static>) -> ! {
    let socket = crate::tx_socket!(stack);
    let mut seq: u16 = 0;
    let mut buf = [0u8; rover_msgs::frame::FRAME_HEADER_LEN + PowerSample::WIRE_LEN];
    let mut ticker = Ticker::every(Duration::from_hz(POWER_PUBLISH_HZ as u64));

    loop {
        ticker.next().await;

        // `PowerSample` carries no timestamp field (see `rover-msgs`) — the
        // 5 Hz `Ticker` cadence itself is the timing reference consumers use.
        let (bus_mv, shunt_uv) = match (power.bus_voltage_millivolts(), power.shunt_voltage_microvolts()) {
            (Ok(bus_mv), Ok(shunt_uv)) => (bus_mv, shunt_uv),
            _ => {
                warn!("power: I2C read failed, skipping sample");
                continue;
            }
        };

        let sample = PowerSample {
            bus_volts: (bus_mv * 1.0e-3) as f32,
            current_amps: ((shunt_uv * 1.0e-6) / SHUNT_OHMS as f64) as f32,
        };

        let n = encode_frame(&sample, seq, &mut buf);
        seq = seq.wrapping_add(1);
        let _ = socket.send_to(&buf[..n], (RPI_IP, RPI_PORT)).await;
    }
}

// Compile-time proof the shared tx_socket! buffer can hold this message.
const _: () = assert!(PowerSample::WIRE_LEN + rover_msgs::FRAME_HEADER_LEN <= 128);
