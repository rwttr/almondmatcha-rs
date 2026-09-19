//! Steering servo and drive motor PWM.
//!
//! # What moved here from the wire
//!
//! The ROS 2 `ChassisCtrl` message carried H-bridge direction as two enum
//! fields (`fdr_msg` 1=right/2=straight/3=left, `bdr_msg` 0=stop/1=fwd/2=bwd)
//! alongside separate magnitude fields. `ChassisCommand` on the new wire
//! carries only `steer`/`throttle` in `[-1, 1]` - direction is a firmware
//! concern now, so this module derives it from the *sign* of each field,
//! restoring exactly the old `fdr`/`bdr` encoding before driving the same
//! hardware the old firmware drove.
//!
//! # Pin assignment - unchanged from `motor_control.h`/`motor_control.cpp`
//!
//! | Signal | Pin | Timer/channel |
//! |---|---|---|
//! | Steering servo PWM | `PA3` | TIM2 CH4 |
//! | Right motor PWM | `PA6` | TIM3 CH1 |
//! | Left motor PWM | `PE11` | TIM1 CH2 |
//! | Right motor forward enable | `PF12` | plain GPIO out |
//! | Right motor backward enable | `PD15` | plain GPIO out |
//! | Left motor forward enable | `PF13` | plain GPIO out |
//! | Left motor backward enable | `PE9` | plain GPIO out |
//!
//! The right motor's enable pins are wired to the opposite sense from the
//! left motor's (`apply_motor_control()`'s comment: "differential drive
//! requires opposite rotation") - both wheels spin the chassis the same way
//! round even though the motors are mirror-mounted. That swap is preserved
//! exactly below.
//!
//! TIM1 is an advanced-control timer (it has a break input and a main-output
//! enable gate, unlike TIM2/TIM3's general-purpose channels). `SimplePwm`
//! handles enabling its main output the same way as any other instance, but
//! this is the one PWM channel in the crate on an advanced timer, so if the
//! left motor alone fails to produce PWM on the bench while the right motor
//! and servo work, TIM1's break/MOE configuration is the first place to look.
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::peripherals::{PA3, PA6, PD15, PE11, PE9, PF12, PF13, TIM1, TIM2, TIM3};
use embassy_stm32::time::hz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::Peri;

use crate::config::{
    CMD_EPSILON, MOTOR_PWM_HZ, SERVO_CENTER_DEG, SERVO_DUTY_MAX, SERVO_DUTY_MIN, SERVO_PWM_HZ, STEER_MAX_DEG,
};

/// `fdr_msg` from the old wire, reconstructed from `sign(steer)`. Named
/// `SteerDir` rather than reusing `fdr`'s bare integers so the mapping is
/// checked by the compiler instead of by a comment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SteerDir {
    Right,
    Straight,
    Left,
}

/// `bdr_msg` from the old wire, reconstructed from `sign(throttle)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DriveDir {
    Forward,
    Backward,
    Stop,
}

fn steer_dir(steer: f32) -> SteerDir {
    if steer > CMD_EPSILON {
        SteerDir::Right
    } else if steer < -CMD_EPSILON {
        SteerDir::Left
    } else {
        SteerDir::Straight
    }
}

fn drive_dir(throttle: f32) -> DriveDir {
    if throttle > CMD_EPSILON {
        DriveDir::Forward
    } else if throttle < -CMD_EPSILON {
        DriveDir::Backward
    } else {
        DriveDir::Stop
    }
}

/// Owns every motor/servo GPIO and PWM timer, and holds the mapping from
/// `(steer, throttle)` to hardware exactly as `motor_control.cpp` did.
///
/// Each `SimplePwm` is kept whole (not reduced to just its channel handle) so
/// this struct is a plain value with no internal borrows: `ch1()`/`ch2()`/
/// `ch4()` are called fresh on `&mut self.<timer>` inside `set_steering`/
/// `set_drive` rather than cached, which sidesteps the self-referential
/// struct that caching a `SimplePwmChannel<'_, T>` would require.
pub struct Motors<'d> {
    servo_pwm: SimplePwm<'d, TIM2>,
    right_pwm: SimplePwm<'d, TIM3>,
    left_pwm: SimplePwm<'d, TIM1>,
    right_fwd: Output<'d>,
    right_bwd: Output<'d>,
    left_fwd: Output<'d>,
    left_bwd: Output<'d>,
}

impl<'d> Motors<'d> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        tim2: Peri<'d, TIM2>,
        servo_pin: Peri<'d, PA3>,
        tim3: Peri<'d, TIM3>,
        right_pwm_pin: Peri<'d, PA6>,
        tim1: Peri<'d, TIM1>,
        left_pwm_pin: Peri<'d, PE11>,
        right_fwd: Peri<'d, PF12>,
        right_bwd: Peri<'d, PD15>,
        left_fwd: Peri<'d, PF13>,
        left_bwd: Peri<'d, PE9>,
    ) -> Self {
        let servo_pwm_pin = PwmPin::new(servo_pin, OutputType::PushPull);
        let mut servo_pwm = SimplePwm::new(
            tim2,
            None,
            None,
            None,
            Some(servo_pwm_pin),
            hz(SERVO_PWM_HZ),
            Default::default(),
        );
        servo_pwm.ch4().enable();

        let right_pwm_pin = PwmPin::new(right_pwm_pin, OutputType::PushPull);
        let mut right_pwm = SimplePwm::new(
            tim3,
            Some(right_pwm_pin),
            None,
            None,
            None,
            hz(MOTOR_PWM_HZ),
            Default::default(),
        );
        right_pwm.ch1().enable();

        let left_pwm_pin = PwmPin::new(left_pwm_pin, OutputType::PushPull);
        let mut left_pwm = SimplePwm::new(
            tim1,
            None,
            Some(left_pwm_pin),
            None,
            None,
            hz(MOTOR_PWM_HZ),
            Default::default(),
        );
        left_pwm.ch2().enable();

        Self {
            servo_pwm,
            right_pwm,
            left_pwm,
            right_fwd: Output::new(right_fwd, Level::Low, Speed::Low),
            right_bwd: Output::new(right_bwd, Level::Low, Speed::Low),
            left_fwd: Output::new(left_fwd, Level::Low, Speed::Low),
            left_bwd: Output::new(left_bwd, Level::Low, Speed::Low),
        }
    }

    /// Apply a full command: derive direction from sign, magnitude from
    /// `abs()`, exactly as `motor_control_task` did after unpacking the old
    /// `fdr_msg`/`bdr_msg`/`ro_ctrl_msg`/`spd_msg` fields.
    pub fn apply(&mut self, steer: f32, throttle: f32) {
        self.set_steering(steer);
        self.set_drive(throttle);
    }

    /// Steering servo only. Used both for normal commands and to centre the
    /// servo independently of the throttle ramp on a watchdog trip.
    pub fn set_steering(&mut self, steer: f32) {
        let angle_deg = steer.clamp(-1.0, 1.0).abs() * STEER_MAX_DEG;
        let target_angle = match steer_dir(steer) {
            SteerDir::Right => SERVO_CENTER_DEG - angle_deg,
            SteerDir::Left => SERVO_CENTER_DEG + angle_deg,
            SteerDir::Straight => SERVO_CENTER_DEG,
        }
        .clamp(0.0, 180.0);

        let duty_fraction = SERVO_DUTY_MIN + (target_angle / 180.0) * (SERVO_DUTY_MAX - SERVO_DUTY_MIN);
        const DENOM: u32 = 10_000;
        let num = (duty_fraction.clamp(0.0, 1.0) * DENOM as f32) as u32;
        self.servo_pwm.ch4().set_duty_cycle_fraction(num, DENOM);
    }

    /// Drive motors only. Centre-on-trip / ramp-to-zero in `watchdog.rs` call
    /// this directly so the ramp can vary throttle without also touching the
    /// steering servo on every step.
    pub fn set_drive(&mut self, throttle: f32) {
        let duty_percent = (throttle.clamp(-1.0, 1.0).abs() * 100.0) as u8;

        let (enable_forward, enable_backward) = match drive_dir(throttle) {
            DriveDir::Forward => (true, false),
            DriveDir::Backward => (false, true),
            DriveDir::Stop => (false, false),
        };

        self.left_fwd.set_level(level(enable_forward));
        self.left_bwd.set_level(level(enable_backward));
        // Right motor: opposite sense from the left motor - see the module
        // doc comment and `apply_motor_control()`'s "OPPOSITE direction"
        // comment in the old firmware.
        self.right_fwd.set_level(level(enable_backward));
        self.right_bwd.set_level(level(enable_forward));

        self.right_pwm.ch1().set_duty_cycle_percent(duty_percent);
        self.left_pwm.ch2().set_duty_cycle_percent(duty_percent);
    }
}

fn level(high: bool) -> Level {
    if high {
        Level::High
    } else {
        Level::Low
    }
}
