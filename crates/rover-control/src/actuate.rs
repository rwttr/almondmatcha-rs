//! Stage 3: actuate. Owns every guard rail so that no controller — the
//! ported `StaticGain` today, an experimental `Lqr`/`Mpc` tomorrow — can
//! reach the hardware unmediated (plan §8.3, "controllers propose,
//! actuation disposes"):
//!
//! - [`SteerLimiter`]: `steer_max_deg` saturation and
//!   `steer_slew_deg_per_s` slew limiting.
//! - [`SpeedController`]: the closed-loop speed PID ported from
//!   `chassis_controller_node.cpp`, including its auto-calibration of
//!   `max_ticks_per_sec` and stall detection — every threshold is carried
//!   over exactly (plan §10).
//! - [`SafetyGate`]: emergency stop on `MissionStatus.active` going false
//!   and on `Command::EStop`.
//! - [`confidence_speed_scale`]: speed derating as the estimator's own
//!   uncertainty grows, replacing the ROS2 `speed_lost_ratio` /
//!   `detection_timeout_sec` cliff (plan §2.1 item 3, deviation 5 in §13.3).
//! - [`Actuator`]: wires all of the above together into one
//!   `RoverState + steer_rad -> ChassisCommand` step, plus the wheel-tick
//!   differencing the speed loop needs.
//!
//! # The `metres_per_tick == 0.0` trap, speed edition
//!
//! `rover_estimator` disables its odometry update when
//! `drivetrain.metres_per_tick` is `0.0` (uncalibrated, plan §2.6) rather
//! than dividing by it. The speed loop has the same trap from the other
//! direction: `speed.reference_mps` is specified in **metres per second**,
//! but the PID itself — ported unchanged from the ROS2 node — operates
//! entirely in **percent of `max_ticks_per_sec`**, exactly like
//! `chassis_controller_node.cpp`'s `target_speed_pct_`/`measured_pct`. Converting
//! a physical m/s target into that percent domain needs `metres_per_tick`,
//! which is currently `0.0` everywhere this system runs (plan §13.5,
//! standing blocker). [`SpeedController::target_duty_pct`] refuses to invent
//! a scale factor from a missing calibration constant — that would be
//! exactly the kind of silent wrong-by-a-constant-factor bug plan §2.6
//! warns about for the 2x/4x encoder decoding change — and instead falls
//! back to [`SpeedController::UNCALIBRATED_FALLBACK_DUTY_PCT`], logging a
//! one-time warning so the degradation is loud, not silent, matching the
//! pattern `rover_estimator::Ekf::correct_wheel_sensors` already
//! establishes for its own instance of this same trap.

use crate::config::{AutocalConfig, StallConfig};
use rover_msgs::{ChassisCommand, Command, RoverState, SpeedLoopDebug, WheelSensors};

// ===========================================================================
// Steering guard rail
// ===========================================================================

/// `steer_max_deg` saturation and `steer_slew_deg_per_s` slew limiting,
/// applied after every controller regardless of which law produced the raw
/// command (plan §8.3). Works in degrees internally — the same unit the
/// ROS 2 law and `config/rover.toml`'s `[control]` table use — and the
/// caller ([`Actuator`]) normalises to `ChassisCommand::steer`'s `[-1, 1]`
/// range as the very last step.
pub struct SteerLimiter {
    max_deg: f32,
    slew_deg_per_s: f32,
    prev_deg: f32,
}

impl SteerLimiter {
    pub fn new(max_deg: f32, slew_deg_per_s: f32) -> Self {
        Self {
            max_deg,
            slew_deg_per_s,
            prev_deg: 0.0,
        }
    }

    /// Saturate, then slew-limit the change from the previously returned
    /// value. Saturating before slewing (rather than after) means the slew
    /// budget is never spent moving toward a command that was going to be
    /// clamped away anyway.
    pub fn apply(&mut self, cmd_deg: f32, dt_s: f32) -> f32 {
        let clamped = cmd_deg.clamp(-self.max_deg, self.max_deg);
        let max_step = self.slew_deg_per_s * dt_s.max(0.0);
        let stepped = clamped.clamp(self.prev_deg - max_step, self.prev_deg + max_step);
        self.prev_deg = stepped;
        stepped
    }

    /// Snap immediately to `deg` with no slew — used to centre the steering
    /// the instant driving stops being permitted (E-stop, mission inactive),
    /// matching `chassis_controller_node.cpp`'s `packAndPublishChassisCtrl`,
    /// which zeroes `ro_ctrl_msg` outright rather than ramping it down. A
    /// latched steering angle turning a runaway into a circle (plan §5.2)
    /// is exactly the failure mode a *slewed* centre during an emergency
    /// would risk leaving briefly uncorrected.
    pub fn force_to(&mut self, deg: f32) {
        self.prev_deg = deg.clamp(-self.max_deg, self.max_deg);
    }
}

// ===========================================================================
// Speed loop — ported from chassis_controller_node.cpp
// ===========================================================================

/// The closed-loop speed PID, its auto-calibration of `max_ticks_per_sec`,
/// and its stall latch. A direct port of
/// `ChassisController::chassisSensorsCallback` and its two helpers
/// (`updateAutoCalibration`, `updateStallDetection`) — see that file for the
/// field-tuning rationale behind every threshold; the comments here only
/// restate what changed in translation.
pub struct SpeedController {
    kp: f32,
    ki: f32,
    kd: f32,
    integral_limit: f32,
    max_duty_step_pct: f32,
    autocal: AutocalConfig,
    stall: StallConfig,

    integral: f32,
    last_error: f32,
    /// Cached output of the last PID step, in percent duty. This is what
    /// `Actuator::tick_chassis_command` reads between the ~10 Hz
    /// `WheelSensors` updates that actually refresh it — mirroring
    /// `speed_pid_output_pct_` in the ROS2 node, read at 50 Hz but written
    /// at the encoder rate.
    output_pct: f32,

    /// Encoder ticks/sec observed at 100% commanded duty on flat ground.
    /// Starts at the ROS2 system's own documented placeholder and is
    /// overwritten once, permanently, by [`Self::update_autocal`].
    max_ticks_per_sec: f32,
    autocal_samples: Vec<f32>,
    autocal_window_open: bool,
    autocal_window_start_s: f64,
    autocal_done: bool,
    autocal_prev_duty: f32,

    stall_latched: bool,
    stall_timing: bool,
    stall_since_s: f64,

    /// What was actually sent as `ChassisCommand.throttle` last tick — not
    /// the pre-cap request. Auto-calibration and stall detection both
    /// compare measured motion against this, exactly like
    /// `last_commanded_duty_pct_` in the ROS2 node.
    last_commanded_duty_pct: f32,

    overspeed_strikes: u32,
    uncalibrated_warned: bool,
}

impl SpeedController {
    /// `max_ticks_per_sec: 1000.0` — the ROS2 system's own
    /// `chassis_speed_control_params.yaml` default, explicitly labelled
    /// `PLACEHOLDER` there (plan §2.6, §10). Auto-calibration overwrites it
    /// once real driving data is available; until then the speed loop runs
    /// exactly as open-loop-flavoured as the ROS2 system did before its
    /// first calibrated run.
    const MAX_TICKS_PER_SEC_PLACEHOLDER: f32 = 1000.0;

    /// Fallback duty percentage used by [`Self::target_duty_pct`] while
    /// `drivetrain.metres_per_tick` is `0.0` (uncalibrated). `16.0` is not
    /// invented: it is `speed_ref` from `docs/CONTROL_LAW.md` §2.1's Stage-1
    /// parameter table — "16% nominal cruise duty when lane detected" — the
    /// last value this rover is documented to have actually cruised at. It
    /// sits comfortably above `autocal_min_duty_pct` (13.0) so a bench run
    /// with no calibration yet still collects auto-calibration samples,
    /// and far below `limit_cap_pct` (40.0) so the safety ceiling still has
    /// room to correct for load.
    pub const UNCALIBRATED_FALLBACK_DUTY_PCT: f32 = 16.0;

    pub fn new(
        kp: f32,
        ki: f32,
        kd: f32,
        integral_limit: f32,
        max_duty_step_pct: f32,
        autocal: AutocalConfig,
        stall: StallConfig,
    ) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_limit,
            max_duty_step_pct,
            autocal,
            stall,
            integral: 0.0,
            last_error: 0.0,
            output_pct: 0.0,
            max_ticks_per_sec: Self::MAX_TICKS_PER_SEC_PLACEHOLDER,
            autocal_samples: Vec::new(),
            autocal_window_open: false,
            autocal_window_start_s: 0.0,
            autocal_done: false,
            autocal_prev_duty: 0.0,
            stall_latched: false,
            stall_timing: false,
            stall_since_s: 0.0,
            last_commanded_duty_pct: 0.0,
            overspeed_strikes: 0,
            uncalibrated_warned: false,
        }
    }

    /// Convert a physical cruise target into the percent-of-full-scale
    /// domain the PID actually runs in. See the module docs for the
    /// `metres_per_tick == 0.0` trap this guards against.
    pub fn target_duty_pct(&mut self, reference_mps: f32, metres_per_tick: f32) -> f32 {
        if metres_per_tick > 0.0 {
            let target_tps = reference_mps / metres_per_tick;
            (target_tps / self.max_ticks_per_sec * 100.0).clamp(0.0, 100.0)
        } else {
            if !self.uncalibrated_warned {
                log::warn!(
                    "rover-control: drivetrain.metres_per_tick is 0.0 (uncalibrated, plan §2.6) \
                     — cannot convert speed.reference_mps ({reference_mps:.3} m/s) into a \
                     physical duty target; falling back to a fixed {:.0}% duty (CONTROL_LAW.md \
                     §2.1's field cruise value) until §2.6 calibration is done.",
                    Self::UNCALIBRATED_FALLBACK_DUTY_PCT
                );
                self.uncalibrated_warned = true;
            }
            Self::UNCALIBRATED_FALLBACK_DUTY_PCT
        }
    }

    /// Reset the integrator/derivative memory and seed the cached output
    /// with `feedforward_pct` rather than zero, so the next transition back
    /// into closed loop is bumpless: at zero error the loop's own output
    /// would be exactly the feedforward term anyway. Matches `resetSpeedPid()`.
    pub fn reset_pid(&mut self, feedforward_pct: f32) {
        self.integral = 0.0;
        self.last_error = 0.0;
        self.output_pct = feedforward_pct.clamp(0.0, 100.0);
    }

    pub fn cached_output_pct(&self) -> f32 {
        self.output_pct
    }

    pub fn is_stall_latched(&self) -> bool {
        self.stall_latched
    }

    /// Record what was actually sent, for the *next* sample's
    /// auto-calibration/stall comparisons.
    pub fn set_last_commanded_duty(&mut self, pct: f32) {
        self.last_commanded_duty_pct = pct;
    }

    pub fn clear_stall_timing(&mut self) {
        self.stall_timing = false;
    }

    #[cfg(test)]
    pub fn max_ticks_per_sec(&self) -> f32 {
        self.max_ticks_per_sec
    }

    /// One `WheelSensors`-rate step: runs auto-calibration and stall
    /// detection (both unconditionally, like the source), then the PID.
    /// `target_duty_pct` is `target_speed_pct_` in the ROS2 source — the
    /// value `applySpeedSafetyCap` computed most recently.
    pub fn on_sample(
        &mut self,
        measured_left_tps: f32,
        measured_right_tps: f32,
        dt_s: f32,
        now_s: f64,
        target_duty_pct: f32,
    ) -> SpeedLoopDebug {
        let measured_tps = (measured_left_tps + measured_right_tps) * 0.5;

        if self.autocal.enabled && !self.autocal_done {
            self.update_autocal(now_s, measured_tps);
        }
        if self.stall.enabled {
            self.update_stall(now_s, measured_tps);
        }

        if target_duty_pct <= 0.0 {
            self.reset_pid(0.0);
            return SpeedLoopDebug {
                measured_left_tps,
                measured_right_tps,
                target_tps: 0.0,
                error_pct: 0.0,
                pid_output_pct: 0.0,
            };
        }

        let measured_pct = if self.max_ticks_per_sec > 1e-6 {
            measured_tps / self.max_ticks_per_sec * 100.0
        } else {
            0.0
        };
        let error_pct = target_duty_pct - measured_pct;
        let derivative = if dt_s > 0.0 {
            (error_pct - self.last_error) / dt_s
        } else {
            0.0
        };

        let integral_candidate =
            (self.integral + error_pct * dt_s).clamp(-self.integral_limit, self.integral_limit);
        let u_candidate = target_duty_pct
            + self.kp * error_pct
            + self.ki * integral_candidate
            + self.kd * derivative;

        // Conditional integration (anti-windup): stop accumulating once the
        // output is saturated and the error would only push it further out
        // of range.
        let would_wind_up =
            (u_candidate > 100.0 && error_pct > 0.0) || (u_candidate < 0.0 && error_pct < 0.0);
        if !would_wind_up {
            self.integral = integral_candidate;
        }
        self.last_error = error_pct;

        let u =
            target_duty_pct + self.kp * error_pct + self.ki * self.integral + self.kd * derivative;

        let desired = u.clamp(0.0, 100.0);
        let max_step = self.max_duty_step_pct;
        let stepped = desired.clamp(self.output_pct - max_step, self.output_pct + max_step);
        self.output_pct = stepped.clamp(0.0, 100.0);

        if measured_pct > 150.0 {
            self.overspeed_strikes += 1;
            if self.overspeed_strikes == 8 {
                log::warn!(
                    "rover-control: measured speed reads {measured_pct:.0}% of full scale — \
                     max_ticks_per_sec ({:.0}) looks too low for the real encoders",
                    self.max_ticks_per_sec
                );
            }
        } else {
            self.overspeed_strikes = 0;
        }

        SpeedLoopDebug {
            measured_left_tps,
            measured_right_tps,
            target_tps: target_duty_pct / 100.0 * self.max_ticks_per_sec,
            error_pct,
            pid_output_pct: self.output_pct,
        }
    }

    /// Learn `max_ticks_per_sec` from the flat opening stretch of a run.
    /// See `chassis_controller_node.cpp::updateAutoCalibration` for why p75
    /// of steady, above-deadband samples in a fixed opening window is the
    /// chosen estimator.
    fn update_autocal(&mut self, now_s: f64, measured_tps: f32) {
        let duty = self.last_commanded_duty_pct;
        let steady = (duty - self.autocal_prev_duty).abs() < f32::EPSILON;
        self.autocal_prev_duty = duty;

        if duty < self.autocal.min_duty_pct || measured_tps <= 0.0 {
            return;
        }

        if !self.autocal_window_open {
            self.autocal_window_open = true;
            self.autocal_window_start_s = now_s;
        }

        if steady {
            self.autocal_samples.push(measured_tps / (duty / 100.0));
        }

        if now_s - self.autocal_window_start_s < self.autocal.window_s as f64 {
            return; // still collecting
        }

        self.autocal_done = true; // window closed: decide once, then never again

        if self.autocal_samples.len() < self.autocal.min_samples {
            log::warn!(
                "rover-control: auto-calibration gave up: only {} steady samples in {:.0}s \
                 (need {}). Keeping max_ticks_per_sec={:.0}.",
                self.autocal_samples.len(),
                self.autocal.window_s,
                self.autocal.min_samples,
                self.max_ticks_per_sec
            );
            return;
        }

        self.autocal_samples
            .sort_by(|a, b| a.partial_cmp(b).expect("samples are finite"));
        let n = self.autocal_samples.len();
        let pctile = |q: f64| -> f32 { self.autocal_samples[((q * n as f64) as usize).min(n - 1)] };
        let learned = pctile(0.75);
        let p25 = pctile(0.25);
        if p25 > 0.0 && (learned / p25) > 1.5 {
            log::warn!(
                "rover-control: auto-calibration samples widely spread (p25={p25:.0}, \
                 p75={learned:.0}) — the rover was not on uniform ground for the whole window"
            );
        }

        let previous = self.max_ticks_per_sec;
        self.max_ticks_per_sec = learned;
        // The error scale just changed under the integrator.
        self.reset_pid(self.last_commanded_duty_pct);
        log::info!(
            "rover-control: auto-calibration complete: max_ticks_per_sec {previous:.0} -> \
             {learned:.0} (75th pct of {} samples)",
            self.autocal_samples.len()
        );
    }

    /// Detect a blocked drivetrain: real duty commanded, wheels not
    /// turning. See `chassis_controller_node.cpp::updateStallDetection`.
    fn update_stall(&mut self, now_s: f64, measured_tps: f32) {
        let duty = self.last_commanded_duty_pct;

        if duty < self.stall.min_duty_pct {
            self.stall_timing = false;
            if self.stall_latched && duty <= 0.0 {
                self.stall_latched = false;
            }
            return;
        }

        if measured_tps.abs() >= self.stall.min_ticks_per_sec {
            self.stall_timing = false;
            return;
        }

        if !self.stall_timing {
            self.stall_timing = true;
            self.stall_since_s = now_s;
            return;
        }

        if !self.stall_latched && (now_s - self.stall_since_s) >= self.stall.timeout_s as f64 {
            self.stall_latched = true;
            log::error!(
                "rover-control: STALL: {duty:.0}% duty commanded but wheels reading \
                 {measured_tps:.1} ticks/s for {:.1}s — stopping motors.",
                self.stall.timeout_s
            );
        }
    }
}

// ===========================================================================
// Emergency stop
// ===========================================================================

/// Whether the actuate stage is currently permitted to drive: the AND of
/// the mission state machine's `active` flag and a local E-stop latch —
/// two independent gates, either of which alone must be able to hold the
/// rover stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SafetyGate {
    mission_active: bool,
    estopped: bool,
}

impl SafetyGate {
    /// Starts closed. `mission_active` defaults to `false` — the ROS2
    /// node's own safe default was `cc_rcon_msg_ = true` ("halted"),
    /// selected before any `tpc_gnss_mission_active` message had arrived.
    /// The rover must not drive before an explicit `MissionStatus` says it
    /// may.
    pub fn new() -> Self {
        Self::default()
    }

    pub fn drive_allowed(&self) -> bool {
        self.mission_active && !self.estopped
    }

    pub fn on_mission_status(&mut self, active: bool) {
        self.mission_active = active;
    }

    /// Apply a base-station command that affects the drive gate.
    ///
    /// `Command::EStop` latches the gate closed; `Command::ClearEStop` is the
    /// only thing that releases it.
    ///
    /// **This used to be `Command::CancelMission`.** An earlier pass treated
    /// "cancel mission" as an unambiguous release, since `rover_msgs` had no
    /// dedicated resume variant and `EStop` is documented as best-effort and
    /// one-directional ("never design safety around this reaching the
    /// rover"). That was flagged as a judgment call, and on reflection it was
    /// the wrong one: an operator pressing "cancel mission" to clear an
    /// emergency stop is surprising, and it meant there was no way to cancel
    /// a mission *without* also releasing the E-stop. `Command::ClearEStop`
    /// (added alongside this change) separates the two actions; cancelling a
    /// mission no longer touches this latch at all.
    pub fn on_command(&mut self, cmd: Command) {
        match cmd {
            Command::EStop => self.estopped = true,
            Command::ClearEStop => self.estopped = false,
            Command::Nop
            | Command::SetSpeedLimit(_)
            | Command::SetMissionGoal(_)
            | Command::CancelMission => {}
        }
    }
}

// ===========================================================================
// Confidence-based speed derating
// ===========================================================================

/// Scale `[0, 1]` applied to the cruise target as estimator confidence
/// falls — replacing the ROS2 `speed_lost_ratio` / `detection_timeout_sec`
/// cliff (plan §2.1 item 3; deviation 5, §13.3) with something continuous.
///
/// **Not a ported ROS2 constant.** The ROS2 system had no continuous
/// confidence signal to derate against — only a binary detected/lost flag
/// and a hard 10 s timeout to zero. These thresholds are a `rover-control`
/// design choice, documented rather than field-tuned; revisit once bench or
/// field data on estimator behaviour through real dropouts exists.
///
/// - Cross-track sigma: full speed at or below `SIGMA_LO_M` (5 cm), linearly
///   down to stopped at `SIGMA_HI_M` (0.5 m — the same bound the ROS2 law
///   clamped raw `b` measurements to, so a rover this uncertain about its
///   lateral position is already outside the range that law ever trusted).
/// - Lane age: full speed at or below `AGE_LO_MS` (500 ms, matching
///   `estimator.lane_stale_ms`), linearly down to stopped at `AGE_HI_MS`
///   (5 s — within the 5-15 s coasting window plan §2.4 says gyro-bias
///   convergence buys before heading drift becomes unusable).
///
/// The two ramps combine by `min`, so either uncertainty source alone can
/// slow the rover; both must be healthy for full speed.
pub fn confidence_speed_scale(cross_track_var_m2: f32, lane_age_ms: u16) -> f32 {
    const SIGMA_LO_M: f32 = 0.05;
    const SIGMA_HI_M: f32 = 0.5;
    let sigma = cross_track_var_m2.max(0.0).sqrt();
    let sigma_scale = ramp_down(sigma, SIGMA_LO_M, SIGMA_HI_M);

    const AGE_LO_MS: f32 = 500.0;
    const AGE_HI_MS: f32 = 5000.0;
    let age_scale = ramp_down(lane_age_ms as f32, AGE_LO_MS, AGE_HI_MS);

    sigma_scale.min(age_scale)
}

/// `1.0` at or below `lo`, `0.0` at or above `hi`, linear in between.
fn ramp_down(x: f32, lo: f32, hi: f32) -> f32 {
    if x <= lo {
        1.0
    } else if x >= hi {
        0.0
    } else {
        1.0 - (x - lo) / (hi - lo)
    }
}

// ===========================================================================
// Actuator: wires the above into one estimate -> ChassisCommand step
// ===========================================================================

pub struct ActuatorConfig {
    pub steer_max_deg: f32,
    pub steer_slew_deg_per_s: f32,
    pub reference_mps: f32,
    pub limit_cap_pct: f32,
    pub sensor_timeout_s: f32,
    pub metres_per_tick: f32,
    pub speed_kp: f32,
    pub speed_ki: f32,
    pub speed_kd: f32,
    pub integral_limit: f32,
    pub max_duty_step_pct: f32,
    pub autocal: AutocalConfig,
    pub stall: StallConfig,
}

/// Owns every guard rail plus the wheel-tick differencing the speed loop
/// needs, and turns one tick's `(steer_rad, RoverState, drive_allowed)` into
/// a `ChassisCommand`.
pub struct Actuator {
    steer_max_deg: f32,
    steer: SteerLimiter,
    speed: SpeedController,

    reference_mps: f32,
    /// Config's own ceiling (`speed.limit_cap_pct`) — the absolute maximum
    /// `spd_limit_cap_pct` may ever be set to, including by
    /// `Command::SetSpeedLimit`. "Hard ceiling, independent of any
    /// controller" per `config/rover.toml`'s own comment: a base-station
    /// operator can only ever move the *operative* cap down from here, not
    /// past it.
    config_limit_cap_pct: f32,
    /// The operative cap: starts at `config_limit_cap_pct`, lowered (or
    /// raised back up, but never past the config ceiling) by
    /// `Command::SetSpeedLimit`.
    spd_limit_cap_pct: f32,
    sensor_timeout_s: f32,
    metres_per_tick: f32,

    last_wheel: Option<(WheelSensors, f64)>,
    last_wheel_seen_s: Option<f64>,
    closed_loop_was_active: bool,

    seq: u16,
}

impl Actuator {
    pub fn new(cfg: ActuatorConfig) -> Self {
        Self {
            steer_max_deg: cfg.steer_max_deg,
            steer: SteerLimiter::new(cfg.steer_max_deg, cfg.steer_slew_deg_per_s),
            speed: SpeedController::new(
                cfg.speed_kp,
                cfg.speed_ki,
                cfg.speed_kd,
                cfg.integral_limit,
                cfg.max_duty_step_pct,
                cfg.autocal,
                cfg.stall,
            ),
            reference_mps: cfg.reference_mps,
            config_limit_cap_pct: cfg.limit_cap_pct,
            spd_limit_cap_pct: cfg.limit_cap_pct,
            sensor_timeout_s: cfg.sensor_timeout_s,
            metres_per_tick: cfg.metres_per_tick,
            last_wheel: None,
            last_wheel_seen_s: None,
            closed_loop_was_active: false,
            seq: 0,
        }
    }

    /// Apply `CommandFrame::SetSpeedLimit`. Clamped to `[0, config_limit_cap_pct]`
    /// — see the field doc on `config_limit_cap_pct`.
    pub fn set_speed_limit_pct(&mut self, pct: u8) {
        self.spd_limit_cap_pct = (pct as f32).clamp(0.0, self.config_limit_cap_pct);
    }

    pub fn is_stall_latched(&self) -> bool {
        self.speed.is_stall_latched()
    }

    fn current_target_duty_pct(&mut self, state: &RoverState) -> f32 {
        let base = self
            .speed
            .target_duty_pct(self.reference_mps, self.metres_per_tick);
        let scale = confidence_speed_scale(state.cross_track_var(), state.lane_age_ms);
        (base * scale).min(self.spd_limit_cap_pct).clamp(0.0, 100.0)
    }

    /// Feed one `WheelSensors` sample to the speed loop. `now_s` is a
    /// monotonic seconds counter owned by the caller (see this crate's
    /// `main.rs`) — kept as a plain `f64` rather than `std::time::Instant`
    /// so this whole module stays clock-free and unit-testable, matching
    /// `rover_estimator`'s own "pure math" style.
    ///
    /// Returns `None` for the first sample ever seen (nothing to
    /// difference yet), a duplicate/out-of-order timestamp, or while
    /// `!drive_allowed` (no `SpeedLoopDebug` is published during an
    /// emergency stop, matching `chassisSensorsCallback`'s early return).
    pub fn handle_wheel_sensors(
        &mut self,
        wheel: WheelSensors,
        now_s: f64,
        state: &RoverState,
        drive_allowed: bool,
    ) -> Option<SpeedLoopDebug> {
        self.last_wheel_seen_s = Some(now_s);
        let (prev, prev_t) = self.last_wheel.replace((wheel, now_s))?;
        let dt_s = (now_s - prev_t) as f32;
        if dt_s <= 1e-3 {
            return None;
        }

        if !drive_allowed {
            self.speed.reset_pid(0.0);
            self.speed.clear_stall_timing();
            return None;
        }

        let target_duty_pct = self.current_target_duty_pct(state);
        let delta_left = wheel.ticks_left.wrapping_sub(prev.ticks_left) as f32;
        let delta_right = wheel.ticks_right.wrapping_sub(prev.ticks_right) as f32;
        let measured_left_tps = delta_left / dt_s;
        let measured_right_tps = delta_right / dt_s;

        Some(self.speed.on_sample(
            measured_left_tps,
            measured_right_tps,
            dt_s,
            now_s,
            target_duty_pct,
        ))
    }

    /// Build this tick's `ChassisCommand`. Called at the configured
    /// emission rate (plan §5.2 / `[safety] command_rate_hz` = 50 Hz) —
    /// **always**, even when `!drive_allowed`,
    /// because the firmware command watchdog (plan §5.2) treats silence
    /// itself as a fault condition.
    pub fn tick_chassis_command(
        &mut self,
        steer_rad: f32,
        state: &RoverState,
        drive_allowed: bool,
        now_s: f64,
        dt_s: f32,
    ) -> ChassisCommand {
        let target_pct = self.current_target_duty_pct(state);

        let steer_deg = if drive_allowed {
            self.steer.apply(steer_rad.to_degrees(), dt_s)
        } else {
            // Immediate centre, no slew — see `SteerLimiter::force_to`.
            self.steer.force_to(0.0);
            0.0
        };
        let steer_norm = (steer_deg / self.steer_max_deg).clamp(-1.0, 1.0);

        let sensor_fresh = self
            .last_wheel_seen_s
            .is_some_and(|t| now_s - t <= self.sensor_timeout_s as f64);
        let closed_loop_active = drive_allowed && sensor_fresh;

        if !closed_loop_active && self.closed_loop_was_active {
            // Falling back to open loop (stale sensor feed, or driving just
            // stopped being allowed): reset so the PID doesn't resume
            // wound-up later.
            self.speed.reset_pid(target_pct);
        }
        self.closed_loop_was_active = closed_loop_active;

        let mut output_pct = if closed_loop_active {
            self.speed.cached_output_pct()
        } else {
            target_pct
        };

        if !drive_allowed {
            // E-stop / mission inactive always wins, regardless of any
            // cached PID output.
            output_pct = 0.0;
        }
        if self.speed.is_stall_latched() {
            output_pct = 0.0;
        }
        output_pct = output_pct.min(self.spd_limit_cap_pct).clamp(0.0, 100.0);

        self.speed.set_last_commanded_duty(output_pct);

        self.seq = self.seq.wrapping_add(1);
        ChassisCommand {
            steer: steer_norm,
            throttle: output_pct / 100.0,
            seq: self.seq,
        }
        .clamped()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn autocal(enabled: bool) -> AutocalConfig {
        AutocalConfig {
            enabled,
            min_duty_pct: 13.0,
            min_samples: 40,
            window_s: 60.0,
        }
    }

    fn stall(enabled: bool) -> StallConfig {
        StallConfig {
            enabled,
            min_duty_pct: 30.0,
            min_ticks_per_sec: 10.0,
            timeout_s: 2.0,
        }
    }

    fn state(cross_track_var: f32, lane_age_ms: u16) -> RoverState {
        RoverState {
            cross_track_m: 0.0,
            heading_err_rad: 0.0,
            curvature_inv_m: 0.0,
            speed_mps: 0.2,
            gyro_bias_radps: 0.0,
            p_diag: [cross_track_var, 0.0, 0.0, 0.0, 0.0],
            lane_age_ms,
        }
    }

    // -- SteerLimiter --------------------------------------------------

    #[test]
    fn steer_limiter_saturates_at_max_deg() {
        let mut l = SteerLimiter::new(45.0, 1_000_000.0); // slew never binds
        assert_eq!(l.apply(100.0, 0.02), 45.0);
        let mut l2 = SteerLimiter::new(45.0, 1_000_000.0);
        assert_eq!(l2.apply(-100.0, 0.02), -45.0);
    }

    #[test]
    fn steer_limiter_limits_slew_rate() {
        let mut l = SteerLimiter::new(45.0, 180.0); // 180 deg/s
        let dt = 0.02; // 20ms tick -> max step 3.6 deg
        let out = l.apply(45.0, dt);
        assert!((out - 3.6).abs() < 1e-4, "expected 3.6deg step, got {out}");
        let out2 = l.apply(45.0, dt);
        assert!(
            (out2 - 7.2).abs() < 1e-4,
            "expected cumulative 7.2deg, got {out2}"
        );
    }

    #[test]
    fn steer_limiter_force_to_ignores_slew() {
        let mut l = SteerLimiter::new(45.0, 1.0); // tiny slew rate
        l.apply(45.0, 0.02); // barely moves
        l.force_to(0.0); // must jump immediately
                         // Next apply starts fresh from 0, not from wherever the slow slew left it.
        let out = l.apply(0.0, 0.02);
        assert_eq!(out, 0.0);
    }

    #[test]
    fn steer_limiter_force_to_is_itself_saturated() {
        let mut l = SteerLimiter::new(45.0, 180.0);
        l.force_to(1000.0);
        assert_eq!(l.apply(1000.0, 0.0), 45.0);
    }

    // -- SpeedController: basic PID behaviour ---------------------------

    #[test]
    fn pid_output_moves_toward_target_when_undershooting() {
        // A large max_duty_step_pct isolates the PID math from the slew
        // limiter (covered separately by `pid_output_slew_limited_per_sample`).
        let mut sc =
            SpeedController::new(0.3, 0.5, 0.0, 100.0, 1000.0, autocal(false), stall(false));
        // max_ticks_per_sec stays at the placeholder (1000) since autocal is
        // disabled. Target 20%, measured well under target (motor loaded).
        let target = 20.0;
        let dbg = sc.on_sample(50.0, 50.0, 0.1, 1.0, target); // 50 tps = 5% of 1000
        assert!(
            dbg.error_pct > 0.0,
            "measured under target must give positive error"
        );
        assert!(
            dbg.pid_output_pct > target,
            "output {} should trim upward above the {target}% feedforward under load",
            dbg.pid_output_pct
        );
    }

    #[test]
    fn pid_output_is_exactly_feedforward_at_zero_error() {
        let mut sc =
            SpeedController::new(0.3, 0.5, 0.0, 100.0, 1000.0, autocal(false), stall(false));
        let target = 20.0;
        // measured == target exactly (200 tps of 1000 = 20%).
        let dbg = sc.on_sample(200.0, 200.0, 0.1, 1.0, target);
        assert!((dbg.error_pct).abs() < 1e-4);
        assert!(
            (dbg.pid_output_pct - target).abs() < 1e-3,
            "zero error should emit exactly the feedforward term, got {}",
            dbg.pid_output_pct
        );
    }

    #[test]
    fn pid_output_slew_limited_per_sample() {
        let mut sc =
            SpeedController::new(10.0, 0.0, 0.0, 100.0, 15.0, autocal(false), stall(false));
        // Huge proportional gain, wants to jump the output far — but the
        // max_duty_step_pct=15 slew must cap the per-sample change.
        let dbg = sc.on_sample(0.0, 0.0, 0.1, 1.0, 20.0);
        assert!(
            dbg.pid_output_pct <= 15.0 + 1e-3,
            "single-sample output {} exceeded the 15%-step slew limit from a 0 start",
            dbg.pid_output_pct
        );
    }

    #[test]
    fn zero_target_resets_and_emits_zero_output() {
        let mut sc = SpeedController::new(0.3, 0.5, 0.0, 100.0, 15.0, autocal(false), stall(false));
        sc.on_sample(200.0, 200.0, 0.1, 1.0, 20.0); // build up some integral state
        let dbg = sc.on_sample(0.0, 0.0, 0.1, 2.0, 0.0);
        assert_eq!(dbg.pid_output_pct, 0.0);
        assert_eq!(dbg.error_pct, 0.0);
        assert_eq!(sc.cached_output_pct(), 0.0);
    }

    // -- Auto-calibration -----------------------------------------------

    #[test]
    fn autocal_learns_max_ticks_per_sec_from_steady_samples() {
        let mut sc = SpeedController::new(0.3, 0.5, 0.0, 100.0, 15.0, autocal(true), stall(false));
        // True full-scale capability: 500 ticks/s at 100% duty, so at a
        // steady 20% commanded duty we'd see ~100 ticks/s.
        let duty = 20.0;
        let mut now = 0.0_f64;
        for _ in 0..50 {
            sc.set_last_commanded_duty(duty);
            sc.on_sample(100.0, 100.0, 0.1, now, duty);
            now += 1.0; // one sample per second; window closes at 60s
        }
        // Window (60s) hasn't closed yet at 49s of samples.
        assert!(!sc.autocal_done);
        // Push past the 60s window.
        sc.set_last_commanded_duty(duty);
        sc.on_sample(100.0, 100.0, 0.1, 61.0, duty);
        assert!(sc.autocal_done);
        // 100 tps / (20/100) = 500 tps at full scale.
        assert!(
            (sc.max_ticks_per_sec() - 500.0).abs() < 1.0,
            "learned max_ticks_per_sec = {}, expected ~500",
            sc.max_ticks_per_sec()
        );
    }

    #[test]
    fn autocal_ignores_samples_below_min_duty() {
        let mut sc = SpeedController::new(0.3, 0.5, 0.0, 100.0, 15.0, autocal(true), stall(false));
        let mut now = 0.0_f64;
        for _ in 0..80 {
            sc.set_last_commanded_duty(5.0); // below min_duty_pct=13.0
            sc.on_sample(1000.0, 1000.0, 0.1, now, 5.0);
            now += 1.0;
        }
        // Never opened a window because duty never crossed the deadband.
        assert!(!sc.autocal_window_open);
        assert!(!sc.autocal_done);
        assert!(
            (sc.max_ticks_per_sec() - SpeedController::MAX_TICKS_PER_SEC_PLACEHOLDER).abs() < 1e-6
        );
    }

    #[test]
    fn autocal_keeps_placeholder_when_too_few_steady_samples() {
        let mut sc = SpeedController::new(0.3, 0.5, 0.0, 100.0, 15.0, autocal(true), stall(false));
        let mut now = 0.0_f64;
        // Duty changes every sample, so `steady` is never true and no
        // sample is ever collected, even though duty is above the deadband.
        for i in 0..80 {
            let duty = if i % 2 == 0 { 20.0 } else { 21.0 };
            sc.set_last_commanded_duty(duty);
            sc.on_sample(100.0, 100.0, 0.1, now, duty);
            now += 1.0;
        }
        assert!(sc.autocal_done, "window should have closed by 80s");
        assert!(
            (sc.max_ticks_per_sec() - SpeedController::MAX_TICKS_PER_SEC_PLACEHOLDER).abs() < 1e-6,
            "too few steady samples must leave the placeholder in place"
        );
    }

    // -- Stall detection --------------------------------------------------

    #[test]
    fn stall_latches_after_timeout_with_high_duty_and_no_motion() {
        let mut sc = SpeedController::new(0.3, 0.5, 0.0, 100.0, 15.0, autocal(false), stall(true));
        sc.set_last_commanded_duty(50.0); // above stall_min_duty_pct=30
        sc.on_sample(0.0, 0.0, 0.1, 0.0, 20.0); // wheels not turning
        assert!(!sc.is_stall_latched(), "must not latch before the timeout");
        sc.set_last_commanded_duty(50.0);
        sc.on_sample(0.0, 0.0, 0.1, 2.5, 20.0); // 2.5s later, still blocked
        assert!(
            sc.is_stall_latched(),
            "must latch once blocked past stall_timeout_s=2.0"
        );
    }

    #[test]
    fn stall_does_not_trigger_under_low_duty() {
        let mut sc = SpeedController::new(0.3, 0.5, 0.0, 100.0, 15.0, autocal(false), stall(true));
        sc.set_last_commanded_duty(10.0); // below stall_min_duty_pct=30
        sc.on_sample(0.0, 0.0, 0.1, 0.0, 10.0);
        sc.set_last_commanded_duty(10.0);
        sc.on_sample(0.0, 0.0, 0.1, 5.0, 10.0);
        assert!(
            !sc.is_stall_latched(),
            "low duty with no motion is under-power, not a stall"
        );
    }

    #[test]
    fn stall_does_not_trigger_while_wheels_are_turning() {
        let mut sc = SpeedController::new(0.3, 0.5, 0.0, 100.0, 15.0, autocal(false), stall(true));
        sc.set_last_commanded_duty(50.0);
        for i in 0..30 {
            sc.set_last_commanded_duty(50.0);
            sc.on_sample(50.0, 50.0, 0.1, i as f64 * 0.5, 20.0); // well above min_ticks_per_sec
        }
        assert!(!sc.is_stall_latched());
    }

    #[test]
    fn stall_latch_clears_only_on_zero_commanded_duty() {
        let mut sc = SpeedController::new(0.3, 0.5, 0.0, 100.0, 15.0, autocal(false), stall(true));
        sc.set_last_commanded_duty(50.0);
        sc.on_sample(0.0, 0.0, 0.1, 0.0, 20.0);
        sc.set_last_commanded_duty(50.0);
        sc.on_sample(0.0, 0.0, 0.1, 3.0, 20.0);
        assert!(sc.is_stall_latched());

        // Still commanding a nonzero duty: latch must survive.
        sc.set_last_commanded_duty(20.0);
        sc.on_sample(0.0, 0.0, 0.1, 3.1, 0.0);
        assert!(
            sc.is_stall_latched(),
            "latch cleared without a zero commanded duty"
        );

        // Now commanded to a full stop: latch clears.
        sc.set_last_commanded_duty(0.0);
        sc.on_sample(0.0, 0.0, 0.1, 3.2, 0.0);
        assert!(!sc.is_stall_latched());
    }

    // -- target_duty_pct / the metres_per_tick trap -----------------------

    #[test]
    fn target_duty_pct_uses_calibrated_conversion_when_available() {
        let mut sc = SpeedController::new(0.3, 0.5, 0.0, 100.0, 15.0, autocal(false), stall(false));
        // max_ticks_per_sec stays at the 1000 placeholder (autocal off).
        // 0.2 m/s at metres_per_tick=0.001 m/tick -> 200 ticks/s -> 20%.
        let pct = sc.target_duty_pct(0.2, 0.001);
        assert!((pct - 20.0).abs() < 1e-3, "got {pct}");
    }

    #[test]
    fn target_duty_pct_falls_back_loudly_when_uncalibrated() {
        let mut sc = SpeedController::new(0.3, 0.5, 0.0, 100.0, 15.0, autocal(false), stall(false));
        let pct = sc.target_duty_pct(0.2, 0.0);
        assert_eq!(pct, SpeedController::UNCALIBRATED_FALLBACK_DUTY_PCT);
    }

    // -- confidence_speed_scale -------------------------------------------

    #[test]
    fn confidence_scale_is_full_speed_when_confident_and_fresh() {
        assert_eq!(confidence_speed_scale(0.0, 0), 1.0);
        assert_eq!(confidence_speed_scale(0.01_f32.powi(2), 100), 1.0);
    }

    #[test]
    fn confidence_scale_drops_to_zero_far_from_the_lane() {
        let scale = confidence_speed_scale(1.0, 0); // sigma = 1.0m, way past 0.5m
        assert_eq!(scale, 0.0);
    }

    #[test]
    fn confidence_scale_drops_to_zero_on_a_long_dropout() {
        let scale = confidence_speed_scale(0.0, 10_000);
        assert_eq!(scale, 0.0);
    }

    #[test]
    fn confidence_scale_decreases_monotonically_with_sigma() {
        let a = confidence_speed_scale(0.05_f32.powi(2), 0);
        let b = confidence_speed_scale(0.2_f32.powi(2), 0);
        let c = confidence_speed_scale(0.4_f32.powi(2), 0);
        assert!(a >= b && b >= c, "{a} {b} {c}");
    }

    #[test]
    fn confidence_scale_takes_the_worse_of_the_two_signals() {
        // Sigma healthy, age terrible.
        let scale = confidence_speed_scale(0.0, 10_000);
        assert_eq!(scale, 0.0);
        // Age healthy, sigma terrible.
        let scale2 = confidence_speed_scale(1.0, 0);
        assert_eq!(scale2, 0.0);
    }

    // -- SafetyGate --------------------------------------------------------

    #[test]
    fn safety_gate_starts_closed() {
        let gate = SafetyGate::new();
        assert!(
            !gate.drive_allowed(),
            "must not drive before any MissionStatus arrives"
        );
    }

    #[test]
    fn safety_gate_opens_on_mission_active_and_closes_on_inactive() {
        let mut gate = SafetyGate::new();
        gate.on_mission_status(true);
        assert!(gate.drive_allowed());
        gate.on_mission_status(false);
        assert!(!gate.drive_allowed());
    }

    #[test]
    fn safety_gate_estop_overrides_an_active_mission() {
        let mut gate = SafetyGate::new();
        gate.on_mission_status(true);
        assert!(gate.drive_allowed());
        gate.on_command(Command::EStop);
        assert!(!gate.drive_allowed());
        // Mission still saying active does not reopen the gate.
        gate.on_mission_status(true);
        assert!(!gate.drive_allowed());
    }

    #[test]
    fn safety_gate_clear_estop_clears_the_latch() {
        let mut gate = SafetyGate::new();
        gate.on_mission_status(true);
        gate.on_command(Command::EStop);
        assert!(!gate.drive_allowed());
        gate.on_command(Command::ClearEStop);
        // Latch cleared, but drive still gated on mission_active.
        gate.on_mission_status(true);
        assert!(gate.drive_allowed());
    }

    /// Regression test for the defect `Command::ClearEStop` fixes: cancelling
    /// a mission must never itself release an E-stop latch.
    #[test]
    fn safety_gate_cancel_mission_does_not_clear_the_estop_latch() {
        let mut gate = SafetyGate::new();
        gate.on_mission_status(true);
        gate.on_command(Command::EStop);
        assert!(!gate.drive_allowed());
        gate.on_command(Command::CancelMission);
        gate.on_mission_status(true);
        assert!(
            !gate.drive_allowed(),
            "CancelMission must not clear an E-stop latch"
        );
    }

    // -- Actuator: end-to-end guard rails -----------------------------------

    fn actuator_cfg() -> ActuatorConfig {
        ActuatorConfig {
            steer_max_deg: 45.0,
            steer_slew_deg_per_s: 180.0,
            reference_mps: 0.20,
            limit_cap_pct: 40.0,
            sensor_timeout_s: 1.0,
            metres_per_tick: 0.001, // calibrated, for a deterministic percent target in these tests
            speed_kp: 0.3,
            speed_ki: 0.5,
            speed_kd: 0.0,
            integral_limit: 100.0,
            max_duty_step_pct: 15.0,
            autocal: autocal(false),
            stall: stall(false),
        }
    }

    #[test]
    fn chassis_command_is_all_stop_when_drive_not_allowed() {
        let mut act = Actuator::new(actuator_cfg());
        let cmd = act.tick_chassis_command(1.0, &state(0.0, 0), false, 1.0, 0.02);
        assert_eq!(cmd.steer, 0.0);
        assert_eq!(cmd.throttle, 0.0);
    }

    #[test]
    fn chassis_command_is_emitted_even_when_stopped() {
        // The watchdog needs a frame every tick regardless of drive state —
        // this just asserts the call always succeeds and produces a valid,
        // clamped command rather than being skipped.
        let mut act = Actuator::new(actuator_cfg());
        for _ in 0..5 {
            let cmd = act.tick_chassis_command(0.3, &state(0.0, 0), false, 1.0, 0.02);
            assert_eq!(cmd.throttle, 0.0);
            assert!(cmd.steer.abs() <= 1.0);
        }
    }

    #[test]
    fn chassis_command_steer_is_normalised_by_steer_max_deg() {
        let mut act = Actuator::new(actuator_cfg());
        // Huge raw command saturates at steer_max_deg=45 once the slew
        // limiter has had enough ticks to get there, so normalised steer
        // converges to exactly +1.0.
        let mut cmd =
            act.tick_chassis_command(std::f32::consts::PI, &state(0.0, 0), true, 1.0, 0.02);
        for _ in 0..20 {
            cmd = act.tick_chassis_command(std::f32::consts::PI, &state(0.0, 0), true, 1.0, 0.02);
        }
        assert!((cmd.steer - 1.0).abs() < 1e-4, "got {}", cmd.steer);
    }

    #[test]
    fn chassis_command_steer_is_slew_limited_on_the_first_tick() {
        let mut act = Actuator::new(actuator_cfg()); // steer_slew_deg_per_s = 180
        let cmd = act.tick_chassis_command(std::f32::consts::PI, &state(0.0, 0), true, 1.0, 0.02);
        // 180 deg/s * 0.02s = 3.6 deg, normalised by steer_max_deg=45.
        let expected = 3.6 / 45.0;
        assert!((cmd.steer - expected).abs() < 1e-4, "got {}", cmd.steer);
    }

    #[test]
    fn speed_limit_command_caps_output_even_under_load() {
        let mut act = Actuator::new(actuator_cfg());
        act.set_speed_limit_pct(5); // well under the 20% target_duty_pct this config implies
        let cmd = act.tick_chassis_command(0.0, &state(0.0, 0), true, 1.0, 0.02);
        assert!(
            cmd.throttle <= 0.05 + 1e-6,
            "cap not respected: throttle={}",
            cmd.throttle
        );
    }

    #[test]
    fn speed_limit_command_cannot_exceed_the_config_ceiling() {
        // A huge reference speed saturates the open-loop target to 100%, so
        // whatever the operator cap resolves to is the binding constraint.
        let cfg = ActuatorConfig {
            reference_mps: 100.0,
            ..actuator_cfg() // config ceiling stays 40%
        };
        let mut act = Actuator::new(cfg);
        act.set_speed_limit_pct(255); // way above both the ceiling and u8 sanity
        let cmd = act.tick_chassis_command(0.0, &state(0.0, 0), true, 1.0, 0.02);
        assert!(
            (cmd.throttle - 0.40).abs() < 1e-6,
            "operator cap must clamp to the config ceiling (40%), got throttle={}",
            cmd.throttle
        );
    }

    #[test]
    fn speed_degrades_toward_zero_as_uncertainty_grows() {
        let mut act = Actuator::new(actuator_cfg());
        let confident = act.tick_chassis_command(0.0, &state(0.0, 0), true, 1.0, 0.02);
        let mut act2 = Actuator::new(actuator_cfg());
        let uncertain = act2.tick_chassis_command(0.0, &state(1.0, 0), true, 1.0, 0.02);
        assert!(
            uncertain.throttle < confident.throttle,
            "uncertain={} confident={}",
            uncertain.throttle,
            confident.throttle
        );
        assert_eq!(uncertain.throttle, 0.0);
    }

    #[test]
    fn open_loop_fallback_used_when_wheel_sensors_are_stale() {
        let mut act = Actuator::new(actuator_cfg());
        // Seed one wheel sample so `last_wheel_seen_s` is set, then let time
        // pass well beyond sensor_timeout_s=1.0 without another sample.
        let wheel = WheelSensors {
            ticks_left: 0,
            ticks_right: 0,
            t_us: 0,
        };
        act.handle_wheel_sensors(wheel, 0.0, &state(0.0, 0), true);
        let cmd = act.tick_chassis_command(0.0, &state(0.0, 0), true, 5.0, 0.02);
        // Open-loop target for this config (0.2 m/s, 0.001 m/tick, 1000
        // placeholder tps) is 20%, unaffected by any PID state.
        assert!(
            (cmd.throttle - 0.20).abs() < 1e-3,
            "expected open-loop 20%, got {}",
            cmd.throttle
        );
    }

    #[test]
    fn wheel_sensors_first_sample_seeds_without_a_debug() {
        let mut act = Actuator::new(actuator_cfg());
        let wheel = WheelSensors {
            ticks_left: 0,
            ticks_right: 0,
            t_us: 0,
        };
        let dbg = act.handle_wheel_sensors(wheel, 0.0, &state(0.0, 0), true);
        assert!(dbg.is_none());
    }

    #[test]
    fn wheel_sensors_produce_no_debug_while_not_drive_allowed() {
        let mut act = Actuator::new(actuator_cfg());
        act.handle_wheel_sensors(
            WheelSensors {
                ticks_left: 0,
                ticks_right: 0,
                t_us: 0,
            },
            0.0,
            &state(0.0, 0),
            false,
        );
        let dbg = act.handle_wheel_sensors(
            WheelSensors {
                ticks_left: 10,
                ticks_right: 10,
                t_us: 100_000,
            },
            1.0,
            &state(0.0, 0),
            false,
        );
        assert!(
            dbg.is_none(),
            "no SpeedLoopDebug should be published during E-stop"
        );
    }

    #[test]
    fn stall_latch_forces_the_chassis_command_to_zero_throttle() {
        // A large reference speed saturates the open-loop target to 100%
        // duty (well above stall_min_duty_pct=30). One `tick_chassis_command`
        // first establishes `last_commanded_duty_pct` (there is no PID
        // history yet, so it takes the open-loop path); then three
        // stationary wheel samples — seed, start the stall timer, exceed
        // `stall_timeout_s=2.0` — latch the stall.
        let cfg = ActuatorConfig {
            reference_mps: 100.0,
            stall: stall(true),
            ..actuator_cfg()
        };
        let mut act = Actuator::new(cfg);
        act.set_speed_limit_pct(100);
        act.tick_chassis_command(0.0, &state(0.0, 0), true, 0.0, 0.02);

        let stationary = WheelSensors {
            ticks_left: 0,
            ticks_right: 0,
            t_us: 0,
        };
        act.handle_wheel_sensors(stationary, 0.0, &state(0.0, 0), true); // seeds
        act.handle_wheel_sensors(stationary, 0.5, &state(0.0, 0), true); // starts the timer
        assert!(!act.is_stall_latched(), "must not latch before the timeout");
        act.handle_wheel_sensors(stationary, 3.0, &state(0.0, 0), true); // 2.5s blocked

        assert!(act.is_stall_latched());
        let cmd = act.tick_chassis_command(0.0, &state(0.0, 0), true, 3.0, 0.02);
        assert_eq!(
            cmd.throttle, 0.0,
            "a latched stall must force zero throttle"
        );
    }
}
