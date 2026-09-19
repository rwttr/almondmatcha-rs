//! Loading `config/rover.toml`'s `[control]`, `[control.static_gain]`,
//! `[speed]`, `[speed.autocal]`, `[speed.stall]`, `[drivetrain]`,
//! `[perception]` and `[estimator]` tables — everything `rover-control` needs
//! beyond the bus wiring `rover_bus::BusConfig` already loads from the same
//! file.
//!
//! Follows the pattern in `crates/rover-bus/src/config.rs`: deserialize into
//! a private `Raw*` shape with `serde`, ignore every table this crate does
//! not own (`[hosts]`, `[routes]`, `[mission]`, ...), and resolve into typed
//! values the rest of the binary works with. Two independent loaders reading
//! one file, each blind to the other's tables, is what lets `rover-bus` and
//! `rover-control` each own their slice without a shared parser to keep in
//! sync — see that file's module docs.

use rover_estimator::{EstimatorConfig, LaneNoise, ProcessNoise};
use rover_model::VehicleParams;
use std::fmt;
use std::path::Path;

/// Everything `rover-control` needs from `config/rover.toml`, beyond bus
/// wiring (which `rover_bus::BusConfig` loads separately from the same
/// file).
#[derive(Debug, Clone, PartialEq)]
pub struct AppConfig {
    /// `estimator.predict_hz` — the control loop's fixed tick rate. Plan
    /// §1.2 and this crate's `main.rs` docs: one std thread, ticking at this
    /// rate, doing estimate -> guide -> actuate every tick.
    pub predict_hz: f32,
    /// `drivetrain.wheelbase_m` + `perception.lookahead_m`, shared with the
    /// EKF's process model (`rover-model`) so estimator and controller can
    /// never disagree about the vehicle.
    pub vehicle: VehicleParams,
    pub estimator: EstimatorConfig,
    /// `control.law` resolved to a concrete controller selection.
    pub law: ControllerLaw,
    /// `control.steer_max_deg` — guard rail applied by the actuate stage to
    /// every controller (plan §8.3), whichever `law` is active.
    pub steer_max_deg: f32,
    /// `control.steer_slew_deg_per_s` — ditto.
    pub steer_slew_deg_per_s: f32,
    pub speed: SpeedConfig,
    /// `safety.command_rate_hz` — how often the actuate stage must emit a
    /// `ChassisCommand`, unconditionally, even while stopped: the firmware
    /// command watchdog (plan §5.2) treats silence itself as a fault.
    pub command_rate_hz: f32,
    /// `drivetrain.metres_per_tick`. `0.0` means uncalibrated (plan §2.6) —
    /// see `rover_estimator`'s module docs for the trap this guards against;
    /// the actuate stage has the same trap on the speed side (converting
    /// `speed.reference_mps` into a duty target).
    pub metres_per_tick: f32,
}

/// Which [`crate::guide::LateralController`] to build. A closed enum rather
/// than a free-form string: `Lqr`/`Mpc` are named in the plan (§8) as future
/// work and deliberately not implemented here (see this crate's top-level
/// docs and plan §13.2) — naming them as *rejected* variants means a config
/// asking for one fails loudly at startup instead of silently falling back
/// to `StaticGain`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControllerLaw {
    StaticGain(StaticGainConfig),
}

/// `[control.static_gain]`. Field-derived, plan §10 — see `crate::guide` for
/// the unit handling that makes these values, and not `k_lat/57.3`-style
/// "fixed" ones, correct.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StaticGainConfig {
    /// `control.static_gain.k_lat`, degrees per metre of lateral offset.
    pub k_lat_deg_per_m: f32,
    /// `control.static_gain.k_head`, degrees per degree of heading error.
    pub k_head_deg_per_deg: f32,
}

/// `[speed]`, `[speed.autocal]`, `[speed.stall]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpeedConfig {
    /// `speed.reference_mps` — cruise target, **metres per second**. See
    /// `crate::actuate`'s module docs for how this is converted into a duty
    /// percentage, and what happens while `metres_per_tick == 0.0`.
    pub reference_mps: f32,
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral_limit: f32,
    pub max_duty_step_pct: f32,
    /// `speed.limit_cap_pct` — hard ceiling, independent of any controller
    /// or `CommandFrame::SetSpeedLimit` override (which can only lower it
    /// further, never raise it past this).
    pub limit_cap_pct: f32,
    pub sensor_timeout_s: f32,
    pub autocal: AutocalConfig,
    pub stall: StallConfig,
}

/// `[speed.autocal]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutocalConfig {
    pub enabled: bool,
    pub min_duty_pct: f32,
    pub min_samples: usize,
    pub window_s: f32,
}

/// `[speed.stall]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StallConfig {
    pub enabled: bool,
    pub min_duty_pct: f32,
    pub min_ticks_per_sec: f32,
    pub timeout_s: f32,
}

impl AppConfig {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, ConfigError> {
        let text = std::fs::read_to_string(path.as_ref())
            .map_err(|e| ConfigError::Io(path.as_ref().display().to_string(), e))?;
        Self::parse(&text)
    }

    pub fn parse(toml_text: &str) -> Result<Self, ConfigError> {
        let raw: RawConfig = toml::from_str(toml_text).map_err(ConfigError::Toml)?;

        let law = match raw.control.law.as_str() {
            "static_gain" => ControllerLaw::StaticGain(StaticGainConfig {
                k_lat_deg_per_m: raw.control.static_gain.k_lat,
                k_head_deg_per_deg: raw.control.static_gain.k_head,
            }),
            // Named explicitly (rather than falling into a wildcard) so the
            // plan's own future names produce a recognisable error, not just
            // "unknown law".
            other @ ("lqr" | "mpc") => {
                return Err(ConfigError::UnimplementedLaw(other.to_string()))
            }
            other => return Err(ConfigError::UnknownLaw(other.to_string())),
        };

        let vehicle = VehicleParams::new(raw.drivetrain.wheelbase_m, raw.perception.lookahead_m);

        let estimator = EstimatorConfig {
            q: ProcessNoise::new(
                raw.estimator.q.cross_track,
                raw.estimator.q.heading,
                raw.estimator.q.curvature,
                raw.estimator.q.speed,
                raw.estimator.q.gyro_bias,
            ),
            r_lane: LaneNoise::new(
                raw.estimator.r.lane_cross_track,
                raw.estimator.r.lane_heading,
                raw.estimator.r.lane_curvature,
            ),
            r_odom_speed: raw.estimator.r.odom_speed,
            r_zero_rate_gyro: raw.estimator.r.zero_rate_gyro,
            nis_gate: raw.estimator.nis_gate,
            metres_per_tick: raw.drivetrain.metres_per_tick,
            zero_rate_max_ticks: raw.estimator.zero_rate_max_ticks,
        };

        Ok(Self {
            predict_hz: raw.estimator.predict_hz,
            vehicle,
            estimator,
            law,
            steer_max_deg: raw.control.steer_max_deg,
            steer_slew_deg_per_s: raw.control.steer_slew_deg_per_s,
            speed: SpeedConfig {
                reference_mps: raw.speed.reference_mps,
                kp: raw.speed.kp,
                ki: raw.speed.ki,
                kd: raw.speed.kd,
                integral_limit: raw.speed.integral_limit,
                max_duty_step_pct: raw.speed.max_duty_step_pct,
                limit_cap_pct: raw.speed.limit_cap_pct,
                sensor_timeout_s: raw.speed.sensor_timeout_s,
                autocal: AutocalConfig {
                    enabled: raw.speed.autocal.enabled,
                    min_duty_pct: raw.speed.autocal.min_duty_pct,
                    min_samples: raw.speed.autocal.min_samples,
                    window_s: raw.speed.autocal.window_s,
                },
                stall: StallConfig {
                    enabled: raw.speed.stall.enabled,
                    min_duty_pct: raw.speed.stall.min_duty_pct,
                    min_ticks_per_sec: raw.speed.stall.min_ticks_per_sec,
                    timeout_s: raw.speed.stall.timeout_s,
                },
            },
            command_rate_hz: raw.safety.command_rate_hz,
            metres_per_tick: raw.drivetrain.metres_per_tick,
        })
    }
}

#[derive(Debug, serde::Deserialize)]
struct RawConfig {
    control: RawControl,
    speed: RawSpeed,
    drivetrain: RawDrivetrain,
    perception: RawPerception,
    estimator: RawEstimator,
    safety: RawSafety,
}

#[derive(Debug, serde::Deserialize)]
struct RawSafety {
    command_rate_hz: f32,
}

#[derive(Debug, serde::Deserialize)]
struct RawControl {
    law: String,
    steer_max_deg: f32,
    steer_slew_deg_per_s: f32,
    static_gain: RawStaticGain,
}

#[derive(Debug, serde::Deserialize)]
struct RawStaticGain {
    k_lat: f32,
    k_head: f32,
}

#[derive(Debug, serde::Deserialize)]
struct RawSpeed {
    reference_mps: f32,
    kp: f32,
    ki: f32,
    kd: f32,
    integral_limit: f32,
    max_duty_step_pct: f32,
    limit_cap_pct: f32,
    sensor_timeout_s: f32,
    autocal: RawAutocal,
    stall: RawStall,
}

#[derive(Debug, serde::Deserialize)]
struct RawAutocal {
    enabled: bool,
    min_duty_pct: f32,
    min_samples: usize,
    window_s: f32,
}

#[derive(Debug, serde::Deserialize)]
struct RawStall {
    enabled: bool,
    min_duty_pct: f32,
    min_ticks_per_sec: f32,
    timeout_s: f32,
}

#[derive(Debug, serde::Deserialize)]
struct RawDrivetrain {
    wheelbase_m: f32,
    metres_per_tick: f32,
}

#[derive(Debug, serde::Deserialize)]
struct RawPerception {
    lookahead_m: f32,
}

#[derive(Debug, serde::Deserialize)]
struct RawEstimator {
    predict_hz: f32,
    nis_gate: f32,
    #[serde(default)]
    zero_rate_max_ticks: i32,
    q: RawQ,
    r: RawR,
}

#[derive(Debug, serde::Deserialize)]
struct RawQ {
    cross_track: f32,
    heading: f32,
    curvature: f32,
    speed: f32,
    gyro_bias: f32,
}

#[derive(Debug, serde::Deserialize)]
struct RawR {
    lane_cross_track: f32,
    lane_heading: f32,
    lane_curvature: f32,
    odom_speed: f32,
    zero_rate_gyro: f32,
}

/// Something was wrong with `rover-control`'s slice of the config file.
#[derive(Debug)]
pub enum ConfigError {
    Io(String, std::io::Error),
    Toml(toml::de::Error),
    /// `control.law` named a controller this binary does not build at all
    /// (plan §8: `StaticGain` is the only implementation that exists — see
    /// this crate's top-level docs for why `Lqr`/`Mpc` are absent rather
    /// than stubbed).
    UnimplementedLaw(String),
    /// `control.law` named something not recognised as a law at all, not
    /// even a future one.
    UnknownLaw(String),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConfigError::Io(path, e) => write!(f, "reading `{path}`: {e}"),
            ConfigError::Toml(e) => write!(f, "parsing config: {e}"),
            ConfigError::UnimplementedLaw(name) => write!(
                f,
                "control.law = \"{name}\" is a planned controller (plan §8) that \
                 rover-control does not implement yet — only \"static_gain\" exists"
            ),
            ConfigError::UnknownLaw(name) => {
                write!(f, "control.law = \"{name}\" is not a recognised controller")
            }
        }
    }
}

impl std::error::Error for ConfigError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            ConfigError::Io(_, e) => Some(e),
            ConfigError::Toml(e) => Some(e),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"
        [control]
        law = "static_gain"
        steer_max_deg = 45.0
        steer_slew_deg_per_s = 180.0

        [control.static_gain]
        k_lat = 181.17
        k_head = 2.024

        [speed]
        reference_mps = 0.20
        kp = 0.3
        ki = 0.5
        kd = 0.0
        integral_limit = 100.0
        max_duty_step_pct = 15.0
        limit_cap_pct = 40.0
        sensor_timeout_s = 1.0

        [speed.autocal]
        enabled = true
        min_duty_pct = 13.0
        min_samples = 40
        window_s = 60.0

        [speed.stall]
        enabled = true
        min_duty_pct = 30.0
        min_ticks_per_sec = 10.0
        timeout_s = 2.0

        [drivetrain]
        wheel_diameter_m = 0.125
        decoding = "quadrature_4x"
        ticks_per_rev = 0.0
        metres_per_tick = 0.0
        track_width_m = 0.0
        wheelbase_m = 0.4875

        [perception]
        bev_px_per_m = 200.0
        lookahead_m = 1.22

        [estimator]
        predict_hz = 100.0
        nis_gate = 11.34
        lane_stale_ms = 500
        use_magnetometer = false
        zero_rate_max_ticks = 0

        [estimator.q]
        cross_track = 1e-2
        heading = 1e-2
        curvature = 1e-1
        speed = 1e-1
        gyro_bias = 1e-5

        [estimator.r]
        lane_cross_track = 2.5e-3
        lane_heading = 2.0e-3
        lane_curvature = 1.0e-2
        odom_speed = 1.0e-3
        zero_rate_gyro = 1e-6

        [safety]
        command_rate_hz = 50
        command_timeout_ms = 200
        throttle_ramp_ms = 300
        iwdg_timeout_ms = 500

        # Tables this crate does not own must not break parsing.
        [hosts]
        rpi = "192.168.1.1"
        [ports]
        rpi = 7001
    "#;

    #[test]
    fn parses_static_gain_law() {
        let cfg = AppConfig::parse(SAMPLE).unwrap();
        assert_eq!(
            cfg.law,
            ControllerLaw::StaticGain(StaticGainConfig {
                k_lat_deg_per_m: 181.17,
                k_head_deg_per_deg: 2.024,
            })
        );
        assert_eq!(cfg.steer_max_deg, 45.0);
        assert_eq!(cfg.vehicle.wheelbase_m, 0.4875);
        assert_eq!(cfg.vehicle.lookahead_m, 1.22);
        assert_eq!(cfg.metres_per_tick, 0.0);
        assert_eq!(cfg.predict_hz, 100.0);
        assert_eq!(cfg.speed.reference_mps, 0.20);
        assert_eq!(cfg.speed.autocal.min_samples, 40);
        assert_eq!(cfg.speed.stall.timeout_s, 2.0);
        assert_eq!(cfg.command_rate_hz, 50.0);
    }

    #[test]
    fn rejects_lqr_and_mpc_as_unimplemented_not_unknown() {
        let text = SAMPLE.replace(r#"law = "static_gain""#, r#"law = "lqr""#);
        let err = AppConfig::parse(&text).unwrap_err();
        assert!(matches!(err, ConfigError::UnimplementedLaw(l) if l == "lqr"));

        let text = SAMPLE.replace(r#"law = "static_gain""#, r#"law = "mpc""#);
        let err = AppConfig::parse(&text).unwrap_err();
        assert!(matches!(err, ConfigError::UnimplementedLaw(l) if l == "mpc"));
    }

    #[test]
    fn rejects_a_law_name_that_is_not_recognised_at_all() {
        let text = SAMPLE.replace(r#"law = "static_gain""#, r#"law = "pure_pursuit""#);
        let err = AppConfig::parse(&text).unwrap_err();
        assert!(matches!(err, ConfigError::UnknownLaw(l) if l == "pure_pursuit"));
    }

    #[test]
    fn loads_the_real_repo_config() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../config/rover.toml");
        let cfg = AppConfig::load(path).expect("config/rover.toml must parse for rover-control");
        assert_eq!(
            cfg.law,
            ControllerLaw::StaticGain(StaticGainConfig {
                k_lat_deg_per_m: 181.17,
                k_head_deg_per_deg: 2.024,
            })
        );
        // Shipped standing blocker (plan §13.5): still uncalibrated.
        assert_eq!(cfg.metres_per_tick, 0.0);
    }
}
