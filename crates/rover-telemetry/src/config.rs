//! Loading the bits of `config/rover.toml` this crate needs beyond
//! `rover-bus`'s `[services]`/`[routes]`: `[speed.stall]` (stall
//! detection) and `[estimator].lane_stale_ms` (lane staleness — reusing the
//! estimator's own threshold rather than inventing a second one; see
//! `health.rs`'s doc comment on `compute_health`).
//!
//! Same pattern as `rover-bus`'s and `rover-navigation`'s config modules:
//! deserialize only the fields owned here, leave every sibling table alone.

use crate::health::stall::StallConfig;
use std::fmt;
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TelemetryConfig {
    pub stall: StallConfig,
    pub lane_stale_ms: u16,
}

impl TelemetryConfig {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, TelemetryConfigError> {
        let text = std::fs::read_to_string(path.as_ref())
            .map_err(|e| TelemetryConfigError::Io(path.as_ref().display().to_string(), e))?;
        Self::parse(&text)
    }

    pub fn parse(toml_text: &str) -> Result<Self, TelemetryConfigError> {
        let raw: RawConfig = toml::from_str(toml_text).map_err(TelemetryConfigError::Toml)?;
        Ok(Self {
            stall: StallConfig {
                enabled: raw.speed.stall.enabled,
                min_duty_pct: raw.speed.stall.min_duty_pct,
                min_ticks_per_sec: raw.speed.stall.min_ticks_per_sec,
                timeout_s: raw.speed.stall.timeout_s,
            },
            lane_stale_ms: raw.estimator.lane_stale_ms,
        })
    }
}

#[derive(Debug, serde::Deserialize)]
struct RawConfig {
    speed: RawSpeed,
    estimator: RawEstimator,
}

#[derive(Debug, serde::Deserialize)]
struct RawSpeed {
    stall: RawStall,
}

#[derive(Debug, serde::Deserialize)]
struct RawStall {
    enabled: bool,
    min_duty_pct: f32,
    min_ticks_per_sec: f32,
    timeout_s: f32,
}

#[derive(Debug, serde::Deserialize)]
struct RawEstimator {
    lane_stale_ms: u16,
}

#[derive(Debug)]
pub enum TelemetryConfigError {
    Io(String, std::io::Error),
    Toml(toml::de::Error),
}

impl fmt::Display for TelemetryConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TelemetryConfigError::Io(path, e) => write!(f, "reading `{path}`: {e}"),
            TelemetryConfigError::Toml(e) => write!(f, "parsing config: {e}"),
        }
    }
}

impl std::error::Error for TelemetryConfigError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            TelemetryConfigError::Io(_, e) => Some(e),
            TelemetryConfigError::Toml(e) => Some(e),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"
        [services]
        control = "192.168.1.1:7001"

        [speed]
        reference_mps = 0.2

        [speed.stall]
        enabled           = true
        min_duty_pct      = 30.0
        min_ticks_per_sec = 10.0
        timeout_s         = 2.0

        [estimator]
        predict_hz    = 100.0
        lane_stale_ms = 500
    "#;

    #[test]
    fn parses_stall_and_lane_stale_from_sibling_sections() {
        let cfg = TelemetryConfig::parse(SAMPLE).unwrap();
        assert!(cfg.stall.enabled);
        assert_eq!(cfg.stall.min_duty_pct, 30.0);
        assert_eq!(cfg.stall.min_ticks_per_sec, 10.0);
        assert_eq!(cfg.stall.timeout_s, 2.0);
        assert_eq!(cfg.lane_stale_ms, 500);
    }

    #[test]
    fn missing_speed_stall_is_a_hard_error() {
        let bad = r#"
            [estimator]
            lane_stale_ms = 500
        "#;
        assert!(TelemetryConfig::parse(bad).is_err());
    }

    #[test]
    fn loads_the_real_repo_config() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../config/rover.toml");
        let cfg = TelemetryConfig::load(path).expect("config/rover.toml must parse");
        assert_eq!(cfg.lane_stale_ms, 500);
        assert!(cfg.stall.enabled);
    }
}
