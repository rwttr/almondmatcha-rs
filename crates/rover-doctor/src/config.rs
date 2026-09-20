//! Loading the bits of `config/rover.toml` this crate needs beyond
//! `rover-bus`'s `[services]`/`[routes]`: `[drivetrain].metres_per_tick`
//! (the calibration check) and `[estimator].lane_stale_ms` (the lane
//! liveness check, reusing the estimator's own threshold for the same
//! reason `rover-telemetry`'s `config.rs` does — see `verdict.rs`'s
//! `lane_check`).
//!
//! Same pattern as `rover-bus`'s and `rover-telemetry`'s config modules:
//! deserialize only the fields owned here, leave every sibling table alone.

use crate::verdict::{DoctorConfig, DrivetrainConfig};
use std::fmt;
use std::path::Path;

impl DoctorConfig {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, DoctorConfigError> {
        let text = std::fs::read_to_string(path.as_ref())
            .map_err(|e| DoctorConfigError::Io(path.as_ref().display().to_string(), e))?;
        Self::parse(&text)
    }

    pub fn parse(toml_text: &str) -> Result<Self, DoctorConfigError> {
        let raw: RawConfig = toml::from_str(toml_text).map_err(DoctorConfigError::Toml)?;
        Ok(Self {
            drivetrain: DrivetrainConfig {
                metres_per_tick: raw.drivetrain.metres_per_tick,
            },
            lane_stale_ms: raw.estimator.lane_stale_ms,
        })
    }
}

#[derive(Debug, serde::Deserialize)]
struct RawConfig {
    drivetrain: RawDrivetrain,
    estimator: RawEstimator,
}

#[derive(Debug, serde::Deserialize)]
struct RawDrivetrain {
    metres_per_tick: f32,
}

#[derive(Debug, serde::Deserialize)]
struct RawEstimator {
    lane_stale_ms: u16,
}

#[derive(Debug)]
pub enum DoctorConfigError {
    Io(String, std::io::Error),
    Toml(toml::de::Error),
}

impl fmt::Display for DoctorConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DoctorConfigError::Io(path, e) => write!(f, "reading `{path}`: {e}"),
            DoctorConfigError::Toml(e) => write!(f, "parsing config: {e}"),
        }
    }
}

impl std::error::Error for DoctorConfigError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            DoctorConfigError::Io(_, e) => Some(e),
            DoctorConfigError::Toml(e) => Some(e),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"
        [services]
        control = "192.168.1.1:7001"

        [drivetrain]
        wheel_diameter_m = 0.125
        metres_per_tick   = 0.0

        [estimator]
        predict_hz    = 100.0
        lane_stale_ms = 500
    "#;

    #[test]
    fn parses_drivetrain_and_lane_stale_from_sibling_sections() {
        let cfg = DoctorConfig::parse(SAMPLE).unwrap();
        assert_eq!(cfg.drivetrain.metres_per_tick, 0.0);
        assert_eq!(cfg.lane_stale_ms, 500);
    }

    #[test]
    fn missing_drivetrain_is_a_hard_error() {
        let bad = r#"
            [estimator]
            lane_stale_ms = 500
        "#;
        assert!(DoctorConfig::parse(bad).is_err());
    }

    #[test]
    fn loads_the_real_repo_config() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../config/rover.toml");
        let cfg = DoctorConfig::load(path).expect("config/rover.toml must parse");
        assert_eq!(cfg.lane_stale_ms, 500);
        // The real config is genuinely uncalibrated today -- see
        // `verdict.rs`'s `drivetrain_check` doc comment. If this ever
        // starts failing because someone calibrated the rover, that is
        // good news and this assertion should be updated to reflect it.
        assert_eq!(cfg.drivetrain.metres_per_tick, 0.0);
    }
}
