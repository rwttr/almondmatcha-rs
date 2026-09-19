//! Loading `config/rover.toml`'s `[mission]` table.
//!
//! Same pattern as `rover-bus`'s `config.rs`: deserialize only the section
//! this crate owns, leave every other table (`[services]`, `[control]`,
//! `[estimator]`, ...) untouched so this file is never a reason a sibling
//! crate's section fails to parse, and vice versa.

use rover_msgs::MissionGoal;
use std::fmt;
use std::path::Path;

/// `[mission]` resolved into the types `Mission` (see `mission.rs`) works
/// with.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MissionConfig {
    pub arrival_radius_m: f32,
    /// `Some` when `rover.toml` pre-loads a goal — the mechanism plan §6.2
    /// requires so the rover can complete a mission with **zero uplink**:
    /// a goal set here is armed at startup with no `CommandFrame` ever
    /// having to arrive. `None` when `preloaded_goal` is empty, meaning wait
    /// for the base station.
    pub preloaded_goal: Option<MissionGoal>,
}

impl MissionConfig {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, MissionConfigError> {
        let text = std::fs::read_to_string(path.as_ref())
            .map_err(|e| MissionConfigError::Io(path.as_ref().display().to_string(), e))?;
        Self::parse(&text)
    }

    pub fn parse(toml_text: &str) -> Result<Self, MissionConfigError> {
        let raw: RawConfig = toml::from_str(toml_text).map_err(MissionConfigError::Toml)?;

        let preloaded_goal = match raw.mission.preloaded_goal.as_slice() {
            [] => None,
            [lat, lon] => Some(MissionGoal {
                lat_deg: *lat,
                lon_deg: *lon,
            }),
            other => {
                return Err(MissionConfigError::BadPreloadedGoal(other.len()));
            }
        };

        Ok(Self {
            arrival_radius_m: raw.mission.arrival_radius_m,
            preloaded_goal,
        })
    }
}

#[derive(Debug, serde::Deserialize)]
struct RawConfig {
    mission: RawMission,
}

#[derive(Debug, serde::Deserialize)]
struct RawMission {
    arrival_radius_m: f32,
    #[serde(default)]
    preloaded_goal: Vec<f64>,
}

/// Something was wrong with the `[mission]` table.
#[derive(Debug)]
pub enum MissionConfigError {
    Io(String, std::io::Error),
    Toml(toml::de::Error),
    /// `preloaded_goal` was neither empty nor exactly `[lat, lon]`.
    BadPreloadedGoal(usize),
}

impl fmt::Display for MissionConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MissionConfigError::Io(path, e) => write!(f, "reading `{path}`: {e}"),
            MissionConfigError::Toml(e) => write!(f, "parsing config: {e}"),
            MissionConfigError::BadPreloadedGoal(n) => write!(
                f,
                "[mission] preloaded_goal must be empty or exactly [lat, lon], got {n} element(s)"
            ),
        }
    }
}

impl std::error::Error for MissionConfigError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            MissionConfigError::Io(_, e) => Some(e),
            MissionConfigError::Toml(e) => Some(e),
            MissionConfigError::BadPreloadedGoal(_) => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_NO_GOAL: &str = r#"
        [services]
        control = "192.168.1.1:7001"

        [mission]
        arrival_radius_m = 2.0
        preloaded_goal   = []
    "#;

    const SAMPLE_WITH_GOAL: &str = r#"
        [mission]
        arrival_radius_m = 3.5
        preloaded_goal   = [13.736717, 100.523186]
    "#;

    #[test]
    fn empty_preloaded_goal_is_none() {
        let cfg = MissionConfig::parse(SAMPLE_NO_GOAL).unwrap();
        assert_eq!(cfg.arrival_radius_m, 2.0);
        assert_eq!(cfg.preloaded_goal, None);
    }

    #[test]
    fn a_two_element_preloaded_goal_is_parsed() {
        let cfg = MissionConfig::parse(SAMPLE_WITH_GOAL).unwrap();
        assert_eq!(cfg.arrival_radius_m, 3.5);
        assert_eq!(
            cfg.preloaded_goal,
            Some(MissionGoal {
                lat_deg: 13.736717,
                lon_deg: 100.523186
            })
        );
    }

    #[test]
    fn wrong_length_preloaded_goal_is_rejected() {
        let bad = r#"
            [mission]
            arrival_radius_m = 2.0
            preloaded_goal   = [1.0, 2.0, 3.0]
        "#;
        let err = MissionConfig::parse(bad).unwrap_err();
        assert!(matches!(err, MissionConfigError::BadPreloadedGoal(3)));
    }

    #[test]
    fn missing_mission_section_is_a_hard_error() {
        let err = MissionConfig::parse("[services]\ncontrol = \"1.2.3.4:7001\"\n").unwrap_err();
        assert!(matches!(err, MissionConfigError::Toml(_)));
    }

    #[test]
    fn sibling_sections_are_ignored() {
        let text = format!("{SAMPLE_WITH_GOAL}\n[control]\nlaw = \"static_gain\"\n");
        assert!(MissionConfig::parse(&text).is_ok());
    }

    #[test]
    fn loads_the_real_repo_config() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../config/rover.toml");
        let cfg = MissionConfig::load(path).expect("config/rover.toml must parse");
        assert_eq!(cfg.arrival_radius_m, 2.0);
        // Shipped default is an empty preloaded_goal.
        assert_eq!(cfg.preloaded_goal, None);
    }
}
