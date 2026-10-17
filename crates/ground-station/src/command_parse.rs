//! Parsing operator input into a [`Command`].
//!
//! Pulled out of `main.rs` (which reads real stdin — not exercisable in a
//! test here) so the actual parsing logic is a
//! plain `&str -> Result` function a unit test can drive directly.
//!
//! Words, not single keystrokes: `main.rs` reads whole lines from stdin, so
//! `estop` (or its short alias `e`) still needs an Enter press. This is the
//! ANSI/plain-terminal trade-off documented in the crate's top-level doc
//! comment — a raw-mode single-keypress E-stop is a reasonable future
//! upgrade (e.g. via `crossterm`), noted rather than built here. What this
//! *does* guarantee is that the E-stop word is short and unambiguous, and
//! that `rover-bus`'s `CommandSender` starts retransmitting it immediately
//! and every second thereafter until acknowledged (see `main.rs`) — no
//! amount of typing speed removes the need for that.

use rover_msgs::{Command, MissionGoal};
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseCommandError {
    Empty,
    UnknownCommand,
    BadArgCount {
        expected: usize,
        got: usize,
    },
    BadNumber,
    /// `nan`/`inf` parse as valid `f64`s but are not coordinates.
    NotFinite,
    LatOutOfRange,
    LonOutOfRange,
    SpeedOutOfRange,
}

impl fmt::Display for ParseCommandError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParseCommandError::Empty => write!(f, "empty input"),
            ParseCommandError::UnknownCommand => {
                write!(f, "unknown command ({})", command_hint())
            }
            ParseCommandError::BadArgCount { expected, got } => {
                write!(f, "expected {expected} argument(s), got {got}")
            }
            ParseCommandError::BadNumber => write!(f, "could not parse a number"),
            ParseCommandError::NotFinite => write!(f, "coordinates must be finite numbers"),
            ParseCommandError::LatOutOfRange => write!(f, "latitude must be within -90..=90"),
            ParseCommandError::LonOutOfRange => write!(f, "longitude must be within -180..=180"),
            ParseCommandError::SpeedOutOfRange => write!(f, "speed limit must be 0..=100"),
        }
    }
}

/// Every command as `(usage shown to the operator, accepted words)`. The
/// dashboard hint, the startup banner and the unknown-command error are all
/// built from this, so a command cannot be accepted by [`parse_command`] yet
/// missing from the hint (`clearestop` once was). A test checks that every
/// word listed here really parses.
pub const COMMANDS: &[(&str, &[&str])] = &[
    ("estop", &["estop", "e"]),
    ("clearestop", &["clearestop", "ce"]),
    ("cancel", &["cancel", "c"]),
    ("goal <lat> <lon>", &["goal", "g"]),
    ("speed <pct>", &["speed", "s"]),
    ("nop", &["nop"]),
];

/// `commands: estop | clearestop | ...`, from [`COMMANDS`].
pub fn command_hint() -> String {
    let usages: Vec<&str> = COMMANDS.iter().map(|(u, _)| *u).collect();
    format!("commands: {}", usages.join(" | "))
}

/// Parse one line of operator input.
///
/// `estop`/`e`, `clearestop`/`ce`, and `cancel`/`c` take no arguments; `goal
/// <lat> <lon>`/`g <lat> <lon>` takes two; `speed <pct>`/`s <pct>` takes one,
/// `0..=100`; `nop` takes none. Matching is case-insensitive on the command
/// word so `ESTOP` under stress is as valid as `estop`.
///
/// `clearestop` is deliberately a distinct word from `cancel`, not an alias:
/// `rover-control::actuate::SafetyGate` used to release its E-stop latch on
/// `Command::CancelMission`, which was flagged as surprising (cancelling a
/// mission is not obviously "please let me drive again"). `Command::
/// ClearEStop` is the explicit, unambiguous release; `cancel` no longer
/// touches the E-stop latch at all.
pub fn parse_command(line: &str) -> Result<Command, ParseCommandError> {
    let mut parts = line.split_whitespace();
    let word = parts.next().ok_or(ParseCommandError::Empty)?.to_lowercase();
    let args: Vec<&str> = parts.collect();

    match word.as_str() {
        "estop" | "e" => {
            require_args(&args, 0)?;
            Ok(Command::EStop)
        }
        "clearestop" | "ce" => {
            require_args(&args, 0)?;
            Ok(Command::ClearEStop)
        }
        "cancel" | "c" => {
            require_args(&args, 0)?;
            Ok(Command::CancelMission)
        }
        "nop" => {
            require_args(&args, 0)?;
            Ok(Command::Nop)
        }
        "goal" | "g" => {
            require_args(&args, 2)?;
            let lat_deg: f64 = args[0].parse().map_err(|_| ParseCommandError::BadNumber)?;
            let lon_deg: f64 = args[1].parse().map_err(|_| ParseCommandError::BadNumber)?;
            // `f64::from_str` accepts "nan" and "inf"; a goal there would be
            // armed and then fail every distance computation downstream.
            // Range checks are the cheap guard against a swapped lat/lon or
            // a dropped decimal point, which would otherwise send the rover
            // toward a point on the other side of the planet.
            if !lat_deg.is_finite() || !lon_deg.is_finite() {
                return Err(ParseCommandError::NotFinite);
            }
            if lat_deg.abs() > 90.0 {
                return Err(ParseCommandError::LatOutOfRange);
            }
            if lon_deg.abs() > 180.0 {
                return Err(ParseCommandError::LonOutOfRange);
            }
            Ok(Command::SetMissionGoal(MissionGoal { lat_deg, lon_deg }))
        }
        "speed" | "s" => {
            require_args(&args, 1)?;
            let pct: u32 = args[0].parse().map_err(|_| ParseCommandError::BadNumber)?;
            if pct > 100 {
                return Err(ParseCommandError::SpeedOutOfRange);
            }
            Ok(Command::SetSpeedLimit(pct as u8))
        }
        _ => Err(ParseCommandError::UnknownCommand),
    }
}

fn require_args(args: &[&str], expected: usize) -> Result<(), ParseCommandError> {
    if args.len() != expected {
        Err(ParseCommandError::BadArgCount {
            expected,
            got: args.len(),
        })
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn estop_and_its_alias() {
        assert_eq!(parse_command("estop"), Ok(Command::EStop));
        assert_eq!(parse_command("e"), Ok(Command::EStop));
        assert_eq!(parse_command("ESTOP"), Ok(Command::EStop));
    }

    #[test]
    fn cancel_and_its_alias() {
        assert_eq!(parse_command("cancel"), Ok(Command::CancelMission));
        assert_eq!(parse_command("c"), Ok(Command::CancelMission));
    }

    #[test]
    fn clearestop_and_its_alias() {
        assert_eq!(parse_command("clearestop"), Ok(Command::ClearEStop));
        assert_eq!(parse_command("ce"), Ok(Command::ClearEStop));
        assert_eq!(parse_command("CLEARESTOP"), Ok(Command::ClearEStop));
    }

    #[test]
    fn nop() {
        assert_eq!(parse_command("nop"), Ok(Command::Nop));
    }

    #[test]
    fn goal_with_two_numbers() {
        assert_eq!(
            parse_command("goal 13.736717 100.523186"),
            Ok(Command::SetMissionGoal(MissionGoal {
                lat_deg: 13.736717,
                lon_deg: 100.523186
            }))
        );
        assert_eq!(
            parse_command("g 1.0 2.0"),
            Ok(Command::SetMissionGoal(MissionGoal {
                lat_deg: 1.0,
                lon_deg: 2.0
            }))
        );
    }

    #[test]
    fn goal_with_wrong_arg_count_is_rejected() {
        assert_eq!(
            parse_command("goal 1.0"),
            Err(ParseCommandError::BadArgCount {
                expected: 2,
                got: 1
            })
        );
        assert_eq!(
            parse_command("goal 1.0 2.0 3.0"),
            Err(ParseCommandError::BadArgCount {
                expected: 2,
                got: 3
            })
        );
    }

    #[test]
    fn goal_with_garbage_coordinates_is_rejected() {
        assert_eq!(
            parse_command("goal abc 2.0"),
            Err(ParseCommandError::BadNumber)
        );
    }

    #[test]
    fn goal_rejects_non_finite_coordinates() {
        for line in [
            "goal nan 100",
            "goal 13 NaN",
            "goal inf 100",
            "goal 13 -inf",
            "goal infinity 1",
        ] {
            assert_eq!(
                parse_command(line),
                Err(ParseCommandError::NotFinite),
                "{line}"
            );
        }
    }

    #[test]
    fn goal_rejects_out_of_range_coordinates_but_accepts_the_limits() {
        assert_eq!(
            parse_command("goal 90.1 100"),
            Err(ParseCommandError::LatOutOfRange)
        );
        assert_eq!(
            parse_command("goal -91 100"),
            Err(ParseCommandError::LatOutOfRange)
        );
        assert_eq!(
            parse_command("goal 13 180.5"),
            Err(ParseCommandError::LonOutOfRange)
        );
        assert_eq!(
            parse_command("goal 13 -181"),
            Err(ParseCommandError::LonOutOfRange)
        );
        assert!(parse_command("goal 90 180").is_ok());
        assert!(parse_command("goal -90 -180").is_ok());
    }

    #[test]
    fn every_listed_command_word_parses_and_is_in_the_hint() {
        let hint = command_hint();
        for (usage, words) in COMMANDS {
            assert!(hint.contains(usage), "{usage} missing from {hint}");
            for w in *words {
                // Dummy args are fine: only "unknown word" is a failure here.
                for line in [w.to_string(), format!("{w} 1"), format!("{w} 1 2")] {
                    assert_ne!(
                        parse_command(&line),
                        Err(ParseCommandError::UnknownCommand),
                        "{line}"
                    );
                }
            }
        }
        assert!(hint.contains("clearestop"));
    }

    #[test]
    fn speed_in_range() {
        assert_eq!(parse_command("speed 40"), Ok(Command::SetSpeedLimit(40)));
        assert_eq!(parse_command("s 0"), Ok(Command::SetSpeedLimit(0)));
        assert_eq!(parse_command("speed 100"), Ok(Command::SetSpeedLimit(100)));
    }

    #[test]
    fn speed_out_of_range_is_rejected() {
        assert_eq!(
            parse_command("speed 101"),
            Err(ParseCommandError::SpeedOutOfRange)
        );
    }

    #[test]
    fn unknown_word_is_rejected() {
        assert_eq!(
            parse_command("banana"),
            Err(ParseCommandError::UnknownCommand)
        );
    }

    #[test]
    fn empty_input_is_rejected() {
        assert_eq!(parse_command(""), Err(ParseCommandError::Empty));
        assert_eq!(parse_command("   "), Err(ParseCommandError::Empty));
    }

    #[test]
    fn extra_whitespace_is_tolerated() {
        assert_eq!(
            parse_command("  goal   1.0   2.0  "),
            Ok(Command::SetMissionGoal(MissionGoal {
                lat_deg: 1.0,
                lon_deg: 2.0
            }))
        );
    }

    #[test]
    fn estop_takes_no_arguments() {
        assert_eq!(
            parse_command("estop now"),
            Err(ParseCommandError::BadArgCount {
                expected: 0,
                got: 1
            })
        );
    }
}
