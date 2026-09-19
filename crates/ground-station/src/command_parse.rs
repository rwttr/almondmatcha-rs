//! Parsing operator input into a [`Command`].
//!
//! Pulled out of `main.rs` (which reads real stdin — not exercisable in a
//! test here, see the top-level report) so the actual parsing logic is a
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
    BadArgCount { expected: usize, got: usize },
    BadNumber,
    SpeedOutOfRange,
}

impl fmt::Display for ParseCommandError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParseCommandError::Empty => write!(f, "empty input"),
            ParseCommandError::UnknownCommand => {
                write!(
                    f,
                    "unknown command (try: estop, clearestop, cancel, goal <lat> <lon>, speed <pct>, nop)"
                )
            }
            ParseCommandError::BadArgCount { expected, got } => {
                write!(f, "expected {expected} argument(s), got {got}")
            }
            ParseCommandError::BadNumber => write!(f, "could not parse a number"),
            ParseCommandError::SpeedOutOfRange => write!(f, "speed limit must be 0..=100"),
        }
    }
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
