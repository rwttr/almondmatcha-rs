//! The GO/NO-GO decision itself, as a pure function over what was observed
//! during the listen window.
//!
//! Deliberately kept apart from `main.rs`: nothing in this module opens a
//! socket, reads a clock, or sleeps, so every check is a plain value-in,
//! value-out comparison a test can drive without a live rover, a real
//! network, or a listen window that actually takes ten seconds to run.
//!
//! # Three states, not two
//!
//! An operator who reads `NO-GO` for "the sensors board never sent a single
//! `BoardDiagnostics` frame" does something different from one who reads
//! `NO-GO` for "the sensors board failed its IMU self-test": the first means
//! check a cable or start a process, the second means look at that specific
//! board. Collapsing "never seen" into "seen and bad" — or worse, into a
//! silent pass — loses exactly the distinction the operator needs, so every
//! check here can independently report [`Verdict::NotSeen`], and
//! `NotSeen` still fails the run (see [`Verdict::is_go`]): **absence of data
//! is never evidence of health.**

use rover_msgs::{BoardDiagnostics, HealthBits, Telemetry};

/// The result of one preflight check.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// Observed, and healthy.
    Go,
    /// Observed, and a real fault was found.
    NoGo,
    /// Never observed during the listen window. Gates the launch exactly
    /// like `NoGo` (see [`Verdict::is_go`]), but is reported under its own
    /// label so the operator does not mistake "nothing arrived" for "a
    /// fault was found" — see the module doc comment.
    NotSeen,
}

impl Verdict {
    /// Whether this check permits the launch to proceed. Only [`Verdict::Go`]
    /// does; `NotSeen` is a NO-GO for the exit code even though it is
    /// reported under a different label on screen.
    pub fn is_go(self) -> bool {
        matches!(self, Verdict::Go)
    }
}

/// One named check's outcome, with a human-readable detail explaining why.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CheckResult {
    pub name: &'static str,
    pub verdict: Verdict,
    pub detail: String,
}

/// Everything seen (or not) on the bus during the listen window.
///
/// `main.rs` builds exactly one of these once the window closes and hands it
/// to [`evaluate`] — nothing in here is captured incrementally by this type
/// itself. `chassis_diag`/`sensors_diag` are tracked separately, keyed by
/// board, for the same reason `rover-telemetry`'s `health::BoardHealth` is:
/// `BoardDiagnostics` shares one `TYPE_ID` for both boards, so a single
/// "latest received" slot would let a healthy sample from one board hide a
/// fault the other is still reporting.
#[derive(Debug, Clone, Default)]
pub struct Observations {
    pub chassis_diag: Option<BoardDiagnostics>,
    pub sensors_diag: Option<BoardDiagnostics>,
    pub telemetry: Option<Telemetry>,
}

/// `[drivetrain]` facts from `config/rover.toml` needed by the calibration
/// check. Read directly from the config file at startup, never observed over
/// the bus — that is why [`drivetrain_check`] can never report `NotSeen`:
/// the config either parses (and this fact is known) or the binary already
/// exited before the listen window began.
#[derive(Debug, Clone, Copy)]
pub struct DrivetrainConfig {
    pub metres_per_tick: f32,
}

/// Configuration-derived facts [`evaluate`] needs beyond what is on the bus.
#[derive(Debug, Clone, Copy)]
pub struct DoctorConfig {
    pub drivetrain: DrivetrainConfig,
    /// `[estimator] lane_stale_ms` — the same threshold the estimator and
    /// `rover-telemetry`'s `HealthBits::LANE_STALE` use, so this check can
    /// never disagree with either about what "stale" means.
    pub lane_stale_ms: u16,
}

/// Run every preflight check and return one [`CheckResult`] per check, in a
/// fixed, stable order. `main.rs` is responsible for printing them and
/// deriving the process exit code from `Verdict::is_go`.
pub fn evaluate(obs: &Observations, cfg: &DoctorConfig) -> Vec<CheckResult> {
    vec![
        feeds_reachable(obs),
        post_check(obs),
        reset_cause_check(obs),
        link_quality_check(obs),
        health_bits_check(obs),
        drivetrain_check(cfg),
        rtk_check(obs),
        lane_check(obs, cfg.lane_stale_ms),
        estimator_check(obs),
    ]
}

fn feeds_reachable(obs: &Observations) -> CheckResult {
    let mut missing = Vec::new();
    if obs.chassis_diag.is_none() {
        missing.push("chassis board diagnostics");
    }
    if obs.sensors_diag.is_none() {
        missing.push("sensors board diagnostics");
    }
    if obs.telemetry.is_none() {
        missing.push("rover telemetry");
    }
    if missing.is_empty() {
        CheckResult {
            name: "feeds reachable",
            verdict: Verdict::Go,
            detail: "chassis diagnostics, sensors diagnostics, and telemetry were all seen"
                .to_string(),
        }
    } else {
        CheckResult {
            name: "feeds reachable",
            verdict: Verdict::NotSeen,
            detail: format!(
                "never received during the listen window: {}",
                missing.join(", ")
            ),
        }
    }
}

fn post_check(obs: &Observations) -> CheckResult {
    let (Some(chassis), Some(sensors)) = (obs.chassis_diag, obs.sensors_diag) else {
        return CheckResult {
            name: "board POST",
            verdict: Verdict::NotSeen,
            detail: "both boards' BoardDiagnostics must be seen before POST can be judged"
                .to_string(),
        };
    };
    let mut failures = Vec::new();
    if !chassis.post_ok() {
        failures.push(format!(
            "chassis (failed mask 0x{:04X})",
            chassis.post_failures().0
        ));
    }
    if !sensors.post_ok() {
        failures.push(format!(
            "sensors (failed mask 0x{:04X})",
            sensors.post_failures().0
        ));
    }
    if failures.is_empty() {
        CheckResult {
            name: "board POST",
            verdict: Verdict::Go,
            detail: "chassis and sensors both passed every POST check that ran".to_string(),
        }
    } else {
        CheckResult {
            name: "board POST",
            verdict: Verdict::NoGo,
            detail: format!("POST failures: {}", failures.join("; ")),
        }
    }
}

fn reset_cause_check(obs: &Observations) -> CheckResult {
    let (Some(chassis), Some(sensors)) = (obs.chassis_diag, obs.sensors_diag) else {
        return CheckResult {
            name: "reset causes",
            verdict: Verdict::NotSeen,
            detail: "both boards' BoardDiagnostics must be seen before reset causes can be judged"
                .to_string(),
        };
    };
    let mut abnormal = Vec::new();
    if chassis.reset_cause.is_abnormal() {
        abnormal.push(format!("chassis ({})", chassis.reset_cause.name()));
    }
    if sensors.reset_cause.is_abnormal() {
        abnormal.push(format!("sensors ({})", sensors.reset_cause.name()));
    }
    if abnormal.is_empty() {
        CheckResult {
            name: "reset causes",
            verdict: Verdict::Go,
            detail: format!(
                "chassis={}, sensors={}",
                chassis.reset_cause.name(),
                sensors.reset_cause.name()
            ),
        }
    } else {
        CheckResult {
            name: "reset causes",
            verdict: Verdict::NoGo,
            detail: format!("abnormal reset cause reported by: {}", abnormal.join("; ")),
        }
    }
}

fn link_quality_check(obs: &Observations) -> CheckResult {
    let (Some(chassis), Some(sensors)) = (obs.chassis_diag, obs.sensors_diag) else {
        return CheckResult {
            name: "board link quality",
            verdict: Verdict::NotSeen,
            detail: "both boards' BoardDiagnostics must be seen before link quality can be judged"
                .to_string(),
        };
    };
    // A speed of 0 is the board saying "I could not resolve this", not "the
    // link is down" -- see `link_speed_mbps`' doc comment. Reporting that as
    // NO-GO would be this tool breaking its own rule: it is unobserved data,
    // and the operator's next move (why did the PHY read fail?) is the
    // NOT SEEN move, not the bad-cable move. Reporting it as GO would break
    // the other rule, so it is neither.
    let unknown: Vec<&str> = [("chassis", &chassis), ("sensors", &sensors)]
        .iter()
        .filter(|(_, d)| d.link_speed_mbps == 0)
        .map(|(name, _)| *name)
        .collect();
    if !unknown.is_empty() {
        return CheckResult {
            name: "board link quality",
            verdict: Verdict::NotSeen,
            detail: format!(
                "{} reported link speed 0 (unknown) -- the board could not read its PHY, \
                 so link quality is unjudged rather than bad",
                unknown.join(" and "),
            ),
        };
    }

    fn degraded(d: &BoardDiagnostics) -> bool {
        d.link_speed_mbps != 100 || !d.link_full_duplex || d.phy_symbol_errors > 0
    }
    fn describe(name: &str, d: &BoardDiagnostics) -> String {
        format!(
            "{name}: {} Mbit/s {}, {} symbol error(s)",
            d.link_speed_mbps,
            if d.link_full_duplex {
                "full-duplex"
            } else {
                "half-duplex"
            },
            d.phy_symbol_errors,
        )
    }
    let mut bad = Vec::new();
    if degraded(&chassis) {
        bad.push(describe("chassis", &chassis));
    }
    if degraded(&sensors) {
        bad.push(describe("sensors", &sensors));
    }
    if bad.is_empty() {
        CheckResult {
            name: "board link quality",
            verdict: Verdict::Go,
            detail: "both boards report 100 Mbit/s full-duplex, zero symbol errors".to_string(),
        }
    } else {
        CheckResult {
            name: "board link quality",
            verdict: Verdict::NoGo,
            detail: format!("degraded link(s): {}", bad.join("; ")),
        }
    }
}

fn health_bits_check(obs: &Observations) -> CheckResult {
    let Some(t) = obs.telemetry else {
        return CheckResult {
            name: "telemetry health bits",
            verdict: Verdict::NotSeen,
            detail: "no Telemetry received during the listen window".to_string(),
        };
    };
    if t.health == HealthBits::NONE {
        CheckResult {
            name: "telemetry health bits",
            verdict: Verdict::Go,
            detail: "no HealthBits set".to_string(),
        }
    } else {
        CheckResult {
            name: "telemetry health bits",
            verdict: Verdict::NoGo,
            detail: format!("HealthBits = 0x{:04X}", t.health.0),
        }
    }
}

/// **This check fails today, on purpose.** `config/rover.toml`'s
/// `[drivetrain] metres_per_tick` is `0.0` — the drivetrain has never been
/// calibrated (`docs/RUST_REWRITE_PLAN.md` §2.6). That is correct behaviour,
/// not a bug in this tool: `speed_mps` everywhere downstream is meaningless
/// until Procedures A and B in that section are run, and there is no flag to
/// skip this check, because a silenced calibration warning is worse than a
/// GO nobody should trust.
fn drivetrain_check(cfg: &DoctorConfig) -> CheckResult {
    if cfg.drivetrain.metres_per_tick > 0.0 {
        CheckResult {
            name: "drivetrain calibrated",
            verdict: Verdict::Go,
            detail: format!(
                "metres_per_tick = {} (see [drivetrain] in config/rover.toml)",
                cfg.drivetrain.metres_per_tick
            ),
        }
    } else {
        CheckResult {
            name: "drivetrain calibrated",
            verdict: Verdict::NoGo,
            detail: "metres_per_tick = 0.0 in [drivetrain] -- the drivetrain has never been \
                     calibrated. Run Procedures A and B in docs/RUST_REWRITE_PLAN.md \
                     section 2.6 before trusting speed_mps or driving autonomously. \
                     There is no override for this check."
                .to_string(),
        }
    }
}

fn rtk_check(obs: &Observations) -> CheckResult {
    let Some(t) = obs.telemetry else {
        return CheckResult {
            name: "RTK fix quality",
            verdict: Verdict::NotSeen,
            detail: "no Telemetry received during the listen window".to_string(),
        };
    };
    if t.rtk.fix.is_rtk() {
        CheckResult {
            name: "RTK fix quality",
            verdict: Verdict::Go,
            detail: format!("fix = {:?}, {} satellites", t.rtk.fix, t.rtk.sats),
        }
    } else {
        CheckResult {
            name: "RTK fix quality",
            verdict: Verdict::NoGo,
            detail: format!(
                "fix = {:?} is not RtkFloat/RtkFixed ({} satellites)",
                t.rtk.fix, t.rtk.sats
            ),
        }
    }
}

/// Reads liveness off `Telemetry::state.lane_age_ms`, **not** a direct
/// `LaneMeasurement` subscription — this binary is base-station-side, and
/// re-bridging raw lane data onto the RPi/base link is exactly what the
/// field-run hand-off ruled out (see `rover-telemetry::main`'s module doc
/// comment). `RoverState::lane_age_ms` is already the one number that
/// crosses the link for this purpose.
fn lane_check(obs: &Observations, lane_stale_ms: u16) -> CheckResult {
    let Some(t) = obs.telemetry else {
        return CheckResult {
            name: "lane detection live",
            verdict: Verdict::NotSeen,
            detail: "no Telemetry received during the listen window".to_string(),
        };
    };
    if t.state.lane_age_ms <= lane_stale_ms {
        CheckResult {
            name: "lane detection live",
            verdict: Verdict::Go,
            detail: format!(
                "lane_age_ms = {} (stale past {lane_stale_ms})",
                t.state.lane_age_ms
            ),
        }
    } else {
        CheckResult {
            name: "lane detection live",
            verdict: Verdict::NoGo,
            detail: format!(
                "lane_age_ms = {} exceeds the estimator's own stale threshold of {lane_stale_ms} ms \
                 -- perception is not delivering usable lane frames",
                t.state.lane_age_ms
            ),
        }
    }
}

fn estimator_check(obs: &Observations) -> CheckResult {
    let Some(t) = obs.telemetry else {
        return CheckResult {
            name: "estimator converged",
            verdict: Verdict::NotSeen,
            detail: "no Telemetry received during the listen window".to_string(),
        };
    };
    if !t.health.contains(HealthBits::ESTIMATOR_DIVERGED) {
        CheckResult {
            name: "estimator converged",
            verdict: Verdict::Go,
            detail: format!(
                "cross-track variance {:.4} m^2 within trust threshold",
                t.state.cross_track_var()
            ),
        }
    } else {
        CheckResult {
            name: "estimator converged",
            verdict: Verdict::NoGo,
            detail: format!(
                "cross-track variance {:.4} m^2 exceeds the trust threshold -- \
                 HealthBits::ESTIMATOR_DIVERGED is set",
                t.state.cross_track_var()
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rover_msgs::{BoardId, FixQuality, GnssFix, PostBits, ResetCause, RoverState};

    fn healthy_diag(board: BoardId) -> BoardDiagnostics {
        BoardDiagnostics {
            board,
            post_run: PostBits(0b1111),
            post_pass: PostBits(0b1111),
            reset_cause: ResetCause::PowerOn,
            phy_id: 0x0007_C130,
            link_speed_mbps: 100,
            link_full_duplex: true,
            phy_symbol_errors: 0,
            uptime_s: 600,
            tx_drops: 0,
        }
    }

    fn healthy_telemetry() -> Telemetry {
        Telemetry {
            state: RoverState {
                lane_age_ms: 50,
                p_diag: [0.01, 0.01, 0.01, 0.01, 0.01],
                ..Default::default()
            },
            rtk: GnssFix {
                fix: FixQuality::RtkFixed,
                sats: 15,
                ..Default::default()
            },
            health: HealthBits::NONE,
            ..Default::default()
        }
    }

    fn healthy_observations() -> Observations {
        Observations {
            chassis_diag: Some(healthy_diag(BoardId::Chassis)),
            sensors_diag: Some(healthy_diag(BoardId::Sensors)),
            telemetry: Some(healthy_telemetry()),
        }
    }

    fn calibrated_config() -> DoctorConfig {
        DoctorConfig {
            drivetrain: DrivetrainConfig {
                metres_per_tick: 0.0004,
            },
            lane_stale_ms: 500,
        }
    }

    fn find<'a>(results: &'a [CheckResult], name: &str) -> &'a CheckResult {
        results
            .iter()
            .find(|r| r.name == name)
            .unwrap_or_else(|| panic!("no check named {name:?}"))
    }

    #[test]
    fn all_good_is_all_go() {
        let results = evaluate(&healthy_observations(), &calibrated_config());
        for r in &results {
            assert_eq!(
                r.verdict,
                Verdict::Go,
                "expected GO for {:?}, got {:?}: {}",
                r.name,
                r.verdict,
                r.detail
            );
        }
    }

    #[test]
    fn empty_observations_are_not_seen_everywhere_except_drivetrain() {
        let results = evaluate(&Observations::default(), &calibrated_config());
        for r in &results {
            if r.name == "drivetrain calibrated" {
                continue; // config-only: always observed, never NotSeen.
            }
            assert_eq!(
                r.verdict,
                Verdict::NotSeen,
                "expected NOT SEEN for {:?}, got {:?}",
                r.name,
                r.verdict
            );
            assert!(!r.verdict.is_go());
        }
    }

    #[test]
    fn feeds_reachable_go_only_when_all_three_are_present() {
        let obs = healthy_observations();
        assert_eq!(
            find(&evaluate(&obs, &calibrated_config()), "feeds reachable").verdict,
            Verdict::Go
        );

        let mut missing_telemetry = obs.clone();
        missing_telemetry.telemetry = None;
        let results = evaluate(&missing_telemetry, &calibrated_config());
        let r = find(&results, "feeds reachable");
        assert_eq!(r.verdict, Verdict::NotSeen);
        assert!(r.detail.contains("telemetry"));
    }

    #[test]
    fn post_failure_on_either_board_is_no_go() {
        let mut obs = healthy_observations();
        let mut chassis = healthy_diag(BoardId::Chassis);
        chassis.post_pass = PostBits(0b1110);
        obs.chassis_diag = Some(chassis);
        let results = evaluate(&obs, &calibrated_config());
        let r = find(&results, "board POST");
        assert_eq!(r.verdict, Verdict::NoGo);
        assert!(r.detail.contains("chassis"));
    }

    #[test]
    fn post_check_is_not_seen_when_one_board_is_missing() {
        let mut obs = healthy_observations();
        obs.sensors_diag = None;
        let results = evaluate(&obs, &calibrated_config());
        let r = find(&results, "board POST");
        assert_eq!(r.verdict, Verdict::NotSeen);
    }

    #[test]
    fn abnormal_reset_cause_is_no_go() {
        let mut obs = healthy_observations();
        let mut sensors = healthy_diag(BoardId::Sensors);
        sensors.reset_cause = ResetCause::IndependentWatchdog;
        obs.sensors_diag = Some(sensors);
        let results = evaluate(&obs, &calibrated_config());
        let r = find(&results, "reset causes");
        assert_eq!(r.verdict, Verdict::NoGo);
        assert!(r.detail.contains("sensors"));
    }

    #[test]
    fn reset_cause_check_is_not_seen_when_a_board_is_missing() {
        let mut obs = healthy_observations();
        obs.chassis_diag = None;
        let results = evaluate(&obs, &calibrated_config());
        let r = find(&results, "reset causes");
        assert_eq!(r.verdict, Verdict::NotSeen);
    }

    #[test]
    fn degraded_link_is_no_go() {
        let mut obs = healthy_observations();
        let mut chassis = healthy_diag(BoardId::Chassis);
        chassis.link_speed_mbps = 10;
        obs.chassis_diag = Some(chassis);
        let results = evaluate(&obs, &calibrated_config());
        let r = find(&results, "board link quality");
        assert_eq!(r.verdict, Verdict::NoGo);
    }

    #[test]
    fn unknown_link_speed_is_not_seen_rather_than_no_go() {
        // Speed 0 means the board could not read its PHY, not that the link
        // is down -- this very sample arrived over that link. Calling it
        // NO-GO would send the operator hunting for a bad cable that is
        // demonstrably fine.
        let mut obs = healthy_observations();
        let mut chassis = healthy_diag(BoardId::Chassis);
        chassis.link_speed_mbps = 0;
        chassis.link_full_duplex = false;
        obs.chassis_diag = Some(chassis);
        let results = evaluate(&obs, &calibrated_config());
        let r = find(&results, "board link quality");
        assert_eq!(r.verdict, Verdict::NotSeen);
        assert!(r.detail.contains("chassis"));
    }

    #[test]
    fn unknown_link_speed_still_blocks_go() {
        // The other half of the rule: unjudged is not healthy either.
        let mut obs = healthy_observations();
        let mut sensors = healthy_diag(BoardId::Sensors);
        sensors.link_speed_mbps = 0;
        obs.sensors_diag = Some(sensors);
        let results = evaluate(&obs, &calibrated_config());
        assert_ne!(find(&results, "board link quality").verdict, Verdict::Go);
    }

    #[test]
    fn link_quality_check_is_not_seen_when_a_board_is_missing() {
        let mut obs = healthy_observations();
        obs.sensors_diag = None;
        let results = evaluate(&obs, &calibrated_config());
        let r = find(&results, "board link quality");
        assert_eq!(r.verdict, Verdict::NotSeen);
    }

    #[test]
    fn any_health_bit_set_is_no_go() {
        let mut obs = healthy_observations();
        let mut t = healthy_telemetry();
        t.health.set(HealthBits::RTK_STALE);
        obs.telemetry = Some(t);
        let results = evaluate(&obs, &calibrated_config());
        let r = find(&results, "telemetry health bits");
        assert_eq!(r.verdict, Verdict::NoGo);
    }

    #[test]
    fn health_bits_check_is_not_seen_without_telemetry() {
        let mut obs = healthy_observations();
        obs.telemetry = None;
        let results = evaluate(&obs, &calibrated_config());
        let r = find(&results, "telemetry health bits");
        assert_eq!(r.verdict, Verdict::NotSeen);
    }

    #[test]
    fn uncalibrated_drivetrain_is_no_go_and_never_not_seen() {
        let cfg = DoctorConfig {
            drivetrain: DrivetrainConfig {
                metres_per_tick: 0.0,
            },
            lane_stale_ms: 500,
        };
        let results = evaluate(&healthy_observations(), &cfg);
        let r = find(&results, "drivetrain calibrated");
        assert_eq!(r.verdict, Verdict::NoGo);
        assert!(r.detail.contains("RUST_REWRITE_PLAN.md"));
        assert!(r.detail.contains("2.6"));
    }

    #[test]
    fn calibrated_drivetrain_is_go() {
        let results = evaluate(&healthy_observations(), &calibrated_config());
        let r = find(&results, "drivetrain calibrated");
        assert_eq!(r.verdict, Verdict::Go);
    }

    #[test]
    fn non_rtk_fix_is_no_go() {
        let mut obs = healthy_observations();
        let mut t = healthy_telemetry();
        t.rtk.fix = FixQuality::Autonomous;
        obs.telemetry = Some(t);
        let results = evaluate(&obs, &calibrated_config());
        let r = find(&results, "RTK fix quality");
        assert_eq!(r.verdict, Verdict::NoGo);
    }

    #[test]
    fn rtk_check_is_not_seen_without_telemetry() {
        let mut obs = healthy_observations();
        obs.telemetry = None;
        let results = evaluate(&obs, &calibrated_config());
        let r = find(&results, "RTK fix quality");
        assert_eq!(r.verdict, Verdict::NotSeen);
    }

    #[test]
    fn stale_lane_is_no_go() {
        let mut obs = healthy_observations();
        let mut t = healthy_telemetry();
        t.state.lane_age_ms = 501;
        obs.telemetry = Some(t);
        let results = evaluate(&obs, &calibrated_config());
        let r = find(&results, "lane detection live");
        assert_eq!(r.verdict, Verdict::NoGo);
    }

    #[test]
    fn lane_check_is_not_seen_without_telemetry() {
        let mut obs = healthy_observations();
        obs.telemetry = None;
        let results = evaluate(&obs, &calibrated_config());
        let r = find(&results, "lane detection live");
        assert_eq!(r.verdict, Verdict::NotSeen);
    }

    #[test]
    fn diverged_estimator_is_no_go() {
        let mut obs = healthy_observations();
        let mut t = healthy_telemetry();
        t.health.set(HealthBits::ESTIMATOR_DIVERGED);
        obs.telemetry = Some(t);
        let results = evaluate(&obs, &calibrated_config());
        let r = find(&results, "estimator converged");
        assert_eq!(r.verdict, Verdict::NoGo);
    }

    #[test]
    fn estimator_check_is_not_seen_without_telemetry() {
        let mut obs = healthy_observations();
        obs.telemetry = None;
        let results = evaluate(&obs, &calibrated_config());
        let r = find(&results, "estimator converged");
        assert_eq!(r.verdict, Verdict::NotSeen);
    }

    #[test]
    fn not_seen_is_not_a_go() {
        assert!(!Verdict::NotSeen.is_go());
        assert!(!Verdict::NoGo.is_go());
        assert!(Verdict::Go.is_go());
    }
}
