//! Computing [`HealthBits`].
//!
//! One bit per feed the rover needs and can lose independently (plan §3.2's
//! doc comment on `HealthBits`), plus three latched conditions computed from
//! other messages. Every threshold below is a judgement call the ROS 2
//! system never had to make — `TelemetryRelay` only ever carried a `*_valid`
//! bool that was `true` for the life of the process once a topic had been
//! seen once (see e.g. `mission_active_valid_` in
//! `mission_monitoring_node_rpi.cpp`, set on the first callback and never
//! cleared). That is not staleness detection, it is "have we ever heard from
//! this topic" — a link that died five minutes ago still reads `valid`.
//! Everything here is a genuine age check instead.

use rover_msgs::{ChassisStatus, HealthBits, RoverState};

// ---------------------------------------------------------------------------
// Staleness thresholds.
//
// Each is roughly 5-10x the feed's native publish period (documented per
// rover_msgs' own rate comments and docs/CSV_LOGGING.md): generous enough to
// absorb a couple of dropped frames without the health bit flapping on
// ordinary jitter, tight enough that a genuinely dead feed is flagged within
// about a second. `lane_stale_ms` is the one exception — it is not a
// constant here at all, see `LANE_STALE` below.
// ---------------------------------------------------------------------------

/// `ChassisStatus` has no documented rate, but it rides along with the 50 Hz
/// command loop (plan §5.2's `command_rate_hz`) on the same board; 500 ms is
/// 25 missed reports.
pub const CHASSIS_STALE_MS: u64 = 500;

/// `PowerSample` is 5 Hz (rover_msgs' own doc comment). 500 ms is the same
/// generous ~2.5x-period margin `docs/CSV_LOGGING.md` used for its 4 Hz
/// equivalent (`chassis_sensors.csv`).
pub const SENSORS_STALE_MS: u64 = 500;

/// Both GNSS streams run at roughly 10 Hz (`docs/CSV_LOGGING.md`). GNSS
/// solutions are noisier and more prone to the occasional multi-sentence
/// gap than a wired sensor board, so this is a full second (10x) rather than
/// the sensor boards' tighter margin.
pub const RTK_STALE_MS: u64 = 1000;
pub const BACKUP_GNSS_STALE_MS: u64 = 1000;

/// One-sigma lateral uncertainty, squared, past which the estimator is
/// considered to have diverged. `1.0 m^2` is `sigma = 1 m` — by the time the
/// EKF is that unsure of lateral position, guidance's own
/// `sqrt(P[e_y])`-driven speed reduction (plan §2.2) has already cut speed
/// hard; this bit exists for an operator/log to *see* that state, not to
/// drive it.
pub const ESTIMATOR_DIVERGED_VAR_M2: f32 = 1.0;

/// Time since each feed was last seen, in milliseconds. `None` means "never
/// seen at all this run" — treated the same as "stale" (older than any
/// threshold), since a feed that has never reported is at least as
/// concerning as one that stopped.
#[derive(Debug, Clone, Copy, Default)]
pub struct FeedAges {
    pub chassis_ms: Option<u64>,
    pub sensors_ms: Option<u64>,
    pub rtk_ms: Option<u64>,
    pub backup_gnss_ms: Option<u64>,
}

/// Compute every [`HealthBits`] flag from the current feed ages plus the
/// three latched/derived conditions.
///
/// `lane_age_ms` and `lane_stale_ms` are passed explicitly rather than
/// folded into [`FeedAges`]: lane staleness comes from
/// [`RoverState::lane_age_ms`], **not** from subscribing to
/// `LaneMeasurement` directly — see the module doc comment on why a second
/// raw-lane bridge onto the RPi/base side is exactly what
/// `HANDOFF_field_run_verification.md` rules out, and `RoverState` already
/// carries the one number (`lane_age_ms`, reset only on an *accepted* camera
/// update — see `docs/RUST_REWRITE_PLAN.md` §13.3 item 5) this bit needs.
/// `lane_stale_ms` is `[estimator] lane_stale_ms` from `config/rover.toml` —
/// the same config key the estimator itself uses to decide the camera feed
/// has gone stale, so this bit and the estimator's own behaviour can never
/// disagree about what "stale" means.
#[allow(clippy::too_many_arguments)]
pub fn compute_health(
    ages: &FeedAges,
    lane_age_ms: u16,
    lane_stale_ms: u16,
    chassis_status: Option<&ChassisStatus>,
    stall_detected: bool,
    state: &RoverState,
) -> HealthBits {
    let mut health = HealthBits::NONE;

    if ages.chassis_ms.is_none_or(|a| a > CHASSIS_STALE_MS) {
        health.set(HealthBits::CHASSIS_STALE);
    }
    if ages.sensors_ms.is_none_or(|a| a > SENSORS_STALE_MS) {
        health.set(HealthBits::SENSORS_STALE);
    }
    if lane_age_ms > lane_stale_ms {
        health.set(HealthBits::LANE_STALE);
    }
    if ages.rtk_ms.is_none_or(|a| a > RTK_STALE_MS) {
        health.set(HealthBits::RTK_STALE);
    }
    if ages.backup_gnss_ms.is_none_or(|a| a > BACKUP_GNSS_STALE_MS) {
        health.set(HealthBits::BACKUP_GNSS_STALE);
    }
    if chassis_status.is_some_and(|c| c.watchdog_tripped) {
        health.set(HealthBits::WATCHDOG_TRIPPED);
    }
    if stall_detected {
        health.set(HealthBits::STALL_DETECTED);
    }
    if state.cross_track_var() > ESTIMATOR_DIVERGED_VAR_M2 {
        health.set(HealthBits::ESTIMATOR_DIVERGED);
    }

    health
}

#[cfg(test)]
mod tests {
    use super::*;
    use rover_msgs::FaultBits;

    fn fresh_ages() -> FeedAges {
        FeedAges {
            chassis_ms: Some(10),
            sensors_ms: Some(10),
            rtk_ms: Some(10),
            backup_gnss_ms: Some(10),
        }
    }

    fn healthy_state() -> RoverState {
        RoverState {
            p_diag: [0.01, 0.01, 0.01, 0.01, 0.01],
            ..Default::default()
        }
    }

    #[test]
    fn all_fresh_is_no_bits_set() {
        let h = compute_health(&fresh_ages(), 0, 500, None, false, &healthy_state());
        assert_eq!(h, HealthBits::NONE);
    }

    #[test]
    fn never_seen_chassis_is_stale() {
        let mut ages = fresh_ages();
        ages.chassis_ms = None;
        let h = compute_health(&ages, 0, 500, None, false, &healthy_state());
        assert!(h.contains(HealthBits::CHASSIS_STALE));
    }

    #[test]
    fn old_chassis_age_is_stale() {
        let mut ages = fresh_ages();
        ages.chassis_ms = Some(CHASSIS_STALE_MS + 1);
        let h = compute_health(&ages, 0, 500, None, false, &healthy_state());
        assert!(h.contains(HealthBits::CHASSIS_STALE));
    }

    #[test]
    fn age_exactly_at_threshold_is_not_yet_stale() {
        let mut ages = fresh_ages();
        ages.sensors_ms = Some(SENSORS_STALE_MS);
        let h = compute_health(&ages, 0, 500, None, false, &healthy_state());
        assert!(!h.contains(HealthBits::SENSORS_STALE));
    }

    #[test]
    fn lane_stale_uses_the_estimator_config_threshold_not_a_constant() {
        let h = compute_health(&fresh_ages(), 501, 500, None, false, &healthy_state());
        assert!(h.contains(HealthBits::LANE_STALE));
        let h = compute_health(&fresh_ages(), 500, 500, None, false, &healthy_state());
        assert!(!h.contains(HealthBits::LANE_STALE));
    }

    #[test]
    fn rtk_and_backup_stale_independently() {
        let mut ages = fresh_ages();
        ages.rtk_ms = Some(RTK_STALE_MS + 1);
        let h = compute_health(&ages, 0, 500, None, false, &healthy_state());
        assert!(h.contains(HealthBits::RTK_STALE));
        assert!(!h.contains(HealthBits::BACKUP_GNSS_STALE));
    }

    #[test]
    fn watchdog_tripped_flows_through_from_chassis_status() {
        let status = ChassisStatus {
            watchdog_tripped: true,
            ..Default::default()
        };
        let h = compute_health(
            &fresh_ages(),
            0,
            500,
            Some(&status),
            false,
            &healthy_state(),
        );
        assert!(h.contains(HealthBits::WATCHDOG_TRIPPED));
    }

    #[test]
    fn no_chassis_status_yet_means_no_watchdog_bit() {
        // Absence of data is not evidence of a tripped watchdog -- it is
        // covered by CHASSIS_STALE instead.
        let h = compute_health(&fresh_ages(), 0, 500, None, false, &healthy_state());
        assert!(!h.contains(HealthBits::WATCHDOG_TRIPPED));
    }

    #[test]
    fn chassis_fault_bits_do_not_by_themselves_set_watchdog_tripped() {
        let status = ChassisStatus {
            watchdog_tripped: false,
            fault: FaultBits::MOTOR_FAULT,
            ..Default::default()
        };
        let h = compute_health(
            &fresh_ages(),
            0,
            500,
            Some(&status),
            false,
            &healthy_state(),
        );
        assert!(!h.contains(HealthBits::WATCHDOG_TRIPPED));
    }

    #[test]
    fn stall_detected_flows_through() {
        let h = compute_health(&fresh_ages(), 0, 500, None, true, &healthy_state());
        assert!(h.contains(HealthBits::STALL_DETECTED));
    }

    #[test]
    fn diverged_covariance_sets_estimator_diverged() {
        let diverged = RoverState {
            p_diag: [ESTIMATOR_DIVERGED_VAR_M2 + 0.01, 0.0, 0.0, 0.0, 0.0],
            ..Default::default()
        };
        let h = compute_health(&fresh_ages(), 0, 500, None, false, &diverged);
        assert!(h.contains(HealthBits::ESTIMATOR_DIVERGED));
    }

    #[test]
    fn covariance_at_threshold_is_not_yet_diverged() {
        let at_threshold = RoverState {
            p_diag: [ESTIMATOR_DIVERGED_VAR_M2, 0.0, 0.0, 0.0, 0.0],
            ..Default::default()
        };
        let h = compute_health(&fresh_ages(), 0, 500, None, false, &at_threshold);
        assert!(!h.contains(HealthBits::ESTIMATOR_DIVERGED));
    }

    #[test]
    fn multiple_bits_combine() {
        let mut ages = fresh_ages();
        ages.chassis_ms = None;
        ages.rtk_ms = None;
        let h = compute_health(&ages, 999, 500, None, true, &healthy_state());
        assert!(h.contains(HealthBits::CHASSIS_STALE));
        assert!(h.contains(HealthBits::RTK_STALE));
        assert!(h.contains(HealthBits::LANE_STALE));
        assert!(h.contains(HealthBits::STALL_DETECTED));
        assert!(!h.contains(HealthBits::SENSORS_STALE));
    }
}

/// Stall detection from the closed-loop speed PID's debug signal.
///
/// Ports the auto-calibration-era heuristic described in
/// `HANDOFF_field_run_verification.md` and encoded in `config/rover.toml`'s
/// `[speed.stall]` table: commanding a high duty cycle while barely moving,
/// sustained for a timeout (not instantaneous — a single noisy sample must
/// not trip it), means a wheel is physically blocked rather than just
/// climbing a ramp.
pub mod stall {
    use rover_msgs::SpeedLoopDebug;

    /// `[speed.stall]` from `config/rover.toml`.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct StallConfig {
        pub enabled: bool,
        pub min_duty_pct: f32,
        pub min_ticks_per_sec: f32,
        pub timeout_s: f32,
    }

    /// Tracks how long the rover has been simultaneously "commanding hard"
    /// and "barely moving", driven by an explicit clock reading rather than
    /// a real timer — see [`StallDetector::observe`].
    pub struct StallDetector {
        config: StallConfig,
        below_threshold_since_s: Option<f64>,
    }

    impl StallDetector {
        pub fn new(config: StallConfig) -> Self {
            Self {
                config,
                below_threshold_since_s: None,
            }
        }

        /// Feed one reading. `now_s` is a monotonic clock reading in
        /// seconds, supplied by the caller rather than read internally —
        /// this is what lets a test drive the timeout without a real sleep.
        ///
        /// Returns whether the rover should be considered stalled *as of
        /// this reading*: `true` only once "commanding hard, barely moving"
        /// has held continuously for `timeout_s`. A single noisy sample
        /// (e.g. one dropped encoder tick) resets the clock rather than
        /// tripping immediately, matching the field-tuned `[speed.stall]`
        /// parameters' intent — this is a sustained-condition detector, not
        /// an edge trigger.
        pub fn observe(&mut self, debug: &SpeedLoopDebug, now_s: f64) -> bool {
            if !self.config.enabled {
                return false;
            }

            let avg_tps = (debug.measured_left_tps + debug.measured_right_tps) / 2.0;
            let commanding_hard = debug.pid_output_pct.abs() >= self.config.min_duty_pct;
            let barely_moving = avg_tps.abs() < self.config.min_ticks_per_sec;

            if commanding_hard && barely_moving {
                let since = *self.below_threshold_since_s.get_or_insert(now_s);
                now_s - since >= self.config.timeout_s as f64
            } else {
                self.below_threshold_since_s = None;
                false
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        fn config() -> StallConfig {
            StallConfig {
                enabled: true,
                min_duty_pct: 30.0,
                min_ticks_per_sec: 10.0,
                timeout_s: 2.0,
            }
        }

        fn stalled_reading(pid_output_pct: f32) -> SpeedLoopDebug {
            SpeedLoopDebug {
                measured_left_tps: 1.0,
                measured_right_tps: 1.0,
                target_tps: 500.0,
                error_pct: 90.0,
                pid_output_pct,
            }
        }

        fn moving_reading() -> SpeedLoopDebug {
            SpeedLoopDebug {
                measured_left_tps: 200.0,
                measured_right_tps: 200.0,
                target_tps: 200.0,
                error_pct: 0.0,
                pid_output_pct: 35.0,
            }
        }

        #[test]
        fn does_not_trip_before_the_timeout() {
            let mut d = StallDetector::new(config());
            assert!(!d.observe(&stalled_reading(35.0), 0.0));
            assert!(!d.observe(&stalled_reading(35.0), 1.0));
        }

        #[test]
        fn trips_once_the_condition_has_held_for_the_timeout() {
            let mut d = StallDetector::new(config());
            assert!(!d.observe(&stalled_reading(35.0), 0.0));
            assert!(d.observe(&stalled_reading(35.0), 2.0));
        }

        #[test]
        fn moving_normally_never_trips() {
            let mut d = StallDetector::new(config());
            for t in 0..10 {
                assert!(!d.observe(&moving_reading(), t as f64));
            }
        }

        #[test]
        fn recovering_before_the_timeout_resets_the_clock() {
            let mut d = StallDetector::new(config());
            assert!(!d.observe(&stalled_reading(35.0), 0.0));
            assert!(!d.observe(&moving_reading(), 1.0)); // recovers
                                                         // Same wall-clock time it would have tripped at, but the clock
                                                         // was reset by the recovery, so this must not be a false trip.
            assert!(!d.observe(&stalled_reading(35.0), 2.0));
        }

        #[test]
        fn low_duty_cycle_never_trips_even_if_barely_moving() {
            // Barely moving at low commanded duty is just "stopped", not stalled.
            let mut d = StallDetector::new(config());
            assert!(!d.observe(&stalled_reading(5.0), 0.0));
            assert!(!d.observe(&stalled_reading(5.0), 10.0));
        }

        #[test]
        fn disabled_detector_never_trips() {
            let mut cfg = config();
            cfg.enabled = false;
            let mut d = StallDetector::new(cfg);
            assert!(!d.observe(&stalled_reading(35.0), 0.0));
            assert!(!d.observe(&stalled_reading(35.0), 100.0));
        }

        #[test]
        fn negative_duty_reverse_stall_also_trips() {
            // A blocked wheel while commanding hard in reverse is still a stall.
            let mut d = StallDetector::new(config());
            assert!(!d.observe(&stalled_reading(-35.0), 0.0));
            assert!(d.observe(&stalled_reading(-35.0), 2.0));
        }
    }
}
