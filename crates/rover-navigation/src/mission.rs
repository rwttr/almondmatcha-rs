//! The mission state machine.
//!
//! Ports `gnss_mission_monitor_node.cpp`'s action server into a plain,
//! synchronous state machine with no I/O — the whole point being that it can
//! be driven from a unit test exactly as it will be driven from `main.rs`'s
//! loop. Two things do **not** carry over, both deliberate:
//!
//! - **Distance is metres, always.** The ROS 2 action reported `dis_remain`
//!   in *kilometres* (`DESTINATION_THRESHOLD_KM = 0.02` for a 20 m arrival
//!   radius) while `docs/CSV_LOGGING.md`'s `mission_state.csv` documents the
//!   same relayed quantity as `Distance_Remaining_m` in *metres* — the same
//!   number, two units, depending which file you read it from. See
//!   `geo.rs`.
//! - **An ordered, typed [`MissionState`] replaces implicit action state.**
//!   The ROS 2 node tracked "are we active" as a bare `bool` plus a
//!   `goal_reached_` flag with no `Cancelled`/`Fault` concept at all — an
//!   E-stop or a watchdog trip had nowhere to register in mission state. See
//!   [`Mission::apply_command`].

use crate::geo::haversine_distance_m;
use rover_msgs::{Command, GnssFix, MissionGoal, MissionState, MissionStatus};

/// Choose which GNSS stream the mission should navigate on.
///
/// **Deliberately not a port of the ROS 2 behaviour.**
/// `gnss_mission_monitor_node.cpp` tracked position from
/// `tpc_gnss_spresense` only — the *standard*, uncorrected receiver — and
/// never looked at the RTK stream at all, despite the RTK receiver being the
/// one described everywhere else in this system as the primary position
/// source. Against a 20 m (ROS 2) or 2 m (this rover.toml) arrival radius,
/// navigating on a receiver with metres of uncorrected error is a real
/// difference in behaviour, not a rounding matter — flagged here rather than
/// silently carried forward. This function prefers the RTK fix whenever it
/// is usable and falls back to the backup receiver only when RTK is not.
pub fn select_navigation_fix(rtk: &GnssFix, backup: &GnssFix) -> Option<(f64, f64)> {
    if usable(rtk) {
        Some((rtk.lat_deg, rtk.lon_deg))
    } else if usable(backup) {
        Some((backup.lat_deg, backup.lon_deg))
    } else {
        None
    }
}

/// A fix older than this is unusable. The receivers send ~1-10 Hz, so 2 s is
/// several missed sentences; before this existed `main` held the last fix
/// forever, so an unplugged or hung receiver kept reporting its last
/// "RTK fixed" position and the mission kept driving on it.
pub const GNSS_FIX_MAX_AGE_S: f64 = 2.0;

/// No usable fix for longer than this pauses the mission (`active = false`);
/// it resumes, uncancelled, when a fix returns.
pub const GNSS_LOSS_PAUSE_S: f64 = 5.0;

/// `GnssFix::is_usable` plus a finite lat/lon: a Spresense `"latitude":NaN`
/// parses as a number and `is_usable` only looks at fix type and satellites.
fn usable(fix: &GnssFix) -> bool {
    fix.is_usable() && fix.lat_deg.is_finite() && fix.lon_deg.is_finite()
}

/// [`select_navigation_fix`] with receive-age expiry: a fix whose age is
/// `None` (never received) or `> max_age_s` is treated as no fix at all.
pub fn select_fresh_navigation_fix(
    rtk: &GnssFix,
    rtk_age_s: Option<f64>,
    backup: &GnssFix,
    backup_age_s: Option<f64>,
    max_age_s: f64,
) -> Option<(f64, f64)> {
    let fresh = |fix: &GnssFix, age: Option<f64>| {
        if age.is_some_and(|a| a <= max_age_s) {
            *fix
        } else {
            GnssFix::default()
        }
    };
    select_navigation_fix(&fresh(rtk, rtk_age_s), &fresh(backup, backup_age_s))
}

/// The mission state machine.
///
/// Owns the current [`MissionState`], the active goal (if any), and the last
/// computed distance (held across a momentary GNSS gap rather than snapping
/// to zero — see [`Mission::update`]).
pub struct Mission {
    state: MissionState,
    goal: Option<MissionGoal>,
    arrival_radius_m: f32,
    last_distance_remaining_m: f32,
    /// Start (caller's clock, seconds) of the current run of ticks with no
    /// usable fix; `None` while a fix is present.
    no_fix_since_s: Option<f64>,
    paused_for_no_fix: bool,
}

impl Mission {
    /// Build a mission from config. A pre-loaded goal arms immediately, with
    /// no [`Command`] ever having to arrive — this is what lets the rover
    /// complete a mission with **zero uplink** (plan §6.2): the base station
    /// is never in the path from "goal exists" to "goal armed".
    pub fn new(config: &crate::config::MissionConfig) -> Self {
        match config.preloaded_goal {
            Some(goal) => Self {
                state: MissionState::Armed,
                goal: Some(goal),
                arrival_radius_m: config.arrival_radius_m,
                last_distance_remaining_m: 0.0,
                no_fix_since_s: None,
                paused_for_no_fix: false,
            },
            None => Self {
                state: MissionState::Idle,
                goal: None,
                arrival_radius_m: config.arrival_radius_m,
                last_distance_remaining_m: 0.0,
                no_fix_since_s: None,
                paused_for_no_fix: false,
            },
        }
    }

    pub fn state(&self) -> MissionState {
        self.state
    }

    pub fn goal(&self) -> Option<MissionGoal> {
        self.goal
    }

    /// Apply an operator command from the base station. Pure state
    /// transition — no I/O, no clock.
    ///
    /// `SetSpeedLimit` and `Nop` are not mission-state transitions: the speed
    /// cap is `rover-control`'s actuation task's concern (plan §8.3), and
    /// `Nop` exists only so the base can prove the link works. `ClearEStop`
    /// likewise does not touch mission state — it only releases
    /// `rover-control::actuate::SafetyGate`'s E-stop latch, on the process
    /// that owns it; a faulted mission still needs a fresh `SetMissionGoal`
    /// to re-arm (see below).
    ///
    /// A new `SetMissionGoal` always re-arms, from *any* current state —
    /// including `Fault` and `Cancelled`. This is what gives an operator (or
    /// a re-run of a pre-loaded goal) a way out of a halted mission without
    /// a process restart.
    pub fn apply_command(&mut self, cmd: Command) {
        match cmd {
            Command::SetMissionGoal(goal) => {
                self.goal = Some(goal);
                self.state = MissionState::Armed;
            }
            Command::CancelMission => {
                self.goal = None;
                self.state = MissionState::Cancelled;
            }
            Command::EStop => {
                // Best-effort by nature (plan §4.2, §6.2) — the guaranteed
                // stop is the firmware command watchdog, not this state
                // transition. This only stops the rover asking to keep
                // driving; it is not the safety mechanism.
                self.state = MissionState::Fault;
            }
            Command::SetSpeedLimit(_) | Command::Nop | Command::ClearEStop => {}
        }
    }

    /// Advance the state machine by one tick.
    ///
    /// `position` must be `Some((lat, lon))` only when the caller has
    /// already judged the source fix usable (see
    /// [`select_navigation_fix`]) — this type has no opinion on fix quality,
    /// satellite count, or which of two receivers to trust; that judgement
    /// stays with the caller, which has the full [`GnssFix`]es to look at.
    ///
    /// Distance is held at its last computed value while `position` is
    /// `None` (GNSS momentarily unusable), rather than reported as `0.0` —
    /// reporting "arrived" purely because the fix dropped out would be
    /// actively misleading to an operator watching remaining distance.
    pub fn update(&mut self, position: Option<(f64, f64)>) -> MissionStatus {
        if let (Some(goal), Some((lat, lon))) = (self.goal, position) {
            self.last_distance_remaining_m =
                haversine_distance_m(lat, lon, goal.lat_deg, goal.lon_deg) as f32;
        }

        self.state = match self.state {
            // A goal that is already inside the arrival radius when the first
            // fix lands is Arrived NOW: going via Running published one tick
            // of `active = true` and let the rover creep before stopping.
            MissionState::Armed if position.is_some() => {
                if self.last_distance_remaining_m <= self.arrival_radius_m {
                    MissionState::Arrived
                } else {
                    MissionState::Running
                }
            }
            MissionState::Running if position.is_some() => {
                if self.last_distance_remaining_m <= self.arrival_radius_m {
                    MissionState::Arrived
                } else {
                    MissionState::Running
                }
            }
            other => other,
        };

        MissionStatus {
            active: self.state.is_driving(),
            distance_remaining_m: self.last_distance_remaining_m,
            target: self.goal,
            state: self.state,
        }
    }
}

impl Mission {
    /// [`Self::update`] plus GNSS-loss pausing, on a caller-owned monotonic
    /// clock (`now_s`, seconds). While a `Running` mission has had no usable
    /// position for more than [`GNSS_LOSS_PAUSE_S`], the published status is
    /// `active = false` — the rover stops rather than dead-reckoning on a
    /// goal it can no longer measure against. The mission state is NOT
    /// changed (not cancelled, not faulted), so `active` returns the moment a
    /// fix does. Logged once per pause/resume edge.
    pub fn update_at(&mut self, position: Option<(f64, f64)>, now_s: f64) -> MissionStatus {
        let mut status = self.update(position);
        if position.is_some() {
            self.no_fix_since_s = None;
            if self.paused_for_no_fix {
                log::info!("rover-navigation: GNSS fix back — resuming the mission");
                self.paused_for_no_fix = false;
            }
        } else {
            let since = *self.no_fix_since_s.get_or_insert(now_s);
            if self.state == MissionState::Running && now_s - since > GNSS_LOSS_PAUSE_S {
                if !self.paused_for_no_fix {
                    log::warn!(
                        "rover-navigation: no usable GNSS fix for > {GNSS_LOSS_PAUSE_S:.0} s — \
                         pausing the mission (active=false) until a fix returns"
                    );
                    self.paused_for_no_fix = true;
                }
                status.active = false;
            }
        }
        status
    }
}

#[cfg(test)]
mod select_navigation_fix_tests {
    use super::*;
    use rover_msgs::FixQuality;

    fn fix(quality: FixQuality, sats: u8, lat: f64, lon: f64) -> GnssFix {
        GnssFix {
            lat_deg: lat,
            lon_deg: lon,
            fix: quality,
            sats,
            ..Default::default()
        }
    }

    #[test]
    fn prefers_rtk_when_usable() {
        let rtk = fix(FixQuality::RtkFixed, 12, 1.0, 2.0);
        let backup = fix(FixQuality::Autonomous, 8, 9.0, 9.0);
        assert_eq!(select_navigation_fix(&rtk, &backup), Some((1.0, 2.0)));
    }

    #[test]
    fn falls_back_to_backup_when_rtk_unusable() {
        let rtk = fix(FixQuality::None, 0, 1.0, 2.0);
        let backup = fix(FixQuality::Autonomous, 6, 9.0, 9.0);
        assert_eq!(select_navigation_fix(&rtk, &backup), Some((9.0, 9.0)));
    }

    #[test]
    fn none_when_both_unusable() {
        let rtk = fix(FixQuality::None, 0, 1.0, 2.0);
        let backup = fix(FixQuality::None, 2, 9.0, 9.0);
        assert_eq!(select_navigation_fix(&rtk, &backup), None);
    }

    #[test]
    fn low_satellite_count_is_unusable_even_with_a_fix_type() {
        // GnssFix::is_usable() requires sats >= 4.
        let rtk = fix(FixQuality::RtkFixed, 3, 1.0, 2.0);
        let backup = fix(FixQuality::Autonomous, 8, 9.0, 9.0);
        assert_eq!(select_navigation_fix(&rtk, &backup), Some((9.0, 9.0)));
    }
}

#[cfg(test)]
mod mission_tests {
    use super::*;
    use crate::config::MissionConfig;

    fn config(arrival_radius_m: f32, preloaded: Option<(f64, f64)>) -> MissionConfig {
        MissionConfig {
            arrival_radius_m,
            preloaded_goal: preloaded.map(|(lat, lon)| MissionGoal {
                lat_deg: lat,
                lon_deg: lon,
            }),
        }
    }

    #[test]
    fn starts_idle_with_no_preloaded_goal() {
        let m = Mission::new(&config(2.0, None));
        assert_eq!(m.state(), MissionState::Idle);
        assert_eq!(m.goal(), None);
    }

    /// The zero-uplink invariant (plan §6.2): a pre-loaded goal must arm with
    /// no `Command` ever applied — no base station involved at all.
    #[test]
    fn preloaded_goal_arms_with_no_command_and_no_base_station() {
        let m = Mission::new(&config(2.0, Some((13.7, 100.5))));
        assert_eq!(m.state(), MissionState::Armed);
        assert_eq!(
            m.goal(),
            Some(MissionGoal {
                lat_deg: 13.7,
                lon_deg: 100.5
            })
        );
    }

    #[test]
    fn armed_moves_to_running_once_position_is_available() {
        let mut m = Mission::new(&config(2.0, Some((0.0, 0.0))));
        let status = m.update(Some((0.001, 0.0))); // ~111 m from the goal
        assert_eq!(m.state(), MissionState::Running);
        assert!(status.active);
    }

    #[test]
    fn armed_stays_armed_without_a_usable_fix() {
        let mut m = Mission::new(&config(2.0, Some((0.0, 0.0))));
        let status = m.update(None);
        assert_eq!(m.state(), MissionState::Armed);
        assert!(!status.active, "must not be permitted to drive yet");
    }

    #[test]
    fn idle_stays_idle_even_with_a_usable_fix() {
        let mut m = Mission::new(&config(2.0, None));
        let status = m.update(Some((0.0, 0.0)));
        assert_eq!(m.state(), MissionState::Idle);
        assert!(!status.active);
    }

    #[test]
    fn arrives_within_the_configured_radius() {
        let mut m = Mission::new(&config(5.0, Some((13.736717, 100.523186))));
        m.update(Some((13.746717, 100.523186))); // Armed -> Running, ~1.1 km out
                                                 // ~2 m away, inside a 5 m arrival radius.
        let status = m.update(Some((13.736735, 100.523186)));
        assert_eq!(m.state(), MissionState::Arrived);
        assert!(!status.active, "arrival must stop the drive permission");
    }

    #[test]
    fn keeps_running_outside_the_arrival_radius() {
        let mut m = Mission::new(&config(2.0, Some((13.736717, 100.523186))));
        m.update(Some((13.746717, 100.523186)));
        // ~1.1 km away — nowhere near a 2 m radius.
        let status = m.update(Some((13.746717, 100.523186)));
        assert_eq!(m.state(), MissionState::Running);
        assert!(status.active);
        assert!(status.distance_remaining_m > 1000.0);
    }

    #[test]
    fn distance_holds_its_last_value_through_a_gnss_gap() {
        let mut m = Mission::new(&config(2.0, Some((13.736717, 100.523186))));
        m.update(Some((13.746717, 100.523186)));
        let first = m.update(Some((13.746717, 100.523186)));
        // GNSS drops out this tick.
        let second = m.update(None);
        assert_eq!(second.distance_remaining_m, first.distance_remaining_m);
        assert_eq!(m.state(), MissionState::Running, "must not snap to Arrived");
    }

    #[test]
    fn cancel_mission_clears_the_goal_and_stops() {
        let mut m = Mission::new(&config(2.0, Some((13.736717, 100.523186))));
        m.update(Some((13.746717, 100.523186))); // now Running
        m.apply_command(Command::CancelMission);
        assert_eq!(m.state(), MissionState::Cancelled);
        assert_eq!(m.goal(), None);
        let status = m.update(Some((13.736717, 100.523186)));
        assert!(!status.active);
    }

    #[test]
    fn estop_halts_into_fault_regardless_of_state() {
        let mut m = Mission::new(&config(2.0, Some((13.736717, 100.523186))));
        m.update(Some((13.746717, 100.523186))); // Running
        m.apply_command(Command::EStop);
        assert_eq!(m.state(), MissionState::Fault);
        let status = m.update(Some((13.736717, 100.523186)));
        assert!(
            !status.active,
            "a faulted mission must never grant drive permission"
        );
    }

    #[test]
    fn a_new_goal_re_arms_out_of_fault() {
        let mut m = Mission::new(&config(2.0, Some((13.736717, 100.523186))));
        m.apply_command(Command::EStop);
        assert_eq!(m.state(), MissionState::Fault);

        m.apply_command(Command::SetMissionGoal(MissionGoal {
            lat_deg: 1.0,
            lon_deg: 2.0,
        }));
        assert_eq!(m.state(), MissionState::Armed);
        assert_eq!(
            m.goal(),
            Some(MissionGoal {
                lat_deg: 1.0,
                lon_deg: 2.0
            })
        );
    }

    #[test]
    fn a_new_goal_re_arms_out_of_cancelled() {
        let mut m = Mission::new(&config(2.0, None));
        m.apply_command(Command::CancelMission);
        assert_eq!(m.state(), MissionState::Cancelled);

        m.apply_command(Command::SetMissionGoal(MissionGoal {
            lat_deg: 1.0,
            lon_deg: 2.0,
        }));
        assert_eq!(m.state(), MissionState::Armed);
    }

    #[test]
    fn speed_limit_and_nop_do_not_change_mission_state() {
        let mut m = Mission::new(&config(2.0, Some((13.736717, 100.523186))));
        m.update(Some((13.746717, 100.523186))); // Running
        m.apply_command(Command::SetSpeedLimit(50));
        m.apply_command(Command::Nop);
        assert_eq!(m.state(), MissionState::Running);
    }

    #[test]
    fn clear_estop_does_not_change_mission_state() {
        // ClearEStop only releases rover-control's SafetyGate latch; a
        // faulted mission still needs a fresh SetMissionGoal to re-arm.
        let mut m = Mission::new(&config(2.0, Some((13.736717, 100.523186))));
        m.apply_command(Command::EStop);
        assert_eq!(m.state(), MissionState::Fault);
        m.apply_command(Command::ClearEStop);
        assert_eq!(m.state(), MissionState::Fault);
    }

    #[test]
    fn arrived_does_not_resume_running_on_its_own() {
        let mut m = Mission::new(&config(5.0, Some((13.736717, 100.523186))));
        m.update(Some((13.746717, 100.523186))); // Armed -> Running, far away
        m.update(Some((13.736717, 100.523186))); // Arrived (distance ~0)
        assert_eq!(m.state(), MissionState::Arrived);
        // Even if fed a position again, Arrived is terminal until a new command.
        let status = m.update(Some((13.746717, 100.523186)));
        assert_eq!(m.state(), MissionState::Arrived);
        assert!(!status.active);
    }

    // ---- audit item 6: arrival, GNSS loss, freshness --------------------

    #[test]
    fn goal_already_inside_the_arrival_radius_goes_straight_to_arrived() {
        let mut m = Mission::new(&config(5.0, Some((13.736717, 100.523186))));
        let status = m.update(Some((13.736717, 100.523186)));
        assert_eq!(m.state(), MissionState::Arrived);
        assert!(!status.active, "not even one tick of active=true");
    }

    #[test]
    fn no_usable_fix_for_over_5s_pauses_then_resumes_without_cancelling() {
        let far = Some((13.746717, 100.523186));
        let mut m = Mission::new(&config(2.0, Some((13.736717, 100.523186))));
        assert!(m.update_at(far, 0.0).active);
        // Loss begins at t=1.
        assert!(m.update_at(None, 1.0).active, "a short gap keeps driving");
        assert!(m.update_at(None, 5.9).active);
        let paused = m.update_at(None, 6.5);
        assert!(
            !paused.active,
            "> 5 s with no fix must publish active=false"
        );
        assert_eq!(m.state(), MissionState::Running, "paused, not cancelled");
        assert!(m.goal().is_some());
        // Fix returns: drive again, same mission.
        let resumed = m.update_at(far, 7.0);
        assert!(resumed.active);
        assert_eq!(m.state(), MissionState::Running);
        // And a later short gap does not inherit the old loss start.
        assert!(m.update_at(None, 8.0).active);
    }

    fn fix(lat: f64, lon: f64) -> GnssFix {
        GnssFix {
            lat_deg: lat,
            lon_deg: lon,
            fix: rover_msgs::FixQuality::RtkFixed,
            sats: 12,
            ..Default::default()
        }
    }

    #[test]
    fn a_stale_fix_is_unusable() {
        let rtk = fix(1.0, 2.0);
        let backup = fix(9.0, 9.0);
        let sel = |ra, ba| select_fresh_navigation_fix(&rtk, ra, &backup, ba, GNSS_FIX_MAX_AGE_S);
        assert_eq!(sel(Some(0.1), Some(0.1)), Some((1.0, 2.0)));
        assert_eq!(
            sel(Some(3.0), Some(0.1)),
            Some((9.0, 9.0)),
            "stale RTK falls back"
        );
        assert_eq!(sel(Some(3.0), Some(3.0)), None);
        assert_eq!(sel(None, None), None, "never received");
    }

    #[test]
    fn non_finite_lat_lon_is_unusable_even_with_a_fix_type() {
        let rtk = fix(f64::NAN, 2.0);
        let backup = fix(9.0, f64::INFINITY);
        assert_eq!(select_navigation_fix(&rtk, &backup), None);
    }
}
