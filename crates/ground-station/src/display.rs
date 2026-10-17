//! Rendering the live dashboard as plain text with ANSI clear/home codes.
//!
//! **Why plain ANSI, not `ratatui`:** the display is eight fixed scalar
//! lines refreshed at a fixed rate — no scrolling lists, no mouse handling,
//! no nested widget layout, nothing a retained-mode TUI framework earns its
//! complexity back on. A `print!("\x1B[2J\x1B[H")` clear-and-redraw, the
//! same trick `mission_monitoring_node_pc.cpp` approximated by leaving a
//! commented-out `"\033[2J\033[1;1H"` in place (it shipped without it,
//! relying on ROS 2's log scrollback instead), is enough, and it keeps every
//! line of this module a plain `&str -> String` function a test can call
//! with no terminal, no event loop and no extra dependency at all.
//!
//! Every function here is pure: given a value, return the string. `main.rs`
//! is the only place that actually writes to a terminal.

use crate::command_parse::command_hint;
use crate::link::LinkStatus;
use rover_msgs::{Command, GnssFix, HealthBits, MissionStatus, PowerSample, RoverState};

use std::time::{Duration, Instant};

/// Printed once, unconditionally, on every redraw and again immediately
/// after an E-stop is sent — the whole point of "label it honestly" is that
/// an operator cannot miss it in the one moment it matters most.
pub const ESTOP_DISCLAIMER: &str =
    "E-STOP is BEST-EFFORT over the network, not a safety guarantee. The \
     guaranteed stop is the onboard firmware command watchdog (200 ms \
     timeout + 300 ms ramp-to-zero) -- it needs no packet from here to fire.";

pub fn format_link_line(status: LinkStatus) -> String {
    match status {
        LinkStatus::Live { age_ms } => format!("Link:     UP   (last telemetry {age_ms} ms ago)"),
        LinkStatus::Lost { age_ms } if age_ms == u64::MAX => {
            "Link:     DOWN (no telemetry received yet)".to_string()
        }
        LinkStatus::Lost { age_ms } => format!(
            "Link:     DOWN (no telemetry for {age_ms} ms) -- rover continues its \
             mission autonomously; this link is observability + optional override only"
        ),
    }
}

pub fn format_position_line(fix: &GnssFix) -> String {
    format!(
        "Position: {:.7}, {:.7}  alt={:.1} m  fix={:?}  sats={}  h_acc={:.2} m",
        fix.lat_deg, fix.lon_deg, fix.alt_m, fix.fix, fix.sats, fix.h_acc_m
    )
}

pub fn format_speed_line(state: &RoverState) -> String {
    format!("Speed:    {:.2} m/s", state.speed_mps)
}

pub fn format_mission_line(mission: &MissionStatus) -> String {
    let target = mission
        .target
        .map(|g| format!("{:.6}, {:.6}", g.lat_deg, g.lon_deg))
        .unwrap_or_else(|| "none".to_string());
    format!(
        "Mission:  {:?}  active={}  distance_remaining={:.1} m  target=({target})",
        mission.state, mission.active, mission.distance_remaining_m
    )
}

pub fn format_power_line(power: &PowerSample) -> String {
    format!(
        "Power:    {:.2} V  {:.2} A  {:.1} W",
        power.bus_volts,
        power.current_amps,
        power.watts()
    )
}

/// Every [`HealthBits`] flag paired with the label shown in the dashboard.
/// One place, so the display and any future consumer name the same flags
/// the same way.
///
/// Must name every flag `rover-msgs` defines: the dashboard is the operator's
/// only view of health, and an unnamed bit reads as "Health: OK" (bits 8-10
/// once did). `format_health_line` also prints any bit missing from this
/// table as `UNKNOWN_BIT_n`, so a flag added later cannot vanish silently.
const HEALTH_FLAG_LABELS: &[(HealthBits, &str)] = &[
    (HealthBits::CHASSIS_STALE, "CHASSIS_STALE"),
    (HealthBits::SENSORS_STALE, "SENSORS_STALE"),
    (HealthBits::LANE_STALE, "LANE_STALE"),
    (HealthBits::RTK_STALE, "RTK_STALE"),
    (HealthBits::BACKUP_GNSS_STALE, "BACKUP_GNSS_STALE"),
    (HealthBits::WATCHDOG_TRIPPED, "WATCHDOG_TRIPPED"),
    (HealthBits::STALL_DETECTED, "STALL_DETECTED"),
    (HealthBits::ESTIMATOR_DIVERGED, "ESTIMATOR_DIVERGED"),
    (HealthBits::BOARD_POST_FAIL, "BOARD_POST_FAIL"),
    (HealthBits::BOARD_RESET, "BOARD_RESET"),
    (HealthBits::LINK_DEGRADED, "LINK_DEGRADED"),
];

pub fn format_health_line(health: HealthBits) -> String {
    let mut flags: Vec<String> = HEALTH_FLAG_LABELS
        .iter()
        .filter(|(bit, _)| health.contains(*bit))
        .map(|(_, label)| label.to_string())
        .collect();
    let known = HEALTH_FLAG_LABELS.iter().fold(0u16, |m, (b, _)| m | b.0);
    for n in 0..16 {
        if health.0 & !known & (1 << n) != 0 {
            flags.push(format!("UNKNOWN_BIT_{n}"));
        }
    }
    if flags.is_empty() {
        "Health:   OK".to_string()
    } else {
        format!("Health:   {}", flags.join(", "))
    }
}

/// How long the last operator message (command error, E-STOP discard
/// warning, "Sending E-STOP") stays on screen. The dashboard clears and
/// redraws every 200 ms, so anything merely `println!`ed between redraws is
/// gone almost at once — exactly when the operator most needs to read it.
pub const NOTICE_HOLD: Duration = Duration::from_secs(5);

/// The last operator-facing message and when it was raised.
#[derive(Debug, Default)]
pub struct Notice {
    current: Option<(String, Instant)>,
}

impl Notice {
    /// Replace the message (the newest always wins) and restart its timer.
    pub fn set(&mut self, text: String, now: Instant) {
        self.current = Some((text, now));
    }

    /// The message, if it is still within [`NOTICE_HOLD`].
    pub fn visible(&self, now: Instant) -> Option<&str> {
        self.current
            .as_ref()
            .filter(|(_, at)| now.saturating_duration_since(*at) < NOTICE_HOLD)
            .map(|(t, _)| t.as_str())
    }
}

/// `CommandSender` state for the dashboard.
#[derive(Debug, Clone, PartialEq)]
pub struct CommandStatus {
    /// `(cmd_seq, command)` awaiting an ack.
    pub in_flight: Option<(u16, Command)>,
    /// Commands queued behind the in-flight one.
    pub queue_len: usize,
    pub session: u16,
    /// `Telemetry::last_cmd_seq` — what the rover last echoed, but only when
    /// the echo belongs to OUR session. `None` = no ack for this session seen
    /// yet: a seq echoed under another session (the previous run of this
    /// program, or another sender) or 0 ("no consensus") would sit next to
    /// our own cmd_seqs and read as if it were ours.
    pub last_acked_seq: Option<u16>,
}

pub fn format_command_line(c: &CommandStatus) -> String {
    let queue = if c.queue_len > 0 {
        format!(", {} queued", c.queue_len)
    } else {
        String::new()
    };
    let echoed = match c.last_acked_seq {
        Some(seq) => format!("rover last echoed {seq}"),
        None => "no ack yet this session".to_string(),
    };
    match &c.in_flight {
        Some((seq, cmd)) => format!(
            "Command:  {cmd:?} cmd_seq={seq} pending ack{queue} ({echoed}; session {:#06x})",
            c.session
        ),
        None => format!(
            "Command:  none pending ({echoed}; session {:#06x})",
            c.session
        ),
    }
}

/// Full-screen redraw. ANSI `\x1B[2J\x1B[H` clears the screen and homes the
/// cursor before the fixed set of lines, so each redraw overwrites the
/// previous one rather than scrolling — the closest plain-terminal
/// equivalent of a TUI's full repaint.
#[allow(clippy::too_many_arguments)]
pub fn render_dashboard(
    link: LinkStatus,
    rtk: &GnssFix,
    backup: &GnssFix,
    state: &RoverState,
    mission: &MissionStatus,
    power: &PowerSample,
    health: HealthBits,
    command: &CommandStatus,
    notice: Option<&str>,
) -> String {
    let mut out = String::new();
    out.push_str("\x1B[2J\x1B[H");
    out.push_str("=== Rover Ground Station ===\n");
    out.push_str(&format_link_line(link));
    out.push('\n');
    out.push_str("-- RTK --\n");
    out.push_str(&format_position_line(rtk));
    out.push('\n');
    out.push_str("-- Backup GNSS --\n");
    out.push_str(&format_position_line(backup));
    out.push('\n');
    out.push_str(&format_speed_line(state));
    out.push('\n');
    out.push_str(&format_mission_line(mission));
    out.push('\n');
    out.push_str(&format_power_line(power));
    out.push('\n');
    out.push_str(&format_health_line(health));
    out.push('\n');
    out.push_str(&format_command_line(command));
    out.push('\n');
    out.push_str(ESTOP_DISCLAIMER);
    out.push('\n');
    out.push_str(&command_hint());
    out.push('\n');
    if let Some(n) = notice {
        out.push_str(">> ");
        out.push_str(n);
        out.push('\n');
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use rover_msgs::{FixQuality, MissionGoal, MissionState};

    #[test]
    fn link_up_shows_age() {
        assert_eq!(
            format_link_line(LinkStatus::Live { age_ms: 42 }),
            "Link:     UP   (last telemetry 42 ms ago)"
        );
    }

    #[test]
    fn link_never_seen_says_so_distinctly_from_a_timed_out_link() {
        let never = format_link_line(LinkStatus::Lost { age_ms: u64::MAX });
        let timed_out = format_link_line(LinkStatus::Lost { age_ms: 5000 });
        assert!(never.contains("no telemetry received yet"));
        assert!(timed_out.contains("5000 ms"));
        assert_ne!(never, timed_out);
    }

    #[test]
    fn health_ok_when_no_bits_set() {
        assert_eq!(format_health_line(HealthBits::NONE), "Health:   OK");
    }

    #[test]
    fn health_lists_every_set_flag() {
        let mut h = HealthBits::NONE;
        h.set(HealthBits::RTK_STALE);
        h.set(HealthBits::WATCHDOG_TRIPPED);
        let line = format_health_line(h);
        assert!(line.contains("RTK_STALE"));
        assert!(line.contains("WATCHDOG_TRIPPED"));
        assert!(!line.contains("LANE_STALE"));
    }

    #[test]
    fn mission_line_reports_no_target_explicitly() {
        let m = MissionStatus {
            active: false,
            distance_remaining_m: 0.0,
            target: None,
            state: MissionState::Idle,
        };
        assert!(format_mission_line(&m).contains("target=(none)"));
    }

    #[test]
    fn mission_line_reports_a_target_when_present() {
        let m = MissionStatus {
            active: true,
            distance_remaining_m: 5.0,
            target: Some(MissionGoal {
                lat_deg: 13.7,
                lon_deg: 100.5,
            }),
            state: MissionState::Running,
        };
        let line = format_mission_line(&m);
        assert!(line.contains("13.700000"));
        assert!(line.contains("100.500000"));
        assert!(line.contains("Running"));
    }

    fn idle_status() -> CommandStatus {
        CommandStatus {
            in_flight: None,
            queue_len: 0,
            session: 0xabcd,
            last_acked_seq: Some(6),
        }
    }

    #[test]
    fn command_line_distinguishes_pending_from_acked() {
        let mut c = idle_status();
        c.in_flight = Some((7, Command::EStop));
        let line = format_command_line(&c);
        assert!(line.contains("cmd_seq=7 pending"));
        assert!(line.contains("EStop"));
        let idle = format_command_line(&idle_status());
        assert!(idle.contains("none pending"));
        assert!(idle.contains("echoed 6"));
        assert!(idle.contains("0xabcd"));
    }

    #[test]
    fn command_line_says_no_ack_yet_for_a_foreign_session_echo() {
        let mut c = idle_status();
        c.last_acked_seq = None;
        let idle = format_command_line(&c);
        assert!(idle.contains("no ack yet this session"));
        assert!(!idle.contains("echoed"));
        c.in_flight = Some((1, Command::Nop));
        assert!(format_command_line(&c).contains("no ack yet this session"));
    }

    #[test]
    fn command_line_shows_queue_length() {
        let mut c = idle_status();
        c.in_flight = Some((7, Command::Nop));
        c.queue_len = 3;
        assert!(format_command_line(&c).contains("3 queued"));
        c.queue_len = 0;
        assert!(!format_command_line(&c).contains("queued"));
    }

    #[test]
    fn every_defined_health_bit_has_a_label() {
        // Bits 0..=10 are everything rover-msgs defines as of writing; the
        // last one named is the highest, so a new bit past it needs a label
        // (and this range extending) -- until then the UNKNOWN_BIT fallback
        // below keeps it visible.
        for n in 0..=10u16 {
            assert!(
                HEALTH_FLAG_LABELS.iter().any(|(b, _)| b.0 == 1 << n),
                "health bit {n} has no dashboard label"
            );
            let line = format_health_line(HealthBits(1 << n));
            assert_ne!(line, "Health:   OK", "bit {n}");
            assert!(!line.contains("UNKNOWN"), "bit {n}: {line}");
        }
        for b in [
            HealthBits::BOARD_POST_FAIL,
            HealthBits::BOARD_RESET,
            HealthBits::LINK_DEGRADED,
        ] {
            assert!(!format_health_line(b).contains("OK"));
        }
    }

    #[test]
    fn undefined_health_bits_are_still_shown() {
        let line = format_health_line(HealthBits(1 << 12));
        assert!(line.contains("UNKNOWN_BIT_12"), "{line}");
    }

    #[test]
    fn notice_persists_for_the_hold_time_then_clears() {
        let t0 = Instant::now();
        let mut n = Notice::default();
        assert_eq!(n.visible(t0), None);
        n.set("command error: x".to_string(), t0);
        assert_eq!(
            n.visible(t0 + Duration::from_millis(400)),
            Some("command error: x")
        );
        assert!(n
            .visible(t0 + NOTICE_HOLD - Duration::from_millis(1))
            .is_some());
        assert_eq!(n.visible(t0 + NOTICE_HOLD), None);
        // The newest message replaces and restarts the timer.
        n.set("second".to_string(), t0 + Duration::from_secs(4));
        assert_eq!(n.visible(t0 + Duration::from_secs(7)), Some("second"));
    }

    #[test]
    fn dashboard_shows_the_notice_and_the_full_command_hint() {
        let out = render_dashboard(
            LinkStatus::Live { age_ms: 10 },
            &GnssFix::default(),
            &GnssFix::default(),
            &RoverState::default(),
            &MissionStatus::default(),
            &PowerSample::default(),
            HealthBits::NONE,
            &idle_status(),
            Some("command error: boom"),
        );
        assert!(out.contains(">> command error: boom"));
        assert!(out.contains("clearestop"));
    }

    #[test]
    fn estop_disclaimer_never_claims_a_guarantee_for_the_network_command() {
        assert!(ESTOP_DISCLAIMER.to_lowercase().contains("best-effort"));
        assert!(ESTOP_DISCLAIMER.to_lowercase().contains("watchdog"));
    }

    #[test]
    fn render_dashboard_includes_the_estop_disclaimer_every_time() {
        let out = render_dashboard(
            LinkStatus::Live { age_ms: 10 },
            &GnssFix {
                fix: FixQuality::RtkFixed,
                ..Default::default()
            },
            &GnssFix::default(),
            &RoverState::default(),
            &MissionStatus::default(),
            &PowerSample::default(),
            HealthBits::NONE,
            &idle_status(),
            None,
        );
        assert!(out.contains(ESTOP_DISCLAIMER));
        assert!(out.starts_with("\x1B[2J\x1B[H"));
    }
}
