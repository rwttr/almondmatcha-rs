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

use crate::link::LinkStatus;
use rover_msgs::{GnssFix, HealthBits, MissionStatus, PowerSample, RoverState};

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
const HEALTH_FLAG_LABELS: &[(HealthBits, &str)] = &[
    (HealthBits::CHASSIS_STALE, "CHASSIS_STALE"),
    (HealthBits::SENSORS_STALE, "SENSORS_STALE"),
    (HealthBits::LANE_STALE, "LANE_STALE"),
    (HealthBits::RTK_STALE, "RTK_STALE"),
    (HealthBits::BACKUP_GNSS_STALE, "BACKUP_GNSS_STALE"),
    (HealthBits::WATCHDOG_TRIPPED, "WATCHDOG_TRIPPED"),
    (HealthBits::STALL_DETECTED, "STALL_DETECTED"),
    (HealthBits::ESTIMATOR_DIVERGED, "ESTIMATOR_DIVERGED"),
];

pub fn format_health_line(health: HealthBits) -> String {
    let flags: Vec<&str> = HEALTH_FLAG_LABELS
        .iter()
        .filter(|(bit, _)| health.contains(*bit))
        .map(|(_, label)| *label)
        .collect();
    if flags.is_empty() {
        "Health:   OK".to_string()
    } else {
        format!("Health:   {}", flags.join(", "))
    }
}

pub fn format_command_line(pending_cmd_seq: Option<u16>, last_acked_cmd_seq: u16) -> String {
    match pending_cmd_seq {
        Some(seq) => {
            format!("Command:  cmd_seq={seq} pending ack (rover last echoed {last_acked_cmd_seq})")
        }
        None => format!("Command:  none pending (rover last echoed {last_acked_cmd_seq})"),
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
    pending_cmd_seq: Option<u16>,
    last_acked_cmd_seq: u16,
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
    out.push_str(&format_command_line(pending_cmd_seq, last_acked_cmd_seq));
    out.push('\n');
    out.push_str(ESTOP_DISCLAIMER);
    out.push('\n');
    out.push_str("commands: estop | cancel | goal <lat> <lon> | speed <pct> | nop\n");
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

    #[test]
    fn command_line_distinguishes_pending_from_acked() {
        assert!(format_command_line(Some(7), 6).contains("cmd_seq=7 pending"));
        assert!(format_command_line(None, 6).contains("none pending"));
        assert!(format_command_line(None, 6).contains("echoed 6"));
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
            None,
            0,
        );
        assert!(out.contains(ESTOP_DISCLAIMER));
        assert!(out.starts_with("\x1B[2J\x1B[H"));
    }
}
