//! Pure CSV row formatting, one function per logged topic.
//!
//! Kept separate from the background writer (`csv_writer.rs`) so the
//! question "is this row shaped right" never needs a thread, a channel or a
//! filesystem to answer — it is a `String -> String` comparison in a unit
//! test.
//!
//! `FixQuality`'s `{:?}` name (`RtkFixed`, `Autonomous`, ...) is used
//! directly as the fix-quality column, rather than a hand-maintained string
//! table like `gnss_ublox_node.cpp`'s `getFixQuality`. The two can never
//! drift apart because there is only one source of the name.

use rover_msgs::{ChassisStatus, GnssFix, MissionStatus, PowerSample, RoverState, SpeedLoopDebug};

pub const ROVER_STATE_HEADER: &str =
    "Timestamp_us,Cross_Track_m,Heading_Err_rad,Curvature_inv_m,Speed_mps,\
     Gyro_Bias_radps,P_CrossTrack,P_Heading,P_Curvature,P_Speed,P_GyroBias,Lane_Age_ms\n";

pub fn format_rover_state_row(timestamp_us: u64, s: &RoverState) -> String {
    format!(
        "{timestamp_us},{},{},{},{},{},{},{},{},{},{},{}\n",
        s.cross_track_m,
        s.heading_err_rad,
        s.curvature_inv_m,
        s.speed_mps,
        s.gyro_bias_radps,
        s.p_diag[0],
        s.p_diag[1],
        s.p_diag[2],
        s.p_diag[3],
        s.p_diag[4],
        s.lane_age_ms,
    )
}

pub const MISSION_STATUS_HEADER: &str =
    "Timestamp_us,Active,Distance_Remaining_m,Target_Lat,Target_Lon,State\n";

pub fn format_mission_status_row(timestamp_us: u64, m: &MissionStatus) -> String {
    let (lat, lon) = m
        .target
        .map(|g| (g.lat_deg, g.lon_deg))
        .unwrap_or((0.0, 0.0));
    format!(
        "{timestamp_us},{},{},{},{},{:?}\n",
        m.active, m.distance_remaining_m, lat, lon, m.state
    )
}

pub const POWER_HEADER: &str = "Timestamp_us,Bus_Volts,Current_A,Power_W\n";

pub fn format_power_row(timestamp_us: u64, p: &PowerSample) -> String {
    format!(
        "{timestamp_us},{},{},{}\n",
        p.bus_volts,
        p.current_amps,
        p.watts()
    )
}

pub const GNSS_HEADER: &str =
    "Timestamp_us,Lat_deg,Lon_deg,Alt_m,Fix_Quality,Satellites,H_Acc_m,Speed_mps,Course_deg,Utc_ms\n";

pub fn format_gnss_row(timestamp_us: u64, f: &GnssFix) -> String {
    format!(
        "{timestamp_us},{:.8},{:.8},{},{:?},{},{},{},{},{}\n",
        f.lat_deg,
        f.lon_deg,
        f.alt_m,
        f.fix,
        f.sats,
        f.h_acc_m,
        f.speed_mps,
        f.course_deg,
        f.utc_ms
    )
}

pub const CHASSIS_STATUS_HEADER: &str =
    "Timestamp_us,Seq_Echo,Watchdog_Tripped,Fault_Bits,Board_t_us\n";

pub fn format_chassis_status_row(timestamp_us: u64, c: &ChassisStatus) -> String {
    format!(
        "{timestamp_us},{},{},{},{}\n",
        c.seq_echo, c.watchdog_tripped, c.fault.0, c.t_us
    )
}

pub const SPEED_LOOP_DEBUG_HEADER: &str =
    "Timestamp_us,Measured_Left_TPS,Measured_Right_TPS,Target_TPS,Error_Pct,PID_Output_Pct\n";

pub fn format_speed_loop_debug_row(timestamp_us: u64, d: &SpeedLoopDebug) -> String {
    format!(
        "{timestamp_us},{},{},{},{},{}\n",
        d.measured_left_tps, d.measured_right_tps, d.target_tps, d.error_pct, d.pid_output_pct
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use rover_msgs::{FixQuality, MissionGoal, MissionState};

    fn assert_same_column_count(header: &str, row: &str) {
        assert_eq!(
            header.trim_end().split(',').count(),
            row.trim_end().split(',').count(),
            "header {header:?} and row {row:?} column counts differ"
        );
    }

    #[test]
    fn rover_state_row_matches_expected_text() {
        let s = RoverState {
            cross_track_m: 0.1,
            heading_err_rad: 0.02,
            curvature_inv_m: 0.5,
            speed_mps: 1.5,
            gyro_bias_radps: 0.001,
            p_diag: [1.0, 2.0, 3.0, 4.0, 5.0],
            lane_age_ms: 42,
        };
        let row = format_rover_state_row(1_000, &s);
        assert_eq!(row, "1000,0.1,0.02,0.5,1.5,0.001,1,2,3,4,5,42\n");
        assert_same_column_count(ROVER_STATE_HEADER, &row);
    }

    #[test]
    fn mission_status_row_with_no_target_uses_zero_placeholders() {
        let m = MissionStatus {
            active: false,
            distance_remaining_m: 0.0,
            target: None,
            state: MissionState::Idle,
        };
        let row = format_mission_status_row(2_000, &m);
        assert_eq!(row, "2000,false,0,0,0,Idle\n");
        assert_same_column_count(MISSION_STATUS_HEADER, &row);
    }

    #[test]
    fn mission_status_row_with_target_reports_its_coordinates() {
        let m = MissionStatus {
            active: true,
            distance_remaining_m: 12.5,
            target: Some(MissionGoal {
                lat_deg: 13.7,
                lon_deg: 100.5,
            }),
            state: MissionState::Running,
        };
        let row = format_mission_status_row(3_000, &m);
        assert_eq!(row, "3000,true,12.5,13.7,100.5,Running\n");
        assert_same_column_count(MISSION_STATUS_HEADER, &row);
    }

    #[test]
    fn power_row_includes_the_derived_watts() {
        let p = PowerSample {
            bus_volts: 12.0,
            current_amps: 2.0,
        };
        let row = format_power_row(4_000, &p);
        assert_eq!(row, "4000,12,2,24\n");
        assert_same_column_count(POWER_HEADER, &row);
    }

    #[test]
    fn gnss_row_shows_the_enum_name_as_fix_quality() {
        let f = GnssFix {
            lat_deg: 13.736717,
            lon_deg: 100.523186,
            alt_m: 12.0,
            fix: FixQuality::RtkFixed,
            sats: 14,
            h_acc_m: 0.02,
            speed_mps: 0.5,
            course_deg: 90.0,
            utc_ms: 1_705_321_845_000,
        };
        let row = format_gnss_row(5_000, &f);
        assert_eq!(
            row,
            "5000,13.73671700,100.52318600,12,RtkFixed,14,0.02,0.5,90,1705321845000\n"
        );
        assert_same_column_count(GNSS_HEADER, &row);
    }

    #[test]
    fn chassis_status_row_reports_the_raw_fault_byte() {
        let c = ChassisStatus {
            seq_echo: 7,
            watchdog_tripped: true,
            fault: rover_msgs::FaultBits::MOTOR_FAULT,
            t_us: 123,
        };
        let row = format_chassis_status_row(6_000, &c);
        assert_eq!(row, "6000,7,true,2,123\n");
        assert_same_column_count(CHASSIS_STATUS_HEADER, &row);
    }

    #[test]
    fn speed_loop_debug_row_matches_expected_text() {
        let d = SpeedLoopDebug {
            measured_left_tps: 100.0,
            measured_right_tps: 98.0,
            target_tps: 100.0,
            error_pct: 1.5,
            pid_output_pct: 32.0,
        };
        let row = format_speed_loop_debug_row(7_000, &d);
        assert_eq!(row, "7000,100,98,100,1.5,32\n");
        assert_same_column_count(SPEED_LOOP_DEBUG_HEADER, &row);
    }

    #[test]
    fn every_header_ends_with_a_newline() {
        for header in [
            ROVER_STATE_HEADER,
            MISSION_STATUS_HEADER,
            POWER_HEADER,
            GNSS_HEADER,
            CHASSIS_STATUS_HEADER,
            SPEED_LOOP_DEBUG_HEADER,
        ] {
            assert!(header.ends_with('\n'), "{header:?} must end with a newline");
        }
    }
}
