//! Logging received `Telemetry` to CSV.
//!
//! **Note on port fidelity:** `docs/CSV_LOGGING.md` is explicit that the
//! ROS 2 base station did **not** log CSVs at all ("No CSV logging on base
//! station (display-only)"), and `mission_monitoring_node_pc.cpp` confirms
//! it — the node only ever calls `RCLCPP_INFO`. This module exists anyway
//! because the task brief for this crate asks for it explicitly ("Log
//! received telemetry to CSV as the ROS 2 base node did"). Flagged here
//! rather than silently reconciled one way or the other: the two sources
//! disagree, and this implementation follows the brief.
//!
//! Formatting (pure, tested below) is kept separate from the background
//! writer thread, for the same reason `rover-telemetry`'s `csv_fmt.rs` /
//! `csv_writer.rs` split exists: so the row shape can be checked without a
//! filesystem or a thread.

use rover_msgs::Telemetry;
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::Path;
use std::sync::mpsc;

pub const HEADER: &str = "Timestamp_us,Seq,Rtk_Lat,Rtk_Lon,Rtk_Fix,Rtk_Sats,\
     Backup_Lat,Backup_Lon,Backup_Fix,Speed_mps,Mission_State,Mission_Active,\
     Distance_Remaining_m,Bus_Volts,Current_A,Last_Cmd_Seq,Health_Bits\n";

/// Format one received `Telemetry` frame as a CSV row. `timestamp_us` is the
/// base station's own receive-time clock, not `Telemetry::t_us` (the
/// rover's clock) — this is what lets a CSV row exist even for a comparison
/// against local wall-clock delay, same distinction
/// `docs/CSV_LOGGING.md` draws for every other `Timestamp_us` column in this
/// system.
pub fn format_row(timestamp_us: u64, t: &Telemetry) -> String {
    format!(
        "{timestamp_us},{},{:.7},{:.7},{:?},{},{:.7},{:.7},{:?},{},{:?},{},{},{},{},{},{}\n",
        t.seq,
        t.rtk.lat_deg,
        t.rtk.lon_deg,
        t.rtk.fix,
        t.rtk.sats,
        t.backup.lat_deg,
        t.backup.lon_deg,
        t.backup.fix,
        t.state.speed_mps,
        t.mission.state,
        t.mission.active,
        t.mission.distance_remaining_m,
        t.power.bus_volts,
        t.power.current_amps,
        t.last_cmd_seq,
        t.health.0,
    )
}

/// Background-thread CSV writer, same non-blocking-caller principle as
/// `rover-telemetry::csv_writer::CsvLogger` (duplicated rather than shared —
/// there is no common internal-utility crate in this workspace, and this
/// crate does not depend on `rover-telemetry`).
pub struct CsvLogger {
    tx: mpsc::Sender<String>,
}

impl CsvLogger {
    pub fn spawn(path: impl AsRef<Path>) -> Self {
        let path = path.as_ref().to_path_buf();
        let (tx, rx) = mpsc::channel::<String>();
        std::thread::spawn(move || {
            let mut file: Option<File> = None;
            for row in rx {
                if file.is_none() {
                    file = open_with_header(&path).ok();
                    if file.is_none() {
                        continue;
                    }
                }
                if let Some(f) = file.as_mut() {
                    let _ = f.write_all(row.as_bytes());
                }
            }
        });
        Self { tx }
    }

    pub fn log(&self, row: String) {
        let _ = self.tx.send(row);
    }
}

fn open_with_header(path: &Path) -> std::io::Result<File> {
    let is_new = !path.exists();
    let mut file = OpenOptions::new().create(true).append(true).open(path)?;
    if is_new {
        file.write_all(HEADER.as_bytes())?;
    }
    Ok(file)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rover_msgs::{FixQuality, GnssFix, MissionState, MissionStatus, PowerSample, RoverState};
    use std::time::{Duration, Instant};

    #[test]
    fn header_ends_with_newline_and_matches_row_column_count() {
        let t = Telemetry {
            seq: 1,
            t_us: 0,
            state: RoverState {
                speed_mps: 0.5,
                ..Default::default()
            },
            mission: MissionStatus {
                state: MissionState::Running,
                active: true,
                distance_remaining_m: 3.0,
                target: None,
            },
            power: PowerSample {
                bus_volts: 12.0,
                current_amps: 1.0,
            },
            rtk: GnssFix {
                fix: FixQuality::RtkFixed,
                sats: 12,
                ..Default::default()
            },
            backup: GnssFix::default(),
            last_cmd_seq: 5,
            health: rover_msgs::HealthBits::NONE,
        };
        let row = format_row(1_000, &t);
        assert!(HEADER.ends_with('\n'));
        assert_eq!(
            HEADER.trim_end().split(',').count(),
            row.trim_end().split(',').count()
        );
    }

    #[test]
    fn row_reports_both_gnss_streams_and_the_ack() {
        let mut t = Telemetry {
            rtk: GnssFix {
                lat_deg: 13.7,
                lon_deg: 100.5,
                fix: FixQuality::RtkFixed,
                sats: 14,
                ..Default::default()
            },
            backup: GnssFix {
                lat_deg: 13.71,
                lon_deg: 100.51,
                fix: FixQuality::Autonomous,
                sats: 6,
                ..Default::default()
            },
            last_cmd_seq: 42,
            ..Default::default()
        };
        t.mission.state = MissionState::Arrived;
        let row = format_row(2_000, &t);
        assert!(row.starts_with("2000,0,13.7000000,100.5000000,RtkFixed,14,"));
        assert!(row.contains("13.7100000,100.5100000,Autonomous"));
        assert!(row.contains(",42,"));
        assert!(row.contains("Arrived"));
    }

    fn wait_for<F: Fn() -> bool>(f: F, timeout: Duration) -> bool {
        let start = Instant::now();
        while start.elapsed() < timeout {
            if f() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        false
    }

    #[test]
    fn csv_logger_writes_header_then_rows_to_disk() {
        let path = std::env::temp_dir().join(format!(
            "ground-station-csvlog-test-{}.csv",
            std::process::id()
        ));
        let _ = std::fs::remove_file(&path);

        let logger = CsvLogger::spawn(&path);
        logger.log("a,b\n".to_string());

        assert!(
            wait_for(
                || std::fs::read_to_string(&path)
                    .map(|s| s.lines().count() >= 2)
                    .unwrap_or(false),
                Duration::from_secs(2),
            ),
            "row was never written"
        );
        let contents = std::fs::read_to_string(&path).unwrap();
        assert!(contents.starts_with(HEADER));
        assert!(contents.ends_with("a,b\n"));

        std::fs::remove_file(&path).ok();
    }
}
