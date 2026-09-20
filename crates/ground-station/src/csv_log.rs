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
//!
//! # `run_NNN_<stamp>/` directories, via `rover-runs`
//!
//! This used to write one flat `ground_station_telemetry.csv` next to the
//! binary. It now lands inside a `run_NNN_<stamp>/` directory, the same
//! scheme `rover-telemetry` uses, via the shared `rover-runs` crate — so a
//! base-station capture and a rover capture from the same session are named
//! the same way and can be paired up by run number. `rover-runs::RunDir`
//! keeps the behaviour this module always had: nothing on disk (not even
//! the run directory itself) is created until the first row is logged, so a
//! session with no telemetry consumes no run number and leaves nothing
//! behind. See `rover-runs`'s own doc comment for why this is a shared
//! crate rather than a second, hand-copied implementation.

use rover_msgs::Telemetry;
use rover_runs::RunDir;
use std::io::Write;
use std::sync::mpsc;

/// Filename inside the run directory. Unchanged from when this was the
/// whole (flat) output path.
pub const FILENAME: &str = "ground_station_telemetry.csv";

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
/// `rover-telemetry::csv_writer::CsvLogger` (the writer-thread plumbing is
/// duplicated rather than shared — there is no common internal-utility
/// crate in this workspace for it, and this crate does not depend on
/// `rover-telemetry` — but the run-directory scheme underneath it is not:
/// both now go through `rover-runs::RunDir`, so the numbering itself cannot
/// drift between them).
pub struct CsvLogger {
    tx: mpsc::Sender<String>,
}

impl CsvLogger {
    /// Spawn the writer thread. Nothing touches the filesystem — not even
    /// `run_dir`'s own directory — until the first row is enqueued;
    /// `RunDir::create_csv` creates the run directory and the file
    /// together, lazily, matching this module's own doc comment on why a
    /// silent session must consume no run number.
    pub fn spawn(run_dir: RunDir) -> Self {
        let (tx, rx) = mpsc::channel::<String>();
        std::thread::spawn(move || {
            let mut file = None;
            for row in rx {
                if file.is_none() {
                    file = run_dir.create_csv(FILENAME, HEADER).ok();
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

    fn temp_runs_root(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "ground-station-csvlog-test-{}-{name}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn csv_logger_writes_header_then_rows_to_disk() {
        let root = temp_runs_root("basic");
        let run_dir = RunDir::resolve(&root);
        let expected_path = run_dir.path().join(FILENAME);
        let logger = CsvLogger::spawn(run_dir);
        logger.log("a,b\n".to_string());

        assert!(
            wait_for(
                || std::fs::read_to_string(&expected_path)
                    .map(|s| s.lines().count() >= 2)
                    .unwrap_or(false),
                Duration::from_secs(2),
            ),
            "row was never written"
        );
        let contents = std::fs::read_to_string(&expected_path).unwrap();
        assert!(contents.starts_with(HEADER));
        assert!(contents.ends_with("a,b\n"));

        std::fs::remove_dir_all(&root).ok();
    }

    #[test]
    fn no_run_directory_is_created_if_nothing_is_ever_logged() {
        let root = temp_runs_root("no_log");
        let run_dir = RunDir::resolve(&root);
        let run_dir_path = run_dir.path().to_path_buf();
        let _logger = CsvLogger::spawn(run_dir);

        std::thread::sleep(Duration::from_millis(50));
        assert!(
            !run_dir_path.exists(),
            "a silent session must not create a run directory or consume a run number"
        );

        std::fs::remove_dir_all(&root).ok();
    }
}
