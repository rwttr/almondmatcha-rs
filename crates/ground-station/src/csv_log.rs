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
use std::io::{BufWriter, Write};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError, TrySendError};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Filename inside the run directory. Unchanged from when this was the
/// whole (flat) output path.
pub const FILENAME: &str = "ground_station_telemetry.csv";

pub const HEADER: &str = "Timestamp_us,Seq,Rtk_Lat,Rtk_Lon,Rtk_Fix,Rtk_Sats,\
     Backup_Lat,Backup_Lon,Backup_Fix,Speed_mps,Mission_State,Mission_Active,\
     Distance_Remaining_m,Bus_Volts,Current_A,Last_Cmd_Session,Last_Cmd_Seq,Health_Bits\n";

/// Format one received `Telemetry` frame as a CSV row. `timestamp_us` is the
/// base station's own receive-time clock, not `Telemetry::t_us` (the
/// rover's clock) — this is what lets a CSV row exist even for a comparison
/// against local wall-clock delay, same distinction
/// `docs/CSV_LOGGING.md` draws for every other `Timestamp_us` column in this
/// system.
pub fn format_row(timestamp_us: u64, t: &Telemetry) -> String {
    format!(
        "{timestamp_us},{},{:.7},{:.7},{:?},{},{:.7},{:.7},{:?},{},{:?},{},{},{},{},{},{},{}\n",
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
        t.last_cmd_session,
        t.last_cmd_seq,
        t.health.0,
    )
}

/// Rows that may wait for the writer thread. `Telemetry` arrives at ~5 Hz, so
/// this is minutes of backlog; it only exists to bound memory if the disk
/// stalls.
const QUEUE_CAPACITY: usize = 1024;
/// Rows are buffered and flushed (and `sync_data`'d) this often, bounding what
/// a crash or power cut loses to about a second of telemetry.
const FLUSH_INTERVAL: Duration = Duration::from_secs(1);
/// CSV errors repeat on every row while a disk is full or gone; log the first
/// and then at most one line per this interval, with a count of the rest.
const ERROR_LOG_INTERVAL: Duration = Duration::from_secs(10);

/// Decides whether a repeating error may be logged now. Pure (time is passed
/// in) so the rate limit is testable without sleeping.
#[derive(Debug)]
struct RateLimit {
    last: Option<Instant>,
    suppressed: u64,
}

impl RateLimit {
    fn new() -> Self {
        Self {
            last: None,
            suppressed: 0,
        }
    }

    /// `Some(n)` = log now, `n` earlier occurrences were suppressed;
    /// `None` = stay quiet (and count it).
    fn check(&mut self, now: Instant) -> Option<u64> {
        match self.last {
            Some(t) if now.saturating_duration_since(t) < ERROR_LOG_INTERVAL => {
                self.suppressed += 1;
                None
            }
            _ => {
                self.last = Some(now);
                Some(std::mem::take(&mut self.suppressed))
            }
        }
    }
}

/// Background-thread CSV writer, same non-blocking-caller principle as
/// `rover-telemetry::csv_writer::CsvLogger` (the writer-thread plumbing is
/// duplicated rather than shared — there is no common internal-utility
/// crate in this workspace for it, and this crate does not depend on
/// `rover-telemetry` — but the run-directory scheme underneath it is not:
/// both now go through `rover-runs::RunDir`, so the numbering itself cannot
/// drift between them).
///
/// Unlike before, nothing here fails silently: create/write/flush errors and
/// dropped rows (bounded queue full) are logged, rate-limited. `create_csv`
/// opens in append mode and writes the header only for a new file, so a
/// restart into the same run directory continues the file.
pub struct CsvLogger {
    tx: Option<mpsc::SyncSender<String>>,
    dropped: Arc<AtomicU64>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl CsvLogger {
    /// Spawn the writer thread. Nothing touches the filesystem — not even
    /// `run_dir`'s own directory — until the first row is enqueued;
    /// `RunDir::create_csv` creates the run directory and the file
    /// together, lazily, matching this module's own doc comment on why a
    /// silent session must consume no run number.
    pub fn spawn(run_dir: RunDir) -> Self {
        let (tx, rx) = mpsc::sync_channel::<String>(QUEUE_CAPACITY);
        let dropped = Arc::new(AtomicU64::new(0));
        let dropped_t = dropped.clone();
        let thread = std::thread::spawn(move || {
            let mut file: Option<BufWriter<std::fs::File>> = None;
            let mut dirty = false;
            let mut limit = RateLimit::new();
            let mut report = |what: String| {
                if let Some(n) = limit.check(Instant::now()) {
                    if n > 0 {
                        log::error!("{FILENAME}: {what} (+{n} similar errors suppressed)");
                    } else {
                        log::error!("{FILENAME}: {what}");
                    }
                }
            };
            let mut next_flush = Instant::now() + FLUSH_INTERVAL;
            loop {
                let timeout = next_flush.saturating_duration_since(Instant::now());
                match rx.recv_timeout(timeout) {
                    Ok(row) => {
                        if file.is_none() {
                            match run_dir.create_csv(FILENAME, HEADER) {
                                Ok(f) => file = Some(BufWriter::new(f)),
                                Err(e) => {
                                    report(format!("cannot create CSV: {e}"));
                                    continue;
                                }
                            }
                        }
                        if let Some(f) = file.as_mut() {
                            match f.write_all(row.as_bytes()) {
                                Ok(()) => dirty = true,
                                Err(e) => report(format!("write failed: {e}")),
                            }
                        }
                    }
                    Err(RecvTimeoutError::Timeout) => {}
                    Err(RecvTimeoutError::Disconnected) => break,
                }
                if Instant::now() >= next_flush {
                    next_flush = Instant::now() + FLUSH_INTERVAL;
                    if dirty {
                        if let Some(Err(e)) = file
                            .as_mut()
                            .map(|f| f.flush().and_then(|_| f.get_ref().sync_data()))
                        {
                            report(format!("flush failed: {e}"));
                        }
                        dirty = false;
                    }
                    let n = dropped_t.swap(0, Ordering::Relaxed);
                    if n > 0 {
                        report(format!("queue full, dropped {n} rows"));
                    }
                }
            }
            if let Some(Err(e)) = file
                .as_mut()
                .map(|f| f.flush().and_then(|_| f.get_ref().sync_data()))
            {
                report(format!("final flush failed: {e}"));
            }
        });
        Self {
            tx: Some(tx),
            dropped,
            thread: Some(thread),
        }
    }

    /// Never blocks: a full queue drops the row and counts it.
    pub fn log(&self, row: String) {
        if let Some(tx) = &self.tx {
            if let Err(TrySendError::Full(_)) = tx.try_send(row) {
                self.dropped.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    #[cfg(test)]
    fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

impl Drop for CsvLogger {
    /// Closing the channel lets the writer drain, flush and exit; joining
    /// makes "dropped" mean "on disk".
    fn drop(&mut self) {
        self.tx.take();
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
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
            last_cmd_session: 3,
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
            last_cmd_session: 777,
            last_cmd_seq: 42,
            ..Default::default()
        };
        t.mission.state = MissionState::Arrived;
        let row = format_row(2_000, &t);
        assert!(row.starts_with("2000,0,13.7000000,100.5000000,RtkFixed,14,"));
        assert!(row.contains("13.7100000,100.5100000,Autonomous"));
        assert!(row.contains(",777,42,"), "session then seq: {row}");
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
    fn full_queue_drops_and_counts() {
        let (tx, _rx) = mpsc::sync_channel::<String>(2);
        let logger = CsvLogger {
            tx: Some(tx),
            dropped: Arc::new(AtomicU64::new(0)),
            thread: None,
        };
        for i in 0..5 {
            logger.log(format!("{i}\n"));
        }
        assert_eq!(logger.dropped(), 3);
    }

    #[test]
    fn dropping_the_logger_flushes_buffered_rows() {
        let root = temp_runs_root("drop");
        let run_dir = RunDir::resolve(&root);
        let path = run_dir.path().join(FILENAME);
        let logger = CsvLogger::spawn(run_dir);
        logger.log("a,b\n".to_string());
        drop(logger);
        let contents = std::fs::read_to_string(&path).unwrap();
        assert_eq!(contents, format!("{HEADER}a,b\n"));
        std::fs::remove_dir_all(&root).ok();
    }

    #[test]
    fn rate_limit_logs_first_then_suppresses_and_counts() {
        let t0 = Instant::now();
        let mut r = RateLimit::new();
        assert_eq!(r.check(t0), Some(0));
        assert_eq!(r.check(t0 + Duration::from_secs(1)), None);
        assert_eq!(r.check(t0 + Duration::from_secs(2)), None);
        assert_eq!(r.check(t0 + ERROR_LOG_INTERVAL), Some(2));
        assert_eq!(
            r.check(t0 + ERROR_LOG_INTERVAL + Duration::from_secs(1)),
            None
        );
    }

    #[test]
    fn a_create_failure_does_not_kill_the_writer_or_panic() {
        // Root is a *file*, so create_dir_all inside create_csv fails.
        let root = temp_runs_root("createfail");
        let blocker = root.join("blocker");
        std::fs::write(&blocker, "x").unwrap();
        let run_dir = RunDir::resolve(&blocker);
        let logger = CsvLogger::spawn(run_dir);
        logger.log("a\n".to_string());
        logger.log("b\n".to_string());
        drop(logger); // joins: would hang or panic if the thread died badly
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
