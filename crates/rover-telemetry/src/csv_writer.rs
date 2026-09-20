//! Background-thread CSV writing.
//!
//! `rover_monitoring_node.cpp` wrote each CSV row synchronously inside its
//! subscription callback, but on its own background-thread-and-queue
//! principle for other I/O (`docs/CSV_LOGGING.md`: "All three write
//! asynchronously: the owning node enqueues a row/frame and a background
//! thread drains the queue and writes it to disk, so a slow eMMC/SD card
//! never blocks the image-processing or control callback" — describing the
//! Jetson-side loggers, but the same reasoning applies here and the task
//! brief calls it out explicitly). One [`CsvLogger`] owns one file and one
//! background thread; [`CsvLogger::log`] only ever pushes onto a channel, so
//! a stalled disk degrades to a growing queue in memory, never a blocked
//! caller.

use rover_runs::RunDir;
use std::io::Write;
use std::sync::mpsc;
use std::sync::Arc;

/// A background-thread CSV writer for one topic's file.
pub struct CsvLogger {
    tx: mpsc::Sender<String>,
}

impl CsvLogger {
    /// Spawn the writer thread. Nothing touches the filesystem until the
    /// first row is enqueued — `RunDir::create_csv` creates the run
    /// directory and the file together, lazily, exactly as
    /// `rover_monitoring_node.cpp`'s `ensure_csv` did.
    pub fn spawn(run_dir: Arc<RunDir>, filename: &'static str, header: &'static str) -> Self {
        let (tx, rx) = mpsc::channel::<String>();
        std::thread::spawn(move || {
            let mut file = None;
            let mut warned = false;
            for row in rx {
                if file.is_none() {
                    match run_dir.create_csv(filename, header) {
                        Ok(f) => file = Some(f),
                        Err(e) => {
                            if !warned {
                                log::error!("{filename}: cannot create CSV: {e}");
                                warned = true;
                            }
                            continue;
                        }
                    }
                }
                if let Some(f) = file.as_mut() {
                    if let Err(e) = f.write_all(row.as_bytes()) {
                        log::error!("{filename}: write failed: {e}");
                    }
                }
            }
        });
        Self { tx }
    }

    /// Enqueue a pre-formatted row (see `csv_fmt.rs` for the formatting
    /// itself — this type has no opinion on row shape). Never blocks: an
    /// unbounded channel send either succeeds immediately or, if the writer
    /// thread has already exited (only during shutdown), the row is
    /// silently dropped rather than the caller panicking or stalling.
    pub fn log(&self, row: String) {
        let _ = self.tx.send(row);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    fn temp_runs_root(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "rover-telemetry-csvwriter-test-{}-{name}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
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
    fn logged_rows_reach_disk_with_the_header_first() {
        let root = temp_runs_root("basic");
        let run_dir = Arc::new(RunDir::resolve(&root));
        let logger = CsvLogger::spawn(run_dir.clone(), "test.csv", "A,B\n");

        logger.log("1,2\n".to_string());
        logger.log("3,4\n".to_string());

        let path = run_dir.path().join("test.csv");
        assert!(
            wait_for(|| path.exists(), Duration::from_secs(2)),
            "file was never created"
        );
        assert!(
            wait_for(
                || std::fs::read_to_string(&path)
                    .map(|s| s.lines().count() >= 3)
                    .unwrap_or(false),
                Duration::from_secs(2),
            ),
            "not all rows were written in time"
        );

        let contents = std::fs::read_to_string(&path).unwrap();
        assert_eq!(contents, "A,B\n1,2\n3,4\n");

        std::fs::remove_dir_all(&root).ok();
    }

    #[test]
    fn no_file_is_created_if_nothing_is_ever_logged() {
        let root = temp_runs_root("no_log");
        let run_dir = Arc::new(RunDir::resolve(&root));
        let _logger = CsvLogger::spawn(run_dir.clone(), "never.csv", "A\n");

        std::thread::sleep(Duration::from_millis(50));
        assert!(!run_dir.path().exists());

        std::fs::remove_dir_all(&root).ok();
    }
}
