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
//! background thread; [`CsvLogger::log`] only ever pushes onto a bounded
//! channel, so a stalled disk degrades to dropped (and counted) rows, never a
//! blocked caller or unbounded memory. Rows are buffered and flushed about
//! once a second, and on drop.

use rover_runs::RunDir;
use std::io::{BufWriter, Write};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError, TrySendError};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// How many rows may wait for the writer thread before new ones are dropped.
/// The fastest feed is `RoverState` at ~100 Hz, so 8192 rows is over a minute
/// of that feed backed up behind a stalled SD card — plenty to ride out a
/// slow write, small enough (a few MB) that a dead disk cannot eat the RPi's
/// RAM, which the old unbounded channel could.
const QUEUE_CAPACITY: usize = 8192;

/// How often buffered rows are flushed (and `sync_data`'d) to disk. Bounds
/// what a power cut or crash can lose to about this much logging, without a
/// syscall per row. Also the cadence at which drops are reported.
const FLUSH_INTERVAL: Duration = Duration::from_secs(1);

/// A background-thread CSV writer for one topic's file.
pub struct CsvLogger {
    tx: Option<mpsc::SyncSender<String>>,
    dropped: Arc<AtomicU64>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl CsvLogger {
    /// Spawn the writer thread. Nothing touches the filesystem until the
    /// first row is enqueued — `RunDir::create_csv` creates the run
    /// directory and the file together, lazily, exactly as
    /// `rover_monitoring_node.cpp`'s `ensure_csv` did. (`create_csv` opens in
    /// append mode and writes the header only for a new file, so a restart
    /// into the same run directory continues the file.)
    pub fn spawn(run_dir: Arc<RunDir>, filename: &'static str, header: &'static str) -> Self {
        let (tx, rx) = mpsc::sync_channel::<String>(QUEUE_CAPACITY);
        let dropped = Arc::new(AtomicU64::new(0));
        let dropped_t = dropped.clone();
        let thread = std::thread::spawn(move || {
            let mut file: Option<BufWriter<std::fs::File>> = None;
            let mut warned = false;
            let mut dirty = false;
            let mut next_flush = Instant::now() + FLUSH_INTERVAL;
            loop {
                let timeout = next_flush.saturating_duration_since(Instant::now());
                match rx.recv_timeout(timeout) {
                    Ok(row) => {
                        if file.is_none() {
                            match run_dir.create_csv(filename, header) {
                                Ok(f) => file = Some(BufWriter::new(f)),
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
                            match f.write_all(row.as_bytes()) {
                                Ok(()) => dirty = true,
                                Err(e) => log::error!("{filename}: write failed: {e}"),
                            }
                        }
                    }
                    Err(RecvTimeoutError::Timeout) => {}
                    Err(RecvTimeoutError::Disconnected) => break,
                }
                if Instant::now() >= next_flush {
                    next_flush = Instant::now() + FLUSH_INTERVAL;
                    if dirty {
                        flush_and_sync(file.as_mut(), filename);
                        dirty = false;
                    }
                    // Reported here, at most once per flush interval, so a
                    // sustained overload is a steady trickle of log lines
                    // rather than one per dropped row.
                    let n = dropped_t.swap(0, Ordering::Relaxed);
                    if n > 0 {
                        log::error!("{filename}: queue full, dropped {n} rows in the last second");
                    }
                }
            }
            // Shutdown: every row accepted before the logger was dropped
            // reaches disk.
            flush_and_sync(file.as_mut(), filename);
        });
        Self {
            tx: Some(tx),
            dropped,
            thread: Some(thread),
        }
    }

    /// Enqueue a pre-formatted row (see `csv_fmt.rs` for the formatting
    /// itself — this type has no opinion on row shape). Never blocks: if the
    /// bounded queue is full (writer stalled on a slow disk) the row is
    /// dropped and counted rather than stalling the caller — the 5 Hz
    /// `Telemetry` publish shares this thread — or growing without limit.
    pub fn log(&self, row: String) {
        if let Some(tx) = &self.tx {
            if let Err(TrySendError::Full(_)) = tx.try_send(row) {
                self.dropped.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// Rows dropped since the writer thread last reported (it resets the
    /// count each flush interval). Exposed for tests.
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

fn flush_and_sync(file: Option<&mut BufWriter<std::fs::File>>, filename: &str) {
    if let Some(f) = file {
        if let Err(e) = f.flush().and_then(|_| f.get_ref().sync_data()) {
            log::error!("{filename}: flush failed: {e}");
        }
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
    fn full_queue_drops_and_counts_instead_of_blocking() {
        // No consumer: the receiver is kept alive but never read, so the
        // queue fills exactly like a writer stuck on a dead disk.
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
    fn rows_are_flushed_within_the_flush_interval_without_shutdown() {
        let root = temp_runs_root("periodic");
        let run_dir = Arc::new(RunDir::resolve(&root));
        let logger = CsvLogger::spawn(run_dir.clone(), "p.csv", "A\n");
        logger.log("1\n".to_string());
        let path = run_dir.path().join("p.csv");
        assert!(
            wait_for(
                || std::fs::read_to_string(&path).is_ok_and(|s| s == "A\n1\n"),
                FLUSH_INTERVAL * 3,
            ),
            "row never reached disk"
        );
        drop(logger);
        std::fs::remove_dir_all(&root).ok();
    }

    #[test]
    fn dropping_the_logger_flushes_everything_immediately() {
        let root = temp_runs_root("drop");
        let run_dir = Arc::new(RunDir::resolve(&root));
        let logger = CsvLogger::spawn(run_dir.clone(), "d.csv", "A\n");
        logger.log("1\n".to_string());
        logger.log("2\n".to_string());
        drop(logger);
        let contents = std::fs::read_to_string(run_dir.path().join("d.csv")).unwrap();
        assert_eq!(contents, "A\n1\n2\n");
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
