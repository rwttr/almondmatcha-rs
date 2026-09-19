//! The `runs/run_NNN_<stamp>/` scheme, ported from `rover_monitoring_node.cpp`.
//!
//! Two behaviours are carried over deliberately:
//!
//! - **Files (and the run directory itself) are created on first write, not
//!   at startup.** A run that never receives data leaves nothing behind and
//!   consumes no run number — `docs/CSV_LOGGING.md` calls this out
//!   explicitly as a fix over an earlier version that always created every
//!   file with headers regardless. A missing CSV is positive evidence that
//!   topic never delivered data.
//! - **Run numbering scans the directory for the current maximum**, same as
//!   `get_next_run_number` (`glob("run_*")`, parse the three digits after
//!   `run_`, take the max, add one) rather than persisting a counter
//!   anywhere.

use std::fs::{self, File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

/// Parse the run number out of directory names already on disk and return
/// one higher than the maximum found — `1` if none match.
///
/// Pure function over the names rather than the filesystem directly, so the
/// numbering logic is testable without creating real directories; `RunDir`
/// below does the actual scan.
pub fn next_run_number(existing_dir_names: &[String]) -> u32 {
    existing_dir_names
        .iter()
        .filter_map(|name| name.strip_prefix("run_"))
        .filter_map(|rest| rest.get(0..3))
        .filter_map(|digits| digits.parse::<u32>().ok())
        .max()
        .map_or(1, |n| n + 1)
}

/// Days since the civil epoch, inverted into `(year, month, day)` — Howard
/// Hinnant's `civil_from_days`, the inverse of the `days_from_civil`
/// algorithm `rover-navigation`'s `time_utils.rs` uses in the other
/// direction. The two crates do not share code (there is no common
/// internal-utility crate in this workspace), so this is a from-scratch,
/// independently-tested implementation rather than a copy.
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097; // [0, 146096]
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365; // [0, 399]
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // [0, 365]
    let mp = (5 * doy + 2) / 153; // [0, 11]
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32; // [1, 31]
    let m = (if mp < 10 { mp + 3 } else { mp - 9 }) as u32; // [1, 12]
    let y = if m <= 2 { y + 1 } else { y };
    (y, m, d)
}

/// Format a Unix timestamp (seconds) as `YYYYMMDD_HHMMSS`, matching
/// `rover_monitoring_node.cpp`'s `strftime("%Y%m%d_%H%M%S")`.
pub fn format_run_timestamp(unix_s: u64) -> String {
    let days = (unix_s / 86_400) as i64;
    let secs_of_day = unix_s % 86_400;
    let (y, m, d) = civil_from_days(days);
    let hh = secs_of_day / 3600;
    let mm = (secs_of_day % 3600) / 60;
    let ss = secs_of_day % 60;
    format!("{y:04}{m:02}{d:02}_{hh:02}{mm:02}{ss:02}")
}

/// Build a full run directory name, e.g. `run_003_20260919_143052`.
pub fn run_dir_name(run_number: u32, unix_s: u64) -> String {
    format!("run_{run_number:03}_{}", format_run_timestamp(unix_s))
}

/// Owns the path to this run's directory and creates it (and each CSV
/// inside it) lazily, on first use.
pub struct RunDir {
    path: PathBuf,
    created: std::sync::atomic::AtomicBool,
}

/// Pure core of [`RunDir::resolve`]: given an already-read environment
/// override, the directory names already on disk, and a clock reading,
/// decide the run path. Split out so the decision — env override wins,
/// otherwise scan-and-increment — is testable without setting a real
/// process-global environment variable (which, being global mutable state
/// shared by every test binary in the process, is exactly the kind of thing
/// worth keeping out of a test) or touching a real filesystem.
fn resolve_run_dir_path(
    env_override: Option<&str>,
    runs_root: &Path,
    existing_dir_names: &[String],
    unix_s: u64,
) -> PathBuf {
    match env_override {
        Some(from_env) if !from_env.is_empty() => PathBuf::from(from_env),
        _ => {
            let run_number = next_run_number(existing_dir_names);
            runs_root.join(run_dir_name(run_number, unix_s))
        }
    }
}

impl RunDir {
    /// Resolve this run's directory without creating anything on disk.
    ///
    /// Prefers `$ROVER_RUN_DIR` when set — the launcher-provided directory
    /// shared by every process on a machine in one launch, so this binary's
    /// CSVs land next to (not scattered from) the run the rest of the
    /// system is using (`docs/CSV_LOGGING.md`'s "Run Directory Numbering"
    /// section). Falls back to scanning `runs_root` for the next run number,
    /// which is what running this binary by hand does.
    pub fn resolve(runs_root: &Path) -> Self {
        let env_override = std::env::var("ROVER_RUN_DIR").ok();
        let existing: Vec<String> = fs::read_dir(runs_root)
            .into_iter()
            .flatten()
            .filter_map(|entry| entry.ok())
            .filter(|entry| entry.path().is_dir())
            .filter_map(|entry| entry.file_name().into_string().ok())
            .collect();
        let unix_s = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let path = resolve_run_dir_path(env_override.as_deref(), runs_root, &existing, unix_s);

        Self {
            path,
            created: std::sync::atomic::AtomicBool::new(false),
        }
    }

    fn ensure_dir(&self) -> std::io::Result<()> {
        if !self.created.load(std::sync::atomic::Ordering::Relaxed) {
            fs::create_dir_all(&self.path)?;
            self.created
                .store(true, std::sync::atomic::Ordering::Relaxed);
        }
        Ok(())
    }

    /// Open `filename` inside this run directory for the first time,
    /// writing `header` immediately, creating the run directory itself if
    /// this is the first file created for this run. Returns an already-open
    /// handle ready for `writeln!` of subsequent rows.
    pub fn create_csv(&self, filename: &str, header: &str) -> std::io::Result<File> {
        self.ensure_dir()?;
        let mut file = OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open(self.path.join(filename))?;
        file.write_all(header.as_bytes())?;
        Ok(file)
    }

    pub fn path(&self) -> &Path {
        &self.path
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_directory_starts_at_run_one() {
        assert_eq!(next_run_number(&[]), 1);
    }

    #[test]
    fn picks_up_after_the_highest_existing_run() {
        let names = vec![
            "run_001_20260101_000000".to_string(),
            "run_003_20260102_000000".to_string(),
            "run_002_20260103_000000".to_string(),
        ];
        assert_eq!(next_run_number(&names), 4);
    }

    #[test]
    fn non_run_directories_are_ignored() {
        let names = vec!["notarun".to_string(), "logs".to_string()];
        assert_eq!(next_run_number(&names), 1);
    }

    #[test]
    fn malformed_run_prefixed_names_are_ignored_not_fatal() {
        let names = vec!["run_abc_garbage".to_string(), "run_002_ok".to_string()];
        assert_eq!(next_run_number(&names), 3);
    }

    #[test]
    fn format_run_timestamp_matches_a_known_instant() {
        // 1705321845 = 2024-01-15 12:30:45 UTC, cross-checked with `date -u`.
        assert_eq!(format_run_timestamp(1_705_321_845), "20240115_123045");
    }

    #[test]
    fn format_run_timestamp_handles_the_epoch() {
        assert_eq!(format_run_timestamp(0), "19700101_000000");
    }

    #[test]
    fn run_dir_name_combines_number_and_timestamp() {
        assert_eq!(run_dir_name(3, 1_705_321_845), "run_003_20240115_123045");
    }

    #[test]
    fn no_directory_is_created_until_the_first_csv_is_opened() {
        let tmp = std::env::temp_dir().join(format!(
            "rover-telemetry-test-{}-{}",
            std::process::id(),
            "no_dir_until_first_write"
        ));
        let _ = fs::remove_dir_all(&tmp);
        fs::create_dir_all(&tmp).unwrap();

        let run_dir = RunDir::resolve(&tmp);
        assert!(!run_dir.path().exists(), "must not create eagerly");

        run_dir.create_csv("a.csv", "H1,H2\n").unwrap();
        assert!(run_dir.path().exists());
        assert!(run_dir.path().join("a.csv").exists());

        fs::remove_dir_all(&tmp).ok();
    }

    #[test]
    fn env_override_takes_priority_over_scanning() {
        let path = resolve_run_dir_path(
            Some("/launcher/chosen/run_dir"),
            Path::new("/nonexistent-runs-root"),
            &["run_005_whatever".to_string()],
            1_705_321_845,
        );
        assert_eq!(path, PathBuf::from("/launcher/chosen/run_dir"));
    }

    #[test]
    fn empty_env_override_falls_back_to_scanning() {
        let path = resolve_run_dir_path(
            Some(""),
            Path::new("/runs"),
            &["run_005_whatever".to_string()],
            1_705_321_845,
        );
        assert_eq!(path, PathBuf::from("/runs/run_006_20240115_123045"));
    }

    #[test]
    fn no_env_override_scans_for_the_next_run_number() {
        let path = resolve_run_dir_path(None, Path::new("/runs"), &[], 0);
        assert_eq!(path, PathBuf::from("/runs/run_001_19700101_000000"));
    }
}
