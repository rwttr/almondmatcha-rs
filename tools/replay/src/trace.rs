//! The trace format this tool reads and writes.
//!
//! # There is no recorded field data to read yet
//!
//! `docs/RUST_REWRITE_PLAN.md` §12 (criterion 2) and §10 both say `replay`
//! must reproduce recorded ROS2 steering output from `runs/*/lane_detection.csv`
//! and its neighbours. **No such recording exists in this repository, and
//! none ever did.** Those were per-run output directories the ROS2 nodes
//! created at record time and never committed; the ROS 2 tree itself has
//! since been removed from this branch (it survives on `main` and in this
//! branch's history). There is no `runs/` directory, no `.csv` anywhere in
//! this repo outside vendored third-party Python packages, and no `.bag`
//! file either — checked again at removal time, which is what made deleting
//! the tree safe.
//!
//! **Consequently: parity against real recorded ROS2 field data is
//! UNVALIDATED by this tool.** What it validates instead — see `crate::synthetic`
//! and `crate::legacy` — is that the new `rover-estimator` + `StaticGain`
//! pipeline reproduces a faithful Rust port of the *old* EMA-based ROS2 law
//! on synthetic input, and that the harness can actually fail (a wrong gain
//! or a dropped unit conversion blows the error stats up, not quietly
//! passes). Do not read a clean report from this tool as field validation;
//! it is a laptop-only regression gate until real `runs/*.csv` exist.
//!
//! # Format
//!
//! One row per **100 Hz IMU tick** (this repo's `estimator.predict_hz`).
//! Slower feeds (camera ~30 Hz, encoders 10 Hz) repeat their last value on
//! rows where they didn't produce a fresh sample; the `*_valid` columns say
//! which rows actually carry a new measurement, so a reader must not treat
//! every row's `cross_track_m`/`ticks_left` etc. as a fresh reading.
//!
//! ```text
//! t_us,gyro_z_radps,lane_valid,cross_track_m,heading_err_rad,curvature_inv_m,wheel_valid,ticks_left,ticks_right,throttle,recorded_steer_deg
//! ```
//!
//! `recorded_steer_deg` is the one optional column: empty means "no ground
//! truth for this row" (a synthetic trace generated without
//! `crate::legacy::LegacyEmaLaw`, or the leading rows of any trace before a
//! comparison target exists). `crate::replay` skips those rows when
//! computing error statistics rather than treating a missing value as zero
//! error.
//!
//! A hand-written reader/writer, not the `csv` crate: the format is one
//! flat row of known columns, in the spirit of `rover_msgs`'s own
//! hand-written wire codec — no schema library earns its keep here.

use std::fmt;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Write};
use std::path::Path;

pub const HEADER: &str = "t_us,gyro_z_radps,lane_valid,cross_track_m,heading_err_rad,curvature_inv_m,wheel_valid,ticks_left,ticks_right,throttle,recorded_steer_deg";

/// One 100 Hz tick of input, plus an optional ground-truth steering value to
/// compare against. See the module docs for the zero-order-hold convention
/// on the slower feeds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TraceRow {
    /// Microsecond timestamp, this trace's own clock — matches
    /// `ImuSample.t_us`'s wrapping 32-bit range in spirit, but kept as `u64`
    /// here so a long synthetic run (or a long field run, one day) never
    /// wraps mid-trace purely as an artifact of the file format; wrapping
    /// arithmetic across a real 32-bit wrap is `rover-estimator`'s and
    /// `rover-control`'s concern, already covered by their own tests, not
    /// this trace format's.
    pub t_us: u64,
    pub gyro_z_radps: f32,
    pub lane_valid: bool,
    pub cross_track_m: f32,
    pub heading_err_rad: f32,
    pub curvature_inv_m: f32,
    pub wheel_valid: bool,
    pub ticks_left: i32,
    pub ticks_right: i32,
    pub throttle: f32,
    pub recorded_steer_deg: Option<f32>,
}

#[derive(Debug)]
pub struct Trace {
    pub rows: Vec<TraceRow>,
}

impl Trace {
    pub fn write_csv(&self, path: impl AsRef<Path>) -> io::Result<()> {
        let mut f = File::create(path)?;
        writeln!(f, "{HEADER}")?;
        for r in &self.rows {
            writeln!(
                f,
                "{},{},{},{},{},{},{},{},{},{},{}",
                r.t_us,
                r.gyro_z_radps,
                r.lane_valid as u8,
                r.cross_track_m,
                r.heading_err_rad,
                r.curvature_inv_m,
                r.wheel_valid as u8,
                r.ticks_left,
                r.ticks_right,
                r.throttle,
                r.recorded_steer_deg
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
            )?;
        }
        Ok(())
    }

    pub fn read_csv(path: impl AsRef<Path>) -> Result<Self, TraceError> {
        let path = path.as_ref();
        let f = File::open(path).map_err(|e| TraceError::Io(path.display().to_string(), e))?;
        let mut lines = BufReader::new(f).lines();

        let header = lines
            .next()
            .ok_or_else(|| TraceError::Empty(path.display().to_string()))?
            .map_err(|e| TraceError::Io(path.display().to_string(), e))?;
        if header.trim() != HEADER {
            return Err(TraceError::BadHeader {
                path: path.display().to_string(),
                got: header,
            });
        }

        let mut rows = Vec::new();
        for (i, line) in lines.enumerate() {
            let line = line.map_err(|e| TraceError::Io(path.display().to_string(), e))?;
            if line.trim().is_empty() {
                continue;
            }
            rows.push(parse_row(&line).map_err(|e| TraceError::BadRow {
                path: path.display().to_string(),
                line: i + 2, // +1 for the header, +1 for 1-indexing
                reason: e,
            })?);
        }
        Ok(Trace { rows })
    }
}

fn parse_row(line: &str) -> Result<TraceRow, String> {
    let f: Vec<&str> = line.split(',').collect();
    if f.len() != 11 {
        return Err(format!("expected 11 fields, got {}", f.len()));
    }
    let field = |i: usize| f[i].trim();
    let parse_f32 = |i: usize| {
        field(i)
            .parse::<f32>()
            .map_err(|e| format!("field {i}: {e}"))
    };
    let parse_i32 = |i: usize| {
        field(i)
            .parse::<i32>()
            .map_err(|e| format!("field {i}: {e}"))
    };
    let parse_u64 = |i: usize| {
        field(i)
            .parse::<u64>()
            .map_err(|e| format!("field {i}: {e}"))
    };
    let parse_bool = |i: usize| match field(i) {
        "1" | "true" => Ok(true),
        "0" | "false" => Ok(false),
        other => Err(format!(
            "field {i}: `{other}` is not a boolean (0/1/true/false)"
        )),
    };

    Ok(TraceRow {
        t_us: parse_u64(0)?,
        gyro_z_radps: parse_f32(1)?,
        lane_valid: parse_bool(2)?,
        cross_track_m: parse_f32(3)?,
        heading_err_rad: parse_f32(4)?,
        curvature_inv_m: parse_f32(5)?,
        wheel_valid: parse_bool(6)?,
        ticks_left: parse_i32(7)?,
        ticks_right: parse_i32(8)?,
        throttle: parse_f32(9)?,
        recorded_steer_deg: if field(10).is_empty() {
            None
        } else {
            Some(parse_f32(10)?)
        },
    })
}

#[derive(Debug)]
pub enum TraceError {
    Io(String, io::Error),
    Empty(String),
    BadHeader {
        path: String,
        got: String,
    },
    BadRow {
        path: String,
        line: usize,
        reason: String,
    },
}

impl fmt::Display for TraceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TraceError::Io(path, e) => write!(f, "reading `{path}`: {e}"),
            TraceError::Empty(path) => write!(f, "`{path}` is empty (no header row)"),
            TraceError::BadHeader { path, got } => {
                write!(f, "`{path}`: expected header `{HEADER}`, got `{got}`")
            }
            TraceError::BadRow { path, line, reason } => {
                write!(f, "`{path}` line {line}: {reason}")
            }
        }
    }
}

impl std::error::Error for TraceError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn row(t_us: u64) -> TraceRow {
        TraceRow {
            t_us,
            gyro_z_radps: 0.01,
            lane_valid: true,
            cross_track_m: 0.05,
            heading_err_rad: 0.02,
            curvature_inv_m: 0.0,
            wheel_valid: false,
            ticks_left: 0,
            ticks_right: 0,
            throttle: 0.2,
            recorded_steer_deg: Some(1.5),
        }
    }

    #[test]
    fn round_trips_through_csv() {
        let trace = Trace {
            rows: vec![row(0), row(10_000), row(20_000)],
        };
        let path =
            std::env::temp_dir().join(format!("replay-trace-test-{}.csv", std::process::id()));
        trace.write_csv(&path).unwrap();
        let read_back = Trace::read_csv(&path).unwrap();
        std::fs::remove_file(&path).ok();

        assert_eq!(read_back.rows.len(), 3);
        assert_eq!(read_back.rows[1].t_us, 10_000);
        assert!((read_back.rows[1].cross_track_m - 0.05).abs() < 1e-6);
        assert_eq!(read_back.rows[1].recorded_steer_deg, Some(1.5));
    }

    #[test]
    fn missing_recorded_steer_round_trips_as_none() {
        let mut r = row(0);
        r.recorded_steer_deg = None;
        let trace = Trace { rows: vec![r] };
        let path =
            std::env::temp_dir().join(format!("replay-trace-test-none-{}.csv", std::process::id()));
        trace.write_csv(&path).unwrap();
        let read_back = Trace::read_csv(&path).unwrap();
        std::fs::remove_file(&path).ok();
        assert_eq!(read_back.rows[0].recorded_steer_deg, None);
    }

    #[test]
    fn rejects_a_mismatched_header() {
        let path = std::env::temp_dir().join(format!(
            "replay-trace-test-badheader-{}.csv",
            std::process::id()
        ));
        std::fs::write(&path, "not,the,right,header\n1,2,3,4\n").unwrap();
        let err = Trace::read_csv(&path).unwrap_err();
        std::fs::remove_file(&path).ok();
        assert!(matches!(err, TraceError::BadHeader { .. }));
    }

    #[test]
    fn rejects_a_row_with_the_wrong_field_count() {
        let path = std::env::temp_dir().join(format!(
            "replay-trace-test-badrow-{}.csv",
            std::process::id()
        ));
        std::fs::write(&path, format!("{HEADER}\n1,2,3\n")).unwrap();
        let err = Trace::read_csv(&path).unwrap_err();
        std::fs::remove_file(&path).ok();
        assert!(matches!(err, TraceError::BadRow { .. }));
    }
}
