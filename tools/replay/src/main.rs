//! `replay` — the offline harness plan §12 (criterion 2) requires before
//! anything drives a motor: `rover-estimator` + `rover_control::guide::StaticGain`
//! driven from a CSV trace, no network, no hardware anywhere in the process.
//!
//! **Read `crate::trace`'s module docs first.** There is no recorded ROS2
//! field data anywhere in this repository — there never was, and the ROS 2
//! tree that would have produced it has since been removed (see `git log`
//! and `main`) — so this tool validates the new pipeline
//! against a faithful Rust port of the *old* EMA-based law
//! (`crate::legacy`) fed synthetic input (`crate::synthetic`) — not against
//! a real recorded run. Treat a clean report from this tool as "the port is
//! internally consistent and the harness can catch a regression," not as
//! "field parity is proven." Plan §12 criterion 2 remains open until real
//! `runs/*.csv` exist and are run through this tool.
//!
//! Three subcommands:
//! - `synthetic`: generate a trace CSV (`crate::synthetic`).
//! - `run`: replay one trace, print/optionally write per-sample
//!   recorded-vs-computed steering and summary error statistics, and exit
//!   non-zero if the RMSE exceeds `--tolerance-rmse-deg` — this is the
//!   actual gate, and it can fail (see `crate::replay`'s tests for two
//!   deliberately-broken-gain cases that do).
//! - `sweep`: replay the same trace under a grid of estimator `Q`/`R` scale
//!   factors, so the filter can be tuned on a laptop (plan §2.3).

#![forbid(unsafe_code)]

mod legacy;
mod replay;
mod synthetic;
mod trace;

use clap::{Parser, Subcommand, ValueEnum};
use rover_control::config::{AppConfig, ControllerLaw};
use rover_control::guide::StaticGainConfig;
use rover_estimator::{EstimatorConfig, LaneNoise, ProcessNoise};
use std::path::PathBuf;
use std::process::ExitCode;

const DEFAULT_CONFIG_PATH: &str = "config/rover.toml";

#[derive(Parser)]
#[command(
    about = "Offline estimate+guide replay harness (plan §12 gate). No network, no hardware."
)]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Generate a synthetic trace CSV. There is no recorded field data to
    /// replay instead — see this crate's top-level docs.
    Synthetic {
        #[arg(long, value_enum, default_value_t = ScenarioArg::Steady)]
        scenario: ScenarioArg,
        #[arg(long, default_value_t = 10.0)]
        duration_s: f32,
        #[arg(long)]
        out: PathBuf,
    },
    /// Replay one trace through rover-estimator + StaticGain and report
    /// recorded-vs-computed steering. Exits non-zero if the RMSE exceeds
    /// the tolerance, or if the trace had nothing to compare against.
    Run {
        #[arg(long)]
        trace: PathBuf,
        /// `config/rover.toml`-shaped file to read `[control.static_gain]`,
        /// `[estimator]`, `[drivetrain]`, `[perception]` from.
        #[arg(long, default_value = DEFAULT_CONFIG_PATH)]
        config: PathBuf,
        /// Seconds of the trace's own clock to exclude from error stats,
        /// letting the EKF's initial-condition transient settle first.
        #[arg(long, default_value_t = 2.0)]
        warmup_s: f32,
        /// Optional per-sample CSV to write (recorded/computed/error plus
        /// estimator confidence signals).
        #[arg(long)]
        out: Option<PathBuf>,
        #[arg(long, default_value_t = 3.0)]
        tolerance_rmse_deg: f32,
        /// Scale every `[estimator.q]` entry by this factor, for one-off
        /// tuning without editing the config file.
        #[arg(long, default_value_t = 1.0)]
        q_scale: f32,
        /// Scale every `[estimator.r]` entry (camera + odometry + zero-rate
        /// gyro noise) by this factor.
        #[arg(long, default_value_t = 1.0)]
        r_scale: f32,
    },
    /// Replay one trace across a grid of Q/R scale factors — the tunable-on-
    /// a-laptop sweep plan §2.3 asks for.
    Sweep {
        #[arg(long)]
        trace: PathBuf,
        #[arg(long, default_value = DEFAULT_CONFIG_PATH)]
        config: PathBuf,
        #[arg(long, default_value_t = 2.0)]
        warmup_s: f32,
        #[arg(long, value_delimiter = ',', default_value = "0.5,1.0,2.0")]
        q_scales: Vec<f32>,
        #[arg(long, value_delimiter = ',', default_value = "0.5,1.0,2.0")]
        r_scales: Vec<f32>,
        /// Optional CSV summary (one row per Q/R combination).
        #[arg(long)]
        out: Option<PathBuf>,
    },
}

#[derive(Clone, Copy, ValueEnum)]
enum ScenarioArg {
    Steady,
    Dropout,
}

impl From<ScenarioArg> for synthetic::Scenario {
    fn from(s: ScenarioArg) -> Self {
        match s {
            ScenarioArg::Steady => synthetic::Scenario::Steady,
            ScenarioArg::Dropout => synthetic::Scenario::Dropout,
        }
    }
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    match cli.cmd {
        Cmd::Synthetic {
            scenario,
            duration_s,
            out,
        } => cmd_synthetic(scenario, duration_s, &out),
        Cmd::Run {
            trace,
            config,
            warmup_s,
            out,
            tolerance_rmse_deg,
            q_scale,
            r_scale,
        } => cmd_run(
            &trace,
            &config,
            warmup_s,
            out.as_deref(),
            tolerance_rmse_deg,
            q_scale,
            r_scale,
        ),
        Cmd::Sweep {
            trace,
            config,
            warmup_s,
            q_scales,
            r_scales,
            out,
        } => cmd_sweep(
            &trace,
            &config,
            warmup_s,
            &q_scales,
            &r_scales,
            out.as_deref(),
        ),
    }
}

fn cmd_synthetic(scenario: ScenarioArg, duration_s: f32, out: &std::path::Path) -> ExitCode {
    let generated =
        synthetic::generate(scenario.into(), duration_s, legacy::LegacyConfig::default());
    match generated.write_csv(out) {
        Ok(()) => {
            println!(
                "replay: wrote {} rows to {}",
                generated.rows.len(),
                out.display()
            );
            println!(
                "replay: NOTE — this is synthetic data with a hand-rolled reference law, \
                 not recorded field data (none exists in this repo). See crate docs."
            );
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("replay: writing {}: {e}", out.display());
            ExitCode::FAILURE
        }
    }
}

fn load_app_config(path: &std::path::Path) -> Result<AppConfig, String> {
    AppConfig::load(path).map_err(|e| format!("loading `{}`: {e}", path.display()))
}

fn static_gain_config_of(app_cfg: &AppConfig) -> StaticGainConfig {
    let ControllerLaw::StaticGain(sg) = app_cfg.law;
    StaticGainConfig {
        k_lat_deg_per_m: sg.k_lat_deg_per_m,
        k_head_deg_per_deg: sg.k_head_deg_per_deg,
    }
}

/// Scale every `Q`/`R` entry in `cfg` by the given factors — the knob
/// `sweep` and `run --q-scale/--r-scale` both turn, without needing to
/// hand-edit `config/rover.toml` for a one-off experiment.
fn scale_estimator_config(cfg: EstimatorConfig, q_scale: f32, r_scale: f32) -> EstimatorConfig {
    EstimatorConfig {
        q: ProcessNoise::new(
            cfg.q.cross_track * q_scale,
            cfg.q.heading * q_scale,
            cfg.q.curvature * q_scale,
            cfg.q.speed * q_scale,
            cfg.q.gyro_bias * q_scale,
        ),
        r_lane: LaneNoise::new(
            cfg.r_lane.cross_track * r_scale,
            cfg.r_lane.heading * r_scale,
            cfg.r_lane.curvature * r_scale,
        ),
        r_odom_speed: cfg.r_odom_speed * r_scale,
        r_zero_rate_gyro: cfg.r_zero_rate_gyro * r_scale,
        ..cfg
    }
}

fn cmd_run(
    trace_path: &std::path::Path,
    config_path: &std::path::Path,
    warmup_s: f32,
    out: Option<&std::path::Path>,
    tolerance_rmse_deg: f32,
    q_scale: f32,
    r_scale: f32,
) -> ExitCode {
    let app_cfg = match load_app_config(config_path) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("replay: {e}");
            return ExitCode::FAILURE;
        }
    };
    let static_gain_cfg = static_gain_config_of(&app_cfg);
    let estimator_cfg = scale_estimator_config(app_cfg.estimator, q_scale, r_scale);

    let trace = match trace::Trace::read_csv(trace_path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("replay: {e}");
            return ExitCode::FAILURE;
        }
    };

    let output = replay::run(
        &trace,
        app_cfg.vehicle,
        estimator_cfg,
        static_gain_cfg,
        warmup_s,
    );

    println!(
        "replay: {} rows, {} compared against a recorded value",
        trace.rows.len(),
        output.stats.n_compared
    );
    println!(
        "  mean |error| = {:.3} deg",
        output.stats.mean_abs_error_deg
    );
    println!("  rmse         = {:.3} deg", output.stats.rmse_deg);
    println!("  max |error|  = {:.3} deg", output.stats.max_abs_error_deg);

    if let Some(out_path) = out {
        if let Err(e) = replay::write_output_csv(&output.rows, out_path) {
            eprintln!("replay: writing {}: {e}", out_path.display());
            return ExitCode::FAILURE;
        }
        println!("  wrote per-sample output to {}", out_path.display());
    }

    if output.stats.n_compared == 0 {
        eprintln!(
            "replay: FAIL — no row had a recorded value to compare against; nothing was validated"
        );
        return ExitCode::FAILURE;
    }
    if output.stats.rmse_deg > tolerance_rmse_deg {
        eprintln!(
            "replay: FAIL — rmse {:.3} deg exceeds tolerance {:.3} deg",
            output.stats.rmse_deg, tolerance_rmse_deg
        );
        return ExitCode::FAILURE;
    }
    println!("replay: PASS (rmse within {tolerance_rmse_deg:.3} deg tolerance)");
    ExitCode::SUCCESS
}

fn cmd_sweep(
    trace_path: &std::path::Path,
    config_path: &std::path::Path,
    warmup_s: f32,
    q_scales: &[f32],
    r_scales: &[f32],
    out: Option<&std::path::Path>,
) -> ExitCode {
    let app_cfg = match load_app_config(config_path) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("replay: {e}");
            return ExitCode::FAILURE;
        }
    };
    let static_gain_cfg = static_gain_config_of(&app_cfg);

    let trace = match trace::Trace::read_csv(trace_path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("replay: {e}");
            return ExitCode::FAILURE;
        }
    };

    println!(
        "{:>8} {:>8} {:>8} {:>12} {:>12} {:>12}",
        "q_scale", "r_scale", "n", "mean_abs_deg", "rmse_deg", "max_abs_deg"
    );
    let mut summary = Vec::new();
    for &qs in q_scales {
        for &rs in r_scales {
            let cfg = scale_estimator_config(app_cfg.estimator, qs, rs);
            let output = replay::run(&trace, app_cfg.vehicle, cfg, static_gain_cfg, warmup_s);
            println!(
                "{:>8.3} {:>8.3} {:>8} {:>12.3} {:>12.3} {:>12.3}",
                qs,
                rs,
                output.stats.n_compared,
                output.stats.mean_abs_error_deg,
                output.stats.rmse_deg,
                output.stats.max_abs_error_deg
            );
            summary.push((qs, rs, output.stats));
        }
    }

    if let Some(out_path) = out {
        if let Err(e) = write_sweep_csv(&summary, out_path) {
            eprintln!("replay: writing {}: {e}", out_path.display());
            return ExitCode::FAILURE;
        }
        println!("wrote sweep summary to {}", out_path.display());
    }

    ExitCode::SUCCESS
}

fn write_sweep_csv(
    rows: &[(f32, f32, replay::Stats)],
    path: &std::path::Path,
) -> std::io::Result<()> {
    use std::io::Write;
    let mut f = std::fs::File::create(path)?;
    writeln!(
        f,
        "q_scale,r_scale,n_compared,mean_abs_error_deg,rmse_deg,max_abs_error_deg"
    )?;
    for (qs, rs, stats) in rows {
        writeln!(
            f,
            "{qs},{rs},{},{},{},{}",
            stats.n_compared, stats.mean_abs_error_deg, stats.rmse_deg, stats.max_abs_error_deg
        )?;
    }
    Ok(())
}
