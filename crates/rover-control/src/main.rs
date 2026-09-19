//! `rover-control` — the RPi binary: estimate (EKF, plan §2) -> guide
//! (pluggable law, plan §8) -> actuate (guard rails, plan §8.3), one
//! process, in-process channels so the hot path never touches the network
//! (plan §1.2).
//!
//! # Threading: plain std threads, not an async runtime
//!
//! The control loop wants a deterministic fixed-rate tick — predict every
//! ~10 ms regardless of what else is happening — not an executor's
//! scheduling jitter, which depends on how many other tasks happen to be
//! runnable and how the runtime chooses to poll them at any given moment.
//! `rover_estimator::Ekf::predict` is explicit that it scales process noise
//! by the *measured* `dt`, precisely so a late tick widens the filter's
//! uncertainty instead of being silently trusted as on-time (see
//! `crate::estimate`'s module docs) — a runtime that can't promise when it
//! will next poll a task works directly against that design. A plain
//! `std::thread` sleeping to a fixed cadence is also far simpler to reason
//! about and to test than tuning an async runtime to behave like one.
//!
//! So there are exactly two threads:
//!
//! - **The control thread** (this file's [`control_loop`]): ticks at
//!   `estimator.predict_hz` (100 Hz), running estimate -> guide -> actuate
//!   as plain sequential function calls — no channels, no async, between
//!   those three stages. It talks to the network thread only through two
//!   `std::sync::mpsc` channels: inbound wire messages in, outbound
//!   publishable messages out.
//! - **The network thread** ([`network_thread`]): owns the `rover_bus::Bus`
//!   exclusively (its `poll`/`publish` need `&mut self`, so it can only
//!   live on one thread), decodes inbound frames into typed messages and
//!   forwards them, and publishes whatever the control thread hands it.
//!
//! Neither thread ever blocks the other: the network thread's `Link::recv`
//! never blocks (see `rover_link::Link`'s own contract), and the control
//! thread never touches a socket.

#![forbid(unsafe_code)]

use rover_bus::{Bus, BusConfig, CommandReceiver};
use rover_control::actuate::{Actuator, ActuatorConfig, SafetyGate};
use rover_control::config::{AppConfig, ControllerLaw};
use rover_control::estimate::Estimator;
use rover_control::guide::{LateralController, StaticGain, StaticGainConfig};
use rover_link::{PeerId, UdpLink};
use rover_msgs::{
    ChassisCommand, Command, CommandFrame, EkfDebug, ImuSample, LaneMeasurement, MissionStatus,
    RoverState, SpeedLoopDebug, WheelSensors,
};
use std::collections::HashMap;
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

/// Default location of the shared config file, relative to the process's
/// working directory — matching every other binary in this workspace
/// (`rover-tap`'s `--config` default, `rover-bus`'s own tests).
const DEFAULT_CONFIG_PATH: &str = "config/rover.toml";

/// Initial covariance for the EKF's five states at startup: the rover is
/// placed on the line at rest, so these are the "trust the zero-error
/// initial condition, but not blindly" priors `Ekf::at_rest` expects. Not a
/// ROS2-ported constant — the ROS2 system had no covariance at all, only a
/// point-estimate EMA — chosen conservatively (wide on curvature/speed,
/// tighter on cross-track/heading given the rover starts on the line) and
/// revisit once bench data on convergence time exists.
const INITIAL_P_DIAG: [f32; rover_msgs::EKF_STATES] = [1e-2, 1e-2, 1e-2, 1e-2, 1e-4];

fn main() {
    env_logger::init();

    let config_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_CONFIG_PATH.to_string());

    let bus_config = BusConfig::load(&config_path).unwrap_or_else(|e| {
        eprintln!("rover-control: loading bus config from `{config_path}`: {e}");
        std::process::exit(1);
    });
    let app_config = AppConfig::load(&config_path).unwrap_or_else(|e| {
        eprintln!("rover-control: loading control config from `{config_path}`: {e}");
        std::process::exit(1);
    });

    let bind_addr = bus_config.addr_of(PeerId::Rpi).unwrap_or_else(|| {
        eprintln!("rover-control: config has no [hosts]/[ports] entry for `rpi`");
        std::process::exit(1);
    });
    let mut peers = HashMap::new();
    for peer in PeerId::ALL {
        if peer != PeerId::Rpi {
            if let Some(addr) = bus_config.addr_of(peer) {
                peers.insert(peer, addr);
            }
        }
    }
    let link = UdpLink::bind(PeerId::Rpi, bind_addr, peers).unwrap_or_else(|e| {
        eprintln!("rover-control: binding as `rpi` on {bind_addr}: {e}");
        std::process::exit(1);
    });

    let (inbound_tx, inbound_rx) = mpsc::channel::<Inbound>();
    let (outbound_tx, outbound_rx) = mpsc::channel::<Outbound>();

    // `Bus` is built *inside* the spawned closure, not here and then moved
    // in: its subscriber callbacks are boxed `dyn FnMut`, not `dyn FnMut +
    // Send`, so a `Bus` that has already registered subscriptions is not
    // itself `Send`. `UdpLink` and `BusConfig` are plain data and cross the
    // thread boundary just fine; the `Bus` they compose into is then built
    // and lives entirely on the network thread.
    let net_handle =
        thread::spawn(move || network_thread(link, bus_config, inbound_tx, outbound_rx));

    control_loop(app_config, &inbound_rx, &outbound_tx);

    // control_loop never returns in normal operation; this is reachable
    // only if it panics and unwinds, in which case there's nothing more
    // useful to do than let the network thread's join (which will itself
    // never return) report the situation.
    drop(outbound_tx);
    let _ = net_handle.join();
}

/// One decoded message the control loop cares about, moved off the network
/// thread. Deliberately not just "the newest of each type" the way
/// `Bus::latest` works: `LaneMeasurement`/`WheelSensors` corrections are
/// stateful (they advance the EKF's covariance), so a control tick that
/// missed one because it only looked at "the latest" would silently lose
/// information a slower consumer wouldn't. The channel preserves arrival
/// order and every sample.
enum Inbound {
    Imu(ImuSample),
    Lane(LaneMeasurement),
    Wheel(WheelSensors),
    Mission(MissionStatus),
    Cmd(Command),
}

/// One message the control loop wants published, moved onto the network
/// thread.
enum Outbound {
    State(RoverState),
    EkfDbg(EkfDebug),
    Chassis(ChassisCommand),
    SpeedDbg(SpeedLoopDebug),
}

/// Owns the `Bus` exclusively. Subscribes to every message type the control
/// loop needs (forwarding each to `tx`), applies the base-station command
/// handshake (`rover_bus::command::CommandReceiver`) to `CommandFrame`
/// before forwarding only the newly-applied `Command`, and publishes
/// whatever arrives on `rx`.
fn network_thread(
    link: UdpLink,
    bus_config: BusConfig,
    tx: mpsc::Sender<Inbound>,
    rx: mpsc::Receiver<Outbound>,
) {
    let mut bus = Bus::new(link, bus_config);
    let mut cmd_rx = CommandReceiver::new();

    {
        let tx = tx.clone();
        bus.subscribe::<ImuSample>(move |m| {
            let _ = tx.send(Inbound::Imu(m));
        });
    }
    {
        let tx = tx.clone();
        bus.subscribe::<LaneMeasurement>(move |m| {
            let _ = tx.send(Inbound::Lane(m));
        });
    }
    {
        let tx = tx.clone();
        bus.subscribe::<WheelSensors>(move |m| {
            let _ = tx.send(Inbound::Wheel(m));
        });
    }
    {
        let tx = tx.clone();
        bus.subscribe::<MissionStatus>(move |m| {
            let _ = tx.send(Inbound::Mission(m));
        });
    }
    {
        let tx = tx.clone();
        bus.subscribe::<CommandFrame>(move |frame: CommandFrame| {
            // Dedup/ordering lives here, on the network thread, so the
            // control thread only ever sees a command exactly once, the
            // moment it newly takes effect — never a stale retransmit.
            if let Some(body) = cmd_rx.apply(frame) {
                let _ = tx.send(Inbound::Cmd(body));
            }
        });
    }
    drop(tx);

    loop {
        bus.poll();

        while let Ok(msg) = rx.try_recv() {
            let result = match msg {
                Outbound::State(s) => bus.publish(&s),
                Outbound::EkfDbg(d) => bus.publish(&d),
                Outbound::Chassis(c) => bus.publish(&c),
                Outbound::SpeedDbg(d) => bus.publish(&d),
            };
            if let Err(e) = result {
                log::warn!("rover-control: publish failed: {e}");
            }
        }

        // `Link::recv`/`poll` never block (see `rover_link::Link`'s
        // contract), so this thread would otherwise spin at 100% CPU doing
        // nothing between packets. A short sleep is not on the hot path —
        // the control thread's own tick rate is what matters for timing —
        // it only bounds how quickly a received frame reaches the control
        // thread, which a millisecond does not meaningfully affect at these
        // message rates (the fastest feed, ImuSample, is 100 Hz).
        thread::sleep(Duration::from_millis(1));
    }
}

/// The control thread: estimate -> guide -> actuate, at a fixed
/// `estimator.predict_hz` tick rate. See this module's top-level docs for
/// why this is a plain sleeping loop rather than an async task.
fn control_loop(
    cfg: AppConfig,
    inbound_rx: &mpsc::Receiver<Inbound>,
    outbound_tx: &mpsc::Sender<Outbound>,
) {
    let mut estimator = Estimator::at_rest(cfg.vehicle, cfg.estimator, INITIAL_P_DIAG);

    let ControllerLaw::StaticGain(sg_cfg) = cfg.law;
    let mut controller = StaticGain::new(
        StaticGainConfig {
            k_lat_deg_per_m: sg_cfg.k_lat_deg_per_m,
            k_head_deg_per_deg: sg_cfg.k_head_deg_per_deg,
        },
        cfg.vehicle,
    );

    let mut actuator = Actuator::new(ActuatorConfig {
        steer_max_deg: cfg.steer_max_deg,
        steer_slew_deg_per_s: cfg.steer_slew_deg_per_s,
        reference_mps: cfg.speed.reference_mps,
        limit_cap_pct: cfg.speed.limit_cap_pct,
        sensor_timeout_s: cfg.speed.sensor_timeout_s,
        metres_per_tick: cfg.metres_per_tick,
        speed_kp: cfg.speed.kp,
        speed_ki: cfg.speed.ki,
        speed_kd: cfg.speed.kd,
        integral_limit: cfg.speed.integral_limit,
        max_duty_step_pct: cfg.speed.max_duty_step_pct,
        autocal: cfg.speed.autocal,
        stall: cfg.speed.stall,
    });

    log::info!("rover-control: guidance law = `{}`", controller.name());

    let mut gate = SafetyGate::new();

    let start = Instant::now();
    let tick_period = Duration::from_secs_f32(1.0 / cfg.predict_hz);
    let command_period = Duration::from_secs_f32(1.0 / cfg.command_rate_hz);

    let mut last_state = estimator.state();
    let mut last_command_at: Option<Duration> = None;
    let mut last_throttle = 0.0_f32;
    let mut was_drive_allowed = false;
    let mut was_stall_latched = false;

    loop {
        let tick_start = Instant::now();

        // Drain everything the network thread has forwarded since the last
        // tick, in arrival order, before computing anything this tick.
        while let Ok(msg) = inbound_rx.try_recv() {
            match msg {
                Inbound::Imu(sample) => {
                    if let Some(state) = estimator.predict_from_imu(&sample) {
                        last_state = state;
                        let _ = outbound_tx.send(Outbound::State(last_state));
                    }
                }
                Inbound::Lane(meas) => {
                    if let Some(dbg) = estimator.correct_lane(&meas) {
                        let _ = outbound_tx.send(Outbound::EkfDbg(dbg));
                    }
                    last_state = estimator.state();
                }
                Inbound::Wheel(wheel) => {
                    // Gyro z for the zero-rate update: the newest estimate's
                    // own bias-uncorrected reading isn't tracked separately
                    // here, so reuse the last known state's speed-adjacent
                    // context is unnecessary — `correct_wheel_sensors` only
                    // needs the raw gyro rate, which the estimator does not
                    // cache between IMU ticks. Passing 0.0 when no fresher
                    // reading is available only affects the zero-rate
                    // gyro-bias update's own measurement, not the odometry
                    // path, and a stale-by-one-tick gyro rate is negligible
                    // at these rates.
                    estimator.correct_wheel(&wheel, last_throttle, 0.0);
                    last_state = estimator.state();

                    let drive_allowed = gate.drive_allowed();
                    let elapsed = tick_start.duration_since(start).as_secs_f64();
                    if let Some(dbg) =
                        actuator.handle_wheel_sensors(wheel, elapsed, &last_state, drive_allowed)
                    {
                        let _ = outbound_tx.send(Outbound::SpeedDbg(dbg));
                    }
                }
                Inbound::Mission(status) => gate.on_mission_status(status.active),
                Inbound::Cmd(Command::SetSpeedLimit(pct)) => actuator.set_speed_limit_pct(pct),
                Inbound::Cmd(cmd) => gate.on_command(cmd),
            }
        }

        let drive_allowed = gate.drive_allowed();
        if drive_allowed && !was_drive_allowed {
            // Resuming after a stop: reset any controller memory rather than
            // letting it pick back up from whatever it held while the
            // rover was stopped. A true no-op for the stateless `StaticGain`
            // today, but load-bearing for a future stateful law (Lqr/Mpc).
            controller.reset();
        }
        was_drive_allowed = drive_allowed;

        // `StaticGain` is stateless and ignores `dt` (see `guide`'s module
        // docs); the nominal tick period is passed for forward
        // compatibility with a future stateful law (Lqr/Mpc) that would
        // need it.
        let steer_rad = controller.steer(&last_state, tick_period.as_secs_f32());

        let stall_latched = actuator.is_stall_latched();
        if stall_latched && !was_stall_latched {
            log::error!("rover-control: drivetrain stall latched — commanding zero speed until an explicit stop is commanded");
        }
        was_stall_latched = stall_latched;

        let now_elapsed = tick_start.duration_since(start);
        let due = last_command_at.is_none_or(|t| now_elapsed.saturating_sub(t) >= command_period);
        if due {
            let dt_s = last_command_at.map_or(tick_period.as_secs_f32(), |t| {
                (now_elapsed - t).as_secs_f32()
            });
            let cmd = actuator.tick_chassis_command(
                steer_rad,
                &last_state,
                drive_allowed,
                now_elapsed.as_secs_f64(),
                dt_s,
            );
            last_throttle = cmd.throttle;
            last_command_at = Some(now_elapsed);
            let _ = outbound_tx.send(Outbound::Chassis(cmd));
        }

        let sleep_for = tick_period.saturating_sub(tick_start.elapsed());
        if sleep_for > Duration::ZERO {
            thread::sleep(sleep_for);
        }
    }
}
