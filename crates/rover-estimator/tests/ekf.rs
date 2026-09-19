//! Behavioural tests for the EKF, matching the checklist in
//! `docs/RUST_REWRITE_PLAN.md` §2: this filter must be trustworthy before it
//! ever drives a motor.

use rover_estimator::{Ekf, EstimatorConfig, LaneNoise, ProcessNoise};
use rover_model::{StateVector, VehicleParams};
use rover_msgs::{state_idx, LaneMeasurement, RoverState, WheelSensors};

const DT: f32 = 0.01; // 100 Hz predict, matching config/rover.toml's estimator.predict_hz

fn params() -> VehicleParams {
    VehicleParams::new(0.4875, 1.22) // plan §10 field-validated constants
}

/// Config mirroring `config/rover.toml`'s `[estimator]` table. `metres_per_tick`
/// is the one field the shipped config does not have calibrated (§2.6), so
/// callers pass it explicitly per test.
///
/// Q here is a per-second spectral density, matching `config/rover.toml`.
fn cfg(metres_per_tick: f32) -> EstimatorConfig {
    EstimatorConfig {
        q: ProcessNoise::new(1e-2, 1e-2, 1e-1, 1e-1, 1e-5),
        r_lane: LaneNoise::new(2.5e-3, 2.0e-3, 1.0e-2),
        r_odom_speed: 1.0e-3,
        r_zero_rate_gyro: 1e-6,
        nis_gate: 11.34,
        zero_rate_max_ticks: 0,
        metres_per_tick,
    }
}

fn lane_measurement(
    cross_track_m: f32,
    heading_err_rad: f32,
    curvature_inv_m: f32,
    t_us: u32,
) -> LaneMeasurement {
    LaneMeasurement {
        curvature_inv_m,
        heading_err_rad,
        cross_track_m,
        valid: true,
        t_us,
    }
}

// ---------------------------------------------------------------------------
// 1. Straight-line, noise-free: filter converges to zero error and stays there.
// ---------------------------------------------------------------------------
#[test]
fn straight_line_converges_to_zero_and_holds() {
    let cfg = cfg(0.0);
    // True state: perfectly on-line, straight, constant speed. Filter starts
    // offset from it.
    let mut x0 = StateVector::zeros();
    x0[state_idx::CROSS_TRACK] = 0.15;
    x0[state_idx::HEADING_ERR] = 0.08;
    x0[state_idx::SPEED] = 0.2;
    let mut ekf = Ekf::new(params(), x0, [1e-2, 1e-2, 1e-2, 1e-2, 1e-4]);

    let total_steps = 500; // 5 s at 100 Hz
    let camera_every = 3; // ~33 Hz, close to the real camera rate
    for step in 0..total_steps {
        ekf.predict(0.0, DT, &cfg); // true yaw rate is 0, no bias
        if step % camera_every == 0 {
            ekf.correct_camera(&lane_measurement(0.0, 0.0, 0.0, step as u32 * 10_000), &cfg);
        }
    }

    let s = ekf.state();
    assert!(
        s.cross_track_m.abs() < 1e-3,
        "cross_track_m did not converge: {}",
        s.cross_track_m
    );
    assert!(
        s.heading_err_rad.abs() < 1e-3,
        "heading_err_rad did not converge: {}",
        s.heading_err_rad
    );

    // Run further and confirm it stays converged rather than drifting back off.
    for step in total_steps..(total_steps + 200) {
        ekf.predict(0.0, DT, &cfg);
        if step % camera_every == 0 {
            ekf.correct_camera(&lane_measurement(0.0, 0.0, 0.0, step as u32 * 10_000), &cfg);
        }
    }
    let s2 = ekf.state();
    assert!(
        s2.cross_track_m.abs() < 1e-3,
        "estimate drifted back off zero: {}",
        s2.cross_track_m
    );
    assert!(
        s2.heading_err_rad.abs() < 1e-3,
        "estimate drifted back off zero: {}",
        s2.heading_err_rad
    );
}

// ---------------------------------------------------------------------------
// 2. Constant-curvature arc: filter tracks curvature.
// ---------------------------------------------------------------------------
#[test]
fn constant_curvature_arc_is_tracked() {
    let cfg = cfg(0.0);
    let true_kappa = 0.5_f32;
    let true_v = 0.2_f32;
    // Vehicle exactly on the arc (e_y = e_psi = 0 throughout): maintaining
    // that requires a true yaw rate of v * kappa.
    let true_yaw_rate = true_v * true_kappa;

    let mut x0 = StateVector::zeros();
    x0[state_idx::SPEED] = true_v; // kappa starts at the wrong value, 0.0
    let mut ekf = Ekf::new(params(), x0, [1e-2, 1e-2, 1e-1, 1e-2, 1e-4]);

    let l_a = params().lookahead_m;
    let true_cross = 0.5 * true_kappa * l_a * l_a; // e_y=0, e_psi=0 => kappa*L_a^2/2
    let true_heading = l_a * true_kappa; // e_psi=0 => L_a*kappa

    let total_steps = 1000; // 10 s
    let camera_every = 3;
    for step in 0..total_steps {
        ekf.predict(true_yaw_rate, DT, &cfg);
        if step % camera_every == 0 {
            ekf.correct_camera(
                &lane_measurement(true_cross, true_heading, true_kappa, step as u32 * 10_000),
                &cfg,
            );
        }
    }

    let s = ekf.state();
    assert!(
        (s.curvature_inv_m - true_kappa).abs() < 0.05,
        "curvature did not converge: got {}, want {true_kappa}",
        s.curvature_inv_m
    );
    assert!(
        s.cross_track_m.abs() < 0.05,
        "front-axle cross track should stay near zero: {}",
        s.cross_track_m
    );
    assert!(
        s.heading_err_rad.abs() < 0.05,
        "front-axle heading error should stay near zero: {}",
        s.heading_err_rad
    );
}

// ---------------------------------------------------------------------------
// 3. Lane dropout: coasts without diverging; lane_age_ms grows; covariance grows.
// ---------------------------------------------------------------------------
#[test]
fn lane_dropout_coasts_without_diverging() {
    let cfg = cfg(0.0);
    let mut ekf = Ekf::at_rest(params(), [1e-3, 1e-3, 1e-3, 1e-3, 1e-6]);
    let p0 = ekf.state().p_diag;

    // A real (small, unmodelled) gyro bias in the raw reading -- the filter
    // starts believing b_g = 0, so this exercises actual drift, not a
    // trivially-static state.
    let raw_gyro = 0.01_f32;

    let dropout_steps = 300; // 3 s at 100 Hz
    for _ in 0..dropout_steps {
        ekf.predict(raw_gyro, DT, &cfg);
        let dropped = LaneMeasurement {
            valid: false,
            ..lane_measurement(0.0, 0.0, 0.0, 0)
        };
        let debug = ekf.correct_camera(&dropped, &cfg);
        assert!(
            debug.is_none(),
            "an invalid measurement must produce no EkfDebug and no update"
        );
    }

    let s = ekf.state();
    assert_eq!(
        s.lane_age_ms, 3000,
        "lane_age_ms must saturate-add up to exactly the elapsed dropout time"
    );

    // Bounded, not diverged: small residual bias over 3 s should only drift
    // the estimate by a small amount, never blow up or go NaN.
    assert!(s.cross_track_m.is_finite() && s.cross_track_m.abs() < 0.5);
    assert!(s.heading_err_rad.is_finite() && s.heading_err_rad.abs() < 0.5);

    for (i, (&before, &after)) in p0.iter().zip(s.p_diag.iter()).enumerate() {
        assert!(
            after > before,
            "P[{i}] should grow through a dropout: {before} -> {after}"
        );
    }
}

// ---------------------------------------------------------------------------
// 4. Gyro bias: zero-rate update converges b_g to a known injected bias.
// ---------------------------------------------------------------------------
#[test]
fn zero_rate_update_converges_gyro_bias() {
    let cfg = cfg(0.0);
    let mut ekf = Ekf::at_rest(params(), [1e-2, 1e-2, 1e-2, 1e-2, 1e-4]);
    let true_bias = 0.02_f32;

    // Stationary: throttle is zero and neither encoder ticks between samples.
    let mut t_us = 0u32;
    let mut wheel = WheelSensors {
        ticks_left: 0,
        ticks_right: 0,
        t_us,
    };
    ekf.correct_wheel_sensors(&wheel, 0.0, true_bias, &cfg); // seeds last_wheel only

    for _ in 0..80 {
        t_us += 100_000; // 100 ms, 10 Hz
        wheel = WheelSensors {
            ticks_left: 0,
            ticks_right: 0,
            t_us,
        };
        // gyro reads true yaw rate (0, stationary) + bias.
        ekf.correct_wheel_sensors(&wheel, 0.0, true_bias, &cfg);
    }

    let s = ekf.state();
    assert!(
        (s.gyro_bias_radps - true_bias).abs() < 0.002,
        "gyro bias did not converge: got {}, want {true_bias}",
        s.gyro_bias_radps
    );
}

// ---------------------------------------------------------------------------
// 5. NIS gate: an outlier measurement is rejected and EkfDebug.gated is true.
// ---------------------------------------------------------------------------
#[test]
fn nis_gate_rejects_outlier_measurement() {
    let cfg = cfg(0.0);
    // A confident filter: small P, so a large innovation is many sigma away.
    let mut ekf = Ekf::at_rest(params(), [1e-4, 1e-4, 1e-4, 1e-4, 1e-6]);

    let outlier = lane_measurement(5.0, 0.0, 0.0, 0); // 5 m cross-track: nonsense
    let debug = ekf
        .correct_camera(&outlier, &cfg)
        .expect("a valid measurement always produces a debug frame");

    assert!(
        debug.gated,
        "an outlier this far from the belief must be gated"
    );
    assert!(
        debug.nis > cfg.nis_gate,
        "NIS should exceed the gate: {}",
        debug.nis
    );

    let s = ekf.state();
    assert!(
        s.cross_track_m.abs() < 1e-6,
        "a gated measurement must not move the state"
    );
}

// ---------------------------------------------------------------------------
// 6. Lookahead reconstruction round-trips against the camera measurement model.
// ---------------------------------------------------------------------------
#[test]
fn lookahead_reconstruction_round_trips_through_camera_model() {
    let p = params();
    let mut x = StateVector::zeros();
    x[state_idx::CROSS_TRACK] = 0.1;
    x[state_idx::HEADING_ERR] = 0.05;
    x[state_idx::CURVATURE] = 0.3;
    x[state_idx::SPEED] = 0.2;

    let synthetic = RoverState {
        cross_track_m: x[state_idx::CROSS_TRACK],
        heading_err_rad: x[state_idx::HEADING_ERR],
        curvature_inv_m: x[state_idx::CURVATURE],
        speed_mps: x[state_idx::SPEED],
        gyro_bias_radps: 0.0,
        p_diag: [0.0; 5],
        lane_age_ms: 0,
    };
    let (expected_cross, expected_heading) = synthetic.at_lookahead(p.lookahead_m);

    // Hand-computed closed form, independent of `at_lookahead`'s own code,
    // so this test does not just check the function against itself.
    let l_a = p.lookahead_m;
    let hand_cross = 0.1 + l_a * 0.05 + 0.5 * 0.3 * l_a * l_a;
    let hand_heading = 0.05 + l_a * 0.3;
    assert!((expected_cross - hand_cross).abs() < 1e-6);
    assert!((expected_heading - hand_heading).abs() < 1e-6);

    // Now confirm the EKF's own camera measurement model (H_cam) agrees: an
    // exact-match measurement at these lookahead values must produce ~zero
    // innovation against a filter whose front-axle state is `x`.
    let cfg = cfg(0.0);
    let mut ekf = Ekf::new(p, x, [1e-6, 1e-6, 1e-6, 1e-6, 1e-6]);
    let meas = lane_measurement(expected_cross, expected_heading, x[state_idx::CURVATURE], 0);
    let debug = ekf.correct_camera(&meas, &cfg).expect("valid measurement");

    assert!(!debug.gated, "an exact match must not be gated");
    for (i, innov) in debug.innovation.iter().enumerate() {
        assert!(
            innov.abs() < 1e-4,
            "innovation[{i}] should be ~0, got {innov}"
        );
    }
}

// ---------------------------------------------------------------------------
// 7. Sign convention: a positive lane offset must push the estimate positive.
// ---------------------------------------------------------------------------
#[test]
fn sign_convention_left_offset_yields_positive_cross_track_and_right_steer() {
    // rover_msgs's crate docs and plan §10: positive cross_track_m /
    // heading_err_rad mean "the correct response is to steer right." Getting
    // this backwards anywhere turns the closed loop into positive feedback.
    let cfg = cfg(0.0);
    let mut ekf = Ekf::at_rest(params(), [1e-2, 1e-2, 1e-2, 1e-2, 1e-4]);

    let meas = lane_measurement(0.1, 0.0, 0.0, 0); // rover offset such that steering right is correct
    ekf.correct_camera(&meas, &cfg);

    let s = ekf.state();
    assert!(
        s.cross_track_m > 0.0,
        "a positive lane offset must move the estimate positive, not negative"
    );

    // Mirrors the ported control law (plan §2.7, §10): both feedback terms
    // carry a PLUS sign, so a positive estimate must yield a positive
    // (rightward) steer command.
    let k_lat = 181.17_f32;
    let k_head = 2.024_f32;
    let steer_deg = k_lat * s.cross_track_m + k_head * s.heading_err_rad;
    assert!(
        steer_deg > 0.0,
        "steer must be positive (right) for a positive cross-track error, got {steer_deg}"
    );
}

// ---------------------------------------------------------------------------
// 8. metres_per_tick == 0.0 must disable odometry loudly, not silently.
// ---------------------------------------------------------------------------
#[test]
fn odometry_disabled_when_metres_per_tick_is_uncalibrated() {
    let cfg = cfg(0.0); // matches config/rover.toml's shipped, uncalibrated value
    let mut ekf = Ekf::at_rest(params(), [1e-2, 1e-2, 1e-2, 1e-2, 1e-4]);

    let w1 = WheelSensors {
        ticks_left: 0,
        ticks_right: 0,
        t_us: 0,
    };
    let w2 = WheelSensors {
        ticks_left: 50,
        ticks_right: 52,
        t_us: 100_000,
    }; // moving

    ekf.correct_wheel_sensors(&w1, 0.5, 0.0, &cfg);
    let speed_before = ekf.state().speed_mps;
    ekf.correct_wheel_sensors(&w2, 0.5, 0.0, &cfg);
    let speed_after = ekf.state().speed_mps;

    assert!(
        ekf.odometry_disabled(),
        "odometry must self-disable when metres_per_tick == 0.0"
    );
    assert_eq!(
        speed_before, speed_after,
        "a disabled odometry update must not touch the speed estimate"
    );
}

#[test]
fn odometry_updates_speed_once_calibrated() {
    let cfg = cfg(0.001); // pretend calibration: 1 mm/tick
    let mut ekf = Ekf::new(
        params(),
        StateVector::zeros(),
        [1e-2, 1e-2, 1e-2, 1e-2, 1e-4],
    );

    let w1 = WheelSensors {
        ticks_left: 0,
        ticks_right: 0,
        t_us: 0,
    };
    let w2 = WheelSensors {
        ticks_left: 200,
        ticks_right: 200,
        t_us: 100_000,
    }; // 100 ms, 200 ticks/wheel

    ekf.correct_wheel_sensors(&w1, 0.5, 0.0, &cfg);
    ekf.correct_wheel_sensors(&w2, 0.5, 0.0, &cfg);

    // true speed = 200 ticks * 0.001 m/tick / 0.1 s = 2.0 m/s
    assert!(!ekf.odometry_disabled());
    assert!(
        (ekf.state().speed_mps - 2.0).abs() < 1.0,
        "speed estimate should move toward the odometry-implied speed: {}",
        ekf.state().speed_mps
    );
}

/// A gated measurement must NOT count as fresh lane information.
///
/// `lane_age_ms` is what guidance consults to decide how far to trust the
/// state and how fast to drive; guidance never sees `EkfDebug`. If a detector
/// producing consistent garbage kept resetting the age, the rover would read
/// as healthy while the filter coasted with zero corrections — a failure that
/// looks like nothing at all from the outside. Camera liveness is a separate
/// signal (`HealthBits::LANE_STALE`), deliberately not this one.
#[test]
fn gated_measurements_do_not_refresh_lane_age() {
    let cfg = cfg(0.0);
    let mut x0 = StateVector::zeros();
    x0[state_idx::SPEED] = 0.2;
    let mut ekf = Ekf::new(params(), x0, [1e-4, 1e-4, 1e-4, 1e-4, 1e-6]);

    // Converge on a straight line so the filter is confident and the gate has
    // something to reject against.
    for step in 0..300 {
        ekf.predict(0.0, DT, &cfg);
        if step % 3 == 0 {
            ekf.correct_camera(&lane_measurement(0.0, 0.0, 0.0, step * 10_000), &cfg);
        }
    }
    // Not exactly 0: the loop ends a couple of predict steps after the last
    // camera update. Anything under one camera period counts as fresh.
    assert!(
        ekf.state().lane_age_ms < 50,
        "should be fresh after good updates, got {} ms",
        ekf.state().lane_age_ms
    );

    // Now feed wild outliers: the camera is alive, but nothing it reports is
    // usable. Age must climb as though the lane were simply absent.
    let mut gated = 0;
    let mut received = 0;
    for step in 300..600 {
        ekf.predict(0.0, DT, &cfg);
        if step % 3 == 0 {
            received += 1;
            if let Some(dbg) =
                ekf.correct_camera(&lane_measurement(5.0, 1.5, 3.0, step * 10_000), &cfg)
            {
                if dbg.gated {
                    gated += 1;
                }
            }
        }
    }

    assert!(
        gated * 2 > received,
        "expected most outliers to be gated, got {gated}/{received}"
    );
    assert!(
        ekf.state().lane_age_ms >= 1_000,
        "lane_age_ms must track ACCEPTED information, not camera liveness; got {} ms",
        ekf.state().lane_age_ms
    );
}
