//! Linearised lateral error dynamics — the vehicle model shared by the EKF's
//! predict step (`rover-estimator`) and, later, the LQR and MPC controllers
//! (plan §8.1, "one model, three consumers"). This crate exists specifically
//! so the estimator and the controller can never quietly disagree about the
//! vehicle: change the physics here and both stay consistent by construction.
//!
//! # State
//!
//! Order and meaning match [`rover_msgs::state_idx`] / [`rover_msgs::EKF_STATES`]
//! exactly, so an index used here is the same index the estimator and the
//! wire format use:
//!
//! ```text
//! x = [ e_y      cross-track error at the FRONT AXLE   (m)
//!       e_psi    heading error vs. the lane tangent    (rad)
//!       kappa    lane curvature ahead                  (1/m)
//!       v        forward speed                         (m/s)
//!       b_g    ] gyro z-axis bias                       (rad/s)
//! ```
//!
//! # Sign convention
//!
//! Positive `e_y` / `e_psi` mean "the correct response is to steer right" —
//! the opposite of ISO 8855. See `rover_msgs`'s crate docs and plan §10. This
//! crate has no "steer" output of its own to get backwards, but every
//! consumer that closes a loop around `A`/`B` does, so the convention is
//! restated here for anyone reading this file in isolation: do not silently
//! renormalise these matrices to the textbook convention.
//!
//! # Why first-order (Euler) discretisation is enough
//!
//! `F_d ≈ I + F_c·dt` drops all `O(dt²)` terms. At `dt = 10 ms` (the
//! estimator's fixed 100 Hz predict rate) and states that evolve on a
//! roughly 1-second timescale — this rover cruises at ~0.2 m/s — the
//! discarded terms are four-plus orders of magnitude smaller than the
//! `O(dt)` term already competing with the process-noise budget `Q`. A
//! matrix exponential would cost real cycles on an RPi 4 at 100 Hz for no
//! measurable accuracy gain. If the loop rate ever drops by an order of
//! magnitude relative to the vehicle's own dynamics (e.g. a much faster
//! rover), revisit this.
//!
//! # A note on "A(v)"
//!
//! The plan (§2.2, §8.1) writes the Jacobian as a function of speed alone,
//! `A(v)`. That is only exactly right at the nominal linearisation point
//! `e_psi = 0`, `kappa = 0`: two entries of the true Jacobian
//! (`∂ e_y_dot/∂v = e_psi`, `∂ e_psi_dot/∂v = -kappa`) vanish only there.
//! [`process_jacobian`] takes the full state rather than `v` alone, because
//! the estimator needs the exact Jacobian at its current belief, not just
//! the nominal one — passing the full state costs nothing and is correct
//! everywhere, whereas a `v`-only version would be exact only on a straight
//! line at zero cross-track error. See this crate's test suite for a check
//! that the two agree at the nominal point.

#![forbid(unsafe_code)]

use nalgebra::{SMatrix, SVector};
use rover_msgs::{state_idx, EKF_STATES};

/// State vector, `[e_y, e_psi, kappa, v, b_g]`.
pub type StateVector = SVector<f32, EKF_STATES>;

/// State-transition / Jacobian matrix, `EKF_STATES x EKF_STATES`.
pub type StateMatrix = SMatrix<f32, EKF_STATES, EKF_STATES>;

/// A single control input mapped onto the full state derivative (i.e. one
/// column of a control input matrix `B`).
pub type InputVector = SVector<f32, EKF_STATES>;

/// Fixed vehicle geometry.
///
/// Loaded from `config/rover.toml`'s `[drivetrain]` and `[perception]`
/// tables by whichever binary owns startup I/O — this crate stays pure data
/// with no parsing of its own, so it has nothing to get wrong about TOML and
/// nothing that stops it running in a unit test.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VehicleParams {
    /// Front-to-rear axle distance, metres. `config/rover.toml`:
    /// `drivetrain.wheelbase_m` (0.4875, measured, field-validated — plan
    /// §10 lists it as a "must not change" constant).
    pub wheelbase_m: f32,
    /// Distance ahead of the front axle at which the camera fits lane
    /// parameters, metres. `config/rover.toml`: `perception.lookahead_m`
    /// (1.22, set by the ROI geometry).
    pub lookahead_m: f32,
}

impl VehicleParams {
    pub const fn new(wheelbase_m: f32, lookahead_m: f32) -> Self {
        Self {
            wheelbase_m,
            lookahead_m,
        }
    }
}

/// Nonlinear continuous-time process model `f(x, gyro_radps)`, plan §2.2.
///
/// Returns the state derivative. Deliberately kept next to
/// [`process_jacobian`] in the same file: a classic EKF bug is for the mean
/// propagation and its Jacobian to be transcribed separately and drift apart.
/// Here there is exactly one transcription of the physics.
///
/// Uses the small-angle form `e_y_dot = v * e_psi` (not `v * sin(e_psi)`)
/// throughout, matching [`process_jacobian`] exactly — the plan derives `F_c`
/// by differentiating the small-angle model, not the exact trig form, and the
/// whole formulation (including the camera's linear `H`) already assumes
/// `e_psi` stays small. Using `sin` here and the linearised derivative there
/// would make the two agree only to first order instead of exactly.
pub fn process_derivative(x: &StateVector, gyro_radps: f32) -> StateVector {
    let e_psi = x[state_idx::HEADING_ERR];
    let kappa = x[state_idx::CURVATURE];
    let v = x[state_idx::SPEED];
    let b_g = x[state_idx::GYRO_BIAS];

    let mut dx = StateVector::zeros();
    // e_y_dot = v * sin(e_psi) ~= v * e_psi
    dx[state_idx::CROSS_TRACK] = v * e_psi;
    // e_psi_dot = (w_gyro - b_g) - v * kappa
    dx[state_idx::HEADING_ERR] = (gyro_radps - b_g) - v * kappa;
    // kappa, v, b_g: random walks. Their deterministic derivative is zero;
    // the estimator's covariance update adds process noise Q separately.
    dx
}

/// Continuous-time state Jacobian `F_c = df/dx` of [`process_derivative`],
/// evaluated at the current estimate `x`. Plan §2.2:
///
/// ```text
///           e_y  e_psi  kappa    v      b_g
///  e_y    [  0     v      0     e_psi    0  ]
///  e_psi  [  0     0     -v    -kappa   -1  ]
///  kappa  [  0     0      0      0       0  ]
///  v      [  0     0      0      0       0  ]
///  b_g    [  0     0      0      0       0  ]
/// ```
///
/// See the module docs for why this takes the full state rather than `v`
/// alone.
pub fn process_jacobian(x: &StateVector) -> StateMatrix {
    let e_psi = x[state_idx::HEADING_ERR];
    let kappa = x[state_idx::CURVATURE];
    let v = x[state_idx::SPEED];

    let mut f = StateMatrix::zeros();
    f[(state_idx::CROSS_TRACK, state_idx::HEADING_ERR)] = v;
    f[(state_idx::CROSS_TRACK, state_idx::SPEED)] = e_psi;
    f[(state_idx::HEADING_ERR, state_idx::CURVATURE)] = -v;
    f[(state_idx::HEADING_ERR, state_idx::SPEED)] = -kappa;
    f[(state_idx::HEADING_ERR, state_idx::GYRO_BIAS)] = -1.0;
    f
}

/// First-order discretisation `F_d ~= I + F_c * dt`. See the module docs for
/// why Euler is adequate at the estimator's 10 ms predict step.
pub fn discretize(f_c: &StateMatrix, dt_s: f32) -> StateMatrix {
    StateMatrix::identity() + f_c * dt_s
}

/// Continuous-time input matrix `B(v)`, for a **steering-angle** input
/// (radians), intended for the future LQR/MPC controllers (plan §8.1).
///
/// Small-angle bicycle kinematics: a steering angle `delta` at speed `v`
/// produces yaw rate `v / wheelbase * delta`, which enters the state
/// derivative in exactly the same row as the gyro-rate term in
/// [`process_derivative`] — both are "some yaw rate arriving at the
/// heading-error equation," one measured, one commanded.
///
/// `rover-estimator` does **not** use this: the EKF takes the gyro reading
/// directly as a measured input (see [`process_derivative`]'s `gyro_radps`
/// argument) rather than inferring yaw rate from a commanded steering angle.
pub fn control_b(v: f32, params: &VehicleParams) -> InputVector {
    let mut b = InputVector::zeros();
    b[state_idx::HEADING_ERR] = v / params.wheelbase_m;
    b
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state(e_y: f32, e_psi: f32, kappa: f32, v: f32, b_g: f32) -> StateVector {
        let mut x = StateVector::zeros();
        x[state_idx::CROSS_TRACK] = e_y;
        x[state_idx::HEADING_ERR] = e_psi;
        x[state_idx::CURVATURE] = kappa;
        x[state_idx::SPEED] = v;
        x[state_idx::GYRO_BIAS] = b_g;
        x
    }

    #[test]
    fn process_derivative_matches_hand_worked_example() {
        // e_psi = 0.1 rad, kappa = 0.5 1/m, v = 0.2 m/s, b_g = 0.01 rad/s,
        // gyro reads 0.02 rad/s.
        let x = state(0.0, 0.1, 0.5, 0.2, 0.01);
        let dx = process_derivative(&x, 0.02);

        assert!((dx[state_idx::CROSS_TRACK] - 0.2 * 0.1).abs() < 1e-6);
        // e_psi_dot = (0.02 - 0.01) - 0.2*0.5 = 0.01 - 0.1 = -0.09
        assert!((dx[state_idx::HEADING_ERR] - (-0.09)).abs() < 1e-6);
        assert_eq!(dx[state_idx::CURVATURE], 0.0);
        assert_eq!(dx[state_idx::SPEED], 0.0);
        assert_eq!(dx[state_idx::GYRO_BIAS], 0.0);
    }

    #[test]
    fn process_jacobian_matches_plan_2_2() {
        let x = state(0.0, 0.1, 0.5, 0.2, 0.01);
        let f = process_jacobian(&x);

        assert_eq!(f[(state_idx::CROSS_TRACK, state_idx::HEADING_ERR)], 0.2); // v
        assert_eq!(f[(state_idx::CROSS_TRACK, state_idx::SPEED)], 0.1); // e_psi
        assert_eq!(f[(state_idx::HEADING_ERR, state_idx::CURVATURE)], -0.2); // -v
        assert_eq!(f[(state_idx::HEADING_ERR, state_idx::SPEED)], -0.5); // -kappa
        assert_eq!(f[(state_idx::HEADING_ERR, state_idx::GYRO_BIAS)], -1.0);

        // Every other entry is zero: rows kappa/v/b_g are pure random walks,
        // and no other e_y/e_psi coupling exists.
        let mut expected_nonzero = StateMatrix::zeros();
        expected_nonzero[(state_idx::CROSS_TRACK, state_idx::HEADING_ERR)] = 1.0;
        expected_nonzero[(state_idx::CROSS_TRACK, state_idx::SPEED)] = 1.0;
        expected_nonzero[(state_idx::HEADING_ERR, state_idx::CURVATURE)] = 1.0;
        expected_nonzero[(state_idx::HEADING_ERR, state_idx::SPEED)] = 1.0;
        expected_nonzero[(state_idx::HEADING_ERR, state_idx::GYRO_BIAS)] = 1.0;
        for r in 0..EKF_STATES {
            for c in 0..EKF_STATES {
                if expected_nonzero[(r, c)] == 0.0 {
                    assert_eq!(f[(r, c)], 0.0, "unexpected nonzero at ({r},{c})");
                }
            }
        }
    }

    #[test]
    fn jacobian_matches_nominal_a_of_v_at_zero_operating_point() {
        // At e_psi = kappa = 0 the "A(v)" the plan writes should fall out of
        // the full Jacobian exactly.
        let v = 0.35;
        let x = state(0.0, 0.0, 0.0, v, 0.0);
        let f = process_jacobian(&x);
        assert_eq!(f[(state_idx::CROSS_TRACK, state_idx::HEADING_ERR)], v);
        assert_eq!(f[(state_idx::CROSS_TRACK, state_idx::SPEED)], 0.0);
        assert_eq!(f[(state_idx::HEADING_ERR, state_idx::CURVATURE)], -v);
        assert_eq!(f[(state_idx::HEADING_ERR, state_idx::SPEED)], 0.0);
    }

    #[test]
    fn discretize_is_first_order_euler() {
        let f_c = process_jacobian(&state(0.0, 0.1, 0.5, 0.2, 0.0));
        let dt = 0.01;
        let f_d = discretize(&f_c, dt);
        let expected = StateMatrix::identity() + f_c * dt;
        assert_eq!(f_d, expected);
        // Off the diagonal it should just be F_c * dt...
        assert!((f_d[(state_idx::CROSS_TRACK, state_idx::HEADING_ERR)] - 0.2 * dt).abs() < 1e-9);
        // ...and the diagonal must stay at 1 (identity contribution).
        for i in 0..EKF_STATES {
            assert!((f_d[(i, i)] - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn control_b_puts_steering_effect_on_heading_row_only() {
        let params = VehicleParams::new(0.4875, 1.22);
        let v = 0.3;
        let b = control_b(v, &params);
        for i in 0..EKF_STATES {
            if i == state_idx::HEADING_ERR {
                assert!((b[i] - v / params.wheelbase_m).abs() < 1e-6);
            } else {
                assert_eq!(b[i], 0.0);
            }
        }
    }

    #[test]
    fn control_b_scales_linearly_with_speed() {
        let params = VehicleParams::new(0.4875, 1.22);
        let b_slow = control_b(0.1, &params);
        let b_fast = control_b(0.3, &params);
        assert!(
            (b_fast[state_idx::HEADING_ERR] - 3.0 * b_slow[state_idx::HEADING_ERR]).abs() < 1e-6
        );
    }
}
