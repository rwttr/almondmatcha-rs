//! A faithful Rust port of the **old** ROS2 law —
//! `rover_kinematic_control_node.py`'s EMA-filtered static-gain feedback plus
//! Ackermann feedforward — used **only** as a comparison oracle for
//! synthetic traces (`crate::synthetic`). This is deliberately a second,
//! independent implementation from `rover_control::guide::StaticGain`: the
//! whole point of replay is to check the new EKF-based pipeline against
//! something that is not itself part of the new pipeline.
//!
//! This is not the `StaticGain` controller shipped in `rover-control`, and
//! it is not meant to be. `rover-control`'s `StaticGain` consumes
//! `RoverState` (the EKF's lookahead-reconstructed estimate); this type
//! consumes raw per-frame `LaneMeasurement`-shaped values and does its own
//! `ExponentialMovingAverageLPF`-equivalent smoothing, exactly like
//! `control_filters.py`'s `ExponentialMovingAverageLPF`. The two should
//! agree closely on the *same underlying physical signal* — that agreement
//! (or its absence) is what `crate::replay` measures.
//!
//! Ported behaviour, from `rover_kinematic_control_node.py`:
//! - Each raw input clamped before filtering: `theta_deg ∈ [-35, 35]`,
//!   `b_m ∈ [-0.50, 0.50]` (curvature is not clamped in the source either).
//! - EMA with `alpha = ema_alpha`, updated only on frames where the lane was
//!   detected; held (not advanced, not reset) on lost frames.
//! - "Warmed up" only once 30 detected frames have been seen — matching the
//!   Python `ExponentialMovingAverageLPF(alpha, maxlen=30)` buffer's
//!   `is_full()` check on all three filters at once (they always fill at
//!   the same rate here, since all three update on the same condition).
//! - `steer_when_lost` (`0.0`) whenever the lane is lost *or* the filters
//!   have not yet warmed up.
//! - `u_fb = k_lat * b_ema + k_head * theta_ema_deg`,
//!   `u_ff = atan(wheelbase_m * curvature_ema).to_degrees()`,
//!   `u_total = u_fb + u_ff`, clamped to `±steer_max_deg`.

/// `ExponentialMovingAverageLPF` from `control_filters.py`: `alpha`-weighted
/// exponential smoothing plus a warm-up counter standing in for the
/// Python's bounded `deque(maxlen=30)` (only its length matters here, not
/// the buffered values themselves — nothing reads the history back out).
struct Ema {
    alpha: f32,
    value: Option<f32>,
    count: usize,
}

impl Ema {
    const MAXLEN: usize = 30;

    fn new(alpha: f32) -> Self {
        Self {
            alpha,
            value: None,
            count: 0,
        }
    }

    fn update(&mut self, x: f32) -> f32 {
        let y = match self.value {
            None => x,
            Some(prev) => self.alpha * x + (1.0 - self.alpha) * prev,
        };
        self.value = Some(y);
        self.count = (self.count + 1).min(Self::MAXLEN);
        y
    }

    fn is_full(&self) -> bool {
        self.count >= Self::MAXLEN
    }

    fn held(&self) -> f32 {
        self.value.unwrap_or(0.0)
    }
}

/// Tuning knobs mirroring `rover_kinematic_control_node.py`'s ROS2
/// parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LegacyConfig {
    pub k_lat_deg_per_m: f32,
    pub k_head_deg_per_deg: f32,
    pub wheelbase_m: f32,
    pub ema_alpha: f32,
    pub steer_max_deg: f32,
    pub steer_when_lost_deg: f32,
}

impl Default for LegacyConfig {
    /// The field-derived values plan §10 lists, plus `ema_alpha = 0.05` and
    /// `steer_when_lost = 0.0` from `rover_kinematic_control_node.py`'s own
    /// parameter declarations.
    fn default() -> Self {
        Self {
            k_lat_deg_per_m: 181.17,
            k_head_deg_per_deg: 2.024,
            wheelbase_m: 0.4875,
            ema_alpha: 0.05,
            steer_max_deg: 45.0,
            steer_when_lost_deg: 0.0,
        }
    }
}

pub struct LegacyEmaLaw {
    cfg: LegacyConfig,
    theta_ema_deg: Ema,
    b_ema_m: Ema,
    curvature_ema_inv_m: Ema,
}

impl LegacyEmaLaw {
    pub fn new(cfg: LegacyConfig) -> Self {
        Self {
            theta_ema_deg: Ema::new(cfg.ema_alpha),
            b_ema_m: Ema::new(cfg.ema_alpha),
            curvature_ema_inv_m: Ema::new(cfg.ema_alpha),
            cfg,
        }
    }

    /// One frame. `valid` mirrors `LaneMeasurement::valid` — the vision
    /// pipeline's own per-frame detection flag, at the camera's rate. A
    /// caller replaying a 100 Hz trace should call this only on rows where
    /// `lane_valid` changed to a fresh reading, exactly like the ROS2 node
    /// only ran this per incoming `tpc_rover_nav_lane` message rather than
    /// once per control tick.
    pub fn step(
        &mut self,
        heading_err_rad: f32,
        cross_track_m: f32,
        curvature_inv_m: f32,
        valid: bool,
    ) -> f32 {
        if valid {
            let theta_deg = heading_err_rad.to_degrees().clamp(-35.0, 35.0);
            let b_m = cross_track_m.clamp(-0.50, 0.50);
            self.theta_ema_deg.update(theta_deg);
            self.b_ema_m.update(b_m);
            self.curvature_ema_inv_m.update(curvature_inv_m);
        }

        let warmed_up = self.theta_ema_deg.is_full()
            && self.b_ema_m.is_full()
            && self.curvature_ema_inv_m.is_full();

        if !valid || !warmed_up {
            return self.cfg.steer_when_lost_deg;
        }

        let theta_e = self.theta_ema_deg.held();
        let b_e = self.b_ema_m.held();
        let k_e = self.curvature_ema_inv_m.held();

        let u_fb = self.cfg.k_lat_deg_per_m * b_e + self.cfg.k_head_deg_per_deg * theta_e;
        let u_ff = (self.cfg.wheelbase_m * k_e).atan().to_degrees();
        (u_fb + u_ff).clamp(-self.cfg.steer_max_deg, self.cfg.steer_max_deg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steers_when_lost_before_warm_up() {
        let mut law = LegacyEmaLaw::new(LegacyConfig::default());
        let u = law.step(0.1, 0.1, 0.0, true);
        assert_eq!(
            u, 0.0,
            "must hold steer_when_lost until 30 detected frames have accumulated"
        );
    }

    #[test]
    fn converges_after_warm_up_on_a_constant_input() {
        let mut law = LegacyEmaLaw::new(LegacyConfig::default());
        let mut last = 0.0;
        for _ in 0..200 {
            last = law.step(0.0, 0.1, 0.0, true);
        }
        // Constant 0.1m offset, zero heading/curvature: after the EMA has
        // fully converged, u_fb = k_lat * 0.1 = 18.117 deg.
        assert!((last - 18.117).abs() < 1e-2, "got {last}");
    }

    #[test]
    fn holds_state_and_reports_lost_on_a_dropout() {
        let mut law = LegacyEmaLaw::new(LegacyConfig::default());
        for _ in 0..200 {
            law.step(0.0, 0.1, 0.0, true);
        }
        let u_lost = law.step(0.0, 999.0, 999.0, false); // garbage input, must be ignored
        assert_eq!(
            u_lost, 0.0,
            "lost frames must report steer_when_lost, not the held EMA"
        );

        // Recovery resumes from the held state, not from the garbage.
        let u_recovered = law.step(0.0, 0.1, 0.0, true);
        assert!((u_recovered - 18.117).abs() < 0.5, "got {u_recovered}");
    }

    #[test]
    fn saturates_at_steer_max_deg() {
        let mut law = LegacyEmaLaw::new(LegacyConfig::default());
        for _ in 0..200 {
            law.step(0.0, 0.50, 0.0, true); // max clamp input
        }
        let u = law.step(0.0, 0.50, 0.0, true);
        assert!((u - 45.0).abs() < 1e-3, "got {u}");
    }
}
