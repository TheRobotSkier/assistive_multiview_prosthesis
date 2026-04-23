//! Single source of truth for all tunable constants in the grasp preshaping pipeline.
//!
//! Adjust values here to tune performance and behavior. No other file defines these.

// ── TSDF construction ──────────────────────────────────────────────────────
pub const TSDF_RESOLUTION_M: f32 = 0.005;
pub const TRUNCATION_CELLS: usize = 4;
pub const RAY_ALIGNMENT_THRESHOLD: f32 = 0.8;

// ── Collision detection ────────────────────────────────────────────────────
pub const COLLISION_TOL_M: f32 = 0.005;
pub const BINARY_SEARCH_TOL: f64 = 0.01;

// ── ROI prediction ─────────────────────────────────────────────────────────
pub const PREDICTION_HORIZON_S: f64 = 5.0;
pub const PREDICTION_SAMPLES: usize = 1000;
pub const HAND_RADIUS_M: f64 = 0.05;
pub const MIN_TSDF_DIM_M: f32 = 0.1;
pub const MAX_TSDF_DIM_M: f32 = 0.3;

// ── Fixed twist covariance (replaces topic-based covariance) ───────────────
pub const FIXED_COV_OMEGA: [f64; 3] = [0.01, 0.01, 0.01];
pub const FIXED_COV_V: [f64; 3] = [0.005, 0.005, 0.005];
