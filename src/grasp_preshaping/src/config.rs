// Runtime-tunable constants for the grasp preshaping pipeline.
//
// Every value here delegates to the `runtime_config` module, which loads
// overrides from a YAML file on first access. If no YAML file is found,
// the compile-time defaults (in `runtime_config::defaults`) are used.
//
// Usage: `config::TSDF_RESOLUTION_M()` (note the parentheses — these are functions now).

use std::sync::OnceLock;

// TSDF construction
pub fn TSDF_RESOLUTION_M() -> f32 { crate::runtime_config::get().tsdf_resolution_m }
pub fn TRUNCATION_CELLS() -> usize { crate::runtime_config::get().truncation_cells }
pub fn RAY_ALIGNMENT_THRESHOLD() -> f32 { crate::runtime_config::get().ray_alignment_threshold }

// Collision detection
pub fn COLLISION_TOL_M() -> f32 { crate::runtime_config::get().collision_tol_m }
pub fn BINARY_SEARCH_TOL() -> f64 { crate::runtime_config::get().binary_search_tol }

// ROI prediction
pub fn PREDICTION_HORIZON_S() -> f64 { crate::runtime_config::get().prediction_horizon_s }
pub fn PREDICTION_SAMPLES() -> usize { crate::runtime_config::get().prediction_samples }
pub fn HAND_RADIUS_M() -> f64 { crate::runtime_config::get().hand_radius_m }
pub fn MIN_TSDF_DIM_M() -> f32 { crate::runtime_config::get().min_tsdf_dim_m }
pub fn MAX_TSDF_DIM_M() -> f32 { crate::runtime_config::get().max_tsdf_dim_m }

// Hit-time-informed sampling
pub fn HIT_TIME_SPREAD_S() -> f64 { crate::runtime_config::get().hit_time_spread_s }
pub fn HIT_TIME_EXPLORATION_FRACTION() -> f64 { crate::runtime_config::get().hit_time_exploration_fraction }

// Wrist rotation allowed range (radians). The wrist rotates around the local
// Y axis (supination/pronation). Value is the half-range; actual rotation is
// sampled uniformly from [-WRIST_ROTATION_RANGE_RAD, WRIST_ROTATION_RANGE_RAD].
pub fn WRIST_ROTATION_RANGE_RAD() -> f64 { crate::runtime_config::get().wrist_rotation_range_rad }

// Fixed twist covariance
pub fn FIXED_COV_OMEGA() -> [f64; 3] { crate::runtime_config::get().fixed_cov_omega }
pub fn FIXED_COV_V() -> [f64; 3] { crate::runtime_config::get().fixed_cov_v }

// SMC Optimization Constants
pub fn ITERATIONS() -> usize { crate::runtime_config::get().iterations }
pub fn DECAY_RATE() -> f64 { crate::runtime_config::get().decay_rate }
pub fn ELITE_RATIO() -> f64 { crate::runtime_config::get().elite_ratio }
pub fn GRASP_TYPE_MIN_PROBABILITY() -> f64 { crate::runtime_config::get().grasp_type_min_probability }

// Starting Proposal Variance
pub fn INITIAL_PROPOSAL_STD_V() -> f64 { crate::runtime_config::get().initial_proposal_std_v }
pub fn INITIAL_PROPOSAL_STD_OMEGA() -> f64 { crate::runtime_config::get().initial_proposal_std_omega }
pub fn INITIAL_PROPOSAL_STD_WRIST() -> f64 { crate::runtime_config::get().initial_proposal_std_wrist }

// Elite injection: fraction of the new population preserved as unchanged
// copies of the best elites. Ensures the best-so-far is never lost.
pub fn ELITE_PRESERVE_RATIO() -> f64 { crate::runtime_config::get().elite_preserve_ratio }

// Grasp type mutation: probability that a resampled particle changes its
// grasp type from the parent elite's type.
pub fn GRASP_TYPE_MUTATION_RATE() -> f64 { crate::runtime_config::get().grasp_type_mutation_rate }

// Grasp scoring weights
pub fn GRASP_WEIGHT_PROBABILITY() -> f64 { crate::runtime_config::get().grasp_weight_probability }
pub fn GRASP_WEIGHT_ALIGNMENT() -> f64 { crate::runtime_config::get().grasp_weight_alignment }
pub fn GRASP_WEIGHT_FORCE_CLOSURE() -> f64 { crate::runtime_config::get().grasp_weight_force_closure }
pub fn GRASP_WEIGHT_CONTACT_SCORE() -> f64 { crate::runtime_config::get().grasp_weight_contact_score }

// Superquadric backside estimation
pub fn SQ_ENABLE_BACKSIDE() -> bool { crate::runtime_config::get().sq_enable_backside }
pub fn SQ_MAX_GN_ITERATIONS() -> usize { crate::runtime_config::get().sq_max_gn_iterations }
pub fn SQ_GN_DAMPING() -> f32 { crate::runtime_config::get().sq_gn_damping }
pub fn SQ_BLEND_DELTA_CELLS() -> usize { crate::runtime_config::get().sq_blend_delta_cells }
pub fn SQ_MIN_FIT_POINTS() -> usize { crate::runtime_config::get().sq_min_fit_points }
pub fn SQ_FIT_ERROR_THRESHOLD() -> f32 { crate::runtime_config::get().sq_fit_error_threshold }
pub fn SQ_MIN_SIGN_OVERRIDE_CELLS() -> usize { crate::runtime_config::get().sq_min_sign_override_cells }

// Debug visualization
// When true, each pipeline invocation writes a single .npz file containing the
// scored grasp candidates. When false the entire export path is skipped.
pub fn DEBUG_VISUALIZATION() -> bool { crate::runtime_config::get().debug_visualization }
// Output directory (relative to the crate manifest directory).
pub fn DEBUG_OUTPUT_DIR() -> &'static str {
    static LEAKED: OnceLock<String> = OnceLock::new();
    LEAKED.get_or_init(|| crate::runtime_config::get().debug_output_dir.clone()).as_str()
}
