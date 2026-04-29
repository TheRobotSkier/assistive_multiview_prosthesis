// Single source of truth for all tunable constants in the grasp preshaping pipeline.
//
// Adjust values here to tune performance and behavior. No other file defines these.

// TSDF construction
pub const TSDF_RESOLUTION_M: f32 = 0.005;
pub const TRUNCATION_CELLS: usize = 4;
pub const RAY_ALIGNMENT_THRESHOLD: f32 = 0.8;

// Collision detection
pub const COLLISION_TOL_M: f32 = 0.005;
pub const BINARY_SEARCH_TOL: f64 = 0.01;

// ROI prediction
pub const PREDICTION_HORIZON_S: f64 = 5.0;
pub const PREDICTION_SAMPLES: usize = 1000000;
pub const HAND_RADIUS_M: f64 = 0.05;
pub const MIN_TSDF_DIM_M: f32 = 0.1;
pub const MAX_TSDF_DIM_M: f32 = 0.3;

// Fixed twist covariance
pub const FIXED_COV_OMEGA: [f64; 3] = [0.001, 0.001, 0.001];
pub const FIXED_COV_V: [f64; 3] = [0.0005, 0.0005, 0.0005];

// Grasp scoring weights
pub const GRASP_WEIGHT_PROBABILITY: f64 = 1.0;
pub const GRASP_WEIGHT_ALIGNMENT: f64 = 1.0;
pub const GRASP_WEIGHT_FORCE_CLOSURE: f64 = 1.0;
pub const GRASP_WEIGHT_CONTACT_COUNT: f64 = 1.5;

// Debug visualization
// When true, each pipeline invocation writes a single .npz file containing the
// scored grasp candidates.  When false the entire export path is eliminated by
// the compiler (zero runtime cost).
pub const DEBUG_VISUALIZATION: bool = true;
// Output directory (relative to the crate manifest directory).
pub const DEBUG_OUTPUT_DIR: &str = "data/debug"; 
