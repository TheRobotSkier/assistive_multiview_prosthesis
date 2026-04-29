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
pub const PREDICTION_SAMPLES: usize = 10000;
pub const HAND_RADIUS_M: f64 = 0.05;
pub const MIN_TSDF_DIM_M: f32 = 0.1;
pub const MAX_TSDF_DIM_M: f32 = 0.3;

// Wrist rotation allowed range (radians). The wrist rotates around the local
// Y axis (supination/pronation). Value is the half-range; actual rotation is
// sampled uniformly from [-WRIST_ROTATION_RANGE_RAD, WRIST_ROTATION_RANGE_RAD].
pub const WRIST_ROTATION_RANGE_RAD: f64 = std::f64::consts::FRAC_PI_2;

// Fixed twist covariance
pub const FIXED_COV_OMEGA: [f64; 3] = [0.001, 0.001, 0.001];
pub const FIXED_COV_V: [f64; 3] = [0.0005, 0.0005, 0.0005];

// Grasp scoring weights
pub const GRASP_WEIGHT_PROBABILITY: f64 = 0.5;
pub const GRASP_WEIGHT_ALIGNMENT: f64 = 1.0;
pub const GRASP_WEIGHT_FORCE_CLOSURE: f64 = 1.0;
pub const GRASP_WEIGHT_CONTACT_SCORE: f64 = 3.0;

// Preshaping closure: fraction of the planner's full closure that is sent
// immediately to the finger controllers as a "pre-grasp" signal.  The
// remaining closure is published on a planner topic for a downstream
// trajectory node to apply progressively as the arm approaches the target.
// Range: [0.0, 1.0].  A value of 0.3 means 30 % of the computed closure is
// applied immediately, leaving 70 % for the trajectory node.
pub const PRESHAPING_CLOSURE_FRACTION: f64 = 0.3;

// Debug visualization
// When true, each pipeline invocation writes a single .npz file containing the
// scored grasp candidates.  When false the entire export path is eliminated by
// the compiler (zero runtime cost).
pub const DEBUG_VISUALIZATION: bool = true;
// Output directory (relative to the crate manifest directory).
pub const DEBUG_OUTPUT_DIR: &str = "data/debug"; 
