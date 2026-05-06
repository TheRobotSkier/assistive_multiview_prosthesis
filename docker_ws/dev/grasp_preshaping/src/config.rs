// Single source of truth for all tunable constants in the grasp preshaping pipeline.
//
// Adjust values here to tune performance and behavior. No other file defines these.

// TSDF construction
pub const TSDF_RESOLUTION_M: f32 = 0.005;
pub const TRUNCATION_CELLS: usize = 4; 
pub const RAY_ALIGNMENT_THRESHOLD: f32 = 0.9;

// Collision detection
pub const COLLISION_TOL_M: f32 = 0.01;
pub const BINARY_SEARCH_TOL: f64 = 0.05;

// ROI prediction
pub const PREDICTION_HORIZON_S: f64 = 5.0;
pub const PREDICTION_SAMPLES: usize = 25000;
pub const HAND_RADIUS_M: f64 = 0.05;
pub const MIN_TSDF_DIM_M: f32 = 0.1;
pub const MAX_TSDF_DIM_M: f32 = 0.5;

// Wrist rotation allowed range (radians). The wrist rotates around the local
// Y axis (supination/pronation). Value is the half-range; actual rotation is
// sampled uniformly from [-WRIST_ROTATION_RANGE_RAD, WRIST_ROTATION_RANGE_RAD].
pub const WRIST_ROTATION_RANGE_RAD: f64 = std::f64::consts::FRAC_PI_2;

// Fixed twist covariance
pub const FIXED_COV_OMEGA: [f64; 3] = [0.001, 0.001, 0.001];
pub const FIXED_COV_V: [f64; 3] = [0.0005, 0.0005, 0.0005];

// SMC Optimization Constants
pub const ITERATIONS: usize = 5; // Number of SMC iterations
pub const DECAY_RATE: f64 = 0.7; // Geometric decay factor per iteration
pub const ELITE_RATIO: f64 = 0.05; // Top fraction selected as elites
pub const SMC_CONVERGENCE_TOL: f64 = 0.01; // Combined score change threshold for early termination
pub const SMC_MIN_ITERATIONS: usize = 3; // Minimum iterations before early termination is allowed
pub const GRASP_TYPE_MIN_PROBABILITY: f64 = 0.15; // Minimum probability floor for each grasp type in weighted resampling

// Starting Proposal Variance
pub const INITIAL_PROPOSAL_STD_V: f64 = 0.002; // metres
pub const INITIAL_PROPOSAL_STD_OMEGA: f64 = 0.005; // radians
pub const INITIAL_PROPOSAL_STD_WRIST: f64 = 0.3; // radians (~17 degrees)

// Elite injection: fraction of the new population preserved as unchanged
// copies of the best elites. Ensures the best-so-far is never lost.
pub const ELITE_PRESERVE_RATIO: f64 = 0.01;

// Grasp type mutation: probability that a resampled particle changes its
// grasp type from the parent elite's type. With probability 1 - this value,
// the child inherits the parent's grasp type.
pub const GRASP_TYPE_MUTATION_RATE: f64 = 0.1;

// Grasp scoring weights
pub const GRASP_WEIGHT_PROBABILITY: f64 = 1.0;
pub const GRASP_WEIGHT_ALIGNMENT: f64 = 1.0;
pub const GRASP_WEIGHT_FORCE_CLOSURE: f64 = 1.0;
pub const GRASP_WEIGHT_CONTACT_SCORE: f64 = 3.0;

// Superquadric backside estimation
pub const SQ_ENABLE_BACKSIDE: bool = true;
pub const SQ_MAX_GN_ITERATIONS: usize = 4;
pub const SQ_GN_DAMPING: f32 = 0.1;
pub const SQ_BLEND_DELTA_CELLS: usize = 3;
pub const SQ_MIN_FIT_POINTS: usize = 20;
pub const SQ_FIT_ERROR_THRESHOLD: f32 = 0.15;
pub const SQ_MIN_SIGN_OVERRIDE_CELLS: usize = 2;

// Debug visualization
// When true, each pipeline invocation writes a single .npz file containing the
// scored grasp candidates.  When false the entire export path is eliminated by
// the compiler (zero runtime cost).
pub const DEBUG_VISUALIZATION: bool = false;
// Output directory (relative to the crate manifest directory).
pub const DEBUG_OUTPUT_DIR: &str = "data/debug"; 
