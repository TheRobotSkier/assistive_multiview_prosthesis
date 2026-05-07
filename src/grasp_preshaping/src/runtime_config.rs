// Runtime configuration loaded from a YAML file.
//
// This module provides a `RuntimeConfig` struct that mirrors all tunable constants
// from the compile-time defaults in `config.rs`. At startup, the library attempts
// to load overrides from a YAML file (path set by `GRASP_CONFIG_PATH` env var,
// defaulting to `config/grasp_preshaping.yaml` relative to the workspace).
//
// If the file is missing or malformed, compile-time defaults are used unchanged.
// The YAML file only needs to contain values you want to override.

use std::sync::OnceLock;

use serde::Deserialize;

// ─────────────────────────────────────────────────────────────────────────────
// Compile-time defaults (single source of truth, mirrors old config.rs)
// These are used when no YAML override is provided.
// ─────────────────────────────────────────────────────────────────────────────

mod defaults {
    // TSDF construction
    pub const TSDF_RESOLUTION_M: f32 = 0.005;
    pub const TRUNCATION_CELLS: usize = 4;
    pub const RAY_ALIGNMENT_THRESHOLD: f32 = 0.9;

    // Collision detection
    pub const COLLISION_TOL_M: f32 = 0.01;
    pub const BINARY_SEARCH_TOL: f64 = 0.05;

    // ROI prediction
    pub const PREDICTION_HORIZON_S: f64 = 5.0;
    pub const PREDICTION_SAMPLES: usize = 20000;
    pub const HAND_RADIUS_M: f64 = 0.05;
    pub const MIN_TSDF_DIM_M: f32 = 0.1;
    pub const MAX_TSDF_DIM_M: f32 = 0.5;

    // Wrist rotation
    pub const WRIST_ROTATION_RANGE_RAD: f64 = std::f64::consts::FRAC_PI_2;

    // Fixed twist covariance
    pub const FIXED_COV_OMEGA: [f64; 3] = [0.001, 0.001, 0.001];
    pub const FIXED_COV_V: [f64; 3] = [0.0005, 0.0005, 0.0005];

    // SMC Optimization
    pub const ITERATIONS: usize = 5;
    pub const DECAY_RATE: f64 = 0.7;
    pub const ELITE_RATIO: f64 = 0.05;
    pub const SMC_CONVERGENCE_TOL: f64 = 0.01;
    pub const SMC_MIN_ITERATIONS: usize = 3;
    pub const GRASP_TYPE_MIN_PROBABILITY: f64 = 0.15;

    // Starting proposal variance
    pub const INITIAL_PROPOSAL_STD_V: f64 = 0.002;
    pub const INITIAL_PROPOSAL_STD_OMEGA: f64 = 0.005;
    pub const INITIAL_PROPOSAL_STD_WRIST: f64 = 0.3;

    // Elite injection
    pub const ELITE_PRESERVE_RATIO: f64 = 0.01;

    // Grasp type mutation
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
    pub const DEBUG_VISUALIZATION: bool = false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Config struct — mirrors every tunable value
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct RuntimeConfig {
    // TSDF construction
    pub tsdf_resolution_m: f32,
    pub truncation_cells: usize,
    pub ray_alignment_threshold: f32,

    // Collision detection
    pub collision_tol_m: f32,
    pub binary_search_tol: f64,

    // ROI prediction
    pub prediction_horizon_s: f64,
    pub prediction_samples: usize,
    pub hand_radius_m: f64,
    pub min_tsdf_dim_m: f32,
    pub max_tsdf_dim_m: f32,

    // Wrist rotation
    pub wrist_rotation_range_rad: f64,

    // Fixed twist covariance
    pub fixed_cov_omega: [f64; 3],
    pub fixed_cov_v: [f64; 3],

    // SMC Optimization
    pub iterations: usize,
    pub decay_rate: f64,
    pub elite_ratio: f64,
    pub smc_convergence_tol: f64,
    pub smc_min_iterations: usize,
    pub grasp_type_min_probability: f64,

    // Starting proposal variance
    pub initial_proposal_std_v: f64,
    pub initial_proposal_std_omega: f64,
    pub initial_proposal_std_wrist: f64,

    // Elite injection
    pub elite_preserve_ratio: f64,

    // Grasp type mutation
    pub grasp_type_mutation_rate: f64,

    // Grasp scoring weights
    pub grasp_weight_probability: f64,
    pub grasp_weight_alignment: f64,
    pub grasp_weight_force_closure: f64,
    pub grasp_weight_contact_score: f64,

    // Superquadric backside estimation
    pub sq_enable_backside: bool,
    pub sq_max_gn_iterations: usize,
    pub sq_gn_damping: f32,
    pub sq_blend_delta_cells: usize,
    pub sq_min_fit_points: usize,
    pub sq_fit_error_threshold: f32,
    pub sq_min_sign_override_cells: usize,

    // Debug visualization
    pub debug_visualization: bool,
    pub debug_output_dir: String,
}

impl Default for RuntimeConfig {
    fn default() -> Self {
        Self {
            tsdf_resolution_m: defaults::TSDF_RESOLUTION_M,
            truncation_cells: defaults::TRUNCATION_CELLS,
            ray_alignment_threshold: defaults::RAY_ALIGNMENT_THRESHOLD,
            collision_tol_m: defaults::COLLISION_TOL_M,
            binary_search_tol: defaults::BINARY_SEARCH_TOL,
            prediction_horizon_s: defaults::PREDICTION_HORIZON_S,
            prediction_samples: defaults::PREDICTION_SAMPLES,
            hand_radius_m: defaults::HAND_RADIUS_M,
            min_tsdf_dim_m: defaults::MIN_TSDF_DIM_M,
            max_tsdf_dim_m: defaults::MAX_TSDF_DIM_M,
            wrist_rotation_range_rad: defaults::WRIST_ROTATION_RANGE_RAD,
            fixed_cov_omega: defaults::FIXED_COV_OMEGA,
            fixed_cov_v: defaults::FIXED_COV_V,
            iterations: defaults::ITERATIONS,
            decay_rate: defaults::DECAY_RATE,
            elite_ratio: defaults::ELITE_RATIO,
            smc_convergence_tol: defaults::SMC_CONVERGENCE_TOL,
            smc_min_iterations: defaults::SMC_MIN_ITERATIONS,
            grasp_type_min_probability: defaults::GRASP_TYPE_MIN_PROBABILITY,
            initial_proposal_std_v: defaults::INITIAL_PROPOSAL_STD_V,
            initial_proposal_std_omega: defaults::INITIAL_PROPOSAL_STD_OMEGA,
            initial_proposal_std_wrist: defaults::INITIAL_PROPOSAL_STD_WRIST,
            elite_preserve_ratio: defaults::ELITE_PRESERVE_RATIO,
            grasp_type_mutation_rate: defaults::GRASP_TYPE_MUTATION_RATE,
            grasp_weight_probability: defaults::GRASP_WEIGHT_PROBABILITY,
            grasp_weight_alignment: defaults::GRASP_WEIGHT_ALIGNMENT,
            grasp_weight_force_closure: defaults::GRASP_WEIGHT_FORCE_CLOSURE,
            grasp_weight_contact_score: defaults::GRASP_WEIGHT_CONTACT_SCORE,
            sq_enable_backside: defaults::SQ_ENABLE_BACKSIDE,
            sq_max_gn_iterations: defaults::SQ_MAX_GN_ITERATIONS,
            sq_gn_damping: defaults::SQ_GN_DAMPING,
            sq_blend_delta_cells: defaults::SQ_BLEND_DELTA_CELLS,
            sq_min_fit_points: defaults::SQ_MIN_FIT_POINTS,
            sq_fit_error_threshold: defaults::SQ_FIT_ERROR_THRESHOLD,
            sq_min_sign_override_cells: defaults::SQ_MIN_SIGN_OVERRIDE_CELLS,
            debug_visualization: defaults::DEBUG_VISUALIZATION,
            debug_output_dir: "data/debug".to_string(),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Global singleton
// ─────────────────────────────────────────────────────────────────────────────

static RUNTIME_CONFIG: OnceLock<RuntimeConfig> = OnceLock::new();

/// Load runtime config from YAML file. Called once on first access.
/// If the file doesn't exist or can't be parsed, defaults are used.
fn load_config() -> RuntimeConfig {
    let config_path = std::env::var("GRASP_CONFIG_PATH").ok().or_else(|| {
        // Try CARGO_MANIFEST_DIR for local development
        option_env!("CARGO_MANIFEST_DIR").map(|dir| {
            format!("{}/config/grasp_preshaping.yaml", dir)
        })
    });

    let Some(path) = config_path else {
        eprintln!("[runtime_config] No config file path found, using compile-time defaults");
        return RuntimeConfig::default();
    };

    let path = std::path::PathBuf::from(&path);
    if !path.exists() {
        eprintln!("[runtime_config] Config file not found at {:?}, using defaults", path);
        return RuntimeConfig::default();
    }

    match std::fs::read_to_string(&path) {
        Ok(content) => match serde_yml::from_str::<RuntimeConfig>(&content) {
            Ok(config) => {
                eprintln!("[runtime_config] Loaded config from {:?}", path);
                config
            }
            Err(e) => {
                eprintln!("[runtime_config] Failed to parse {:?}: {}. Using defaults", path, e);
                RuntimeConfig::default()
            }
        },
        Err(e) => {
            eprintln!("[runtime_config] Failed to read {:?}: {}. Using defaults", path, e);
            RuntimeConfig::default()
        }
    }
}

/// Get the global runtime config. Initialized once on first access.
pub fn get() -> &'static RuntimeConfig {
    RUNTIME_CONFIG.get_or_init(load_config)
}
