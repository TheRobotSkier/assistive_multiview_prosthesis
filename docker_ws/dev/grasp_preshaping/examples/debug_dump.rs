//! Standalone debug-dump generator.
//!
//! Runs the full grasp preshaping pipeline with synthetic data and writes a
//! single `.npz` file that can be visualised with `scripts/visualize_grasp_debug.py`.
//!
//! The synthetic point cloud is a sphere with the camera-occluded backside
//! removed (backface culling), and the planner runs the full 5-iteration SMC
//! optimization loop so the visualizer's iteration filter (`+`/`-`) is
//! populated.
//!
//! ```text
//! cargo run --example debug_dump
//! ```

use grasp_preshaping::config;
use grasp_preshaping::debug_export::{
    debug_output_path, export_npz, DebugDump, ScoredGraspExport,
};
use grasp_preshaping::lut_helper::{Contact, DualQuaternion, FingerLUT};
use grasp_preshaping::planner::{
    score_cylindrical, score_lateral, score_pinch, GraspScoreResult, GraspWeights,
};
use grasp_preshaping::pointcloud_helper::{get_tsdf, morton, prune, Camera, PointCloud, Tsdf};
use grasp_preshaping::predictor::{
    compute_grasp_type_weights, predict_roi_with_samples, resample_around_elites,
    sample_initial_particles, select_elite_indices, PredictionConfig, SmcParticle, Twist6,
    TwistCovariance,
};
use grasp_preshaping::superquadric::{fit_best_superquadric, SuperquadricParams};
use nalgebra::{Matrix4, Vector3};
use rayon::prelude::*;

fn demo_sphere(center: Vector3<f32>, radius: f32, n: usize) -> PointCloud {
    use std::f32::consts::PI;
    let golden = (1.0 + 5.0_f32.sqrt()) / 2.0;
    let points: Vec<Vector3<f32>> = (0..n)
        .map(|i| {
            let theta = 2.0 * PI * (i as f32) / golden;
            let phi = (1.0 - 2.0 * (i as f32 + 0.5) / n as f32).acos();
            center
                + Vector3::new(
                    radius * phi.sin() * theta.cos(),
                    radius * phi.sin() * theta.sin(),
                    radius * phi.cos(),
                )
        })
        .collect();
    PointCloud::new(points)
}

/// Filter a point cloud to retain only points whose surface normal faces the
/// camera (backface culling).
///
/// For a convex object centred at `center`, the outward normal at point `p` is
/// `(p - center).normalized()`.  A point is camera-visible when this normal has
/// a positive dot product with the direction from the point to the camera.
/// This simulates the partial coverage of a single-view depth sensor.
fn filter_camera_visible(
    points: &[Vector3<f32>],
    center: Vector3<f32>,
    camera_pos: Vector3<f32>,
) -> Vec<Vector3<f32>> {
    points
        .iter()
        .copied()
        .filter(|p| {
            let normal = (p - center).try_normalize(1e-9);
            let to_cam = (camera_pos - p).try_normalize(1e-9);
            match (normal, to_cam) {
                (Some(n), Some(d)) => n.dot(&d) > 0.0,
                _ => false,
            }
        })
        .collect()
}

fn lut_path() -> String {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let candidates = vec![
        format!("{}/data/finger_contact_lut.npz", manifest_dir),
        format!("{}/../../../data/finger_contact_lut.npz", manifest_dir),
        "./data/finger_contact_lut.npz".to_string(),
    ];
    for path in candidates {
        if std::path::Path::new(&path).exists() {
            return path;
        }
    }
    format!("{}/data/finger_contact_lut.npz", manifest_dir)
}

fn load_lut() -> Option<FingerLUT> {
    let path = lut_path();
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| FingerLUT::load(&path))).ok()
}

fn main() {
    let lut = match load_lut() {
        Some(l) => l,
        None => {
            eprintln!("Skipping: LUT file not found at {}", lut_path());
            return;
        }
    };

    let pred_config = PredictionConfig {
        t_max: config::PREDICTION_HORIZON_S,
        n_samples: config::PREDICTION_SAMPLES,
        hand_radius: config::HAND_RADIUS_M,
        min_tsdf_dims: Vector3::new(
            config::MIN_TSDF_DIM_M as f64,
            config::MIN_TSDF_DIM_M as f64,
            config::MIN_TSDF_DIM_M as f64,
        ),
        max_tsdf_dims: Vector3::new(
            config::MAX_TSDF_DIM_M as f64,
            config::MAX_TSDF_DIM_M as f64,
            config::MAX_TSDF_DIM_M as f64,
        ),
    };

    let pose = DualQuaternion::from_se3(&Matrix4::identity());
    let twist = Twist6 {
        omega: Vector3::new(0.0, 0.0, 0.1),
        v: Vector3::new(0.01, 0.0, 0.0),
    };
    let twist_cov = TwistCovariance::fixed();
    let index_tip = lut.get_location(Contact::IndexTip, 0.0);

    // Predict ROI (samples are not needed — the SMC loop generates its own).
    let (roi, _samples) = predict_roi_with_samples(&pose, &twist, &twist_cov, &index_tip, &pred_config);

    // Create synthetic point cloud (sphere in front of the hand).
    let sphere_center = Vector3::new(0.0, 0.1, 0.05);
    let camera_pos = Vector3::new(0.0, 0.1, -0.3);
    let pc = demo_sphere(sphere_center, 0.02, 5000);

    // Remove the camera-occluded backside to simulate a single-view depth scan.
    let visible_points = filter_camera_visible(&pc.points, sphere_center, camera_pos);
    eprintln!(
        "Camera visibility filter: {}/{} points retained ({:.1}%)",
        visible_points.len(),
        pc.points.len(),
        100.0 * visible_points.len() as f64 / pc.points.len() as f64,
    );
    let pc = PointCloud::new(visible_points);

    // Prune to ROI.
    let mut pruned = prune(&pc, Some(roi));
    if pruned.is_empty() {
        eprintln!("No points in ROI after pruning — using full cloud");
        pruned = prune(&pc, None);
        if pruned.is_empty() {
            eprintln!("Point cloud is empty, aborting");
            return;
        }
    }

    // Build TSDF.
    let cameras: Vec<Camera> = vec![Camera { position: camera_pos }];
    let (morton_arr, offsets, start) = morton(&pruned, config::TSDF_RESOLUTION_M);
    let tsdf = get_tsdf(
        &morton_arr,
        &offsets,
        config::TRUNCATION_CELLS,
        start,
        config::TSDF_RESOLUTION_M,
        &cameras,
        None,
    );

    // --- Superquadric backside estimation ---
    // Fit a parametric shape to the (partial) point cloud.  This estimates the
    // object's full geometry, including the camera-occluded backside, and is
    // rendered by the visualizer's `b` key as a semi-transparent primitive.
    let sq_params: Option<SuperquadricParams> = fit_best_superquadric(&pruned.points);
    match &sq_params {
        Some(sq) => eprintln!(
            "Superquadric fit: template={} (e1={:.2}, e2={:.2}) scale=({:.4}, {:.4}, {:.4}) fit_error={:.4}",
            sq.template_index, sq.epsilon1, sq.epsilon2, sq.a, sq.b, sq.c, sq.fit_error,
        ),
        None => eprintln!(
            "Superquadric fit: skipped/failed (< {} points)",
            config::SQ_MIN_FIT_POINTS
        ),
    }

    // --- SMC Optimization Loop ---
    let collision_tol = config::COLLISION_TOL_M;
    let weights = GraspWeights::default();

    let n_samples = config::PREDICTION_SAMPLES;
    let n_iterations = config::ITERATIONS;

    let mut rng = rand::rng();
    let mut particles = sample_initial_particles(
        &pose,
        &twist,
        &twist_cov,
        n_samples,
        pred_config.t_max,
        &mut rng,
    );

    let mut grasp_exports: Vec<ScoredGraspExport> = Vec::new();

    for iteration in 0..n_iterations {
        // Score all particles in parallel.
        let scored: Vec<(GraspScoreResult, f64, usize)> = particles
            .par_iter()
            .map(|p| {
                let base_transform = p.pose.to_se3();
                let scorer: fn(&FingerLUT, &Tsdf, &Matrix4<f64>, f32) -> GraspScoreResult =
                    match p.grasp_type {
                        0 => score_cylindrical,
                        1 => score_pinch,
                        _ => score_lateral,
                    };
                let result = scorer(&lut, &tsdf, &base_transform, collision_tol);
                let combined = result.combined_score(&weights, p.sample_probability);
                (result, combined, p.grasp_type)
            })
            .collect();

        // Write scores back into particles and build export entries.
        for (i, (result, combined, grasp_type)) in scored.iter().enumerate() {
            particles[i].score = *combined;

            let base_transform = particles[i].pose.to_se3();
            let mut pose_se3 = [0.0f64; 16];
            for row in 0..4 {
                for col in 0..4 {
                    pose_se3[row * 4 + col] = base_transform[(row, col)];
                }
            }

            grasp_exports.push(ScoredGraspExport {
                sample_index: i,
                grasp_type_i32: *grasp_type as i32 + 1,
                closure_amount: result.closure_amount,
                alignment_score: result.alignment_score,
                force_closure_score: result.force_closure_score,
                contact_count_score: result.contact_count_score,
                contact_score: result.contact_score,
                active_contact_count: result.active_contact_count,
                found_collision: result.found_collision,
                combined_score: *combined,
                sample_probability: particles[i].sample_probability,
                pose_se3,
                wrist_rotation: particles[i].wrist_rotation,
                smc_iteration: iteration,
            });
        }

        // Last iteration: no resampling needed.
        if iteration == n_iterations - 1 {
            break;
        }

        // Select elites and resample with decaying proposal variance.
        let elite_indices = select_elite_indices(&particles, config::ELITE_RATIO);
        let elites: Vec<SmcParticle> = elite_indices
            .iter()
            .map(|&idx| particles[idx].clone())
            .collect();

        let grasp_type_weights = compute_grasp_type_weights(&particles);

        let decay = config::DECAY_RATE.powi(iteration as i32);
        let proposal_std_v = config::INITIAL_PROPOSAL_STD_V * decay;
        let proposal_std_omega = config::INITIAL_PROPOSAL_STD_OMEGA * decay;
        let proposal_std_wrist = config::INITIAL_PROPOSAL_STD_WRIST * decay;

        particles = resample_around_elites(
            &elites,
            n_samples,
            proposal_std_v,
            proposal_std_omega,
            proposal_std_wrist,
            &grasp_type_weights,
            &mut rng,
        );
    }

    // Build debug dump
    let dump = DebugDump {
        tsdf: &tsdf,
        point_cloud: &pruned,
        roi: &roi,
        cameras: &cameras,
        scored_grasps: &grasp_exports,
        input_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        input_twist: [0.01, 0.0, 0.0, 0.0, 0.0, 0.1],
        sq_params: sq_params.as_ref(),
    };

    // Write
    let path = debug_output_path();
    match export_npz(&dump, &path) {
        Ok(()) => {
            eprintln!("Debug dump written to {}", path.display());
            eprintln!(
                "  TSDF grid:  {}x{}x{} = {} voxels",
                tsdf.dimensions().0,
                tsdf.dimensions().1,
                tsdf.dimensions().2,
                tsdf.data().len(),
            );
            eprintln!(
                "  Points:     {} (pruned, camera-visible)",
                pruned.points.len()
            );
            eprintln!(
                "  Grasps:     {} candidates across {} SMC iterations",
                grasp_exports.len(),
                n_iterations
            );
            eprintln!("  Cameras:    {}", cameras.len());
        }
        Err(e) => eprintln!("FAILED: {}", e),
    }
}
