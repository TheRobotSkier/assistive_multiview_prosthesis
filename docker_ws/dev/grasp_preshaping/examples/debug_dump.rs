//! Standalone debug-dump generator.
//!
//! Runs the full grasp preshaping pipeline with synthetic data and writes a
//! single `.npz` file that can be visualised with `scripts/visualize_grasp_debug.py`.
//!
//! ```text
//! cargo run --example debug_dump
//! ```

use grasp_preshaping::config;
use grasp_preshaping::debug_export::{
    export_npz, debug_output_path, DebugDump, ScoredGraspExport,
};
use grasp_preshaping::lut_helper::{Contact, DualQuaternion, FingerLUT};
use grasp_preshaping::planner::{score_cylindrical, score_lateral, score_pinch};
use grasp_preshaping::pointcloud_helper::{get_tsdf, morton, prune, Camera, PointCloud};
use grasp_preshaping::predictor::{
    predict_roi_with_samples, PredictionConfig, Twist6, TwistCovariance,
};
use nalgebra::{Matrix4, Vector3};

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

    // Predict ROI
    let (roi, samples) = predict_roi_with_samples(&pose, &twist, &twist_cov, &index_tip, &pred_config);

    // Create synthetic point cloud (sphere in front of the hand)
    let pc = demo_sphere(Vector3::new(0.0, 0.1, 0.05), 0.02, 5000);

    // Prune to ROI
    let pruned = prune(&pc, Some(roi));
    if pruned.is_empty() {
        eprintln!("No points in ROI after pruning — using full cloud");
        let pruned = prune(&pc, None);
        if pruned.is_empty() {
            eprintln!("Point cloud is empty, aborting");
            return;
        }
    }

    // Build TSDF
    let cameras: Vec<Camera> = vec![Camera {
        position: Vector3::new(0.0, 0.1, -0.3),
    }];
    let (morton_arr, offsets, start) = morton(&pruned, config::TSDF_RESOLUTION_M);
    let tsdf = get_tsdf(
        &morton_arr,
        &offsets,
        config::TRUNCATION_CELLS,
        start,
        config::TSDF_RESOLUTION_M,
        &cameras,
    );

    // Score all samples (unified: each sample has its own grasp type)
    let collision_tol = config::COLLISION_TOL_M;
    let weights = grasp_preshaping::planner::GraspWeights::default();

    let grasp_scorers: [(i32, fn(&FingerLUT, &grasp_preshaping::pointcloud_helper::Tsdf, &Matrix4<f64>, f32) -> Option<grasp_preshaping::planner::GraspScoreResult>); 3] = [
        (1i32, score_cylindrical),
        (2i32, score_pinch),
        (3i32, score_lateral),
    ];

    let mut grasp_exports: Vec<ScoredGraspExport> = Vec::new();

    for (si, sp) in samples.iter().enumerate() {
        let base_transform = sp.pose.to_se3();
        let mut pose_se3 = [0.0f64; 16];
        for row in 0..4 {
            for col in 0..4 {
                pose_se3[row * 4 + col] = base_transform[(row, col)];
            }
        }

        let (gt_int, scorer) = grasp_scorers[sp.grasp_type];

        match scorer(&lut, &tsdf, &base_transform, collision_tol) {
            Some(result) => {
                let combined = if result.found_collision {
                    result.combined_score(&weights, sp.sample_probability)
                } else {
                    f64::NEG_INFINITY
                };
                grasp_exports.push(ScoredGraspExport {
                    sample_index: si,
                    grasp_type_i32: gt_int,
                    closure_amount: result.closure_amount,
                    alignment_score: result.alignment_score,
                    force_closure_score: result.force_closure_score,
                    contact_count_score: result.contact_count_score,
                    found_collision: result.found_collision,
                    combined_score: combined,
                    sample_probability: sp.sample_probability,
                    pose_se3,
                    wrist_rotation: sp.wrist_rotation,
                });
            }
            None => {
                // Start-position collision — invalid pose.
                grasp_exports.push(ScoredGraspExport {
                    sample_index: si,
                    grasp_type_i32: gt_int,
                    closure_amount: 0.0,
                    alignment_score: 0.0,
                    force_closure_score: 0.0,
                    contact_count_score: 0.0,
                    found_collision: false,
                    combined_score: f64::NEG_INFINITY,
                    sample_probability: sp.sample_probability,
                    pose_se3,
                    wrist_rotation: sp.wrist_rotation,
                });
            }
        }
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
            eprintln!("  Points:     {} (pruned)", pruned.points.len());
            eprintln!("  Grasps:     {} candidates", grasp_exports.len());
            eprintln!("  Cameras:    {}", cameras.len());
        }
        Err(e) => eprintln!("FAILED: {}", e),
    }
}
