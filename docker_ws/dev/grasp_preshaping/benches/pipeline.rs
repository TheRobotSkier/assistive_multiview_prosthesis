use criterion::{black_box, criterion_group, criterion_main, Criterion};
use grasp_preshaping::lut_helper::{Contact, DualQuaternion, FingerLUT};
use grasp_preshaping::planner::{score_cylindrical, score_lateral, score_pinch};
use grasp_preshaping::pointcloud_helper::{get_tsdf, morton, prune, PointCloud};
use grasp_preshaping::predictor::{
    predict_roi_with_samples, PredictionConfig, Twist6, TwistCovariance,
};
use nalgebra::{Matrix4, Vector3};

const TSDF_RESOLUTION_MM: f32 = 5.0;
const TRUNCATION_CELLS: usize = 4;
const COLLISION_TOL_MM: f32 = 5.0;
const HORIZON: f64 = 5.0;
const SAMPLES: usize = 1000;

const PREDICTION_HAND_RADIUS_M: f64 = 0.05;
const MIN_TSDF_DIM_M: f32 = 0.1;
const MAX_TSDF_DIM_M: f32 = 0.3;

fn lut_path() -> String {
    // Try to find the LUT file in common locations
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

    // Return the most likely location; test will skip if not found
    format!("{}/data/finger_contact_lut.npz", manifest_dir)
}

fn load_lut_or_skip() -> Option<FingerLUT> {
    let path = lut_path();
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| FingerLUT::load(&path))).ok()
}

fn identity_pose() -> DualQuaternion {
    DualQuaternion::from_se3(&Matrix4::identity())
}

fn create_pred_config() -> PredictionConfig {
    PredictionConfig {
        t_max: HORIZON,
        n_samples: SAMPLES,
        hand_radius: PREDICTION_HAND_RADIUS_M,
        min_tsdf_dims: Vector3::new(
            MIN_TSDF_DIM_M as f64,
            MIN_TSDF_DIM_M as f64,
            MIN_TSDF_DIM_M as f64,
        ),
        max_tsdf_dims: Vector3::new(
            MAX_TSDF_DIM_M as f64,
            MAX_TSDF_DIM_M as f64,
            MAX_TSDF_DIM_M as f64,
        ),
    }
}

/// Benchmark ROI prediction from pose and twist
fn bench_roi_prediction(c: &mut Criterion) {
    let lut = match load_lut_or_skip() {
        Some(l) => l,
        None => {
            println!("Skipping ROI prediction benchmark: LUT file not found");
            return;
        }
    };

    let pred_config = black_box(create_pred_config());
    let pose = black_box(identity_pose());
    let twist = black_box(Twist6::dummy());
    let twist_cov = black_box(TwistCovariance::dummy());
    let index_tip = black_box(lut.get_location(Contact::IndexTip, 0.0));

    c.bench_function("roi_prediction_1000_samples", |b| {
        b.iter(|| predict_roi_with_samples(&pose, &twist, &twist_cov, &index_tip, &pred_config))
    });
}

/// Benchmark point cloud pruning to ROI
fn bench_pointcloud_pruning(c: &mut Criterion) {
    let pc = PointCloud::demo_sphere(Vector3::new(0.0, 0.1, 0.05), 0.02, 5000);

    let lut = match load_lut_or_skip() {
        Some(l) => l,
        None => return,
    };

    let pred_config = create_pred_config();
    let pose = identity_pose();
    let twist = Twist6::dummy();
    let twist_cov = TwistCovariance::dummy();
    let index_tip = lut.get_location(Contact::IndexTip, 0.0);

    let (roi, _) = predict_roi_with_samples(&pose, &twist, &twist_cov, &index_tip, &pred_config);

    c.bench_function("pointcloud_prune_to_roi_5000pts", |b| {
        b.iter(|| prune(&pc, Some(roi)))
    });
}

/// Benchmark TSDF construction from point cloud
fn bench_tsdf_construction(c: &mut Criterion) {
    let center = Vector3::new(0.0, 0.1, 0.05);
    let pc = black_box(PointCloud::demo_sphere(center, 0.02, 5000));

    c.bench_function("tsdf_construction_5000pts", |b| {
        b.iter(|| {
            let (morton_arr, offsets, start) = morton(&pc, TSDF_RESOLUTION_MM);
            get_tsdf(
                &morton_arr,
                &offsets,
                TRUNCATION_CELLS,
                start,
                TSDF_RESOLUTION_MM,
                &[],
            )
        })
    });
}

/// Benchmark full scoring pipeline: ROI -> TSDF -> score all grasps
fn bench_full_pipeline(c: &mut Criterion) {
    let lut = match load_lut_or_skip() {
        Some(l) => l,
        None => {
            println!("Skipping full pipeline benchmark: LUT file not found");
            return;
        }
    };

    let pred_config = create_pred_config();
    let pose = black_box(identity_pose());
    let twist = black_box(Twist6::dummy());
    let twist_cov = black_box(TwistCovariance::dummy());
    let index_tip = black_box(lut.get_location(Contact::IndexTip, 0.0));

    let pc = black_box(PointCloud::demo_sphere(Vector3::new(0.0, 0.1, 0.05), 0.02, 5000));

    c.bench_function("full_pipeline_predict_to_score", |b| {
        b.iter(|| {
            // Predict ROI and sample grasps
            let (roi, samples) =
                predict_roi_with_samples(&pose, &twist, &twist_cov, &index_tip, &pred_config);

            // Prune point cloud
            let pruned = prune(&pc, Some(roi));
            if pruned.is_empty() {
                return ();
            }

            // Build TSDF
            let (morton_arr, offsets, start) = morton(&pruned, TSDF_RESOLUTION_MM);
            let tsdf = get_tsdf(
                &morton_arr,
                &offsets,
                TRUNCATION_CELLS,
                start,
                TSDF_RESOLUTION_MM,
                &[],
            );

            // Score all grasps
            let collision_tol = COLLISION_TOL_MM / 1000.0;
            for sp in &samples {
                let base_transform = sp.pose.to_se3();
                let _r1 = score_cylindrical(&lut, &tsdf, &base_transform, collision_tol);
                let _r2 = score_pinch(&lut, &tsdf, &base_transform, collision_tol);
                let _r3 = score_lateral(&lut, &tsdf, &base_transform, collision_tol);
            }
        })
    });
}

/// Benchmark individual scoring functions
fn bench_scoring_functions(c: &mut Criterion) {
    let lut = match load_lut_or_skip() {
        Some(l) => l,
        None => return,
    };

    // Create a small TSDF for scoring
    let pc = PointCloud::demo_sphere(Vector3::new(0.0, 0.1, 0.05), 0.02, 1000);
    let (morton_arr, offsets, start) = morton(&pc, TSDF_RESOLUTION_MM);
    let tsdf = get_tsdf(
        &morton_arr,
        &offsets,
        TRUNCATION_CELLS,
        start,
        TSDF_RESOLUTION_MM,
        &[],
    );

    let base_transform = black_box(Matrix4::identity());
    let collision_tol = black_box(COLLISION_TOL_MM / 1000.0);

    c.bench_function("score_cylindrical", |b| {
        b.iter(|| score_cylindrical(&lut, &tsdf, &base_transform, collision_tol))
    });

    c.bench_function("score_pinch", |b| {
        b.iter(|| score_pinch(&lut, &tsdf, &base_transform, collision_tol))
    });

    c.bench_function("score_lateral", |b| {
        b.iter(|| score_lateral(&lut, &tsdf, &base_transform, collision_tol))
    });
}

criterion_group!(
    benches,
    bench_roi_prediction,
    bench_pointcloud_pruning,
    bench_tsdf_construction,
    bench_full_pipeline,
    bench_scoring_functions
);
criterion_main!(benches);
