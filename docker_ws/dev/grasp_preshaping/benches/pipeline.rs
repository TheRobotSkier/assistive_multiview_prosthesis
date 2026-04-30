use criterion::{black_box, criterion_group, criterion_main, Criterion};
use grasp_preshaping::lut_helper::{Contact, DualQuaternion, FingerLUT};
use grasp_preshaping::planner::{score_cylindrical, score_lateral, score_pinch};
use grasp_preshaping::pointcloud_helper::{get_tsdf, morton, prune, PointCloud};
use grasp_preshaping::predictor::{
    predict_roi_with_samples, sample_initial_particles, resample_around_elites,
    select_elite_indices, SmcParticle, PredictionConfig, Twist6, TwistCovariance,
};
use grasp_preshaping::config;
use nalgebra::{Matrix4, Vector3};

fn create_pred_config() -> PredictionConfig {
    PredictionConfig {
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
    }
}

fn identity_pose() -> DualQuaternion {
    DualQuaternion::from_se3(&Matrix4::identity())
}

fn dummy_twist() -> Twist6 {
    Twist6 {
        omega: Vector3::new(0.0, 0.0, 0.1),
        v: Vector3::new(0.01, 0.0, 0.0),
    }
}

fn demo_sphere(center: Vector3<f32>, radius: f32, n: usize) -> PointCloud {
    use std::f32::consts::PI;
    let golden = (1.0 + 5.0_f32.sqrt()) / 2.0;
    let points: Vec<Vector3<f32>> = (0..n)
        .map(|i| {
            let theta = 2.0 * PI * (i as f32) / golden;
            let phi = (1.0 - 1.0 - 2.0 * (i as f32 + 0.5) / n as f32).acos();
            center + Vector3::new(
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

fn load_lut_or_skip() -> Option<FingerLUT> {
    let path = lut_path();
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| FingerLUT::load(&path))).ok()
}

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
    let twist = black_box(dummy_twist());
    let twist_cov = black_box(TwistCovariance::fixed());
    let index_tip = black_box(lut.get_location(Contact::IndexTip, 0.0));

    c.bench_function("roi_prediction_1000_samples", |b| {
        b.iter(|| predict_roi_with_samples(&pose, &twist, &twist_cov, &index_tip, &pred_config))
    });
}

fn bench_pointcloud_pruning(c: &mut Criterion) {
    let pc = demo_sphere(Vector3::new(0.0, 0.1, 0.05), 0.02, 5000);

    // Use a fixed deterministic ROI for the prune benchmark.
    // The prune benchmark measures the prune function itself, not the ROI size.
    // Previously predict_roi_with_samples was used, but it now includes random wrist rotation
    // which inflates the AABB non-deterministically.
    let roi = grasp_preshaping::pointcloud_helper::Aabb {
        min: Vector3::new(-0.05, 0.05, -0.02),
        max: Vector3::new(0.05, 0.15, 0.08),
    };

    c.bench_function("pointcloud_prune_to_roi_5000pts", |b| {
        b.iter(|| prune(&pc, Some(roi)))
    });
}

fn bench_tsdf_construction(c: &mut Criterion) {
    let center = Vector3::new(0.0, 0.1, 0.05);
    let pc = black_box(demo_sphere(center, 0.02, 5000));

    c.bench_function("tsdf_construction_5000pts", |b| {
        b.iter(|| {
            let (morton_arr, offsets, start) = morton(&pc, config::TSDF_RESOLUTION_M);
            get_tsdf(
                &morton_arr,
                &offsets,
                config::TRUNCATION_CELLS,
                start,
                config::TSDF_RESOLUTION_M,
                &[],
            )
        })
    });
}

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
    let twist = black_box(dummy_twist());
    let twist_cov = black_box(TwistCovariance::fixed());
    let index_tip = black_box(lut.get_location(Contact::IndexTip, 0.0));

    let pc = black_box(demo_sphere(Vector3::new(0.0, 0.1, 0.05), 0.02, 5000));

    c.bench_function("full_pipeline_predict_to_score", |b| {
        b.iter(|| {
            let (roi, samples) =
                predict_roi_with_samples(&pose, &twist, &twist_cov, &index_tip, &pred_config);

            let pruned = prune(&pc, Some(roi));
            if pruned.is_empty() {
                return ();
            }

            let (morton_arr, offsets, start) = morton(&pruned, config::TSDF_RESOLUTION_M);
            let tsdf = get_tsdf(
                &morton_arr,
                &offsets,
                config::TRUNCATION_CELLS,
                start,
                config::TSDF_RESOLUTION_M,
                &[],
            );

            let collision_tol = config::COLLISION_TOL_M;
            for sp in &samples {
                let base_transform = sp.pose.to_se3();
                let scorer: fn(&FingerLUT, &grasp_preshaping::pointcloud_helper::Tsdf, &Matrix4<f64>, f32) -> grasp_preshaping::planner::GraspScoreResult = match sp.grasp_type {
                    0 => score_cylindrical,
                    1 => score_pinch,
                    _ => score_lateral,
                };
                let _ = scorer(&lut, &tsdf, &base_transform, collision_tol);
            }
        })
    });
}

fn bench_scoring_functions(c: &mut Criterion) {
    let lut = match load_lut_or_skip() {
        Some(l) => l,
        None => return,
    };

    let pc = demo_sphere(Vector3::new(0.0, 0.1, 0.05), 0.02, 1000);
    let (morton_arr, offsets, start) = morton(&pc, config::TSDF_RESOLUTION_M);
    let tsdf = get_tsdf(
        &morton_arr,
        &offsets,
        config::TRUNCATION_CELLS,
        start,
        config::TSDF_RESOLUTION_M,
        &[],
    );

    let base_transform = black_box(Matrix4::identity());
    let collision_tol = black_box(config::COLLISION_TOL_M);

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

fn bench_smc_pipeline(c: &mut Criterion) {
    let lut = match load_lut_or_skip() {
        Some(l) => l,
        None => {
            println!("Skipping SMC pipeline benchmark: LUT file not found");
            return;
        }
    };

    let pred_config = create_pred_config();
    let pose = black_box(identity_pose());
    let twist = black_box(dummy_twist());
    let twist_cov = black_box(TwistCovariance::fixed());
    let index_tip = black_box(lut.get_location(Contact::IndexTip, 0.0));
    let pc = black_box(demo_sphere(Vector3::new(0.0, 0.1, 0.05), 0.02, 5000));

    c.bench_function("smc_pipeline_8iter_1000samples", |b| {
        b.iter(|| {
            // Build ROI and TSDF once.
            let (roi, _) =
                predict_roi_with_samples(&pose, &twist, &twist_cov, &index_tip, &pred_config);
            let pruned = prune(&pc, Some(roi));
            if pruned.is_empty() {
                return ();
            }
            let (morton_arr, offsets, start) = morton(&pruned, config::TSDF_RESOLUTION_M);
            let tsdf = get_tsdf(
                &morton_arr,
                &offsets,
                config::TRUNCATION_CELLS,
                start,
                config::TSDF_RESOLUTION_M,
                &[],
            );

            let collision_tol = config::COLLISION_TOL_M;
            let n_samples = config::PREDICTION_SAMPLES;

            // SMC loop.
            let mut rng = rand::rng();
            let mut particles = sample_initial_particles(
                &pose, &twist, &twist_cov, n_samples, pred_config.t_max, &mut rng,
            );

            for iteration in 0..config::ITERATIONS {
                // Score all particles (sequential in benchmark for simplicity).
                for p in particles.iter_mut() {
                    let base_transform = p.pose.to_se3();
                    let scorer: fn(&FingerLUT, &grasp_preshaping::pointcloud_helper::Tsdf, &Matrix4<f64>, f32) -> grasp_preshaping::planner::GraspScoreResult = match p.grasp_type {
                        0 => score_cylindrical,
                        1 => score_pinch,
                        _ => score_lateral,
                    };
                    let result = scorer(&lut, &tsdf, &base_transform, collision_tol);
                    let weights = grasp_preshaping::planner::GraspWeights::default();
                    p.score = result.combined_score(&weights, p.sample_probability);
                }

                if iteration == config::ITERATIONS - 1 {
                    break;
                }

                let elite_indices = select_elite_indices(&particles, config::ELITE_RATIO);
                let elites: Vec<SmcParticle> = elite_indices.iter().map(|&idx| particles[idx].clone()).collect();
                let decay = config::DECAY_RATE.powi(iteration as i32);
                particles = resample_around_elites(
                    &elites,
                    n_samples,
                    config::INITIAL_PROPOSAL_STD_V * decay,
                    config::INITIAL_PROPOSAL_STD_OMEGA * decay,
                    &mut rng,
                );
            }
        })
    });
}

fn bench_resample_around_elites(c: &mut Criterion) {
    let pose = black_box(identity_pose());
    let twist = black_box(dummy_twist());
    let twist_cov = black_box(TwistCovariance::fixed());

    let mut rng = rand::rng();
    let particles = sample_initial_particles(
        &pose, &twist, &twist_cov, 100, 5.0, &mut rng,
    );

    c.bench_function("resample_around_elites_1000_from_100", |b| {
        b.iter(|| {
            let mut rng = rand::rng();
            resample_around_elites(
                &particles,
                1000,
                config::INITIAL_PROPOSAL_STD_V,
                config::INITIAL_PROPOSAL_STD_OMEGA,
                &mut rng,
            )
        })
    });
}

criterion_group!(
    benches,
    bench_roi_prediction,
    bench_pointcloud_pruning,
    bench_tsdf_construction,
    bench_full_pipeline,
    bench_smc_pipeline,
    bench_resample_around_elites,
    bench_scoring_functions
);
criterion_main!(benches);
