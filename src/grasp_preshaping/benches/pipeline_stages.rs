//! Per-stage latency benchmark for the grasp preshaping pipeline.
//!
//! Measures the four pipeline stages that appear in the LaTeX timing table:
//!   1. ROI Filtering & Morton Sort
//!   2. Superquadric Backside Estimation
//!   3. TSDF Volumetric Construction
//!   4. Evolutionary Optimization Loop (per iteration)
//!   5. Total Pipeline (end-to-end cross-check)
//!
//! Uses a synthetic ~10k-point cylinder, 20k SMC samples, 5 iterations.
//! Run with: cargo bench --bench pipeline_stages

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use nalgebra::{Matrix4, Vector3};
use rayon::prelude::*;

use grasp_preshaping::config;
use grasp_preshaping::lut_helper::{DualQuaternion, FingerLUT};
use grasp_preshaping::planner::{
    score_cylindrical, score_lateral, score_pinch, GraspScoreResult, GraspWeights,
};
use grasp_preshaping::pointcloud_helper::{
    get_tsdf, morton, prune, Camera, PointCloud,
};
use grasp_preshaping::predictor::{
    compute_grasp_type_weights, resample_around_elites, sample_initial_particles,
    select_elite_indices, SmcParticle, PredictionConfig, Twist6, TwistCovariance,
};
use grasp_preshaping::superquadric::fit_best_superquadric;

// ---------------------------------------------------------------------------
// Synthetic data generation
// ---------------------------------------------------------------------------

/// Generate ~10k points on the surface of a cylinder (radius 3cm, height 10cm),
/// centered at (0, 0, center_z). Simulates a typical grasp target.
fn generate_cylinder_points(n_points: usize, center_z: f32) -> Vec<Vector3<f32>> {
    let radius = 0.03_f32;
    let half_height = 0.05_f32;
    let mut points = Vec::with_capacity(n_points);

    // Side surface: ~70% of points
    let n_side = (n_points as f32 * 0.7) as usize;
    for i in 0..n_side {
        let angle = (i as f32 / n_side as f32) * 2.0 * std::f32::consts::PI;
        let z = (i as f32 / n_side as f32) * 2.0 * half_height - half_height + center_z;
        points.push(Vector3::new(
            radius * angle.cos(),
            radius * angle.sin(),
            z,
        ));
    }

    // Top cap: ~15% of points
    let n_top = (n_points as f32 * 0.15) as usize;
    for i in 0..n_top {
        let angle = (i as f32 / n_top as f32) * 2.0 * std::f32::consts::PI;
        let r = radius * ((i as f32 / n_top as f32) * 0.9 + 0.1);
        points.push(Vector3::new(r * angle.cos(), r * angle.sin(), half_height + center_z));
    }

    // Bottom cap: ~15% of points
    let n_bot = n_points - n_side - n_top;
    for i in 0..n_bot {
        let angle = (i as f32 / n_bot as f32) * 2.0 * std::f32::consts::PI;
        let r = radius * ((i as f32 / n_bot as f32) * 0.9 + 0.1);
        points.push(Vector3::new(r * angle.cos(), r * angle.sin(), -half_height + center_z));
    }

    points
}

// ---------------------------------------------------------------------------
// Shared benchmark state (computed once)
// ---------------------------------------------------------------------------

struct BenchState {
    // Input
    cloud: PointCloud,
    current_pose: DualQuaternion,
    twist: Twist6,
    covariance: TwistCovariance,
    cameras: Vec<Camera>,

    // Pre-loaded resources
    lut: FingerLUT,
    pred_config: PredictionConfig,
    index_tip_local: Vector3<f64>,
    collision_tol: f32,

    // Derived (stage 1 outputs)
    roi_cloud: PointCloud,
    morton_arr: Vec<grasp_preshaping::pointcloud_helper::MortonPoint>,
    morton_offsets: Vec<usize>,
    morton_start: Vector3<f32>,

    // Derived (stage 2 output)
    sq_params: Option<grasp_preshaping::superquadric::SuperquadricParams>,

    // Derived (stage 3 output)
    tsdf: grasp_preshaping::pointcloud_helper::Tsdf,
}

impl BenchState {
    fn new() -> Self {
        // Initialize config (triggers OnceLock init)
        let _ = config::TSDF_RESOLUTION_M();

        // Load LUT
        let lut_path =
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("data/finger_contact_lut.npz");
        let lut = FingerLUT::load(lut_path.to_str().expect("LUT path is valid UTF-8"));

        // Index tip location
        let index_tip_local = lut.get_location(grasp_preshaping::lut_helper::Contact::IndexTip, 0.0);

        // Prediction config
        let pred_config = PredictionConfig {
            t_max: config::PREDICTION_HORIZON_S(),
            n_samples: config::PREDICTION_SAMPLES(),
            hand_radius: config::HAND_RADIUS_M(),
            min_tsdf_dims: Vector3::new(
                config::MIN_TSDF_DIM_M() as f64,
                config::MIN_TSDF_DIM_M() as f64,
                config::MIN_TSDF_DIM_M() as f64,
            ),
            max_tsdf_dims: Vector3::new(
                config::MAX_TSDF_DIM_M() as f64,
                config::MAX_TSDF_DIM_M() as f64,
                config::MAX_TSDF_DIM_M() as f64,
            ),
        };

        // Hand pose: 15cm above the cylinder, pointing down
        let mut pose_m = Matrix4::identity();
        pose_m[(0, 3)] = 0.0;
        pose_m[(1, 3)] = 0.0;
        pose_m[(2, 3)] = 0.15;
        // 180-degree rotation around X so hand points down
        let rot_x_180 = nalgebra::Rotation3::from_axis_angle(
            &nalgebra::Vector3::x_axis(),
            std::f64::consts::PI,
        );
        pose_m.fixed_view_mut::<3, 3>(0, 0).copy_from(rot_x_180.matrix());
        let current_pose = DualQuaternion::from_se3(&pose_m);

        // Twist: small forward velocity
        let twist = Twist6 {
            omega: Vector3::zeros(),
            v: Vector3::new(0.0, 0.0, -0.10),
        };
        let covariance = TwistCovariance::fixed();

        // Cameras: 4 views around the object
        let cameras = vec![
            Camera { position: Vector3::new(0.3, 0.0, 0.3) },
            Camera { position: Vector3::new(-0.3, 0.0, 0.3) },
            Camera { position: Vector3::new(0.0, 0.3, 0.3) },
            Camera { position: Vector3::new(0.0, -0.3, 0.3) },
        ];

        // Generate point cloud: cylinder centered at z=0.12 so it's near the hand
        // at z=0.15. The ROI prediction will naturally include this region.
        let points = generate_cylinder_points(10_000, 0.12);
        let cloud = PointCloud::new(points);

        let collision_tol = config::COLLISION_TOL_M();

        // --- Stage 1: ROI + prune + morton ---
        let (roi, _samples) = grasp_preshaping::predictor::predict_roi_with_samples(
            &current_pose,
            &twist,
            &covariance,
            &index_tip_local,
            &pred_config,
            None, // no hit time
        );
        let roi_cloud = prune(&cloud, Some(roi));
        let (morton_arr, morton_offsets, morton_start) = morton(&roi_cloud, config::TSDF_RESOLUTION_M());

        // --- Stage 2: SQ backside ---
        let sq_params = if config::SQ_ENABLE_BACKSIDE() {
            fit_best_superquadric(&roi_cloud.points)
        } else {
            None
        };

        // --- Stage 3: TSDF ---
        let tsdf = get_tsdf(
            &morton_arr,
            &morton_offsets,
            config::TRUNCATION_CELLS(),
            morton_start,
            config::TSDF_RESOLUTION_M(),
            &cameras,
            sq_params.as_ref(),
        );

        Self {
            cloud,
            current_pose,
            twist,
            covariance,
            cameras,
            lut,
            pred_config,
            index_tip_local,
            collision_tol,
            roi_cloud,
            morton_arr,
            morton_offsets,
            morton_start,
            sq_params,
            tsdf,
        }
    }
}

// ---------------------------------------------------------------------------
// Scoring helper (replicates c_api::score_all_particles logic)
// ---------------------------------------------------------------------------

fn score_particles(
    lut: &FingerLUT,
    tsdf: &grasp_preshaping::pointcloud_helper::Tsdf,
    particles: &mut [SmcParticle],
    collision_tol: f32,
    sq_params: Option<&grasp_preshaping::superquadric::SuperquadricParams>,
) {
    let weights = GraspWeights::default();
    let results: Vec<f64> = particles
        .par_iter()
        .map(|p| {
            let base_transform = p.pose.to_se3();
            let result: GraspScoreResult = match p.grasp_type {
                0 => score_cylindrical(lut, tsdf, &base_transform, collision_tol, sq_params),
                1 => score_pinch(lut, tsdf, &base_transform, collision_tol, sq_params),
                _ => score_lateral(lut, tsdf, &base_transform, collision_tol, sq_params),
            };
            result.combined_score(&weights, p.sample_probability)
        })
        .collect();

    for (i, score) in results.iter().enumerate() {
        particles[i].score = *score;
    }
}

// ---------------------------------------------------------------------------
// Benchmark groups
// ---------------------------------------------------------------------------

fn bench_roi_filter_morton(c: &mut Criterion) {
    let state = BenchState::new();

    let mut group = c.benchmark_group("1_roi_filter_morton");
    group.sample_size(20);
    group.bench_function("full", |b| {
        b.iter(|| {
            let (roi, _) = grasp_preshaping::predictor::predict_roi_with_samples(
                black_box(&state.current_pose),
                black_box(&state.twist),
                black_box(&state.covariance),
                black_box(&state.index_tip_local),
                black_box(&state.pred_config),
                black_box(None),
            );
            let pruned = prune(black_box(&state.cloud), Some(roi));
            let _ = morton(black_box(&pruned), black_box(config::TSDF_RESOLUTION_M()));
        })
    });
    group.finish();
}

fn bench_sq_backside(c: &mut Criterion) {
    let state = BenchState::new();

    let mut group = c.benchmark_group("2_sq_backside");
    group.sample_size(20);
    group.bench_function("full", |b| {
        b.iter(|| {
            let _ = fit_best_superquadric(black_box(&state.roi_cloud.points));
        })
    });
    group.finish();
}

fn bench_tsdf_construction(c: &mut Criterion) {
    let state = BenchState::new();

    let mut group = c.benchmark_group("3_tsdf_construction");
    group.sample_size(20);
    group.bench_function("full", |b| {
        b.iter(|| {
            let _ = get_tsdf(
                black_box(&state.morton_arr),
                black_box(&state.morton_offsets),
                black_box(config::TRUNCATION_CELLS()),
                black_box(state.morton_start),
                black_box(config::TSDF_RESOLUTION_M()),
                black_box(&state.cameras),
                black_box(state.sq_params.as_ref()),
            );
        })
    });
    group.finish();
}

fn bench_smc_per_iteration(c: &mut Criterion) {
    let state = BenchState::new();
    let n_samples = config::PREDICTION_SAMPLES();

    let mut group = c.benchmark_group("4_smc_per_iteration");
    group.sample_size(10); // fewer samples because each iteration is expensive

    group.bench_function("scoring_only", |b| {
        b.iter_batched(
            || {
                // Setup: generate fresh particles for each sample
                let mut rng = rand::rng();
                sample_initial_particles(
                    &state.current_pose,
                    &state.twist,
                    &state.covariance,
                    n_samples,
                    state.pred_config.t_max,
                    &mut rng,
                    None,
                )
            },
            |mut particles| {
                // Measured: one scoring pass (the dominant cost per iteration)
                score_particles(
                    black_box(&state.lut),
                    black_box(&state.tsdf),
                    black_box(&mut particles),
                    black_box(state.collision_tol),
                    black_box(state.sq_params.as_ref()),
                );
            },
            criterion::BatchSize::LargeInput,
        )
    });

    group.finish();
}

fn bench_smc_full_loop(c: &mut Criterion) {
    let state = BenchState::new();
    let n_samples = config::PREDICTION_SAMPLES();
    let n_iterations = config::ITERATIONS();

    let mut group = c.benchmark_group("4b_smc_full_loop");
    group.sample_size(10);

    group.bench_function(
        BenchmarkId::new("iterations", n_iterations),
        |b| {
            b.iter_batched(
                || {
                    let mut rng = rand::rng();
                    sample_initial_particles(
                        &state.current_pose,
                        &state.twist,
                        &state.covariance,
                        n_samples,
                        state.pred_config.t_max,
                        &mut rng,
                        None,
                    )
                },
                |mut particles| {
                    for iteration in 0..n_iterations {
                        score_particles(
                            &state.lut,
                            &state.tsdf,
                            &mut particles,
                            state.collision_tol,
                            state.sq_params.as_ref(),
                        );

                        if iteration == n_iterations - 1 {
                            break;
                        }

                        let elite_indices =
                            select_elite_indices(&particles, config::ELITE_RATIO());
                        let elites: Vec<SmcParticle> =
                            elite_indices.iter().map(|&idx| particles[idx].clone()).collect();
                        let grasp_type_weights = compute_grasp_type_weights(&particles);

                        let decay = config::DECAY_RATE().powi(iteration as i32);
                        let proposal_std_v = config::INITIAL_PROPOSAL_STD_V() * decay;
                        let proposal_std_omega = config::INITIAL_PROPOSAL_STD_OMEGA() * decay;
                        let proposal_std_wrist = config::INITIAL_PROPOSAL_STD_WRIST() * decay;

                        let mut rng = rand::rng();
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
                },
                criterion::BatchSize::LargeInput,
            )
        },
    );

    group.finish();
}

fn bench_total_pipeline(c: &mut Criterion) {
    let state = BenchState::new();
    let n_samples = config::PREDICTION_SAMPLES();
    let n_iterations = config::ITERATIONS();

    let mut group = c.benchmark_group("5_total_pipeline");
    group.sample_size(10);

    group.bench_function("end_to_end", |b| {
        b.iter(|| {
            // Stage 1: ROI + prune + morton
            let (roi, _) = grasp_preshaping::predictor::predict_roi_with_samples(
                black_box(&state.current_pose),
                black_box(&state.twist),
                black_box(&state.covariance),
                black_box(&state.index_tip_local),
                black_box(&state.pred_config),
                black_box(None),
            );
            let pruned = prune(black_box(&state.cloud), Some(roi));
            let (morton_arr, morton_offsets, morton_start) =
                morton(black_box(&pruned), black_box(config::TSDF_RESOLUTION_M()));

            // Stage 2: SQ backside
            let sq_params = if config::SQ_ENABLE_BACKSIDE() {
                fit_best_superquadric(black_box(&pruned.points))
            } else {
                None
            };

            // Stage 3: TSDF
            let tsdf = get_tsdf(
                black_box(&morton_arr),
                black_box(&morton_offsets),
                black_box(config::TRUNCATION_CELLS()),
                black_box(morton_start),
                black_box(config::TSDF_RESOLUTION_M()),
                black_box(&state.cameras),
                black_box(sq_params.as_ref()),
            );

            // Stage 4: SMC loop
            let mut rng = rand::rng();
            let mut particles = sample_initial_particles(
                &state.current_pose,
                &state.twist,
                &state.covariance,
                n_samples,
                state.pred_config.t_max,
                &mut rng,
                None,
            );

            for iteration in 0..n_iterations {
                score_particles(
                    &state.lut,
                    &tsdf,
                    &mut particles,
                    state.collision_tol,
                    sq_params.as_ref(),
                );

                if iteration == n_iterations - 1 {
                    break;
                }

                let elite_indices = select_elite_indices(&particles, config::ELITE_RATIO());
                let elites: Vec<SmcParticle> =
                    elite_indices.iter().map(|&idx| particles[idx].clone()).collect();
                let grasp_type_weights = compute_grasp_type_weights(&particles);

                let decay = config::DECAY_RATE().powi(iteration as i32);
                particles = resample_around_elites(
                    &elites,
                    n_samples,
                    config::INITIAL_PROPOSAL_STD_V() * decay,
                    config::INITIAL_PROPOSAL_STD_OMEGA() * decay,
                    config::INITIAL_PROPOSAL_STD_WRIST() * decay,
                    &grasp_type_weights,
                    &mut rng,
                );
            }
        })
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_roi_filter_morton,
    bench_sq_backside,
    bench_tsdf_construction,
    bench_smc_per_iteration,
    bench_smc_full_loop,
    bench_total_pipeline,
);
criterion_main!(benches);
