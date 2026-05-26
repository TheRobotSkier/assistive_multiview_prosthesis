use crate::lut_helper::{Contact, DualQuaternion, FingerLUT};
use crate::planner::{
    score_cylindrical, score_lateral, score_pinch, GraspScoreResult, GraspWeights,
};
use crate::pointcloud_helper::{get_tsdf, morton, prune, PointCloud};
use crate::predictor::{
    predict_roi_with_samples, sample_initial_particles, resample_around_elites,
    select_elite_indices, compute_grasp_type_weights, SmcParticle, PredictionConfig, Twist6, TwistCovariance,
    TwistWithCovariance,
};
use crate::config;
use nalgebra::{Matrix4, Vector3};
use rayon::prelude::*;
use std::ffi::c_char;
use std::sync::OnceLock;

pub const GRASP_TYPE_UNKNOWN: i32 = 0;
pub const GRASP_TYPE_CYLINDRICAL: i32 = 1;
pub const GRASP_TYPE_PINCH: i32 = 2;
pub const GRASP_TYPE_LATERAL: i32 = 3;

pub const GRASP_COMPUTE_OK: i32 = 0;
pub const GRASP_COMPUTE_INVALID_ARGS: i32 = 1;
pub const GRASP_COMPUTE_PANIC: i32 = 2;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GraspPoseFFI {
    pub px: f64,
    pub py: f64,
    pub pz: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
    pub qw: f64,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GraspTwistFFI {
    pub lx: f64,
    pub ly: f64,
    pub lz: f64,
    pub ax: f64,
    pub ay: f64,
    pub az: f64,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct PointCloudViewFFI {
    pub width: usize,
    pub height: usize,
    pub point_step: usize,
    pub x_off: usize,
    pub y_off: usize,
    pub z_off: usize,
    pub data_ptr: *const u8,
    pub data_len: usize,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct CameraPositionFFI {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GraspComputeRequestFFI {
    pub pose: GraspPoseFFI,
    pub twist: GraspTwistFFI,
    pub cloud: PointCloudViewFFI,
    pub cameras: [CameraPositionFFI; 4],
    pub n_cameras: u32,
    /// Predicted time-to-hit in seconds from twist propagation.
    /// Negative value means no hit time is available; fall back to uniform sampling.
    pub hit_time_s: f64,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GraspComputeResponseFFI {
    pub success: u8,
    pub closure_amount: f64,
    pub combined_score: f64,
    pub grasp_type: i32,
    pub alignment_score: f64,
    pub force_closure_score: f64,
    pub contact_count_score: f64,
    pub contact_score: f64,
    pub second_best_combined_score: f64,
    pub second_best_grasp_type: i32,
    pub thumb_closure: f64,
    pub index_closure: f64,
    pub mrl_closure: f64,
    pub pipeline_time_ms: u32,
    pub smc_iterations_used: u32,
    /// Target hand position from the best scored grasp sample (world frame).
    pub target_px: f64,
    pub target_py: f64,
    pub target_pz: f64,
    /// Wrist orientation quaternion [qx, qy, qz, qw].
    pub wrist_qx: f64,
    pub wrist_qy: f64,
    pub wrist_qz: f64,
    pub wrist_qw: f64,
    /// Wrist rotation angle in degrees (signed delta from current position).
    pub wrist_rotation_deg: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GraspType {
    Cylindrical,
    Pinch,
    Lateral,
}

impl std::fmt::Display for GraspType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GraspType::Cylindrical => write!(f, "cylindrical"),
            GraspType::Pinch => write!(f, "pinch"),
            GraspType::Lateral => write!(f, "lateral"),
        }
    }
}

impl GraspType {
    fn to_ffi(self) -> i32 {
        match self {
            GraspType::Cylindrical => GRASP_TYPE_CYLINDRICAL,
            GraspType::Pinch => GRASP_TYPE_PINCH,
            GraspType::Lateral => GRASP_TYPE_LATERAL,
        }
    }
}

#[derive(Clone)]
struct ScoredGrasp {
    grasp_type: GraspType,
    result: GraspScoreResult,
    combined: f64,
}

type ScorerFn = fn(
    &FingerLUT,
    &crate::pointcloud_helper::Tsdf,
    &Matrix4<f64>,
    f32,
    Option<&crate::superquadric::SuperquadricParams>,
) -> GraspScoreResult;

struct ComputeOutput {
    grasp_type: GraspType,
    closure_amount: f64,
    combined_score: f64,
    alignment_score: f64,
    force_closure_score: f64,
    contact_count_score: f64,
    contact_score: f64,
    second_best_combined_score: f64,
    second_best_grasp_type: i32,
    pipeline_time_ms: u32,
    smc_iterations_used: u32,
    thumb_closure: f64,
    index_closure: f64,
    mrl_closure: f64,
    /// Target hand position from the best scored grasp sample [px, py, pz].
    target_position: [f64; 3],
    /// Wrist orientation quaternion [qx, qy, qz, qw].
    wrist_quaternion: [f64; 4],
    /// Wrist rotation angle in degrees (signed delta from current position).
    wrist_rotation_deg: f64,
}

fn compute_per_finger_output(grasp_type: GraspType, closure_amount: f64) -> (f64, f64, f64) {
    match grasp_type {
        GraspType::Cylindrical => (closure_amount, closure_amount, closure_amount),
        GraspType::Pinch => (closure_amount, closure_amount, 0.0),
        GraspType::Lateral => (closure_amount, closure_amount, 0.0),
    }
}

static LUT: OnceLock<FingerLUT> = OnceLock::new();
static PRED_CONFIG: OnceLock<PredictionConfig> = OnceLock::new();
static INDEX_TIP_LOCAL: OnceLock<Vector3<f64>> = OnceLock::new();

fn select_best_grasps(scored: &[ScoredGrasp]) -> (Option<&ScoredGrasp>, Option<&ScoredGrasp>) {
    let mut best: Option<&ScoredGrasp> = None;
    let mut second_best: Option<&ScoredGrasp> = None;

    for grasp in scored {
        if let Some(b) = best {
            if grasp.combined > b.combined {
                second_best = best;
                best = Some(grasp);
            } else if second_best.map_or(true, |sb| grasp.combined > sb.combined) {
                second_best = Some(grasp);
            }
        } else {
            best = Some(grasp);
        }
    }

    (best, second_best)
}

/// Score all SMC particles in parallel, returning scored grasps and updating
/// each particle's `score` field in place.
fn score_all_particles(
    lut: &FingerLUT,
    tsdf: &crate::pointcloud_helper::Tsdf,
    particles: &mut [SmcParticle],
    collision_tol: f32,
    sq_params: Option<&crate::superquadric::SuperquadricParams>,
) -> Vec<ScoredGrasp> {
    let weights = GraspWeights::default();

    // Score in parallel, collect (ScoredGrasp, combined_score) pairs.
    let results: Vec<(ScoredGrasp, f64)> = particles
        .par_iter()
        .map(|p| {
            let base_transform = p.pose.to_se3();
            let scorer: ScorerFn = match p.grasp_type {
                0 => score_cylindrical,
                1 => score_pinch,
                _ => score_lateral,
            };
            let grasp_type = match p.grasp_type {
                0 => GraspType::Cylindrical,
                1 => GraspType::Pinch,
                _ => GraspType::Lateral,
            };

            let result = scorer(lut, tsdf, &base_transform, collision_tol, sq_params);
            let combined = result.combined_score(&weights, p.sample_probability);
            (
                ScoredGrasp {
                    grasp_type,
                    result,
                    combined,
                },
                combined,
            )
        })
        .collect();

    // Write scores back into particles.
    for (i, (_, score)) in results.iter().enumerate() {
        particles[i].score = *score;
    }

    results.into_iter().map(|(sg, _)| sg).collect()
}

fn pose_to_dual_quaternion(pose: &GraspPoseFFI) -> DualQuaternion {
    let mut m = Matrix4::identity();
    let uq = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        pose.qw, pose.qx, pose.qy, pose.qz,
    ));
    m.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(uq.to_rotation_matrix().matrix());
    m[(0, 3)] = pose.px;
    m[(1, 3)] = pose.py;
    m[(2, 3)] = pose.pz;
    DualQuaternion::from_se3(&m)
}

fn twist_to_runtime(twist: &GraspTwistFFI) -> TwistWithCovariance {
    TwistWithCovariance {
        twist: Twist6 {
            omega: Vector3::new(twist.ax, twist.ay, twist.az),
            v: Vector3::new(twist.lx, twist.ly, twist.lz),
        },
        covariance: TwistCovariance::fixed(),
    }
}

fn pointcloud_view_to_pointcloud(view: &PointCloudViewFFI) -> Result<PointCloud, String> {
    if view.point_step == 0 {
        return Err("PointCloud2 has point_step=0".into());
    }
    if view.data_ptr.is_null() {
        return Err("PointCloud data pointer is null".into());
    }

    let n_points = view.width.saturating_mul(view.height);
    let required_offsets = [view.x_off, view.y_off, view.z_off];
    if required_offsets.iter().any(|offset| *offset + 4 > view.point_step) {
        return Err("PointCloud offsets exceed point_step".into());
    }

    // SAFETY: data_ptr/data_len are owned by the FFI caller for the duration of this call.
    let data = unsafe { std::slice::from_raw_parts(view.data_ptr, view.data_len) };
    let mut points = Vec::with_capacity(n_points);

    for i in 0..n_points {
        let base = i.saturating_mul(view.point_step);
        if base + view.z_off + 4 > data.len() {
            break;
        }

        let x = f32::from_le_bytes([
            data[base + view.x_off],
            data[base + view.x_off + 1],
            data[base + view.x_off + 2],
            data[base + view.x_off + 3],
        ]);
        let y = f32::from_le_bytes([
            data[base + view.y_off],
            data[base + view.y_off + 1],
            data[base + view.y_off + 2],
            data[base + view.y_off + 3],
        ]);
        let z = f32::from_le_bytes([
            data[base + view.z_off],
            data[base + view.z_off + 1],
            data[base + view.z_off + 2],
            data[base + view.z_off + 3],
        ]);

        if x.is_finite() && y.is_finite() && z.is_finite() {
            points.push(Vector3::new(x, y, z));
        }
    }

    if points.is_empty() {
        return Err("No valid points parsed from PointCloud2".into());
    }

    Ok(PointCloud::new(points))
}

fn get_lut() -> &'static FingerLUT {
    LUT.get_or_init(|| {
        let lut_path = crate::runtime_config::crate_root().join("data/finger_contact_lut.npz");
        FingerLUT::load(lut_path.to_str().expect("LUT path is not valid UTF-8"))
    })
}

fn get_prediction_config() -> &'static PredictionConfig {
    PRED_CONFIG.get_or_init(|| PredictionConfig {
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
    })
}

fn get_index_tip_local() -> &'static Vector3<f64> {
    INDEX_TIP_LOCAL.get_or_init(|| get_lut().get_location(Contact::IndexTip, 0.0))
}

fn compute_from_request(request: &GraspComputeRequestFFI) -> Result<ComputeOutput, String> {
    let pipeline_start = std::time::Instant::now();
    let current_pose = pose_to_dual_quaternion(&request.pose);
    let twist_with_cov = twist_to_runtime(&request.twist);
    let cloud = pointcloud_view_to_pointcloud(&request.cloud)?;

    let lut = get_lut();
    let pred_config = get_prediction_config();
    let index_tip_local = get_index_tip_local();

    // --- Extract hit time (negative sentinel = no hit time available) ---
    let hit_time_opt = if request.hit_time_s >= 0.0 {
        Some(request.hit_time_s)
    } else {
        None
    };
    if let Some(ht) = hit_time_opt {
        eprintln!(
            "[hit_time] using hit_time_s={:.3}s (spread={:.3}s)",
            ht,
            config::HIT_TIME_SPREAD_S(),
        );
    } else {
        eprintln!("[hit_time] no hit time available, using uniform sampling");
    }

    // --- ROI and TSDF construction (one-time cost) ---
    // With hit time available, sample_future_poses concentrates particles
    // around the predicted contact horizon, which also focuses the ROI.
    let (roi, _samples_for_roi) = predict_roi_with_samples(
        &current_pose,
        &twist_with_cov.twist,
        &twist_with_cov.covariance,
        index_tip_local,
        pred_config,
        hit_time_opt,
    );

    let pruned = prune(&cloud, Some(roi));
    if pruned.is_empty() {
        return Err("No points in ROI, cannot score grasps".into());
    }

    let cameras: Vec<crate::pointcloud_helper::Camera> = request
        .cameras
        .iter()
        .take(request.n_cameras as usize)
        .map(|c| crate::pointcloud_helper::Camera {
            position: Vector3::new(c.x, c.y, c.z),
        })
        .collect();

    let (morton_arr, offsets, start) = morton(&pruned, config::TSDF_RESOLUTION_M());

    // --- Superquadric backside estimation ---
    let sq_params: Option<crate::superquadric::SuperquadricParams> =
        if config::SQ_ENABLE_BACKSIDE() {
            crate::superquadric::fit_best_superquadric(&pruned.points)
        } else {
            None
        };

    let tsdf = get_tsdf(
        &morton_arr,
        &offsets,
        config::TRUNCATION_CELLS(),
        start,
        config::TSDF_RESOLUTION_M(),
        &cameras,
        sq_params.as_ref(),
    );

    let collision_tol = config::COLLISION_TOL_M();

    // --- SMC Optimization Loop ---
    let n_samples = config::PREDICTION_SAMPLES();
    let n_iterations = config::ITERATIONS();

    // Iteration 0: broad sampling from the motion model.
    let mut rng = rand::rng();
    let mut particles = sample_initial_particles(
        &current_pose,
        &twist_with_cov.twist,
        &twist_with_cov.covariance,
        n_samples,
        pred_config.t_max,
        &mut rng,
        hit_time_opt,
    );

    // Collect debug data across all iterations: (particle_index, scored_grasp, iteration).
    let mut all_debug: Option<Vec<(usize, ScoredGrasp, usize, SmcParticle)>> =
        if config::DEBUG_VISUALIZATION() {
            Some(Vec::new())
        } else {
            None
        };

    let mut scored: Vec<ScoredGrasp> = Vec::new();

    let mut iterations_used: u32 = 0;

    for iteration in 0..n_iterations {
        iterations_used = iteration as u32 + 1;
        scored = score_all_particles(lut, &tsdf, &mut particles, collision_tol, sq_params.as_ref());

        // Store debug data for this iteration.
        if let Some(ref mut debug) = all_debug {
            for (i, sg) in scored.iter().enumerate() {
                debug.push((i, sg.clone(), iteration, particles[i].clone()));
            }
        }

        // Last iteration: no resampling needed.
        if iteration == n_iterations - 1 {
            break;
        }

        // Select elites and resample with decaying proposal variance.
        let elite_indices = select_elite_indices(&particles, config::ELITE_RATIO());
        let elites: Vec<SmcParticle> = elite_indices
            .iter()
            .map(|&idx| particles[idx].clone())
            .collect();

        // Compute weighted grasp type probabilities based on current population performance.
        let grasp_type_weights = compute_grasp_type_weights(&particles);

        let decay = config::DECAY_RATE().powi(iteration as i32);
        let proposal_std_v = config::INITIAL_PROPOSAL_STD_V() * decay;
        let proposal_std_omega = config::INITIAL_PROPOSAL_STD_OMEGA() * decay;
        let proposal_std_wrist = config::INITIAL_PROPOSAL_STD_WRIST() * decay;

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

    // --- Debug visualization export ---
    if config::DEBUG_VISUALIZATION() {
        if let Some(ref debug) = all_debug {
            let grasp_exports: Vec<crate::debug_export::ScoredGraspExport> = debug
                .iter()
                .map(|(_idx, sg, iteration, particle)| {
                    let se3 = particle.pose.to_se3();
                    let mut pose_se3 = [0.0f64; 16];
                    for row in 0..4 {
                        for col in 0..4 {
                            pose_se3[row * 4 + col] = se3[(row, col)];
                        }
                    }
                    crate::debug_export::ScoredGraspExport {
                        sample_index: *_idx,
                        grasp_type_i32: sg.grasp_type.to_ffi(),
                        closure_amount: sg.result.closure_amount,
                        alignment_score: sg.result.alignment_score,
                        force_closure_score: sg.result.force_closure_score,
                        contact_count_score: sg.result.contact_count_score,
                        contact_score: sg.result.contact_score,
                        active_contact_count: sg.result.active_contact_count,
                        found_collision: sg.result.found_collision,
                        combined_score: sg.combined,
                        sample_probability: particle.sample_probability,
                        pose_se3,
                        wrist_rotation: particle.wrist_rotation,
                        smc_iteration: *iteration,
                    }
                })
                .collect();

            let dump = crate::debug_export::DebugDump {
                tsdf: &tsdf,
                point_cloud: &pruned,
                roi: &roi,
                cameras: &cameras,
                scored_grasps: &grasp_exports,
                input_pose: [
                    request.pose.px,
                    request.pose.py,
                    request.pose.pz,
                    request.pose.qx,
                    request.pose.qy,
                    request.pose.qz,
                    request.pose.qw,
                ],
                input_twist: [
                    request.twist.lx,
                    request.twist.ly,
                    request.twist.lz,
                    request.twist.ax,
                    request.twist.ay,
                    request.twist.az,
                ],
                sq_params: sq_params.as_ref(),
            };

            let path = crate::debug_export::debug_output_path();
            match crate::debug_export::export_npz(&dump, &path) {
                Ok(()) => eprintln!("[debug_viz] wrote {}", path.display()),
                Err(e) => eprintln!("[debug_viz] FAILED to write {}: {}", path.display(), e),
            }
        }
    }

    // --- Select best grasp from final iteration ---
    let (best, second_best) = select_best_grasps(&scored);
    let best = best.ok_or("No valid grasps found")?;

    if best.result.contact_score == 0.0 {
        return Err(format!(
            "No object in reach for any grasp type ({} particles x {} iterations, best contact_score=0.0)",
            n_samples, n_iterations
        ));
    }

    let (thumb_closure, index_closure, mrl_closure) =
        compute_per_finger_output(best.grasp_type, best.result.closure_amount);

    // Extract wrist orientation quaternion from the best sample's pose.
    let best_idx = scored
        .iter()
        .enumerate()
        .filter(|(_, sg)| sg.combined == best.combined)
        .map(|(i, _)| i)
        .next()
        .unwrap_or(0);
    let best_se3 = particles[best_idx].pose.to_se3();
    let target_position = [best_se3[(0, 3)], best_se3[(1, 3)], best_se3[(2, 3)]];
    let rot = best_se3.fixed_view::<3, 3>(0, 0);
    let rot3 = nalgebra::Rotation3::from_matrix_unchecked(rot.clone_owned());
    let uq = nalgebra::UnitQuaternion::from_rotation_matrix(&rot3);
    let q = uq.quaternion();
    let wrist_quaternion = [q.i, q.j, q.k, q.w];
    let wrist_rotation_deg = particles[best_idx].wrist_rotation.to_degrees();
    let pipeline_time_ms = pipeline_start.elapsed().as_millis() as u32;
    let (second_best_combined_score, second_best_grasp_type) = match second_best {
        Some(grasp) => (grasp.combined, grasp.grasp_type.to_ffi()),
        None => (0.0, GRASP_TYPE_UNKNOWN),
    };

    Ok(ComputeOutput {
        grasp_type: best.grasp_type,
        closure_amount: best.result.closure_amount,
        combined_score: best.combined,
        alignment_score: best.result.alignment_score,
        force_closure_score: best.result.force_closure_score,
        contact_count_score: best.result.contact_count_score,
        contact_score: best.result.contact_score,
        second_best_combined_score,
        second_best_grasp_type,
        pipeline_time_ms,
        smc_iterations_used: iterations_used,
        thumb_closure,
        index_closure,
        mrl_closure,
        target_position,
        wrist_quaternion,
        wrist_rotation_deg,
    })
}

fn write_message(buf: *mut c_char, buf_len: usize, msg: &str) {
    if buf.is_null() || buf_len == 0 {
        return;
    }

    let msg_bytes = msg.as_bytes();
    let copy_len = msg_bytes.len().min(buf_len.saturating_sub(1));

    // SAFETY: caller provides a writable C buffer of size `buf_len`.
    unsafe {
        std::ptr::copy_nonoverlapping(msg_bytes.as_ptr(), buf as *mut u8, copy_len);
        *buf.add(copy_len) = 0;
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn grasp_preshaping_reload_config() {
    crate::runtime_config::reload();
}

/// Expose API version constant for FFI consumers.
#[unsafe(no_mangle)]
pub extern "C" fn grasp_preshaping_api_version() -> u32 {
    1
}
#[unsafe(no_mangle)]
pub extern "C" fn grasp_preshaping_compute(
    request: *const GraspComputeRequestFFI,
    response: *mut GraspComputeResponseFFI,
    message_out: *mut c_char,
    message_out_len: usize,
) -> i32 {
    if request.is_null() || response.is_null() {
        write_message(message_out, message_out_len, "Invalid null pointer arguments");
        return GRASP_COMPUTE_INVALID_ARGS;
    }

    // SAFETY: request/response pointers were validated against null above.
    let (request_ref, response_ref) = unsafe { (&*request, &mut *response) };
    *response_ref = GraspComputeResponseFFI {
        success: 0,
        closure_amount: 0.0,
        combined_score: 0.0,
        grasp_type: GRASP_TYPE_UNKNOWN,
        alignment_score: 0.0,
        force_closure_score: 0.0,
        contact_count_score: 0.0,
        contact_score: 0.0,
        second_best_combined_score: 0.0,
        second_best_grasp_type: GRASP_TYPE_UNKNOWN,
        thumb_closure: 0.0,
        index_closure: 0.0,
        mrl_closure: 0.0,
        pipeline_time_ms: 0,
        smc_iterations_used: 0,
        target_px: 0.0,
        target_py: 0.0,
        target_pz: 0.0,
        wrist_qx: 0.0,
        wrist_qy: 0.0,
        wrist_qz: 0.0,
        wrist_qw: 1.0,
        wrist_rotation_deg: 0.0,
    };

    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        compute_from_request(request_ref)
    }));

    match result {
        Ok(Ok(output)) => {
            response_ref.success = 1;
            response_ref.closure_amount = output.closure_amount;
            response_ref.combined_score = output.combined_score;
            response_ref.grasp_type = output.grasp_type.to_ffi();
            response_ref.alignment_score = output.alignment_score;
            response_ref.force_closure_score = output.force_closure_score;
            response_ref.contact_count_score = output.contact_count_score;
            response_ref.contact_score = output.contact_score;
            response_ref.second_best_combined_score = output.second_best_combined_score;
            response_ref.second_best_grasp_type = output.second_best_grasp_type;
            response_ref.thumb_closure = output.thumb_closure;
            response_ref.index_closure = output.index_closure;
            response_ref.mrl_closure = output.mrl_closure;
            response_ref.pipeline_time_ms = output.pipeline_time_ms;
            response_ref.smc_iterations_used = output.smc_iterations_used;
            response_ref.wrist_qx = output.wrist_quaternion[0];
            response_ref.wrist_qy = output.wrist_quaternion[1];
            response_ref.wrist_qz = output.wrist_quaternion[2];
            response_ref.wrist_qw = output.wrist_quaternion[3];
            response_ref.target_px = output.target_position[0];
            response_ref.target_py = output.target_position[1];
            response_ref.target_pz = output.target_position[2];
            response_ref.wrist_rotation_deg = output.wrist_rotation_deg;
            let message = format!(
                "{} grasp, closure={:.4}, combined={:.4}, alignment={:.4}, force_closure={:.4}, contact_count={:.4}, contact={:.4}, second_best={:.4}, thumb={:.4}, index={:.4}, mrl={:.4}, target=({:.4},{:.4},{:.4}), wrist_rot={:.1} deg, pipeline={} ms, iterations={}",
                output.grasp_type,
                output.closure_amount,
                output.combined_score,
                output.alignment_score,
                output.force_closure_score,
                output.contact_count_score,
                output.contact_score,
                output.second_best_combined_score,
                output.thumb_closure,
                output.index_closure,
                output.mrl_closure,
                output.target_position[0],
                output.target_position[1],
                output.target_position[2],
                output.wrist_rotation_deg,
                output.pipeline_time_ms,
                output.smc_iterations_used,
            );
            write_message(message_out, message_out_len, &message);
            GRASP_COMPUTE_OK
        }
        Ok(Err(message)) => {
            write_message(message_out, message_out_len, &message);
            GRASP_COMPUTE_OK
        }
        Err(_) => {
            write_message(message_out, message_out_len, "Rust planner panic during compute");
            GRASP_COMPUTE_PANIC
        }
    }
}
