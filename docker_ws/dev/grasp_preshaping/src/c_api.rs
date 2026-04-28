use crate::lut_helper::{Contact, DualQuaternion, FingerLUT};
use crate::planner::{
    score_cylindrical, score_lateral, score_pinch, GraspScoreResult, GraspWeights,
};
use crate::pointcloud_helper::{get_tsdf, morton, prune, PointCloud};
use crate::predictor::{
    predict_roi_with_samples, PredictionConfig, SampledPose, Twist6, TwistCovariance,
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
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GraspComputeResponseFFI {
    pub success: u8,
    pub closure_amount: f64,
    pub combined_score: f64,
    pub grasp_type: i32,
    pub thumb_closure: f64,
    pub index_closure: f64,
    pub mrl_closure: f64,
    /// Wrist orientation quaternion [qx, qy, qz, qw].
    pub wrist_qx: f64,
    pub wrist_qy: f64,
    pub wrist_qz: f64,
    pub wrist_qw: f64,
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
) -> Option<GraspScoreResult>;

struct ComputeOutput {
    grasp_type: GraspType,
    closure_amount: f64,
    combined_score: f64,
    thumb_closure: f64,
    index_closure: f64,
    mrl_closure: f64,
    /// Wrist orientation quaternion [qx, qy, qz, qw].
    wrist_quaternion: [f64; 4],
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

fn score_all_samples(
    lut: &FingerLUT,
    tsdf: &crate::pointcloud_helper::Tsdf,
    samples: &[SampledPose],
    collision_tol: f32,
) -> Vec<ScoredGrasp> {
    let weights = GraspWeights::default();

    samples
        .par_iter()
        .map(|sp| {
            let base_transform = sp.pose.to_se3();
            let scorer: ScorerFn = match sp.grasp_type {
                0 => score_cylindrical,
                1 => score_pinch,
                _ => score_lateral,
            };
            let grasp_type = match sp.grasp_type {
                0 => GraspType::Cylindrical,
                1 => GraspType::Pinch,
                _ => GraspType::Lateral,
            };

            match scorer(lut, tsdf, &base_transform, collision_tol) {
                Some(result) => {
                    let combined = if result.found_collision {
                        result.combined_score(&weights, sp.sample_probability)
                    } else {
                        f64::NEG_INFINITY
                    };
                    ScoredGrasp {
                        grasp_type,
                        result,
                        combined,
                    }
                }
                None => {
                    // Start-position collision — pose is invalid.
                    ScoredGrasp {
                        grasp_type,
                        result: GraspScoreResult {
                            closure_amount: 0.0,
                            alignment_score: 0.0,
                            force_closure_score: 0.0,
                            contact_count_score: 0.0,
                            found_collision: false,
                        },
                        combined: f64::NEG_INFINITY,
                    }
                }
            }
        })
        .collect()
}

fn select_best_grasp(scored: &[ScoredGrasp]) -> Option<&ScoredGrasp> {
    scored.iter().max_by(|a, b| {
        a.combined
            .partial_cmp(&b.combined)
            .unwrap_or(std::cmp::Ordering::Equal)
    })
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
    LUT.get_or_init(|| FingerLUT::load(concat!(env!("CARGO_MANIFEST_DIR"), "/data/finger_contact_lut.npz")))
}

fn get_prediction_config() -> &'static PredictionConfig {
    PRED_CONFIG.get_or_init(|| PredictionConfig {
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
    })
}

fn get_index_tip_local() -> &'static Vector3<f64> {
    INDEX_TIP_LOCAL.get_or_init(|| get_lut().get_location(Contact::IndexTip, 0.0))
}

fn compute_from_request(request: &GraspComputeRequestFFI) -> Result<ComputeOutput, String> {
    let current_pose = pose_to_dual_quaternion(&request.pose);
    let twist_with_cov = twist_to_runtime(&request.twist);
    let cloud = pointcloud_view_to_pointcloud(&request.cloud)?;

    let lut = get_lut();
    let pred_config = get_prediction_config();
    let index_tip_local = get_index_tip_local();

    let (roi, samples) = predict_roi_with_samples(
        &current_pose,
        &twist_with_cov.twist,
        &twist_with_cov.covariance,
        index_tip_local,
        pred_config,
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

    let (morton_arr, offsets, start) = morton(&pruned, config::TSDF_RESOLUTION_M);
    let tsdf = get_tsdf(
        &morton_arr,
        &offsets,
        config::TRUNCATION_CELLS,
        start,
        config::TSDF_RESOLUTION_M,
        &cameras,
    );

    let collision_tol = config::COLLISION_TOL_M;
    let scored = score_all_samples(lut, &tsdf, &samples, collision_tol);

    // --- Debug visualization export ---
    if config::DEBUG_VISUALIZATION {
        let grasp_exports: Vec<crate::debug_export::ScoredGraspExport> = scored
            .iter()
            .enumerate()
            .map(|(i, sg)| {
                let se3 = samples[i].pose.to_se3();
                let mut pose_se3 = [0.0f64; 16];
                for row in 0..4 {
                    for col in 0..4 {
                        pose_se3[row * 4 + col] = se3[(row, col)];
                    }
                }
                crate::debug_export::ScoredGraspExport {
                    sample_index: i,
                    grasp_type_i32: sg.grasp_type.to_ffi(),
                    closure_amount: sg.result.closure_amount,
                    alignment_score: sg.result.alignment_score,
                    force_closure_score: sg.result.force_closure_score,
                    contact_count_score: sg.result.contact_count_score,
                    found_collision: sg.result.found_collision,
                    combined_score: sg.combined,
                    sample_probability: samples[i].sample_probability,
                    pose_se3,
                    wrist_rotation: samples[i].wrist_rotation,
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
        };

        let path = crate::debug_export::debug_output_path();
        match crate::debug_export::export_npz(&dump, &path) {
            Ok(()) => eprintln!("[debug_viz] wrote {}", path.display()),
            Err(e) => eprintln!("[debug_viz] FAILED to write {}: {}", path.display(), e),
        }
    }

    let best = select_best_grasp(&scored).ok_or("No valid grasps found")?;

    if !best.result.found_collision {
        return Err(format!(
            "No collision found for any grasp type ({} samples evaluated)",
            scored.len()
        ));
    }

    let (thumb_closure, index_closure, mrl_closure) =
        compute_per_finger_output(best.grasp_type, best.result.closure_amount);

    // Extract wrist orientation quaternion from the best sample's pose.
    let best_sample_idx = scored.iter().enumerate()
        .filter(|(_, sg)| sg.combined == best.combined)
        .map(|(i, _)| i)
        .next()
        .unwrap_or(0);
    let best_se3 = samples[best_sample_idx].pose.to_se3();
    let rot = best_se3.fixed_view::<3, 3>(0, 0);
    let rot3 = nalgebra::Rotation3::from_matrix_unchecked(rot.clone_owned());
    let uq = nalgebra::UnitQuaternion::from_rotation_matrix(&rot3);
    let q = uq.quaternion();
    let wrist_quaternion = [q.i, q.j, q.k, q.w];

    Ok(ComputeOutput {
        grasp_type: best.grasp_type,
        closure_amount: best.result.closure_amount,
        combined_score: best.combined,
        thumb_closure,
        index_closure,
        mrl_closure,
        wrist_quaternion,
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
pub extern "C" fn grasp_preshaping_api_version() -> u32 {
    2
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
        thumb_closure: 0.0,
        index_closure: 0.0,
        mrl_closure: 0.0,
        wrist_qx: 0.0,
        wrist_qy: 0.0,
        wrist_qz: 0.0,
        wrist_qw: 1.0,
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
            response_ref.thumb_closure = output.thumb_closure;
            response_ref.index_closure = output.index_closure;
            response_ref.mrl_closure = output.mrl_closure;
            response_ref.wrist_qx = output.wrist_quaternion[0];
            response_ref.wrist_qy = output.wrist_quaternion[1];
            response_ref.wrist_qz = output.wrist_quaternion[2];
            response_ref.wrist_qw = output.wrist_quaternion[3];
            let message = format!(
                "{} grasp, closure={:.4}, combined={:.4}, thumb={:.4}, index={:.4}, mrl={:.4}, wrist_q=({:.3},{:.3},{:.3},{:.3})",
                output.grasp_type,
                output.closure_amount,
                output.combined_score,
                output.thumb_closure,
                output.index_closure,
                output.mrl_closure,
                output.wrist_quaternion[0],
                output.wrist_quaternion[1],
                output.wrist_quaternion[2],
                output.wrist_quaternion[3],
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
