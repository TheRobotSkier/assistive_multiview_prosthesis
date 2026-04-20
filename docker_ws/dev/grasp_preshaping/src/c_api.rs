use crate::lut_helper::{Contact, DualQuaternion, FingerLUT};
use crate::planner::{
    score_cylindrical, score_lateral, score_pinch, GraspScoreResult, GraspWeights,
};
use crate::pointcloud_helper::{get_tsdf, morton, prune, PointCloud};
use crate::predictor::{
    predict_roi_with_samples, PredictionConfig, SampledPose, Twist6, TwistCovariance,
    TwistWithCovariance,
};
use nalgebra::{Matrix4, Vector3};
use std::ffi::c_char;
use std::sync::OnceLock;

const LUT_PATH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/data/finger_contact_lut.npz"
);

const TSDF_RESOLUTION_MM: f32 = 5.0;
const TRUNCATION_CELLS: usize = 4;
const COLLISION_TOL_MM: f32 = 5.0;
const HORIZON: f64 = 5.0;
const SAMPLES: usize = 1000;

const PREDICTION_HAND_RADIUS_M: f64 = 0.05;
const MIN_TSDF_DIM_M: f32 = 0.1;
const MAX_TSDF_DIM_M: f32 = 0.3;

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
    pub covariance: [f64; 36],
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
pub struct GraspComputeRequestFFI {
    pub pose: GraspPoseFFI,
    pub twist: GraspTwistFFI,
    pub cloud: PointCloudViewFFI,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GraspComputeResponseFFI {
    pub success: u8,
    pub closure_amount: f64,
    pub combined_score: f64,
    pub grasp_type: i32,
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
    sample_pose: DualQuaternion,
    sample_probability: f64,
    result: GraspScoreResult,
    combined: f64,
}

type ScorerFn = fn(
    &FingerLUT,
    &crate::pointcloud_helper::Tsdf,
    &Matrix4<f64>,
    f32,
) -> GraspScoreResult;

struct GraspScorer {
    grasp_type: GraspType,
    scorer: ScorerFn,
    weights: GraspWeights,
}

struct ComputeOutput {
    grasp_type: GraspType,
    closure_amount: f64,
    combined_score: f64,
}

static LUT: OnceLock<FingerLUT> = OnceLock::new();
static PRED_CONFIG: OnceLock<PredictionConfig> = OnceLock::new();
static INDEX_TIP_LOCAL: OnceLock<Vector3<f64>> = OnceLock::new();

fn grasp_scorers() -> Vec<GraspScorer> {
    vec![
        GraspScorer {
            grasp_type: GraspType::Cylindrical,
            scorer: score_cylindrical,
            weights: GraspWeights::cylindrical(),
        },
        GraspScorer {
            grasp_type: GraspType::Pinch,
            scorer: score_pinch,
            weights: GraspWeights::pinch(),
        },
        GraspScorer {
            grasp_type: GraspType::Lateral,
            scorer: score_lateral,
            weights: GraspWeights::lateral(),
        },
    ]
}

fn score_all_samples(
    lut: &FingerLUT,
    tsdf: &crate::pointcloud_helper::Tsdf,
    samples: &[SampledPose],
    collision_tol: f32,
) -> Vec<ScoredGrasp> {
    let scorers = grasp_scorers();
    let mut results = Vec::new();

    for sp in samples {
        let base_transform = sp.pose.to_se3();
        for gs in &scorers {
            let result = (gs.scorer)(lut, tsdf, &base_transform, collision_tol);
            let combined = result.combined_score(&gs.weights, sp.sample_probability);
            results.push(ScoredGrasp {
                grasp_type: gs.grasp_type,
                sample_pose: sp.pose,
                sample_probability: sp.sample_probability,
                result,
                combined,
            });
        }
    }

    results
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
        covariance: TwistCovariance {
            diagonal: Vector3::new(twist.covariance[0], twist.covariance[7], twist.covariance[14]),
            diagonal_v: Vector3::new(
                twist.covariance[21],
                twist.covariance[28],
                twist.covariance[35],
            ),
        },
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
    LUT.get_or_init(|| FingerLUT::load(LUT_PATH))
}

fn get_prediction_config() -> &'static PredictionConfig {
    PRED_CONFIG.get_or_init(|| PredictionConfig {
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

    let (morton_arr, offsets, start) = morton(&pruned, TSDF_RESOLUTION_MM);
    let tsdf = get_tsdf(
        &morton_arr,
        &offsets,
        TRUNCATION_CELLS,
        start,
        TSDF_RESOLUTION_MM,
        &[],
    );

    let collision_tol = COLLISION_TOL_MM / 1000.0;
    let scored = score_all_samples(lut, &tsdf, &samples, collision_tol);

    let best = select_best_grasp(&scored).ok_or("No valid grasps found")?;
    let _best_loc = best.sample_pose.location();
    let _sample_prob = best.sample_probability;

    Ok(ComputeOutput {
        grasp_type: best.grasp_type,
        closure_amount: best.result.closure_amount,
        combined_score: best.combined,
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
            let message = format!(
                "{} grasp, closure={:.4}, combined={:.4}",
                output.grasp_type, output.closure_amount, output.combined_score
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
