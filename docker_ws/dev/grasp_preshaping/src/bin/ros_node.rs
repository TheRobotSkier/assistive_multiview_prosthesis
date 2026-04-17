#![cfg(feature = "ros")]

use anyhow::{Error, Result};
use grasp_preshaping::lut_helper::{Contact, DualQuaternion, FingerLUT};
use grasp_preshaping::planner::{
    score_cylindrical, score_lateral, score_pinch, GraspScoreResult, GraspWeights,
};
use grasp_preshaping::pointcloud_helper::{get_tsdf, morton, prune, PointCloud};
use grasp_preshaping::predictor::{
    predict_roi_with_samples, PredictionConfig, SampledPose, Twist6, TwistCovariance,
    TwistWithCovariance,
};
use nalgebra::{Matrix4, Vector3};
use rclrs::*;
use std::convert::TryInto;
use std::sync::{Arc, Mutex};

const TOPIC_COMMAND_THUMB: &str = "/thumb_pos_ff_controller/commands";
const TOPIC_COMMAND_INDEX: &str = "/index_pos_ff_controller/commands";
const TOPIC_COMMAND_MRL: &str = "/mrl_pos_ff_controller/commands";

const TOPIC_HAND_POSE: &str = "/hand_pose";
const TOPIC_HAND_TWIST: &str = "/hand_twist";
const TOPIC_SEGMENTED_OBJECT_CLOUD: &str = "/segmented_object_cloud";

mod std_srvs_trigger {
    pub mod rmw {
        #[link(name = "std_srvs__rosidl_typesupport_c")]
        unsafe extern "C" {
            fn rosidl_typesupport_c__get_message_type_support_handle__std_srvs__srv__Trigger_Request(
            ) -> *const std::ffi::c_void;
            fn rosidl_typesupport_c__get_message_type_support_handle__std_srvs__srv__Trigger_Response(
            ) -> *const std::ffi::c_void;
            fn rosidl_typesupport_c__get_service_type_support_handle__std_srvs__srv__Trigger(
            ) -> *const std::ffi::c_void;
        }

        #[link(name = "std_srvs__rosidl_generator_c")]
        unsafe extern "C" {
            fn std_srvs__srv__Trigger_Request__init(msg: *mut Trigger_Request) -> bool;
            fn std_srvs__srv__Trigger_Request__Sequence__init(
                seq: *mut rosidl_runtime_rs::Sequence<Trigger_Request>,
                size: usize,
            ) -> bool;
            fn std_srvs__srv__Trigger_Request__Sequence__fini(
                seq: *mut rosidl_runtime_rs::Sequence<Trigger_Request>,
            );
            fn std_srvs__srv__Trigger_Request__Sequence__copy(
                in_seq: &rosidl_runtime_rs::Sequence<Trigger_Request>,
                out_seq: *mut rosidl_runtime_rs::Sequence<Trigger_Request>,
            ) -> bool;

            fn std_srvs__srv__Trigger_Response__init(msg: *mut Trigger_Response) -> bool;
            fn std_srvs__srv__Trigger_Response__Sequence__init(
                seq: *mut rosidl_runtime_rs::Sequence<Trigger_Response>,
                size: usize,
            ) -> bool;
            fn std_srvs__srv__Trigger_Response__Sequence__fini(
                seq: *mut rosidl_runtime_rs::Sequence<Trigger_Response>,
            );
            fn std_srvs__srv__Trigger_Response__Sequence__copy(
                in_seq: &rosidl_runtime_rs::Sequence<Trigger_Response>,
                out_seq: *mut rosidl_runtime_rs::Sequence<Trigger_Response>,
            ) -> bool;
        }

        #[repr(C)]
        #[derive(Clone, Debug, PartialEq, PartialOrd)]
        pub struct Trigger_Request {
            pub structure_needs_at_least_one_member: u8,
        }

        impl Default for Trigger_Request {
            fn default() -> Self {
                unsafe {
                    let mut msg = std::mem::zeroed();
                    if !std_srvs__srv__Trigger_Request__init(&mut msg as *mut _) {
                        panic!("Call to std_srvs__srv__Trigger_Request__init() failed");
                    }
                    msg
                }
            }
        }

        impl rosidl_runtime_rs::SequenceAlloc for Trigger_Request {
            fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
                unsafe { std_srvs__srv__Trigger_Request__Sequence__init(seq as *mut _, size) }
            }

            fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
                unsafe { std_srvs__srv__Trigger_Request__Sequence__fini(seq as *mut _) }
            }

            fn sequence_copy(
                in_seq: &rosidl_runtime_rs::Sequence<Self>,
                out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
            ) -> bool {
                unsafe { std_srvs__srv__Trigger_Request__Sequence__copy(in_seq, out_seq as *mut _) }
            }
        }

        impl rosidl_runtime_rs::Message for Trigger_Request {
            type RmwMsg = Self;

            fn into_rmw_message(
                msg_cow: std::borrow::Cow<'_, Self>,
            ) -> std::borrow::Cow<'_, Self::RmwMsg> {
                msg_cow
            }

            fn from_rmw_message(msg: Self::RmwMsg) -> Self {
                msg
            }
        }

        impl rosidl_runtime_rs::RmwMessage for Trigger_Request {
            const TYPE_NAME: &'static str = "std_srvs/srv/Trigger_Request";

            fn get_type_support() -> *const std::ffi::c_void {
                unsafe {
                    rosidl_typesupport_c__get_message_type_support_handle__std_srvs__srv__Trigger_Request()
                }
            }
        }

        #[repr(C)]
        #[derive(Clone, Debug, PartialEq, PartialOrd)]
        pub struct Trigger_Response {
            pub success: bool,
            pub message: rosidl_runtime_rs::String,
        }

        impl Default for Trigger_Response {
            fn default() -> Self {
                unsafe {
                    let mut msg = std::mem::zeroed();
                    if !std_srvs__srv__Trigger_Response__init(&mut msg as *mut _) {
                        panic!("Call to std_srvs__srv__Trigger_Response__init() failed");
                    }
                    msg
                }
            }
        }

        impl rosidl_runtime_rs::SequenceAlloc for Trigger_Response {
            fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
                unsafe { std_srvs__srv__Trigger_Response__Sequence__init(seq as *mut _, size) }
            }

            fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
                unsafe { std_srvs__srv__Trigger_Response__Sequence__fini(seq as *mut _) }
            }

            fn sequence_copy(
                in_seq: &rosidl_runtime_rs::Sequence<Self>,
                out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
            ) -> bool {
                unsafe { std_srvs__srv__Trigger_Response__Sequence__copy(in_seq, out_seq as *mut _) }
            }
        }

        impl rosidl_runtime_rs::Message for Trigger_Response {
            type RmwMsg = Self;

            fn into_rmw_message(
                msg_cow: std::borrow::Cow<'_, Self>,
            ) -> std::borrow::Cow<'_, Self::RmwMsg> {
                msg_cow
            }

            fn from_rmw_message(msg: Self::RmwMsg) -> Self {
                msg
            }
        }

        impl rosidl_runtime_rs::RmwMessage for Trigger_Response {
            const TYPE_NAME: &'static str = "std_srvs/srv/Trigger_Response";

            fn get_type_support() -> *const std::ffi::c_void {
                unsafe {
                    rosidl_typesupport_c__get_message_type_support_handle__std_srvs__srv__Trigger_Response()
                }
            }
        }

        pub struct Trigger;

        impl rosidl_runtime_rs::Service for Trigger {
            type Request = Trigger_Request;
            type Response = Trigger_Response;

            fn get_type_support() -> *const std::ffi::c_void {
                unsafe {
                    rosidl_typesupport_c__get_service_type_support_handle__std_srvs__srv__Trigger()
                }
            }
        }
    }
}

const LUT_PATH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../data/finger_contact_lut.npz"
);

const TSDF_RESOLUTION_MM: f32 = 5.0;
const TRUNCATION_CELLS: usize = 4;
const COLLISION_TOL_MM: f32 = 5.0;
const HORIZON: f64 = 5.0;
const SAMPLES: usize = 1000;

const PREDICTION_HAND_RADIUS_M: f64 = 0.05;
const MIN_TSDF_DIM_M: f32 = 0.1;
const MAX_TSDF_DIM_M: f32 = 0.3;

#[derive(Clone)]
struct PoseData {
    px: f64,
    py: f64,
    pz: f64,
    qx: f64,
    qy: f64,
    qz: f64,
    qw: f64,
}

#[derive(Clone)]
struct TwistData {
    lx: f64,
    ly: f64,
    lz: f64,
    ax: f64,
    ay: f64,
    az: f64,
    covariance: [f64; 36],
}

#[derive(Clone)]
struct PointCloudData {
    width: usize,
    height: usize,
    point_step: usize,
    x_off: usize,
    y_off: usize,
    z_off: usize,
    data: Vec<u8>,
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

struct ScoredGrasp {
    grasp_type: GraspType,
    sample_pose: DualQuaternion,
    sample_probability: f64,
    result: GraspScoreResult,
    combined: f64,
}

type ScorerFn =
    fn(&FingerLUT, &grasp_preshaping::pointcloud_helper::Tsdf, &Matrix4<f64>, f32) -> GraspScoreResult;

struct GraspScorer {
    grasp_type: GraspType,
    scorer: ScorerFn,
    weights: GraspWeights,
}

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
    tsdf: &grasp_preshaping::pointcloud_helper::Tsdf,
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

fn value_as_message(value: Value<'_>) -> Option<DynamicMessageView<'_>> {
    match value {
        Value::Simple(SimpleValue::Message(msg)) => Some(msg),
        _ => None,
    }
}

fn value_as_f64(value: Value<'_>) -> Option<f64> {
    match value {
        Value::Simple(SimpleValue::Double(v)) => Some(*v),
        Value::Simple(SimpleValue::Float(v)) => Some(*v as f64),
        Value::Simple(SimpleValue::Int64(v)) => Some(*v as f64),
        Value::Simple(SimpleValue::Uint64(v)) => Some(*v as f64),
        Value::Simple(SimpleValue::Int32(v)) => Some(*v as f64),
        Value::Simple(SimpleValue::Uint32(v)) => Some(*v as f64),
        _ => None,
    }
}

fn value_as_usize(value: Value<'_>) -> Option<usize> {
    match value {
        Value::Simple(SimpleValue::Uint32(v)) => Some(*v as usize),
        Value::Simple(SimpleValue::Uint64(v)) => Some(*v as usize),
        Value::Simple(SimpleValue::Int32(v)) if *v >= 0 => Some(*v as usize),
        Value::Simple(SimpleValue::Int64(v)) if *v >= 0 => Some(*v as usize),
        _ => None,
    }
}

fn value_as_bool(value: Value<'_>) -> Option<bool> {
    match value {
        Value::Simple(SimpleValue::Boolean(v)) => Some(*v),
        _ => None,
    }
}

fn value_as_string(value: Value<'_>) -> Option<String> {
    match value {
        Value::Simple(SimpleValue::String(v)) => Some(v.to_string()),
        Value::Simple(SimpleValue::BoundedString(v)) => Some(v.to_string()),
        _ => None,
    }
}

fn parse_pose(msg: &DynamicMessage) -> Result<PoseData, String> {
    let pose = value_as_message(msg.get("pose").ok_or("PoseStamped missing pose field")?)
        .ok_or("PoseStamped.pose has unexpected type")?;

    let position = value_as_message(pose.get("position").ok_or("pose missing position")?)
        .ok_or("pose.position has unexpected type")?;
    let orientation =
        value_as_message(pose.get("orientation").ok_or("pose missing orientation")?)
            .ok_or("pose.orientation has unexpected type")?;

    Ok(PoseData {
        px: value_as_f64(position.get("x").ok_or("position.x missing")?)
            .ok_or("position.x has unexpected type")?,
        py: value_as_f64(position.get("y").ok_or("position.y missing")?)
            .ok_or("position.y has unexpected type")?,
        pz: value_as_f64(position.get("z").ok_or("position.z missing")?)
            .ok_or("position.z has unexpected type")?,
        qx: value_as_f64(orientation.get("x").ok_or("orientation.x missing")?)
            .ok_or("orientation.x has unexpected type")?,
        qy: value_as_f64(orientation.get("y").ok_or("orientation.y missing")?)
            .ok_or("orientation.y has unexpected type")?,
        qz: value_as_f64(orientation.get("z").ok_or("orientation.z missing")?)
            .ok_or("orientation.z has unexpected type")?,
        qw: value_as_f64(orientation.get("w").ok_or("orientation.w missing")?)
            .ok_or("orientation.w has unexpected type")?,
    })
}

fn parse_twist(msg: &DynamicMessage) -> Result<TwistData, String> {
    let twist_cov =
        value_as_message(msg.get("twist").ok_or("TwistWithCovarianceStamped missing twist")?)
            .ok_or("twist field has unexpected type")?;
    let twist = value_as_message(twist_cov.get("twist").ok_or("twist.twist missing")?)
        .ok_or("twist.twist has unexpected type")?;
    let linear = value_as_message(twist.get("linear").ok_or("twist.linear missing")?)
        .ok_or("twist.linear has unexpected type")?;
    let angular = value_as_message(twist.get("angular").ok_or("twist.angular missing")?)
        .ok_or("twist.angular has unexpected type")?;

    let covariance_value = twist_cov
        .get("covariance")
        .ok_or("twist.covariance missing")?;
    let mut covariance = [0.0_f64; 36];
    match covariance_value {
        Value::Array(ArrayValue::DoubleArray(values)) => {
            for (idx, value) in values.iter().take(36).enumerate() {
                covariance[idx] = *value;
            }
        }
        _ => return Err("twist.covariance has unexpected type".into()),
    }

    Ok(TwistData {
        lx: value_as_f64(linear.get("x").ok_or("linear.x missing")?)
            .ok_or("linear.x has unexpected type")?,
        ly: value_as_f64(linear.get("y").ok_or("linear.y missing")?)
            .ok_or("linear.y has unexpected type")?,
        lz: value_as_f64(linear.get("z").ok_or("linear.z missing")?)
            .ok_or("linear.z has unexpected type")?,
        ax: value_as_f64(angular.get("x").ok_or("angular.x missing")?)
            .ok_or("angular.x has unexpected type")?,
        ay: value_as_f64(angular.get("y").ok_or("angular.y missing")?)
            .ok_or("angular.y has unexpected type")?,
        az: value_as_f64(angular.get("z").ok_or("angular.z missing")?)
            .ok_or("angular.z has unexpected type")?,
        covariance,
    })
}

fn parse_pointcloud2(msg: &DynamicMessage) -> Result<PointCloudData, String> {
    let width = value_as_usize(msg.get("width").ok_or("PointCloud2.width missing")?)
        .ok_or("PointCloud2.width has unexpected type")?;
    let height = value_as_usize(msg.get("height").ok_or("PointCloud2.height missing")?)
        .ok_or("PointCloud2.height has unexpected type")?;
    let point_step =
        value_as_usize(msg.get("point_step").ok_or("PointCloud2.point_step missing")?)
            .ok_or("PointCloud2.point_step has unexpected type")?;
    if point_step == 0 {
        return Err("PointCloud2 has point_step=0".into());
    }

    let is_bigendian =
        value_as_bool(msg.get("is_bigendian").ok_or("PointCloud2.is_bigendian missing")?)
            .ok_or("PointCloud2.is_bigendian has unexpected type")?;
    if is_bigendian {
        return Err("Big-endian PointCloud2 is not supported".into());
    }

    let mut x_off: usize = 0;
    let mut y_off: usize = 4;
    let mut z_off: usize = 8;

    if let Some(fields_value) = msg.get("fields") {
        if let Value::Sequence(SequenceValue::MessageSequence(fields_seq)) = fields_value {
            for field in fields_seq.as_slice() {
                let field_name = field
                    .get("name")
                    .and_then(value_as_string)
                    .unwrap_or_default();
                let offset = field
                    .get("offset")
                    .and_then(value_as_usize)
                    .unwrap_or(0);
                match field_name.as_str() {
                    "x" => x_off = offset,
                    "y" => y_off = offset,
                    "z" => z_off = offset,
                    _ => {}
                }
            }
        }
    }

    let data = match msg.get("data").ok_or("PointCloud2.data missing")? {
        Value::Sequence(SequenceValue::Uint8Sequence(seq)) => seq.as_slice().to_vec(),
        _ => return Err("PointCloud2.data has unexpected type".into()),
    };

    Ok(PointCloudData {
        width,
        height,
        point_step,
        x_off,
        y_off,
        z_off,
        data,
    })
}

fn pose_to_dual_quaternion(pose: &PoseData) -> DualQuaternion {
    let mut m = Matrix4::identity();
    let uq =
        nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            pose.qw, pose.qx, pose.qy, pose.qz,
        ));
    m.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(uq.to_rotation_matrix().matrix());
    m[(0, 3)] = pose.px;
    m[(1, 3)] = pose.py;
    m[(2, 3)] = pose.pz;
    DualQuaternion::from_se3(&m)
}

fn twist_to_runtime(twist: &TwistData) -> TwistWithCovariance {
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

fn pointcloud_data_to_pointcloud(data: &PointCloudData) -> Result<PointCloud, String> {
    let n_points = data.width.saturating_mul(data.height);
    let mut points = Vec::with_capacity(n_points);

    for i in 0..n_points {
        let base = i.saturating_mul(data.point_step);
        if base + data.z_off + 4 > data.data.len() {
            break;
        }
        let x = f32::from_le_bytes([
            data.data[base + data.x_off],
            data.data[base + data.x_off + 1],
            data.data[base + data.x_off + 2],
            data.data[base + data.x_off + 3],
        ]);
        let y = f32::from_le_bytes([
            data.data[base + data.y_off],
            data.data[base + data.y_off + 1],
            data.data[base + data.y_off + 2],
            data.data[base + data.y_off + 3],
        ]);
        let z = f32::from_le_bytes([
            data.data[base + data.z_off],
            data.data[base + data.z_off + 1],
            data.data[base + data.z_off + 2],
            data.data[base + data.z_off + 3],
        ]);
        points.push(Vector3::new(x, y, z));
    }

    if points.is_empty() {
        return Err("No valid points parsed from PointCloud2".into());
    }

    Ok(PointCloud::new(points))
}

fn build_joint_command(position: f64) -> Result<DynamicMessage, String> {
    let message_type: MessageTypeName = "std_msgs/msg/Float64MultiArray"
        .try_into()
        .map_err(|e| format!("Failed to parse command message type: {}", e))?;
    let mut msg = DynamicMessage::new(message_type)
        .map_err(|e| format!("Failed to create command message: {}", e))?;

    match msg.get_mut("data") {
        Some(ValueMut::Sequence(SequenceValueMut::DoubleSequence(seq))) => {
            *seq = vec![position].into();
            Ok(msg)
        }
        _ => Err("Float64MultiArray.data has unexpected type".into()),
    }
}

fn main() -> Result<(), Error> {
    let lut = FingerLUT::load(LUT_PATH);
    println!("LUT loaded: resolution={}", lut.get_resolution());

    let index_tip_local = lut.get_location(Contact::IndexTip, 0.0);
    println!(
        "Index tip local (open hand): ({:.4}, {:.4}, {:.4})",
        index_tip_local.x, index_tip_local.y, index_tip_local.z
    );

    let pred_config = PredictionConfig {
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
    };

    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("grasp_preshaping")?;

    let pose_cache: Arc<Mutex<Option<PoseData>>> = Arc::new(Mutex::new(None));
    let twist_cache: Arc<Mutex<Option<TwistData>>> = Arc::new(Mutex::new(None));
    let cloud_cache: Arc<Mutex<Option<PointCloudData>>> = Arc::new(Mutex::new(None));

    let pose_cache_sub = pose_cache.clone();
    let _pose_sub = node.create_dynamic_subscription::<_>(
        "geometry_msgs/msg/PoseStamped".try_into()?,
        TOPIC_HAND_POSE,
        Box::new(move |msg: DynamicMessage, _info: MessageInfo| {
            if let Ok(parsed) = parse_pose(&msg) {
                *pose_cache_sub.lock().unwrap() = Some(parsed);
            }
        }),
    )?;

    let twist_cache_sub = twist_cache.clone();
    let _twist_sub = node.create_dynamic_subscription::<_>(
        "geometry_msgs/msg/TwistWithCovarianceStamped".try_into()?,
        TOPIC_HAND_TWIST,
        Box::new(move |msg: DynamicMessage, _info: MessageInfo| {
            if let Ok(parsed) = parse_twist(&msg) {
                *twist_cache_sub.lock().unwrap() = Some(parsed);
            }
        }),
    )?;

    let cloud_cache_sub = cloud_cache.clone();
    let _cloud_sub = node.create_dynamic_subscription::<_>(
        "sensor_msgs/msg/PointCloud2".try_into()?,
        TOPIC_SEGMENTED_OBJECT_CLOUD,
        Box::new(move |msg: DynamicMessage, _info: MessageInfo| {
            if let Ok(parsed) = parse_pointcloud2(&msg) {
                *cloud_cache_sub.lock().unwrap() = Some(parsed);
            }
        }),
    )?;

    let thumb_pub = node.create_dynamic_publisher(
        "std_msgs/msg/Float64MultiArray".try_into()?,
        TOPIC_COMMAND_THUMB,
    )?;
    let index_pub = node.create_dynamic_publisher(
        "std_msgs/msg/Float64MultiArray".try_into()?,
        TOPIC_COMMAND_INDEX,
    )?;
    let mrl_pub = node.create_dynamic_publisher(
        "std_msgs/msg/Float64MultiArray".try_into()?,
        TOPIC_COMMAND_MRL,
    )?;

    let lut_arc = Arc::new(lut);
    let pred_config_arc = Arc::new(pred_config);
    let index_tip_local_arc = Arc::new(index_tip_local);

    let lut_svc = lut_arc.clone();
    let pred_config_svc = pred_config_arc.clone();
    let index_tip_local_svc = index_tip_local_arc.clone();
    let pose_cache_svc = pose_cache.clone();
    let twist_cache_svc = twist_cache.clone();
    let cloud_cache_svc = cloud_cache.clone();
    let thumb_pub_svc = thumb_pub.clone();
    let index_pub_svc = index_pub.clone();
    let mrl_pub_svc = mrl_pub.clone();

    let _service = node.create_service::<std_srvs_trigger::rmw::Trigger, _>(
        "/grasp_preshaping/compute_grasp",
        move |_req: std_srvs_trigger::rmw::Trigger_Request,
              _info: ServiceInfo|
              -> std_srvs_trigger::rmw::Trigger_Response {
            let pose_msg = pose_cache_svc.lock().unwrap().clone();
            let twist_msg = twist_cache_svc.lock().unwrap().clone();
            let cloud_msg = cloud_cache_svc.lock().unwrap().clone();

            let pose_msg = match pose_msg {
                Some(p) => p,
                None => {
                    return std_srvs_trigger::rmw::Trigger_Response {
                        success: false,
                        message: "No pose data received yet".into(),
                    };
                }
            };

            let twist_msg = match twist_msg {
                Some(t) => t,
                None => {
                    return std_srvs_trigger::rmw::Trigger_Response {
                        success: false,
                        message: "No twist data received yet".into(),
                    };
                }
            };

            let cloud_msg = match cloud_msg {
                Some(c) => c,
                None => {
                    return std_srvs_trigger::rmw::Trigger_Response {
                        success: false,
                        message: "No point cloud data received yet".into(),
                    };
                }
            };

            let current_pose = pose_to_dual_quaternion(&pose_msg);
            let twist_with_cov = twist_to_runtime(&twist_msg);
            let cloud = match pointcloud_data_to_pointcloud(&cloud_msg) {
                Ok(c) => c,
                Err(e) => {
                    return std_srvs_trigger::rmw::Trigger_Response {
                        success: false,
                        message: format!("PointCloud conversion failed: {}", e).into(),
                    };
                }
            };

            let (roi, samples) = predict_roi_with_samples(
                &current_pose,
                &twist_with_cov.twist,
                &twist_with_cov.covariance,
                &index_tip_local_svc,
                &pred_config_svc,
            );

            let pruned = prune(&cloud, Some(roi));
            if pruned.is_empty() {
                return std_srvs_trigger::rmw::Trigger_Response {
                    success: false,
                    message: "No points in ROI, cannot score grasps".into(),
                };
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
            let scored = score_all_samples(&lut_svc, &tsdf, &samples, collision_tol);

            let best = match select_best_grasp(&scored) {
                Some(b) => b,
                None => {
                    return std_srvs_trigger::rmw::Trigger_Response {
                        success: false,
                        message: "No valid grasps found".into(),
                    };
                }
            };

            let best_loc = best.sample_pose.location();
            println!(
                "Best: {} at ({:.4}, {:.4}, {:.4}) combined={:.4} (closure={:.4}, alignment={:.4}, force_closure={:.4}, prob={:.4})",
                best.grasp_type,
                best_loc.x,
                best_loc.y,
                best_loc.z,
                best.combined,
                best.result.closure_amount,
                best.result.alignment_score,
                best.result.force_closure_score,
                best.sample_probability,
            );

            let ctrl = best.result.closure_amount;
            let cmd_thumb = match build_joint_command(ctrl) {
                Ok(msg) => msg,
                Err(e) => {
                    return std_srvs_trigger::rmw::Trigger_Response {
                        success: false,
                        message: format!("Failed to build thumb command: {}", e).into(),
                    };
                }
            };
            let cmd_index = match build_joint_command(ctrl) {
                Ok(msg) => msg,
                Err(e) => {
                    return std_srvs_trigger::rmw::Trigger_Response {
                        success: false,
                        message: format!("Failed to build index command: {}", e).into(),
                    };
                }
            };
            let cmd_mrl = match build_joint_command(ctrl) {
                Ok(msg) => msg,
                Err(e) => {
                    return std_srvs_trigger::rmw::Trigger_Response {
                        success: false,
                        message: format!("Failed to build mrl command: {}", e).into(),
                    };
                }
            };

            if let Err(e) = thumb_pub_svc.publish(cmd_thumb) {
                eprintln!("Failed to publish thumb command: {}", e);
            }
            if let Err(e) = index_pub_svc.publish(cmd_index) {
                eprintln!("Failed to publish index command: {}", e);
            }
            if let Err(e) = mrl_pub_svc.publish(cmd_mrl) {
                eprintln!("Failed to publish mrl command: {}", e);
            }

            std_srvs_trigger::rmw::Trigger_Response {
                success: true,
                message: format!(
                    "{} grasp, closure={:.4}, combined={:.4}",
                    best.grasp_type, ctrl, best.combined
                )
                .into(),
            }
        },
    )?;

    println!("Grasp preshaping node ready. Waiting for trigger on /grasp_preshaping/compute_grasp");
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
