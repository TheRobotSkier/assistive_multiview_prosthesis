use nalgebra::{Matrix4, Vector3};
use preshaping::lut_helper::{Contact, DualQuaternion, FingerLUT};
use preshaping::planner::{
    score_cylindrical, score_lateral, score_pinch, GraspScoreResult, GraspWeights,
};
use preshaping::pointcloud_helper::{get_tsdf, morton, prune, Aabb, PointCloud};
use preshaping::predictor::{
    predict_roi_with_samples, read_pose_from_ros2, read_twist_from_ros2, PredictionConfig,
    SampledPose, Twist6, TwistCovariance,
};
use preshaping::ros_command_helper::publish_joint_trajectory_once;
use std::process::Command;
use std::thread;
use std::time::Duration;
use std::time::Instant;

const TOPIC_TRAJECTORY_THUMB: &str = "/thumb_trajectory_controller/joint_trajectory";
const TOPIC_TRAJECTORY_INDEX: &str = "/index_trajectory_controller/joint_trajectory";
const TOPIC_TRAJECTORY_MRL: &str = "/mrl_trajectory_controller/joint_trajectory";

const JOINT_THUMB: &str = "j_thumb_fle";
const JOINT_INDEX: &str = "j_index_fle";
const JOINT_MRL: &str = "j_mrl_fle";

const TRAJECTORY_COMMAND_TIME_FROM_START_SEC: f64 = 1.0;

const LUT_PATH: &str = "./data/finger_contact_lut.npz";
const POSE_TOPIC: &str = "/hand_pose";
const TWIST_TOPIC: &str = "/hand_twist";
const POINTCLOUD_TOPIC: &str = "/segmented_object_cloud";

const TSDF_RESOLUTION_MM: f32 = 5.0;
const TRUNCATION_CELLS: usize = 4;
const COLLISION_TOL_MM: f32 = 5.0;
const HORIZON: f64 = 5.0;
const SAMPLES: usize = 1000;
const FREQUENCY_HZ: f64 = 1.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    Demo,
    Normal,
    OneShot,
}

fn parse_mode() -> Mode {
    let args: Vec<String> = std::env::args().collect();
    let mode_str = if args.len() > 1 {
        args[1].as_str()
    } else {
        "demo"
    };
    match mode_str {
        "demo" => Mode::Demo,
        "normal" => Mode::Normal,
        "one-shot" => Mode::OneShot,
        other => {
            eprintln!(
                "Unknown mode '{}'. Usage: preshaping [demo|normal|one-shot]",
                other
            );
            std::process::exit(2);
        }
    }
}

fn read_ros_pointcloud(topic: &str) -> Result<PointCloud, String> {
    let output = Command::new("ros2")
        .arg("topic")
        .arg("echo")
        .arg("--full-length")
        .arg("--once")
        .arg(topic)
        .arg("sensor_msgs/msg/PointCloud2")
        .output()
        .map_err(|e| format!("Failed to run ros2 topic echo: {}", e))?;

    if !output.status.success() {
        return Err(format!(
            "ros2 topic echo failed on '{}' with code {:?}",
            topic,
            output.status.code()
        ));
    }

    let raw = String::from_utf8(output.stdout)
        .map_err(|e| format!("PointCloud2 output is not valid UTF-8: {}", e))?;

    let mut width: usize = 0;
    let mut height: usize = 0;
    let mut point_step: usize = 0;
    let mut data_bytes: Vec<u8> = Vec::new();
    let mut fields: Vec<(String, usize, usize)> = Vec::new();
    let mut in_fields = false;
    let mut field_name = String::new();
    let mut field_offset: usize = 0;
    let mut field_count: usize = 1;
    let mut in_data = false;

    for line in raw.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("fields:") {
            in_fields = true;
            in_data = false;
            continue;
        }
        if trimmed.starts_with("is_bigendian:") {
            in_fields = false;
        }
        if trimmed.starts_with("width:") {
            let v: usize = trimmed
                .strip_prefix("width:")
                .unwrap()
                .trim()
                .parse()
                .unwrap_or(0);
            width = v;
        }
        if trimmed.starts_with("height:") {
            let v: usize = trimmed
                .strip_prefix("height:")
                .unwrap()
                .trim()
                .parse()
                .unwrap_or(0);
            height = v;
        }
        if trimmed.starts_with("point_step:") {
            let v: usize = trimmed
                .strip_prefix("point_step:")
                .unwrap()
                .trim()
                .parse()
                .unwrap_or(0);
            point_step = v;
        }
        if trimmed.starts_with("data:") {
            in_fields = false;
            in_data = true;
            let rest = trimmed.strip_prefix("data:").unwrap().trim();
            let rest = rest.strip_prefix('[').unwrap_or(rest);
            let rest = rest.strip_suffix(']').unwrap_or(rest);
            if !rest.is_empty() {
                for val in rest.split(',') {
                    if let Ok(b) = val.trim().parse::<u8>() {
                        data_bytes.push(b);
                    }
                }
            }
            continue;
        }
        if in_data {
            let cleaned = trimmed.trim_end_matches(']');
            for val in cleaned.split(',') {
                if let Ok(b) = val.trim().parse::<u8>() {
                    data_bytes.push(b);
                }
            }
            if trimmed.ends_with(']') {
                in_data = false;
            }
            continue;
        }
        if in_fields {
            if let Some(rest) = trimmed.strip_prefix("name:") {
                field_name = rest.trim().trim_matches('\'').trim_matches('"').to_string();
            }
            if let Some(rest) = trimmed.strip_prefix("offset:") {
                field_offset = rest.trim().parse().unwrap_or(0);
            }
            if let Some(rest) = trimmed.strip_prefix("count:") {
                field_count = rest.trim().parse().unwrap_or(1);
            }
            if trimmed.starts_with('-') && !field_name.is_empty() {
                if field_count == 1 {
                    fields.push((field_name.clone(), field_offset, field_count));
                }
                field_name = String::new();
                field_offset = 0;
                field_count = 1;
            }
        }
    }

    if !field_name.is_empty() && field_count == 1 {
        fields.push((field_name, field_offset, field_count));
    }

    let x_off = fields.iter().find(|(n, _, _)| n == "x").map(|(_, o, _)| *o);
    let y_off = fields.iter().find(|(n, _, _)| n == "y").map(|(_, o, _)| *o);
    let z_off = fields.iter().find(|(n, _, _)| n == "z").map(|(_, o, _)| *o);

    let (x_off, y_off, z_off) = match (x_off, y_off, z_off) {
        (Some(x), Some(y), Some(z)) => (x, y, z),
        _ => return Err("PointCloud2 missing x/y/z fields".into()),
    };

    if point_step == 0 {
        return Err("PointCloud2 has point_step=0".into());
    }

    let n_points = width * height;
    let mut points = Vec::with_capacity(n_points);

    for i in 0..n_points {
        let base = i * point_step;
        if base + z_off + 4 > data_bytes.len() {
            break;
        }
        let x = f32::from_le_bytes([
            data_bytes[base + x_off],
            data_bytes[base + x_off + 1],
            data_bytes[base + x_off + 2],
            data_bytes[base + x_off + 3],
        ]);
        let y = f32::from_le_bytes([
            data_bytes[base + y_off],
            data_bytes[base + y_off + 1],
            data_bytes[base + y_off + 2],
            data_bytes[base + y_off + 3],
        ]);
        let z = f32::from_le_bytes([
            data_bytes[base + z_off],
            data_bytes[base + z_off + 1],
            data_bytes[base + z_off + 2],
            data_bytes[base + z_off + 3],
        ]);
        points.push(Vector3::new(x, y, z));
    }

    if points.is_empty() {
        return Err("No valid points parsed from PointCloud2".into());
    }

    Ok(PointCloud::new(points))
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
    weights: GraspWeights,
    combined: f64,
}

type ScorerFn =
    fn(&FingerLUT, &preshaping::pointcloud_helper::Tsdf, &Matrix4<f64>, f32) -> GraspScoreResult;

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
    tsdf: &preshaping::pointcloud_helper::Tsdf,
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
                weights: gs.weights,
                combined,
            });
        }
    }

    results
}

fn select_best_grasp(scored: &[ScoredGrasp]) -> &ScoredGrasp {
    scored
        .iter()
        .max_by(|a, b| {
            a.combined
                .partial_cmp(&b.combined)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .expect("at least one grasp")
}

fn print_aabb(label: &str, aabb: &Aabb) {
    let size = aabb.max - aabb.min;
    println!(
        "  {} min=({:.4}, {:.4}, {:.4}) max=({:.4}, {:.4}, {:.4}) size=({:.4}, {:.4}, {:.4})",
        label,
        aabb.min.x,
        aabb.min.y,
        aabb.min.z,
        aabb.max.x,
        aabb.max.y,
        aabb.max.z,
        size.x,
        size.y,
        size.z,
    );
}

fn identity_pose() -> DualQuaternion {
    DualQuaternion::from_se3(&Matrix4::identity())
}

fn run_iteration(lut: &FingerLUT, mode: Mode, pred_config: &PredictionConfig, iter_idx: usize) {
    let current_pose = match mode {
        Mode::Demo => identity_pose(),
        Mode::Normal | Mode::OneShot => read_pose_from_ros2(POSE_TOPIC).unwrap_or_else(|e| {
            eprintln!("Failed to read pose from ROS2: {}", e);
            std::process::exit(1);
        }),
    };

    let (roi, samples) = match mode {
        Mode::Demo => predict_roi_with_samples(
            &current_pose,
            &Twist6::dummy(),
            &TwistCovariance::dummy(),
            &lut.get_location(Contact::IndexTip, 0.0),
            pred_config,
        ),
        Mode::Normal | Mode::OneShot => {
            let twist_and_cov = read_twist_from_ros2(TWIST_TOPIC).unwrap_or_else(|e| {
                eprintln!("Failed to read twist from ROS2: {}", e);
                std::process::exit(1);
            });
            predict_roi_with_samples(
                &current_pose,
                &twist_and_cov.twist,
                &twist_and_cov.covariance,
                &lut.get_location(Contact::IndexTip, 0.0),
                pred_config,
            )
        }
    };

    println!("[iter {}] Predicted ROI:", iter_idx);
    print_aabb("ROI", &roi);
    println!(
        "[iter {}] Generated {} candidate grasp poses",
        iter_idx,
        samples.len()
    );

    let pc = match mode {
        Mode::Demo => {
            let center = current_pose.location();
            PointCloud::demo_sphere(
                Vector3::new(center.x as f32, center.y as f32, center.z as f32 + 0.05),
                0.02,
                5000,
            )
        }
        Mode::Normal | Mode::OneShot => read_ros_pointcloud(POINTCLOUD_TOPIC).unwrap_or_else(|e| {
            eprintln!("Failed to read PointCloud2: {}", e);
            std::process::exit(1);
        }),
    };

    println!("[iter {}] Point cloud: {} points", iter_idx, pc.len());

    let pruned = prune(&pc, Some(roi));
    let n_pruned = pruned.len();
    println!(
        "[iter {}] Points in ROI (used for TSDF): {}",
        iter_idx, n_pruned
    );

    let tsdf = if pruned.is_empty() {
        eprintln!(
            "[iter {}] Warning: no points in ROI, skipping scoring",
            iter_idx
        );
        return;
    } else {
        let (morton_arr, offsets, start) = morton(&pruned, TSDF_RESOLUTION_MM);
        get_tsdf(
            &morton_arr,
            &offsets,
            TRUNCATION_CELLS,
            start,
            TSDF_RESOLUTION_MM,
            &[],
        )
    };

    let collision_tol = COLLISION_TOL_MM / 1000.0;

    let scoring_start = Instant::now();
    let scored = score_all_samples(lut, &tsdf, &samples, collision_tol);

    println!(
        "[iter {}] Scored {} (sample, grasp) combinations",
        iter_idx,
        scored.len()
    );
    for sg in &scored {
        println!(
            "  {}: closure={:.4} alignment={:.4} force_closure={:.4} prob={:.4} combined={:.4}",
            sg.grasp_type,
            sg.result.closure_amount,
            sg.result.alignment_score,
            sg.result.force_closure_score,
            sg.sample_probability,
            sg.combined,
        );
    }

    let best = select_best_grasp(&scored);
    let best_loc = best.sample_pose.location();
    println!(
        "[iter {}] Best: {} at ({:.4}, {:.4}, {:.4}) combined={:.4} (closure={:.4}, alignment={:.4}, prob={:.4})",
        iter_idx,
        best.grasp_type,
        best_loc.x,
        best_loc.y,
        best_loc.z,
        best.combined,
        best.result.closure_amount,
        best.result.alignment_score,
        best.sample_probability,
    );
    println!(
        "[iter {}] Scoring time: {:.2?}",
        iter_idx,
        scoring_start.elapsed()
    );

    if mode == Mode::Normal || mode == Mode::OneShot {
        let ctrl = best.result.closure_amount;
        publish_joint_trajectory_once(
            TOPIC_TRAJECTORY_THUMB,
            JOINT_THUMB,
            ctrl,
            TRAJECTORY_COMMAND_TIME_FROM_START_SEC,
        )
        .unwrap_or_else(|e| {
            eprintln!("{}", e);
            std::process::exit(1);
        });
        publish_joint_trajectory_once(
            TOPIC_TRAJECTORY_INDEX,
            JOINT_INDEX,
            ctrl,
            TRAJECTORY_COMMAND_TIME_FROM_START_SEC,
        )
        .unwrap_or_else(|e| {
            eprintln!("{}", e);
            std::process::exit(1);
        });
        publish_joint_trajectory_once(
            TOPIC_TRAJECTORY_MRL,
            JOINT_MRL,
            ctrl,
            TRAJECTORY_COMMAND_TIME_FROM_START_SEC,
        )
        .unwrap_or_else(|e| {
            eprintln!("{}", e);
            std::process::exit(1);
        });
        println!("[iter {}] Published trajectory commands.", iter_idx);
    }
}

fn main() {
    let mode = parse_mode();
    println!("Mode: {:?}", mode);

    let now = Instant::now();
    let lut = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| FingerLUT::load(LUT_PATH)))
        .unwrap_or_else(|_| {
            eprintln!("Failed to load LUT file: {}", LUT_PATH);
            std::process::exit(1);
        });
    println!("LUT loaded: resolution={}", lut.get_resolution());

    let index_tip_local = lut.get_location(Contact::IndexTip, 0.0);
    println!(
        "Index tip local (open hand): ({:.4}, {:.4}, {:.4})",
        index_tip_local.x, index_tip_local.y, index_tip_local.z
    );

    let pred_config = PredictionConfig {
        t_max: HORIZON,
        n_samples: SAMPLES,
        ..Default::default()
    };

    println!("Setup time: {:.2?}", now.elapsed());

    match mode {
        Mode::Demo => {
            run_iteration(&lut, mode, &pred_config, 1);
            println!("[iter 1] Total time: {:.2?}", now.elapsed());
        }
        Mode::Normal => {
            let period = Duration::from_secs_f64(1.0 / FREQUENCY_HZ);
            let mut iter_idx: usize = 0;
            loop {
                iter_idx += 1;
                let tick_start = Instant::now();
                run_iteration(&lut, mode, &pred_config, iter_idx);
                let elapsed = tick_start.elapsed();
                println!("[iter {}] Total time: {:.2?}", iter_idx, elapsed);
                if elapsed < period {
                    thread::sleep(period - elapsed);
                } else {
                    eprintln!(
                        "Loop overrun: {:.2?} exceeds period {:.2?}",
                        elapsed, period
                    );
                }
            }
        }
        Mode::OneShot => {
            run_iteration(&lut, mode, &pred_config, 1);
            println!("[iter 1] Total time: {:.2?}", now.elapsed());
        }
    }
}
