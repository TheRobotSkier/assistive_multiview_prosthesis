use nalgebra::{Matrix4, Quaternion, UnitQuaternion, Vector3};
use preshaping::lut_helper::FingerLUT;
use preshaping::planner::{compute_preshape, PlannerConfig, PreshapeResult};
use preshaping::pointcloud_helper::{AabbMask, PointCloud, PointCloudProximityChecker};
use preshaping::ros_command_helper::{
    publish_float64_multi_array_once,
    publish_joint_trajectory_once,
};
use std::env;
use std::process::Command;
use std::thread;
use std::time::Duration;
use std::time::Instant;

const TOPIC_POS_FF_THUMB: &str = "/thumb_pos_ff_controller/commands";
const TOPIC_POS_FF_INDEX: &str = "/index_pos_ff_controller/commands";
const TOPIC_POS_FF_MRL: &str = "/mrl_pos_ff_controller/commands";

const TOPIC_TRAJECTORY_THUMB: &str = "/thumb_trajectory_controller/joint_trajectory";
const TOPIC_TRAJECTORY_INDEX: &str = "/index_trajectory_controller/joint_trajectory";
const TOPIC_TRAJECTORY_MRL: &str = "/mrl_trajectory_controller/joint_trajectory";

const JOINT_THUMB: &str = "j_thumb_fle";
const JOINT_INDEX: &str = "j_index_fle";
const JOINT_MRL: &str = "j_mrl_fle";

const TRAJECTORY_COMMAND_TIME_FROM_START_SEC: f64 = 1.0;
const TOPIC_POINTCLOUD: &str = "/segmented_object_cloud";

const DEFAULT_MUJOCO_RIGHT_HAND_POS_X: f64 = -0.1;
const DEFAULT_MUJOCO_RIGHT_HAND_POS_Y: f64 = 0.0;
const DEFAULT_MUJOCO_RIGHT_HAND_POS_Z: f64 = 0.2;
const DEFAULT_MUJOCO_RIGHT_HAND_QUAT_W: f64 = 0.707388;
const DEFAULT_MUJOCO_RIGHT_HAND_QUAT_X: f64 = 0.706825;
const DEFAULT_MUJOCO_RIGHT_HAND_QUAT_Y: f64 = 0.0;
const DEFAULT_MUJOCO_RIGHT_HAND_QUAT_Z: f64 = 0.0;

const DEFAULT_CUSTOM_SCENE_OBJECT_POS_X: f64 = -0.1;
const DEFAULT_CUSTOM_SCENE_OBJECT_POS_Y: f64 = -0.049_912_4;
const DEFAULT_CUSTOM_SCENE_OBJECT_POS_Z: f64 = 0.310_039_8;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PointCloudMode {
    File,
    Ros,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CommandBackend {
    Trajectory,
    PosFf,
}

#[derive(Debug, Clone)]
struct CliArgs {
    mode: PointCloudMode,
    lut_path: String,
    xyz_cloud_path: String,
    pointcloud_topic: String,
    pointcloud_scale: f64,
    collision_tol: f64,
    frequency_hz: f64,
    iterations: usize,
    publish_commands: bool,
    command_backend: CommandBackend,
    search_base_transform: bool,
    base_search_step: f64,
    base_search_span: f64,
    aabb_mask: Option<AabbMask>,
    distal_proximal_offset: f64,
    palmar_dorsal_offset: f64,
}

impl Default for CliArgs {
    fn default() -> Self {
        Self {
            mode: PointCloudMode::File,
            lut_path: "./data/finger_tip_lut.npz".to_string(),
            xyz_cloud_path: "./data/sphere.xyz".to_string(),
            pointcloud_topic: TOPIC_POINTCLOUD.to_string(),
            pointcloud_scale: 0.03,
            collision_tol: 0.005,
            frequency_hz: 1.0,
            iterations: 1,
            publish_commands: false,
            command_backend: CommandBackend::Trajectory,
            search_base_transform: false,
            base_search_step: 0.01,
            base_search_span: 0.2,
            aabb_mask: None,
            distal_proximal_offset: 0.0,
            palmar_dorsal_offset: 0.0,
        }
    }
}

fn parse_cli_args() -> Result<CliArgs, String> {
    let mut cfg = CliArgs::default();
    let mut iter = env::args().skip(1);

    while let Some(arg) = iter.next() {
        match arg.as_str() {
            "--mode" => {
                let raw = iter
                    .next()
                    .ok_or_else(|| "Missing value for --mode".to_string())?;
                cfg.mode = match raw.as_str() {
                    "file" => PointCloudMode::File,
                    "ros" => PointCloudMode::Ros,
                    _ => {
                        return Err(format!(
                            "Invalid --mode value '{}'. Supported: file|ros",
                            raw
                        ));
                    }
                };
            }
            "--lut" => {
                cfg.lut_path = iter
                    .next()
                    .ok_or_else(|| "Missing value for --lut".to_string())?;
            }
            "--cloud" => {
                cfg.xyz_cloud_path = iter
                    .next()
                    .ok_or_else(|| "Missing value for --cloud".to_string())?;
            }
            "--collision-tol" => {
                let raw = iter
                    .next()
                    .ok_or_else(|| "Missing value for --collision-tol".to_string())?;
                cfg.collision_tol = raw
                    .parse::<f64>()
                    .map_err(|e| format!("Invalid --collision-tol value {}: {}", raw, e))?;
            }
            "--pointcloud-topic" => {
                cfg.pointcloud_topic = iter
                    .next()
                    .ok_or_else(|| "Missing value for --pointcloud-topic".to_string())?;
            }
            "--pc-scale" | "--pointcloud-scale" => {
                let raw = iter
                    .next()
                    .ok_or_else(|| "Missing value for --pc-scale".to_string())?;
                cfg.pointcloud_scale = raw
                    .parse::<f64>()
                    .map_err(|e| format!("Invalid --pc-scale value {}: {}", raw, e))?;
            }
            "--frequency-hz" => {
                let raw = iter
                    .next()
                    .ok_or_else(|| "Missing value for --frequency-hz".to_string())?;
                cfg.frequency_hz = raw
                    .parse::<f64>()
                    .map_err(|e| format!("Invalid --frequency-hz value {}: {}", raw, e))?;
            }
            "--iterations" => {
                let raw = iter
                    .next()
                    .ok_or_else(|| "Missing value for --iterations".to_string())?;
                cfg.iterations = raw
                    .parse::<usize>()
                    .map_err(|e| format!("Invalid --iterations value {}: {}", raw, e))?;
            }
            "--publish-commands" => {
                cfg.publish_commands = true;
            }
            "--command-backend" => {
                let raw = iter
                    .next()
                    .ok_or_else(|| "Missing value for --command-backend".to_string())?;
                cfg.command_backend = match raw.as_str() {
                    "trajectory" => CommandBackend::Trajectory,
                    "pos_ff" | "pos-ff" => CommandBackend::PosFf,
                    _ => {
                        return Err(format!(
                            "Invalid --command-backend value '{}'. Supported: trajectory|pos_ff",
                            raw
                        ));
                    }
                };
            }
            "--search-base-transform" => {
                cfg.search_base_transform = true;
            }
            "--base-search-step" => {
                cfg.base_search_step = parse_next_f64(&mut iter, "--base-search-step")?;
            }
            "--base-search-span" => {
                cfg.base_search_span = parse_next_f64(&mut iter, "--base-search-span")?;
            }
            "--aabb" => {
                let xmin = parse_next_f64(&mut iter, "--aabb xmin")?;
                let ymin = parse_next_f64(&mut iter, "--aabb ymin")?;
                let zmin = parse_next_f64(&mut iter, "--aabb zmin")?;
                let xmax = parse_next_f64(&mut iter, "--aabb xmax")?;
                let ymax = parse_next_f64(&mut iter, "--aabb ymax")?;
                let zmax = parse_next_f64(&mut iter, "--aabb zmax")?;

                cfg.aabb_mask = Some(AabbMask {
                    min: Vector3::new(xmin, ymin, zmin),
                    max: Vector3::new(xmax, ymax, zmax),
                });
            }
            "--offset-distal-proximal" => {
                cfg.distal_proximal_offset = parse_next_f64(&mut iter, "--offset-distal-proximal")?;
            }
            "--offset-palmar-dorsal" => {
                cfg.palmar_dorsal_offset = parse_next_f64(&mut iter, "--offset-palmar-dorsal")?;
            }
            "--help" | "-h" => {
                print_usage();
                std::process::exit(0);
            }
            unknown => {
                return Err(format!("Unknown argument: {}", unknown));
            }
        }
    }

    if cfg.frequency_hz <= 0.0 {
        return Err("--frequency-hz must be > 0".to_string());
    }

    if !cfg.pointcloud_scale.is_finite() || cfg.pointcloud_scale <= 0.0 {
        return Err("--pc-scale must be a finite value > 0".to_string());
    }

    if !cfg.base_search_step.is_finite() || cfg.base_search_step <= 0.0 {
        return Err("--base-search-step must be a finite value > 0".to_string());
    }

    if !cfg.base_search_span.is_finite() || cfg.base_search_span < 0.0 {
        return Err("--base-search-span must be a finite value >= 0".to_string());
    }

    if let Some(mask) = cfg.aabb_mask {
        if mask.min.x > mask.max.x || mask.min.y > mask.max.y || mask.min.z > mask.max.z {
            return Err(
                "--aabb requires xmin <= xmax, ymin <= ymax, and zmin <= zmax".to_string(),
            );
        }
    }

    Ok(cfg)
}

fn parse_next_f64(
    iter: &mut impl Iterator<Item = String>,
    label: &str,
) -> Result<f64, String> {
    let raw = iter
        .next()
        .ok_or_else(|| format!("Missing value for {}", label))?;
    raw.parse::<f64>()
        .map_err(|e| format!("Invalid value for {} ({}): {}", label, raw, e))
}

fn print_usage() {
    println!("Usage: cargo run -- [options]");
    println!("  --mode file|ros        point cloud source mode (default: file)");
    println!("  --lut PATH             LUT file (default: ./data/finger_tip_lut.npz)");
    println!("  --cloud PATH           point cloud .xyz file (default: ./data/sphere.xyz)");
    println!("  --pointcloud-topic TOPIC point cloud topic for --mode ros (default: /segmented_object_cloud)");
    println!("  --pc-scale VALUE       isotropic point cloud scale factor (default: 0.03)");
    println!("  --collision-tol VALUE  collision tolerance in meters (default: 0.005)");
    println!("  --frequency-hz VALUE   execution frequency in Hz (default: 1.0)");
    println!("  --iterations N         number of iterations (0 => run forever, default: 1)");
    println!("  --command-backend trajectory|pos_ff");
    println!("                         controller command target when --publish-commands is used (default: trajectory)");
    println!("  Default hand pose matches mia_hand_mujoco/mia_hand/mia_hand_right.xml:");
    println!("    pos=({:.3}, {:.3}, {:.3}), quat(wxyz)=({:.6}, {:.6}, {:.6}, {:.6})",
        DEFAULT_MUJOCO_RIGHT_HAND_POS_X,
        DEFAULT_MUJOCO_RIGHT_HAND_POS_Y,
        DEFAULT_MUJOCO_RIGHT_HAND_POS_Z,
        DEFAULT_MUJOCO_RIGHT_HAND_QUAT_W,
        DEFAULT_MUJOCO_RIGHT_HAND_QUAT_X,
        DEFAULT_MUJOCO_RIGHT_HAND_QUAT_Y,
        DEFAULT_MUJOCO_RIGHT_HAND_QUAT_Z,
    );
    println!("  --search-base-transform run a 3D grid search over base translation around the default MuJoCo hand pose");
    println!("  --base-search-step VALUE  translation increment in meters (default: 0.01)");
    println!("  --base-search-span VALUE  search span in each axis, [-span,+span] (default: 0.2)");
    println!("  --aabb xmin ymin zmin xmax ymax zmax");
    println!("                         limit collision checks to points inside the axis-aligned box");
    println!("  --offset-distal-proximal VALUE  finger-local X translation in meters");
    println!("  --offset-palmar-dorsal VALUE    finger-local Z translation in meters");
    println!("  --publish-commands     publish commands to the selected backend topics");
}

fn default_mujoco_right_hand_base_transform() -> Matrix4<f64> {
    let rotation = UnitQuaternion::new_normalize(Quaternion::new(
        DEFAULT_MUJOCO_RIGHT_HAND_QUAT_W,
        DEFAULT_MUJOCO_RIGHT_HAND_QUAT_X,
        DEFAULT_MUJOCO_RIGHT_HAND_QUAT_Y,
        DEFAULT_MUJOCO_RIGHT_HAND_QUAT_Z,
    ));

    let mut transform = rotation.to_homogeneous();
    transform[(0, 3)] = DEFAULT_MUJOCO_RIGHT_HAND_POS_X;
    transform[(1, 3)] = DEFAULT_MUJOCO_RIGHT_HAND_POS_Y;
    transform[(2, 3)] = DEFAULT_MUJOCO_RIGHT_HAND_POS_Z;
    transform
}

fn default_custom_scene_object_transform() -> Matrix4<f64> {
    let mut transform = Matrix4::identity();
    transform[(0, 3)] = DEFAULT_CUSTOM_SCENE_OBJECT_POS_X;
    transform[(1, 3)] = DEFAULT_CUSTOM_SCENE_OBJECT_POS_Y;
    transform[(2, 3)] = DEFAULT_CUSTOM_SCENE_OBJECT_POS_Z;
    transform
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
            "ros2 topic echo failed on topic '{}' with code {:?}",
            topic,
            output.status.code()
        ));
    }

    let raw = String::from_utf8(output.stdout)
        .map_err(|e| format!("PointCloud2 output is not valid UTF-8: {}", e))?;
    let normalized = raw
        .lines()
        .skip_while(|line| {
            let trimmed = line.trim();
            trimmed.is_empty() || trimmed == "---" || trimmed == "..."
        })
        .take_while(|line| {
            let trimmed = line.trim();
            trimmed != "---" && trimmed != "..."
        })
        .collect::<Vec<_>>()
        .join("\n");

    PointCloud::from_pointcloud2_yaml(&normalized)
}

fn count_collisions(result: &PreshapeResult) -> usize {
    [
        result.thumb_sample,
        result.index_sample,
        result.middle_sample,
        result.ring_sample,
        result.little_sample,
    ]
    .iter()
    .filter(|sample| sample.is_some())
    .count()
}

fn select_better_result(
    best: &(Matrix4<f64>, PreshapeResult),
    candidate: &(Matrix4<f64>, PreshapeResult),
) -> bool {
    let best_collisions = count_collisions(&best.1);
    let candidate_collisions = count_collisions(&candidate.1);
    if candidate_collisions != best_collisions {
        return candidate_collisions > best_collisions;
    }

    if (candidate.1.closest_distance - best.1.closest_distance).abs() > f64::EPSILON {
        return candidate.1.closest_distance < best.1.closest_distance;
    }

    let best_norm_sq = best.0[(0, 3)].powi(2) + best.0[(1, 3)].powi(2) + best.0[(2, 3)].powi(2);
    let candidate_norm_sq =
        candidate.0[(0, 3)].powi(2) + candidate.0[(1, 3)].powi(2) + candidate.0[(2, 3)].powi(2);
    candidate_norm_sq < best_norm_sq
}

fn search_best_base_transform(
    lut: &FingerLUT,
    checker: &PointCloudProximityChecker,
    planner_cfg: &PlannerConfig,
    step: f64,
    span: f64,
) -> Result<(Matrix4<f64>, PreshapeResult, usize), String> {
    let cells_per_axis = (span / step).floor() as isize;
    let mut best: Option<(Matrix4<f64>, PreshapeResult)> = None;
    let mut evaluated = 0usize;

    for ix in -cells_per_axis..=cells_per_axis {
        for iy in -cells_per_axis..=cells_per_axis {
            for iz in -cells_per_axis..=cells_per_axis {
                let tx = ix as f64 * step;
                let ty = iy as f64 * step;
                let tz = iz as f64 * step;

                let mut cfg = planner_cfg.clone();
                let mut tf = planner_cfg.base_transform;
                tf[(0, 3)] += tx;
                tf[(1, 3)] += ty;
                tf[(2, 3)] += tz;
                cfg.base_transform = tf;

                let result = compute_preshape(lut, checker, &cfg)
                    .map_err(|e| format!("Grid-search planning failed: {}", e))?;
                let candidate = (tf, result);
                evaluated += 1;

                if let Some(current_best) = &best {
                    if select_better_result(current_best, &candidate) {
                        best = Some(candidate);
                    }
                } else {
                    best = Some(candidate);
                }
            }
        }
    }

    let (best_tf, best_result) = best.ok_or_else(|| "Grid search had no candidates".to_string())?;
    Ok((best_tf, best_result, evaluated))
}

fn main() {
    let cli = parse_cli_args().unwrap_or_else(|e| {
        eprintln!("{}", e);
        print_usage();
        std::process::exit(2);
    });

    let now = Instant::now();
    let lut = FingerLUT::load(&cli.lut_path).unwrap_or_else(|e| {
        eprintln!("Failed to load LUT file: {}", e);
        std::process::exit(1);
    });
    println!("\nLUT loaded with resolution: {}", lut.get_resolution());
    println!("Available fingers: {:?}", lut.get_available_fingers());

    let mut planner_cfg = PlannerConfig::default();
    planner_cfg.collision_tol = cli.collision_tol;
    if let Some(mask) = cli.aabb_mask {
        println!(
            "Using AABB mask: min=({:.4}, {:.4}, {:.4}), max=({:.4}, {:.4}, {:.4})",
            mask.min.x,
            mask.min.y,
            mask.min.z,
            mask.max.x,
            mask.max.y,
            mask.max.z,
        );
    }
    planner_cfg.mask = cli.aabb_mask;
    planner_cfg.distal_proximal_offset = cli.distal_proximal_offset;
    planner_cfg.palmar_dorsal_offset = cli.palmar_dorsal_offset;
    planner_cfg.base_transform = default_mujoco_right_hand_base_transform();

    println!(
        "Using hardcoded MuJoCo default hand pose: pos=({:.3}, {:.3}, {:.3}), quat(wxyz)=({:.6}, {:.6}, {:.6}, {:.6})",
        DEFAULT_MUJOCO_RIGHT_HAND_POS_X,
        DEFAULT_MUJOCO_RIGHT_HAND_POS_Y,
        DEFAULT_MUJOCO_RIGHT_HAND_POS_Z,
        DEFAULT_MUJOCO_RIGHT_HAND_QUAT_W,
        DEFAULT_MUJOCO_RIGHT_HAND_QUAT_X,
        DEFAULT_MUJOCO_RIGHT_HAND_QUAT_Y,
        DEFAULT_MUJOCO_RIGHT_HAND_QUAT_Z,
    );

    if cli.publish_commands {
        println!(
            "Command publishing backend: {}",
            match cli.command_backend {
                CommandBackend::Trajectory => "trajectory",
                CommandBackend::PosFf => "pos_ff",
            }
        );
    }

    println!("Time taken for setup: {:.2?}", now.elapsed());
    let period = Duration::from_secs_f64(1.0 / cli.frequency_hz);
    let run_forever = cli.iterations == 0;
    let mut iter_idx: usize = 0;

    loop {
        if !run_forever && iter_idx >= cli.iterations {
            break;
        }
        iter_idx += 1;

        let tick_start = Instant::now();

        let pc = match cli.mode {
            PointCloudMode::File => PointCloud::from_xyz_file(&cli.xyz_cloud_path).unwrap_or_else(|e| {
                eprintln!(
                    "Failed to load point cloud from '{}': {}",
                    cli.xyz_cloud_path,
                    e
                );
                std::process::exit(1);
            }),
            PointCloudMode::Ros => read_ros_pointcloud(&cli.pointcloud_topic).unwrap_or_else(|e| {
                eprintln!("Failed to read PointCloud2 from ROS topic: {}", e);
                std::process::exit(1);
            }),
        };

        let pc = pc.scaled(cli.pointcloud_scale);
        let pc = match cli.mode {
            PointCloudMode::File => {
                let transform = default_custom_scene_object_transform();
                println!(
                    "[iter {}] Applying fixed file-cloud world translation: x={:.4}, y={:.4}, z={:.4}",
                    iter_idx,
                    transform[(0, 3)],
                    transform[(1, 3)],
                    transform[(2, 3)],
                );
                pc.transformed(&transform)
            }
            PointCloudMode::Ros => pc,
        };

        println!(
            "[iter {}] Point cloud loaded with {} points (scale {:.6})",
            iter_idx,
            pc.len(),
            cli.pointcloud_scale
        );
        let checker = PointCloudProximityChecker::new(pc);

        let collision_start = Instant::now();

        let (active_base_tf, result, evaluated_candidates) = if cli.search_base_transform {
            search_best_base_transform(
                &lut,
                &checker,
                &planner_cfg,
                cli.base_search_step,
                cli.base_search_span,
            )
            .map(|(tf, res, n)| (tf, res, n))
            .unwrap_or_else(|e| {
                eprintln!("Base-transform grid search failed: {}", e);
                std::process::exit(1);
            })
        } else {
            let res = compute_preshape(&lut, &checker, &planner_cfg).unwrap_or_else(|e| {
                eprintln!("Planning failed: {}", e);
                std::process::exit(1);
            });
            (planner_cfg.base_transform, res, 1)
        };

        println!(
            "[iter {}] Time taken for collision checking: {:.2?}",
            iter_idx,
            collision_start.elapsed()
        );
        println!(
            "[iter {}] Base transform search candidates evaluated: {}",
            iter_idx, evaluated_candidates
        );
        println!(
            "[iter {}] Active base translation: x={:.4}, y={:.4}, z={:.4}",
            iter_idx,
            active_base_tf[(0, 3)],
            active_base_tf[(1, 3)],
            active_base_tf[(2, 3)]
        );
        println!(
            "[iter {}] Finger collisions in best result: {}/5",
            iter_idx,
            count_collisions(&result)
        );
        println!("Closest distance found: {:.4} m", result.closest_distance);
        println!(
            "Collision samples: thumb={:?}, index={:?}, middle={:?}, ring={:?}, little={:?}",
            result.thumb_sample,
            result.index_sample,
            result.middle_sample,
            result.ring_sample,
            result.little_sample
        );
        println!(
            "AABB mask active: {}",
            if result.used_aabb_mask { "yes" } else { "no (full cloud)" }
        );
        println!(
            "Selected controls: thumb={:.4}, index={:.4}, mrl={:.4}",
            result.controls.thumb,
            result.controls.index,
            result.controls.mrl
        );

        if cli.publish_commands {
            match cli.command_backend {
                CommandBackend::Trajectory => {
                    publish_joint_trajectory_once(
                        TOPIC_TRAJECTORY_THUMB,
                        JOINT_THUMB,
                        result.controls.thumb,
                        TRAJECTORY_COMMAND_TIME_FROM_START_SEC,
                    )
                    .unwrap_or_else(|e| {
                        eprintln!("{}", e);
                        std::process::exit(1);
                    });
                    publish_joint_trajectory_once(
                        TOPIC_TRAJECTORY_INDEX,
                        JOINT_INDEX,
                        result.controls.index,
                        TRAJECTORY_COMMAND_TIME_FROM_START_SEC,
                    )
                    .unwrap_or_else(|e| {
                        eprintln!("{}", e);
                        std::process::exit(1);
                    });
                    publish_joint_trajectory_once(
                        TOPIC_TRAJECTORY_MRL,
                        JOINT_MRL,
                        result.controls.mrl,
                        TRAJECTORY_COMMAND_TIME_FROM_START_SEC,
                    )
                    .unwrap_or_else(|e| {
                        eprintln!("{}", e);
                        std::process::exit(1);
                    });
                    println!("Published trajectory commands to controller topics.");
                }
                CommandBackend::PosFf => {
                    publish_float64_multi_array_once(TOPIC_POS_FF_THUMB, result.controls.thumb)
                        .unwrap_or_else(|e| {
                            eprintln!("{}", e);
                            std::process::exit(1);
                        });
                    publish_float64_multi_array_once(TOPIC_POS_FF_INDEX, result.controls.index)
                        .unwrap_or_else(|e| {
                            eprintln!("{}", e);
                            std::process::exit(1);
                        });
                    publish_float64_multi_array_once(TOPIC_POS_FF_MRL, result.controls.mrl)
                        .unwrap_or_else(|e| {
                            eprintln!("{}", e);
                            std::process::exit(1);
                        });
                    println!("Published pos_ff commands to controller topics.");
                }
            }
        } else {
            println!("Dry-run only. Use --publish-commands to send commands.");
        }

        let elapsed = tick_start.elapsed();
        println!(
            "[iter {}] Total time for iteration: {:.2?}",
            iter_idx, elapsed
        );
        if elapsed < period {
            thread::sleep(period - elapsed);
        } else {
            eprintln!(
                "Loop overrun: compute took {:.2?} which exceeds period {:.2?}",
                elapsed,
                period
            );
        }
    }
}
