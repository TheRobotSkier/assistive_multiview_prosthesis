use nalgebra::Vector3;
use preshaping::lut_helper::FingerLUT;
use preshaping::planner::{compute_preshape, PlannerConfig};
use preshaping::pointcloud_helper::{AabbMask, PointCloud, PointCloudProximityChecker};
use preshaping::ros_command_helper::publish_float64_multi_array_once;
use std::env;
use std::process::Command;
use std::thread;
use std::time::Duration;
use std::time::Instant;

const TOPIC_THUMB: &str = "/thumb_pos_ff_controller/commands";
const TOPIC_INDEX: &str = "/index_pos_ff_controller/commands";
const TOPIC_MRL: &str = "/mrl_pos_ff_controller/commands";
const TOPIC_POINTCLOUD: &str = "/segmented_object_cloud";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PointCloudMode {
    File,
    Ros,
}

#[derive(Debug, Clone)]
struct CliArgs {
    mode: PointCloudMode,
    lut_path: String,
    xyz_cloud_path: String,
    pointcloud_topic: String,
    collision_tol: f64,
    frequency_hz: f64,
    iterations: usize,
    publish_commands: bool,
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
            collision_tol: 0.005,
            frequency_hz: 1.0,
            iterations: 1,
            publish_commands: false,
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
    println!("  --collision-tol VALUE  collision tolerance in meters (default: 0.005)");
    println!("  --frequency-hz VALUE   execution frequency in Hz (default: 1.0)");
    println!("  --iterations N         number of iterations (0 => run forever, default: 1)");
    println!("  --aabb xmin ymin zmin xmax ymax zmax");
    println!("                         optional AABB mask; if omitted full cloud is used");
    println!("  --offset-distal-proximal VALUE  finger-local X translation in meters");
    println!("  --offset-palmar-dorsal VALUE    finger-local Z translation in meters");
    println!("  --publish-commands     publish Float64MultiArray commands to MuJoCo topics");
}

fn read_ros_pointcloud(topic: &str) -> Result<PointCloud, String> {
    let output = Command::new("ros2")
        .arg("topic")
        .arg("echo")
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
    PointCloud::from_pointcloud2_yaml(&raw)
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
    planner_cfg.mask = cli.aabb_mask;
    planner_cfg.distal_proximal_offset = cli.distal_proximal_offset;
    planner_cfg.palmar_dorsal_offset = cli.palmar_dorsal_offset;

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
                eprintln!("Failed to load point cloud: {}, using synthetic cloud", e);
                PointCloud::new(vec![
                    Vector3::new(0.09, 0.02, 0.07),
                    Vector3::new(0.10, 0.01, 0.06),
                    Vector3::new(0.12, 0.04, 0.08),
                    Vector3::new(0.50, 0.50, 0.50),
                ])
            }),
            PointCloudMode::Ros => read_ros_pointcloud(&cli.pointcloud_topic).unwrap_or_else(|e| {
                eprintln!("Failed to read PointCloud2 from ROS topic: {}", e);
                std::process::exit(1);
            }),
        };

        println!("[iter {}] Point cloud loaded with {} points", iter_idx, pc.len());
        let checker = PointCloudProximityChecker::new(pc);

        let result = compute_preshape(&lut, &checker, &planner_cfg).unwrap_or_else(|e| {
            eprintln!("Planning failed: {}", e);
            std::process::exit(1);
        });

        println!(
            "[iter {}] Time taken for collision checking: {:.2?}",
            iter_idx,
            tick_start.elapsed()
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
            publish_float64_multi_array_once(TOPIC_THUMB, result.controls.thumb).unwrap_or_else(|e| {
                eprintln!("{}", e);
                std::process::exit(1);
            });
            publish_float64_multi_array_once(TOPIC_INDEX, result.controls.index).unwrap_or_else(|e| {
                eprintln!("{}", e);
                std::process::exit(1);
            });
            publish_float64_multi_array_once(TOPIC_MRL, result.controls.mrl).unwrap_or_else(|e| {
                eprintln!("{}", e);
                std::process::exit(1);
            });
            println!("Published commands to MuJoCo controller topics.");
        } else {
            println!("Dry-run only. Use --publish-commands to send commands.");
        }

        let elapsed = tick_start.elapsed();
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
