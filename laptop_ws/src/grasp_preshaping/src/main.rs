use nalgebra::Vector3;
use preshaping::lut_helper::FingerLUT;
use preshaping::planner::{compute_preshape, PlannerConfig};
use preshaping::pointcloud_helper::{AabbMask, PointCloud, PointCloudProximityChecker};
use preshaping::ros_command_helper::publish_float64_multi_array_once;
use std::env;
use std::time::Instant;

const TOPIC_THUMB: &str = "/thumb_pos_ff_controller/commands";
const TOPIC_INDEX: &str = "/index_pos_ff_controller/commands";
const TOPIC_MRL: &str = "/mrl_pos_ff_controller/commands";

#[derive(Debug, Clone)]
struct CliArgs {
    lut_path: String,
    xyz_cloud_path: String,
    collision_tol: f64,
    publish_commands: bool,
    aabb_mask: Option<AabbMask>,
}

impl Default for CliArgs {
    fn default() -> Self {
        Self {
            lut_path: "./data/finger_tip_lut.npz".to_string(),
            xyz_cloud_path: "./data/sphere.xyz".to_string(),
            collision_tol: 0.005,
            publish_commands: false,
            aabb_mask: None,
        }
    }
}

fn parse_cli_args() -> Result<CliArgs, String> {
    let mut cfg = CliArgs::default();
    let mut iter = env::args().skip(1);

    while let Some(arg) = iter.next() {
        match arg.as_str() {
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
            "--help" | "-h" => {
                print_usage();
                std::process::exit(0);
            }
            unknown => {
                return Err(format!("Unknown argument: {}", unknown));
            }
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
    println!("  --lut PATH             LUT file (default: ./data/finger_tip_lut.npz)");
    println!("  --cloud PATH           point cloud .xyz file (default: ./data/sphere.xyz)");
    println!("  --collision-tol VALUE  collision tolerance in meters (default: 0.005)");
    println!("  --aabb xmin ymin zmin xmax ymax zmax");
    println!("                         optional AABB mask; if omitted full cloud is used");
    println!("  --publish-commands     publish Float64MultiArray commands to MuJoCo topics");
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

    // Load point cloud
    let pc = PointCloud::from_xyz_file(&cli.xyz_cloud_path).unwrap_or_else(|e| {
        eprintln!("Failed to load point cloud: {}, using synthetic cloud", e);
        PointCloud::new(vec![
            Vector3::new(0.09, 0.02, 0.07),
            Vector3::new(0.10, 0.01, 0.06),
            Vector3::new(0.12, 0.04, 0.08),
            Vector3::new(0.50, 0.50, 0.50),
        ])
    });
    println!("Point cloud loaded with {} points", pc.len());
    let checker = PointCloudProximityChecker::new(pc);

    let mut planner_cfg = PlannerConfig::default();
    planner_cfg.collision_tol = cli.collision_tol;
    planner_cfg.mask = cli.aabb_mask;

    println!("Time taken for setup: {:.2?}", now.elapsed());
    let now = Instant::now();

    let result = compute_preshape(&lut, &checker, &planner_cfg).unwrap_or_else(|e| {
        eprintln!("Planning failed: {}", e);
        std::process::exit(1);
    });

    println!("Time taken for collision checking: {:.2?}", now.elapsed());
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

}
