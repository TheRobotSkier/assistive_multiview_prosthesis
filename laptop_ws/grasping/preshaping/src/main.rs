mod lut_helper;
mod pointcloud_helper;

use std::path::Path;
use std::time::Instant;

use lut_helper::{FingerLUT, FingerType};
use nalgebra::{Matrix4, Vector3};
use pointcloud_helper::{AabbMask, PointCloud, PointCloudProximityChecker, ProximityQuery};

fn main() {
    // Load the LUT file
    let lut_path = "../finger_tip_lut.npz";
    let xyz_cloud_path = "../sphere.xyz";

    let now = Instant::now();

    match FingerLUT::load(lut_path) {
        Ok(lut) => {
            println!("Successfully loaded LUT file!");

            // Get resolution
            println!("Resolution: {}", lut.get_resolution());

            // List available fingers
            println!("Available fingers: {:?}", lut.get_available_fingers());

            // Get a specific transform
            if let Some(transform) = lut.get_transform(FingerType::Index, 0) {
                println!("\nIndex finger transform at sample 0:");
                println!("{:.4}", transform.matrix);
            }

            // Interpolate between samples
            if let Some(transform) = lut.interpolate_transform(FingerType::Index, 0.5) {
                println!("\nIndex finger interpolated transform at t=0.5:");
                println!("{:.4}", transform.matrix);

                let cloud = if Path::new(xyz_cloud_path).exists() {
                    match PointCloud::from_xyz_file(xyz_cloud_path) {
                        Ok(pc) => {
                            println!("Loaded point cloud from {}", xyz_cloud_path);
                            pc
                        }
                        Err(err) => {
                            eprintln!(
                                "Failed to load {} ({}), falling back to synthetic cloud",
                                xyz_cloud_path, err
                            );
                            PointCloud::new(vec![
                                Vector3::new(0.09, 0.02, 0.07),
                                Vector3::new(0.10, 0.01, 0.06),
                                Vector3::new(0.12, 0.04, 0.08),
                                Vector3::new(0.50, 0.50, 0.50),
                            ])
                        }
                    }
                } else {
                    PointCloud::new(vec![
                        Vector3::new(0.09, 0.02, 0.07),
                        Vector3::new(0.10, 0.01, 0.06),
                        Vector3::new(0.12, 0.04, 0.08),
                        Vector3::new(0.50, 0.50, 0.50),
                    ])
                };
                let checker = PointCloudProximityChecker::new(cloud);

                let mut base_transform = Matrix4::identity();
                base_transform[(0, 3)] = 0.0;
                base_transform[(1, 3)] = 0.0;
                base_transform[(2, 3)] = 0.0;

                let query = ProximityQuery {
                    base_transform,
                    finger_transform: transform.matrix,
                    mask: None//Some(AabbMask {min: Vector3::new(-0.2, -0.2, -0.2),max: Vector3::new(0.2, 0.2, 0.2),}),
                };

                let result = checker.nearest_distance(&query);
                println!("\nProximity query result:");
                println!("World tip position: {:?}", result.world_tip);
                println!("Candidates checked: {}", result.candidates_checked);
                match result.nearest_distance {
                    Some(distance) => println!("Nearest point distance: {:.6} m", distance),
                    None => println!("No points found inside AABB mask"),
                }
            }

            // Combine thumb transforms
            let combined = lut.combine_thumb_transforms(5, 5);
            println!("\nCombined thumb transform (flex=5, opp=5):");
            println!("{:.4}", combined.matrix);

            println!("\nTime taken: {:.2?}", now.elapsed());
        }
        Err(e) => {
            eprintln!("Failed to load LUT file: {}", e);
        }
    }
}
