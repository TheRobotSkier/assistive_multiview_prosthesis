mod lut_helper;
mod pointcloud_helper;

use std::time::Instant;
use std::thread;
use std::sync::Arc;
use std::sync::Mutex;

use lut_helper::{FingerLUT, FingerType};
use nalgebra::{Matrix4, Vector3};
use pointcloud_helper::{AabbMask, PointCloud, PointCloudProximityChecker, ProximityQuery};

fn main() {
    let now = Instant::now();
    // Load the LUT file
    let lut_path = "./data/finger_tip_lut.npz";
    let xyz_cloud_path = "./data/sphere.xyz";

    let lut = FingerLUT::load(lut_path).unwrap_or_else(|e| {
        eprintln!("Failed to load LUT file: {}", e);
        std::process::exit(1);
    });
    println!("\nLUT loaded with resolution: {}", lut.get_resolution());
    println!("Available fingers: {:?}", lut.get_available_fingers());

    // Load point cloud
    let pc = PointCloud::from_xyz_file(xyz_cloud_path).unwrap_or_else(|e| {
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

    // Base transform (identity for simplicity)
    let mut base_transform = Matrix4::identity();
    base_transform[(0, 3)] = -0.1;
    base_transform[(1, 3)] = -1.16;
    base_transform[(2, 3)] = 0.0;

    // AABB mask for proximity checking (optional)
    let aabb_mask = AabbMask {
        min: Vector3::new(-0.2, -0.2, -0.2),
        max: Vector3::new(0.2, 0.2, 0.2),
    };

    let collitions_tol = 0.005; // 5 mm tolerance for collision checking
    let closest_distance = Arc::new(Mutex::new(f64::INFINITY));

    println!("Time taken for setup: {:.2?}", now.elapsed());
    let now = Instant::now();

    // Compute proximity for each finger type in parallel
    let main_fingers = vec![FingerType::ThumbFlex, FingerType::Index, FingerType::Middle, FingerType::Ring, FingerType::Little];
    let preshape_main_fingers: Vec<_> = thread::scope(|s| {
        let mut handles = Vec::new();
        for i in 0..main_fingers.len() {
            let lut = &lut;
            let checker = &checker;
            let main_fingers = main_fingers.clone();
            let closest_distance = Arc::clone(&closest_distance);

            let base_transform = base_transform.clone();
            let aabb_mask = aabb_mask.clone();

            handles.push(s.spawn(move || {
                for sample in 0..lut.get_resolution() {
                    // Get a specific transform
                    let transform = if main_fingers[i] != FingerType::ThumbFlex {
                        match lut.get_transform_result(main_fingers[i], sample) {
                            Ok(transform) => transform,
                            Err(err) => {
                                eprintln!("{}", err);
                                continue;
                            }
                        }
                    } else {
                        // For thumb, we need to combine the thumb flex transform with the thumb abduction transform
                        match lut.combine_thumb_transforms(sample, 0) {
                            Ok(transform) => transform,
                            Err(err) => {
                                eprintln!("{}", err);
                                continue;
                            }
                        }
                    };

                    // Check proximity to point cloud
                    let query = ProximityQuery {
                        base_transform,
                        finger_transform: transform.matrix,
                        mask: None, // Some(aabb_mask.clone()),
                    };
                    let result = checker.nearest_distance(&query);
                    let distance = result.nearest_distance.unwrap_or(f64::INFINITY);
                    let mut closest = closest_distance.lock().unwrap();
                    if distance < *closest {
                        *closest = distance;
                    }
                    if distance < collitions_tol {
                        return Some(sample);
                    }
                    // println!("Finger {:?} sample {}: nearest distance = {:.4}", main_fingers[i], sample, distance);
                }
                None
            }));
        }
        // return collision index for each finger type
        let results: Vec<_> = handles.into_iter()
            .map(|h| h.join().unwrap())
            .collect();
        return results;
    });

    println!("Time taken for collision checking: {:.2?}", now.elapsed());
    println!("Closest distance found: {:.4} m", *closest_distance.lock().unwrap());
    println!("Collision indices for fingers: {:?}", preshape_main_fingers);    
}
