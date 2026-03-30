use crate::lut_helper::{FingerLUT, FingerType, LutError};
use crate::pointcloud_helper::{AabbMask, PointCloudProximityChecker, ProximityQuery};
use nalgebra::Matrix4;
use std::sync::{Arc, Mutex};
use std::thread;

const ORDERED_FINGERS: [FingerType; 5] = [
    FingerType::ThumbFlex,
    FingerType::Index,
    FingerType::Middle,
    FingerType::Ring,
    FingerType::Little,
];

#[derive(Debug, Clone)]
pub struct PlannerConfig {
    pub base_transform: Matrix4<f64>,
    pub collision_tol: f64,
    pub thumb_opp_sample: usize,
    pub mask: Option<AabbMask>,
    pub distal_proximal_offset: f64,
    pub palmar_dorsal_offset: f64,
}

impl Default for PlannerConfig {
    fn default() -> Self {
        Self {
            base_transform: Matrix4::identity(),
            collision_tol: 0.005,
            thumb_opp_sample: 0,
            mask: None,
            distal_proximal_offset: 0.0,
            palmar_dorsal_offset: 0.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PreshapeControls {
    pub thumb: f64,
    pub index: f64,
    pub mrl: f64,
}

#[derive(Debug, Clone)]
pub struct PreshapeResult {
    pub thumb_sample: Option<usize>,
    pub index_sample: Option<usize>,
    pub middle_sample: Option<usize>,
    pub ring_sample: Option<usize>,
    pub little_sample: Option<usize>,
    pub controls: PreshapeControls,
    pub closest_distance: f64,
    pub used_aabb_mask: bool,
}

pub fn compute_preshape(
    lut: &FingerLUT,
    checker: &PointCloudProximityChecker,
    config: &PlannerConfig,
) -> Result<PreshapeResult, LutError> {
    let closest_distance = Arc::new(Mutex::new(f64::INFINITY));
    let finger_offset_tf = make_finger_offset_transform(config);

    let collisions: Vec<Option<usize>> = thread::scope(|scope| {
        let mut handles = Vec::new();

        for finger in ORDERED_FINGERS {
            let closest_distance = Arc::clone(&closest_distance);
            let base_transform = config.base_transform;
            let mask = config.mask;

            handles.push(scope.spawn(move || {
                for sample in 0..lut.get_resolution() {
                    let transform = if finger != FingerType::ThumbFlex {
                        match lut.get_transform_result(finger, sample) {
                            Ok(transform) => transform,
                            Err(err) => return Err(err),
                        }
                    } else {
                        match lut.combine_thumb_transforms(sample, config.thumb_opp_sample) {
                            Ok(transform) => transform,
                            Err(err) => return Err(err),
                        }
                    };

                    let query = ProximityQuery {
                        base_transform,
                        // Apply the same fingertip offset in finger local frame once for all fingers.
                        // For thumb this happens after thumb flex+opposition composition, so it is not double-applied.
                        finger_transform: transform.matrix * finger_offset_tf,
                        mask,
                    };

                    let result = checker.nearest_distance(&query);
                    let distance = result.nearest_distance.unwrap_or(f64::INFINITY);

                    let mut closest = closest_distance.lock().expect("closest mutex poisoned");
                    if distance < *closest {
                        *closest = distance;
                    }

                    if distance < config.collision_tol {
                        return Ok(Some(sample));
                    }
                }

                Ok(None)
            }));
        }

        let mut out = Vec::with_capacity(handles.len());
        for handle in handles {
            out.push(handle.join().expect("planner worker panicked")?);
        }
        Ok::<Vec<Option<usize>>, LutError>(out)
    })?;

    let thumb_sample = collisions[0];
    let index_sample = collisions[1];
    let middle_sample = collisions[2];
    let ring_sample = collisions[3];
    let little_sample = collisions[4];

    let mrl_sample = [middle_sample, ring_sample, little_sample]
        .iter()
        .filter_map(|v| *v)
        .min()
        .unwrap_or(0);

    let controls = PreshapeControls {
        thumb: sample_to_control(thumb_sample.unwrap_or(0), lut.get_resolution()) * 1.134, // scale to 0-100% range of thumb flexion
        index: sample_to_control(index_sample.unwrap_or(0), lut.get_resolution()) * -1.4, // invert index control and scale to 0-100% range
        mrl: sample_to_control(mrl_sample, lut.get_resolution()) * 1.396, // scale to 0-100% range of mrl fingers
    };

    Ok(PreshapeResult {
        thumb_sample,
        index_sample,
        middle_sample,
        ring_sample,
        little_sample,
        controls,
        closest_distance: *closest_distance
            .lock()
            .expect("closest mutex poisoned while finalizing"),
        used_aabb_mask: config.mask.is_some(),
    })
}

fn sample_to_control(sample: usize, resolution: usize) -> f64 {
    if resolution <= 1 {
        return 0.0;
    }
    sample as f64 / (resolution - 1) as f64
}

fn make_finger_offset_transform(config: &PlannerConfig) -> Matrix4<f64> {
    let mut tf = Matrix4::identity();
    // Local finger axes convention used here:
    // +X: distal/proximal axis, +Z: palmar/dorsal axis.
    tf[(0, 3)] = config.distal_proximal_offset;
    tf[(2, 3)] = config.palmar_dorsal_offset;
    tf
}
