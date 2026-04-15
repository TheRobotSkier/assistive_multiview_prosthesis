use crate::lut_helper::{FingerLUT, Contact};
use crate::pointcloud_helper::Tsdf;
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
pub struct ScoreResult {
    pub alignment_score: f64,
    pub force_closure_score: f64,
    pub closure_amount: f64,
}

pub fn get_score(
    lut: &FingerLUT,
    checker: &Tsdf,
    base_transform: &Matrix4<f64>,
) -> ScoreResult {
    // this is where i got
    let closest_distance = Arc::new(Mutex::new(f32::INFINITY));

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
