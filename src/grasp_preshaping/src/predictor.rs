use crate::config;
use crate::lut_helper::DualQuaternion;
use crate::pointcloud_helper::Aabb;
use nalgebra::{Matrix3, Matrix4, UnitQuaternion, Vector3};
use rand::Rng;
use rand_distr::Normal;


#[derive(Debug, Clone)]
pub struct SampledPose {
    pub pose: DualQuaternion,
    pub sample_probability: f64,
    /// Randomly assigned grasp type: 0 = cylindrical, 1 = pinch, 2 = lateral.
    pub grasp_type: usize,
    /// Wrist rotation angle around local Y axis in radians, in [-WRIST_ROTATION_RANGE_RAD, WRIST_ROTATION_RANGE_RAD].
    pub wrist_rotation: f64,
}

pub struct Twist6 {
    pub omega: Vector3<f64>,
    pub v: Vector3<f64>,
}

#[derive(Debug, Clone)]
pub struct TwistCovariance {
    pub diagonal: Vector3<f64>,
    pub diagonal_v: Vector3<f64>,
}

#[derive(Debug, Clone)]
pub struct PredictionConfig {
    pub t_max: f64,
    pub n_samples: usize,
    pub hand_radius: f64,
    pub min_tsdf_dims: Vector3<f64>,
    pub max_tsdf_dims: Vector3<f64>,
}

impl TwistCovariance {
    /// Construct from the fixed constants in config.rs.
    pub fn fixed() -> Self {
        Self {
            diagonal: Vector3::new(
                config::FIXED_COV_OMEGA()[0],
                config::FIXED_COV_OMEGA()[1],
                config::FIXED_COV_OMEGA()[2],
            ),
            diagonal_v: Vector3::new(
                config::FIXED_COV_V()[0],
                config::FIXED_COV_V()[1],
                config::FIXED_COV_V()[2],
            ),
        }
    }
}

pub struct TwistWithCovariance {
    pub twist: Twist6,
    pub covariance: TwistCovariance,
}

fn skew_symmetric(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

pub fn twist_to_se3(omega: &Vector3<f64>, v: &Vector3<f64>) -> Matrix4<f64> {
    let theta = omega.norm();
    let mut m = Matrix4::identity();

    if theta < 1e-10 {
        m[(0, 3)] = v.x;
        m[(1, 3)] = v.y;
        m[(2, 3)] = v.z;
        return m;
    }

    let n = omega / theta;
    let k = skew_symmetric(&n);
    let k2 = &k * &k;

    let st = theta.sin();
    let ct = theta.cos();
    let r: Matrix3<f64> = Matrix3::identity() + st * k + (1.0 - ct) * k2;

    let left_jacobian: Matrix3<f64> =
        Matrix3::identity() + ((1.0 - ct) / theta) * k + ((theta - st) / (theta * theta)) * k2;

    let t = left_jacobian * v;

    m.fixed_view_mut::<3, 3>(0, 0).copy_from(&r);
    m[(0, 3)] = t.x;
    m[(1, 3)] = t.y;
    m[(2, 3)] = t.z;
    m
}

struct SampledTwist {
    twist: Twist6,
    noise_omega: Vector3<f64>,
    noise_v: Vector3<f64>,
    t: f64,
}

fn sample_twist(
    twist: &Twist6,
    covariance: &TwistCovariance,
    t: f64,
    rng: &mut impl Rng,
) -> SampledTwist {
    let omega_mean = twist.omega * t;
    let v_mean = twist.v * t;

    let sqrt_t = t.sqrt().max(0.0);
    let d_ox: f64 = rng.sample(Normal::new(0.0, covariance.diagonal.x.sqrt() * sqrt_t).unwrap());
    let d_oy: f64 = rng.sample(Normal::new(0.0, covariance.diagonal.y.sqrt() * sqrt_t).unwrap());
    let d_oz: f64 = rng.sample(Normal::new(0.0, covariance.diagonal.z.sqrt() * sqrt_t).unwrap());
    let d_vx: f64 = rng.sample(Normal::new(0.0, covariance.diagonal_v.x.sqrt() * sqrt_t).unwrap());
    let d_vy: f64 = rng.sample(Normal::new(0.0, covariance.diagonal_v.y.sqrt() * sqrt_t).unwrap());
    let d_vz: f64 = rng.sample(Normal::new(0.0, covariance.diagonal_v.z.sqrt() * sqrt_t).unwrap());

    let noise_omega = Vector3::new(d_ox, d_oy, d_oz);
    let noise_v = Vector3::new(d_vx, d_vy, d_vz);

    SampledTwist {
        twist: Twist6 {
            omega: omega_mean + noise_omega,
            v: v_mean + noise_v,
        },
        noise_omega,
        noise_v,
        t,
    }
}

fn compute_sample_probability(
    noise_omega: &Vector3<f64>,
    noise_v: &Vector3<f64>,
    covariance: &TwistCovariance,
    t: f64,
) -> f64 {
    let t_safe = t.max(1e-12);
    let var_omega = covariance.diagonal * t_safe;
    let var_v = covariance.diagonal_v * t_safe;

    let d_omega_sq = noise_omega.x * noise_omega.x / var_omega.x.max(1e-12)
        + noise_omega.y * noise_omega.y / var_omega.y.max(1e-12)
        + noise_omega.z * noise_omega.z / var_omega.z.max(1e-12);
    let d_v_sq = noise_v.x * noise_v.x / var_v.x.max(1e-12)
        + noise_v.y * noise_v.y / var_v.y.max(1e-12)
        + noise_v.z * noise_v.z / var_v.z.max(1e-12);

    (-0.5 * (d_omega_sq + d_v_sq)).exp().clamp(0.0, 1.0)
}

pub fn sample_future_poses(
    current_pose: &DualQuaternion,
    twist: &Twist6,
    covariance: &TwistCovariance,
    config: &PredictionConfig,
    rng: &mut impl Rng,
) -> Vec<SampledPose> {
    let mut poses = Vec::with_capacity(config.n_samples);
    for _ in 0..config.n_samples {
        let t: f64 = rng.random_range(0.0..config.t_max);
        let sampled = sample_twist(twist, covariance, t, rng);
        let displacement_se3 = twist_to_se3(&sampled.twist.omega, &sampled.twist.v);
        let displacement_dq = DualQuaternion::from_se3(&displacement_se3);
        let future_pose = current_pose.multiply(&displacement_dq);

        // Random grasp type: uniform over 0, 1, 2.
        let grasp_type: usize = rng.random_range(0..3);

        // Random wrist rotation around local Y axis.
        let wrist_rotation: f64 = rng.random_range(-config::WRIST_ROTATION_RANGE_RAD()..config::WRIST_ROTATION_RANGE_RAD());

        // Apply wrist rotation to the sampled pose.
        let wrist_se3 = {
            let rot = UnitQuaternion::from_axis_angle(
                &nalgebra::Vector3::y_axis(),
                wrist_rotation,
            );
            let mut m = Matrix4::identity();
            m.fixed_view_mut::<3, 3>(0, 0)
                .copy_from(rot.to_rotation_matrix().matrix());
            m
        };
        let wrist_dq = DualQuaternion::from_se3(&wrist_se3);
        let final_pose = future_pose.multiply(&wrist_dq);

        let prob = compute_sample_probability(
            &sampled.noise_omega,
            &sampled.noise_v,
            covariance,
            sampled.t,
        );
        poses.push(SampledPose {
            pose: final_pose,
            sample_probability: prob,
            grasp_type,
            wrist_rotation,
        });
    }
    poses
}

/// A particle in the Sequential Monte Carlo optimizer.
///
/// Unlike `SampledPose` which represents a one-shot sample from the motion model,
/// `SmcParticle` carries forward state across SMC iterations and caches its score.
#[derive(Debug, Clone)]
pub struct SmcParticle {
    pub pose: DualQuaternion,
    pub sample_probability: f64,
    /// Randomly assigned grasp type: 0 = cylindrical, 1 = pinch, 2 = lateral.
    pub grasp_type: usize,
    /// Wrist rotation angle around local Y axis in radians.
    pub wrist_rotation: f64,
    /// Combined score from the last evaluation iteration.
    pub score: f64,
}

/// Generate the initial broad particle set for SMC iteration 0.
///
/// This is equivalent to `sample_future_poses` but produces `SmcParticle`s
/// with a default score of 0.0 (to be filled by the scorer).
pub fn sample_initial_particles(
    current_pose: &DualQuaternion,
    twist: &Twist6,
    covariance: &TwistCovariance,
    n_samples: usize,
    t_max: f64,
    rng: &mut impl Rng,
) -> Vec<SmcParticle> {
    let mut particles = Vec::with_capacity(n_samples);
    for _ in 0..n_samples {
        let t: f64 = rng.random_range(0.0..t_max);
        let sampled = sample_twist(twist, covariance, t, rng);
        let displacement_se3 = twist_to_se3(&sampled.twist.omega, &sampled.twist.v);
        let displacement_dq = DualQuaternion::from_se3(&displacement_se3);
        let future_pose = current_pose.multiply(&displacement_dq);

        let grasp_type: usize = rng.random_range(0..3);
        let wrist_rotation: f64 =
            rng.random_range(-config::WRIST_ROTATION_RANGE_RAD()..config::WRIST_ROTATION_RANGE_RAD());

        let wrist_se3 = {
            let rot = UnitQuaternion::from_axis_angle(&nalgebra::Vector3::y_axis(), wrist_rotation);
            let mut m = Matrix4::identity();
            m.fixed_view_mut::<3, 3>(0, 0)
                .copy_from(rot.to_rotation_matrix().matrix());
            m
        };
        let wrist_dq = DualQuaternion::from_se3(&wrist_se3);
        let final_pose = future_pose.multiply(&wrist_dq);

        let prob = compute_sample_probability(
            &sampled.noise_omega,
            &sampled.noise_v,
            covariance,
            sampled.t,
        );

        particles.push(SmcParticle {
            pose: final_pose,
            sample_probability: prob,
            grasp_type,
            wrist_rotation,
            score: 0.0,
        });
    }
    particles
}

/// Resample a full population of particles around the elite set.
///
/// The top `ELITE_PRESERVE_RATIO` fraction of the new population is filled with
/// unchanged copies of the best elites (elite injection). The remaining particles
/// are generated by jittering a **score-weighted** parent elite's pose.
///
/// Grasp type is inherited from the parent with probability `1 - GRASP_TYPE_MUTATION_RATE`;
/// otherwise it mutates to a different type sampled from `grasp_type_weights`.
///
/// Wrist rotation is perturbed around the parent's wrist angle with a Gaussian
/// whose std decays with the iteration schedule (passed via `proposal_std_wrist`).
///
/// `sample_probability` is set to 1.0 for resampled particles since they no longer
/// originate from the motion model.
pub fn resample_around_elites(
    elites: &[SmcParticle],
    n_total: usize,
    proposal_std_v: f64,
    proposal_std_omega: f64,
    proposal_std_wrist: f64,
    grasp_type_weights: &[f64; 3],
    rng: &mut impl Rng,
) -> Vec<SmcParticle> {
    assert!(!elites.is_empty(), "elite set must not be empty");

    let n_elites = elites.len();

    // Pre-compute cumulative weights for score-weighted parent selection.
    let elite_scores: Vec<f64> = elites.iter().map(|e| e.score.max(0.0)).collect();
    let total_weight: f64 = elite_scores.iter().sum();
    let cum_weights: Vec<f64> = {
        let mut cw = Vec::with_capacity(n_elites);
        let mut acc = 0.0;
        for &s in &elite_scores {
            acc += s;
            cw.push(acc);
        }
        cw
    };

    // How many particles to preserve as elite injection.
    let n_preserve = ((n_total as f64) * config::ELITE_PRESERVE_RATIO()).round() as usize;
    let n_preserve = n_preserve.min(n_elites).min(n_total);

    let mut particles = Vec::with_capacity(n_total);

    let normal_v = Normal::new(0.0, proposal_std_v).unwrap();
    let normal_omega = Normal::new(0.0, proposal_std_omega).unwrap();
    let normal_wrist = Normal::new(0.0, proposal_std_wrist).unwrap();

    // --- Elite injection: preserve the top `n_preserve` elites unchanged ---
    // Elites are assumed sorted by score descending (select_elite_indices sorts them).
    for i in 0..n_preserve {
        particles.push(elites[i].clone());
    }

    // --- Remaining particles: score-weighted parent selection + jitter ---
    for _ in n_preserve..n_total {
        // Score-weighted parent selection.
        let elite = if total_weight > 0.0 {
            let r = rng.random::<f64>() * total_weight;
            let idx = cum_weights
                .binary_search_by(|probe| {
                    if *probe < r {
                        std::cmp::Ordering::Less
                    } else {
                        std::cmp::Ordering::Greater
                    }
                })
                .unwrap_or_else(|i| i.min(n_elites - 1));
            &elites[idx]
        } else {
            // All scores zero — fall back to uniform.
            &elites[rng.random_range(0..n_elites)]
        };

        let elite_se3 = elite.pose.to_se3();

        // Extract position and rotation from elite pose.
        let pos = Vector3::new(elite_se3[(0, 3)], elite_se3[(1, 3)], elite_se3[(2, 3)]);
        let rot = elite_se3.fixed_view::<3, 3>(0, 0).clone_owned();
        let rot3 = nalgebra::Rotation3::from_matrix_unchecked(rot);
        let uq = UnitQuaternion::from_rotation_matrix(&rot3);

        // Add position jitter.
        let jitter_v = Vector3::new(
            rng.sample(normal_v),
            rng.sample(normal_v),
            rng.sample(normal_v),
        );
        let new_pos = pos + jitter_v;

        // Add orientation jitter as a small incremental rotation.
        let jitter_omega = Vector3::new(
            rng.sample(normal_omega),
            rng.sample(normal_omega),
            rng.sample(normal_omega),
        );
        let jitter_se3 = twist_to_se3(&jitter_omega, &Vector3::zeros());
        let jitter_rot = jitter_se3.fixed_view::<3, 3>(0, 0).clone_owned();
        let jitter_uq = UnitQuaternion::from_rotation_matrix(
            &nalgebra::Rotation3::from_matrix_unchecked(jitter_rot),
        );
        let new_uq = jitter_uq * uq;

        // Reconstruct SE(3) with jittered position and orientation.
        let mut new_se3 = Matrix4::identity();
        new_se3
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(new_uq.to_rotation_matrix().matrix());
        new_se3[(0, 3)] = new_pos.x;
        new_se3[(1, 3)] = new_pos.y;
        new_se3[(2, 3)] = new_pos.z;

        let new_pose = DualQuaternion::from_se3(&new_se3);

        // Grasp type: inherit from parent with mutation probability.
        let grasp_type = if rng.random::<f64>() < config::GRASP_TYPE_MUTATION_RATE() {
            // Mutate: sample from weighted distribution.
            weighted_choice(grasp_type_weights, rng)
        } else {
            // Inherit parent's type.
            elite.grasp_type
        };

        // Wrist rotation: perturb around parent's wrist angle.
        let wrist_rotation = elite.wrist_rotation + rng.sample(normal_wrist);

        // Apply wrist rotation to the resampled pose.
        let wrist_se3 = {
            let rot = UnitQuaternion::from_axis_angle(&nalgebra::Vector3::y_axis(), wrist_rotation);
            let mut m = Matrix4::identity();
            m.fixed_view_mut::<3, 3>(0, 0)
                .copy_from(rot.to_rotation_matrix().matrix());
            m
        };
        let wrist_dq = DualQuaternion::from_se3(&wrist_se3);
        let final_pose = new_pose.multiply(&wrist_dq);

        particles.push(SmcParticle {
            pose: final_pose,
            sample_probability: 1.0, // No longer from motion model.
            grasp_type,
            wrist_rotation,
            score: 0.0,
        });
    }

    particles
}

/// Select elites by sorting particles by score and returning the top fraction.
/// Returns indices into the original particle array.
pub fn select_elite_indices(particles: &[SmcParticle], elite_ratio: f64) -> Vec<usize> {
    let n_elite = ((particles.len() as f64) * elite_ratio).ceil() as usize;
    let n_elite = n_elite.max(1).min(particles.len());

    let mut indexed: Vec<usize> = (0..particles.len()).collect();
    
    if n_elite < indexed.len() {
        indexed.select_nth_unstable_by(n_elite - 1, |&a, &b| {
            particles[b]
                .score
                .partial_cmp(&particles[a].score)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        indexed.truncate(n_elite);
    }
    
    // Sort only the elites so that the absolute best are at the front
    indexed.sort_by(|&a, &b| {
        particles[b]
            .score
            .partial_cmp(&particles[a].score)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    
    indexed
}

/// Compute weighted probabilities for each grasp type based on average scores.
/// Ensures each type maintains a minimum probability floor to prevent collapse.
pub fn compute_grasp_type_weights(particles: &[SmcParticle]) -> [f64; 3] {
    let mut sums = [0.0f64; 3];
    let mut counts = [0usize; 3];

    for p in particles {
        if p.grasp_type < 3 {
            sums[p.grasp_type] += p.score;
            counts[p.grasp_type] += 1;
        }
    }

    // Compute average scores per type
    let mut avg_scores = [0.0f64; 3];
    for i in 0..3 {
        if counts[i] > 0 {
            avg_scores[i] = sums[i] / counts[i] as f64;
        }
    }

    // Shift to non-negative and apply minimum probability floor
    let min_avg = avg_scores.iter().cloned().reduce(f64::min).unwrap_or(0.0);
    let epsilon = 1e-6;
    let mut shifted = [0.0f64; 3];
    for i in 0..3 {
        shifted[i] = (avg_scores[i] - min_avg + epsilon).max(0.0);
    }

    // Apply minimum probability floor
    let min_prob = config::GRASP_TYPE_MIN_PROBABILITY();
    let sum_shifted: f64 = shifted.iter().sum();

    if sum_shifted > 0.0 {
        for i in 0..3 {
            shifted[i] = (shifted[i] / sum_shifted).max(min_prob);
        }
    } else {
        // All scores are equal - use uniform distribution
        for i in 0..3 {
            shifted[i] = 1.0 / 3.0;
        }
    }

    // Renormalize to sum to 1.0
    let sum_after_floor: f64 = shifted.iter().sum();
    for i in 0..3 {
        shifted[i] /= sum_after_floor;
    }

    shifted
}

/// Sample a grasp type (0, 1, or 2) using weighted probabilities.
fn weighted_choice(weights: &[f64; 3], rng: &mut impl Rng) -> usize {
    let r: f64 = rng.random();
    let mut cumulative = 0.0;
    for (i, &w) in weights.iter().enumerate() {
        cumulative += w;
        if r < cumulative {
            return i;
        }
    }
    2 // Fallback to last type
}

pub fn project_index_tips(
    sampled_poses: &[SampledPose],
    current_pose: &DualQuaternion,
    index_tip_local: &Vector3<f64>,
) -> Vec<Vector3<f32>> {
    let mut points = Vec::with_capacity(sampled_poses.len() + 2);

    let base = current_pose.location();
    points.push(Vector3::new(base.x as f32, base.y as f32, base.z as f32));

    let tip = current_pose.transform_point(index_tip_local);
    points.push(Vector3::new(tip.x as f32, tip.y as f32, tip.z as f32));

    for sp in sampled_poses {
        let p = sp.pose.transform_point(index_tip_local);
        points.push(Vector3::new(p.x as f32, p.y as f32, p.z as f32));
    }

    points
}

pub fn compute_prediction_aabb(
    points: &[Vector3<f32>],
    config: &PredictionConfig,
    anchor: &Vector3<f32>,
) -> Aabb {
    let mut aabb = Aabb::from_points(points);
    aabb.inflate(config.hand_radius as f32);
    aabb.enforce_min_dims(&Vector3::new(
        config.min_tsdf_dims.x as f32,
        config.min_tsdf_dims.y as f32,
        config.min_tsdf_dims.z as f32,
    ));
    aabb.clip_max_dims(
        &Vector3::new(
            config.max_tsdf_dims.x as f32,
            config.max_tsdf_dims.y as f32,
            config.max_tsdf_dims.z as f32,
        ),
        anchor,
    );
    aabb
}

pub fn predict_roi_with_samples(
    current_pose: &DualQuaternion,
    twist: &Twist6,
    covariance: &TwistCovariance,
    index_tip_local: &Vector3<f64>,
    config: &PredictionConfig,
) -> (Aabb, Vec<SampledPose>) {
    let mut rng = rand::rng();
    let sampled_poses = sample_future_poses(current_pose, twist, covariance, config, &mut rng);
    let points = project_index_tips(&sampled_poses, current_pose, index_tip_local);
    let anchor = current_pose.location();
    let anchor_f32 = Vector3::new(anchor.x as f32, anchor.y as f32, anchor.z as f32);
    let aabb = compute_prediction_aabb(&points, config, &anchor_f32);
    (aabb, sampled_poses)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn twist_to_se3_pure_translation() {
        let m = twist_to_se3(&Vector3::zeros(), &Vector3::new(1.0, 2.0, 3.0));
        assert!((m[(0, 3)] - 1.0).abs() < 1e-9);
        assert!((m[(1, 3)] - 2.0).abs() < 1e-9);
        assert!((m[(2, 3)] - 3.0).abs() < 1e-9);
        assert!((m[(0, 0)] - 1.0).abs() < 1e-9);
        assert!((m[(1, 1)] - 1.0).abs() < 1e-9);
        assert!((m[(2, 2)] - 1.0).abs() < 1e-9);
    }

    #[test]
    fn twist_to_se3_pure_rotation_z() {
        let theta = std::f64::consts::FRAC_PI_2;
        let m = twist_to_se3(&Vector3::new(0.0, 0.0, theta), &Vector3::zeros());
        assert!((m[(0, 3)] - 0.0).abs() < 1e-9);
        assert!((m[(1, 3)] - 0.0).abs() < 1e-9);
        assert!((m[(0, 0)] - 0.0).abs() < 1e-6);
        assert!((m[(0, 1)] - (-1.0)).abs() < 1e-6);
        assert!((m[(1, 0)] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn twist_to_se3_screw_motion() {
        let omega = Vector3::new(0.0, 0.0, 1.0);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let theta = std::f64::consts::PI;
        let m = twist_to_se3(&(omega * theta), &v);

        // After rotation by pi around z, the x-axis points to -x
        assert!((m[(0, 0)] - (-1.0)).abs() < 1e-6);
        assert!((m[(1, 1)] - (-1.0)).abs() < 1e-6);
        // Translation should be non-zero due to screw
        assert!(m[(0, 3)].abs() > 0.1);
    }

    #[test]
    fn sample_future_poses_zero_twist_stays_at_current() {
        let pose = DualQuaternion::from_se3(&{
            let mut m = Matrix4::identity();
            m[(0, 3)] = 1.0;
            m
        });
        let config = PredictionConfig {
            t_max: 5.0,
            n_samples: 20,
            hand_radius: 0.05,
            min_tsdf_dims: Vector3::new(0.1, 0.1, 0.1),
            max_tsdf_dims: Vector3::new(0.3, 0.3, 0.3),
        };
        let zero_twist = Twist6 {
            omega: Vector3::zeros(),
            v: Vector3::zeros(),
        };
        let zero_cov = TwistCovariance {
            diagonal: Vector3::zeros(),
            diagonal_v: Vector3::zeros(),
        };
        let mut rng = rand::rng();
        let poses = sample_future_poses(
            &pose,
            &zero_twist,
            &zero_cov,
            &config,
            &mut rng,
        );
        assert_eq!(poses.len(), 20);
        for sp in &poses {
            let loc = sp.pose.location();
            assert!(
                (loc[0] - 1.0).abs() < 1e-6,
                "zero twist should keep x ≈ 1.0, got {}",
                loc[0]
            );
        }
    }

    #[test]
    fn project_index_tips_includes_base_and_current_tip() {
        let pose = DualQuaternion::from_se3(&{
            let mut m = Matrix4::identity();
            m[(0, 3)] = 0.1;
            m[(1, 3)] = 0.2;
            m[(2, 3)] = 0.3;
            m
        });
        let tip_local = Vector3::new(0.05, 0.0, 0.0);
        let points = project_index_tips(&[], &pose, &tip_local);
        assert_eq!(points.len(), 2);
        assert!((points[0].x - 0.1).abs() < 1e-6, "base x");
        assert!((points[1].x - 0.15).abs() < 1e-6, "tip x = base + local");
    }

    #[test]
    fn predict_roi_returns_valid_aabb() {
        let pose = DualQuaternion::from_se3(&{
            let mut m = Matrix4::identity();
            m[(0, 3)] = 0.1;
            m[(2, 3)] = 0.2;
            m
        });
        let tip_local = Vector3::new(0.05, 0.0, 0.0);
        let config = PredictionConfig {
            t_max: 5.0,
            n_samples: 50,
            hand_radius: 0.05,
            min_tsdf_dims: Vector3::new(0.1, 0.1, 0.1),
            max_tsdf_dims: Vector3::new(0.3, 0.3, 0.3),
        };
        let zero_twist = Twist6 {
            omega: Vector3::zeros(),
            v: Vector3::zeros(),
        };
        let cov = TwistCovariance::fixed();

        let (aabb, _) = predict_roi_with_samples(&pose, &zero_twist, &cov, &tip_local, &config);

        let size = aabb.max - aabb.min;
        assert!(
            size.x >= 0.1,
            "x size should be >= min_tsdf_dims.x, got {}",
            size.x
        );
        assert!(
            size.y >= 0.1,
            "y size should be >= min_tsdf_dims.y, got {}",
            size.y
        );
        assert!(
            size.z >= 0.1,
            "z size should be >= min_tsdf_dims.z, got {}",
            size.z
        );

        let center = (aabb.min + aabb.max) * 0.5;
        assert!(center.x > -1.0 && center.x < 1.0, "center x = {}", center.x);
        assert!(center.z > -1.0 && center.z < 1.0, "center z = {}", center.z);
    }
}
