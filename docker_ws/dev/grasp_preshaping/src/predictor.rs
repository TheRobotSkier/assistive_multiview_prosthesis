use crate::config;
use crate::lut_helper::DualQuaternion;
use crate::pointcloud_helper::Aabb;
use nalgebra::{Matrix3, Matrix4, Vector3};
use rand::Rng;
use rand_distr::Normal;

#[derive(Debug, Clone)]
pub struct SampledPose {
    pub pose: DualQuaternion,
    pub sample_probability: f64,
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
                config::FIXED_COV_OMEGA[0],
                config::FIXED_COV_OMEGA[1],
                config::FIXED_COV_OMEGA[2],
            ),
            diagonal_v: Vector3::new(
                config::FIXED_COV_V[0],
                config::FIXED_COV_V[1],
                config::FIXED_COV_V[2],
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

fn sample_future_poses(
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
        let prob = compute_sample_probability(
            &sampled.noise_omega,
            &sampled.noise_v,
            covariance,
            sampled.t,
        );
        poses.push(SampledPose {
            pose: future_pose,
            sample_probability: prob,
        });
    }
    poses
}

fn project_index_tips(
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

fn compute_prediction_aabb(
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
