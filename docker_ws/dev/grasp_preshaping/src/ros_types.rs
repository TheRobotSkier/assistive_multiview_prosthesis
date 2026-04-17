#[cfg(feature = "ros")]
use nalgebra::{Matrix4, Vector3};
#[cfg(feature = "ros")]
use crate::lut_helper::DualQuaternion;
#[cfg(feature = "ros")]
use crate::pointcloud_helper::PointCloud;
#[cfg(feature = "ros")]
use crate::predictor::{Twist6, TwistCovariance, TwistWithCovariance};

#[cfg(feature = "ros")]
pub fn pointcloud2_to_pointcloud(
    msg: &sensor_msgs::msg::PointCloud2,
) -> Result<PointCloud, String> {
    let mut x_off: usize = 0;
    let mut y_off: usize = 4;
    let mut z_off: usize = 8;
    let mut point_step: usize = 0;

    for field in &msg.fields {
        match field.name.as_str() {
            "x" => x_off = field.offset as usize,
            "y" => y_off = field.offset as usize,
            "z" => z_off = field.offset as usize,
            _ => {}
        }
    }

    point_step = msg.point_step as usize;
    if point_step == 0 {
        return Err("PointCloud2 has point_step=0".into());
    }

    let n_points = (msg.width as usize) * (msg.height as usize);
    let data = &msg.data;
    let mut points = Vec::with_capacity(n_points);

    for i in 0..n_points {
        let base = i * point_step;
        if base + z_off + 4 > data.len() {
            break;
        }
        let x = f32::from_le_bytes([
            data[base + x_off],
            data[base + x_off + 1],
            data[base + x_off + 2],
            data[base + x_off + 3],
        ]);
        let y = f32::from_le_bytes([
            data[base + y_off],
            data[base + y_off + 1],
            data[base + y_off + 2],
            data[base + y_off + 3],
        ]);
        let z = f32::from_le_bytes([
            data[base + z_off],
            data[base + z_off + 1],
            data[base + z_off + 2],
            data[base + z_off + 3],
        ]);
        points.push(Vector3::new(x, y, z));
    }

    if points.is_empty() {
        return Err("No valid points parsed from PointCloud2".into());
    }

    Ok(PointCloud::new(points))
}

#[cfg(feature = "ros")]
pub fn pose_stamped_to_dq(msg: &geometry_msgs::msg::PoseStamped) -> DualQuaternion {
    let p = &msg.pose.position;
    let q = &msg.pose.orientation;

    let uq =
        nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(q.w, q.x, q.y, q.z));
    let mut m = Matrix4::identity();
    m.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(uq.to_rotation_matrix().matrix());
    m[(0, 3)] = p.x;
    m[(1, 3)] = p.y;
    m[(2, 3)] = p.z;

    DualQuaternion::from_se3(&m)
}

#[cfg(feature = "ros")]
pub fn twist_msg_to_twist(
    msg: &geometry_msgs::msg::TwistWithCovarianceStamped,
) -> TwistWithCovariance {
    let twist = &msg.twist.twist;
    let cov = &msg.twist.covariance;

    let omega = Vector3::new(twist.angular.x, twist.angular.y, twist.angular.z);
    let v = Vector3::new(twist.linear.x, twist.linear.y, twist.linear.z);

    let covariance = if cov.len() >= 36 {
        TwistCovariance {
            diagonal: Vector3::new(cov[0], cov[7], cov[14]),
            diagonal_v: Vector3::new(cov[21], cov[28], cov[35]),
        }
    } else {
        TwistCovariance::dummy()
    };

    TwistWithCovariance {
        twist: Twist6 { omega, v },
        covariance,
    }
}

#[cfg(feature = "ros")]
pub fn build_joint_command(position: f64) -> std_msgs::msg::Float64MultiArray {
    let mut msg = std_msgs::msg::Float64MultiArray::default();
    msg.data.push(position);
    msg
}
