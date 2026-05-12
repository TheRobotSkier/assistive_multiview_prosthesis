/*
 * Dynamic arm pose update path for head-observed arm-mounted markers.
 */

#ifndef OV_MSCKF_UPDATER_DYNAMIC_ARM_POSE_H
#define OV_MSCKF_UPDATER_DYNAMIC_ARM_POSE_H

#include <Eigen/Eigen>
#include <array>
#include <memory>
#include <string>

namespace ov_msckf {

class State;

struct DynamicArmPoseUpdaterOptions {
  bool enabled = false;
  bool measurement_only = true;
  std::string topic = "/arm/marker_pose/dynamic_arm_pose_observation";
  std::string status_topic = "/ov_msckf_arm/dynamic_arm_update/status";
  std::string global_frame_id = "marker_map";
  std::string target_frame = "arm_imu";
  std::string source_camera_frame = "head_d435i_head_color_optical_frame";
  std::string marker_frame = "arm_marker_2";
  int marker_id = 2;
  double time_tolerance_s = 0.05;
  double chi2_gate = 16.81;
  double noise_multiplier = 4.0;
  double max_update_translation_m = 0.35;
  double max_update_rotation_deg = 15.0;
  double min_update_interval_s = 0.10;
  double skip_after_fixed_marker_s = 0.50;
};

struct DynamicArmPoseMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = -1.0;
  int marker_id = -1;
  std::string frame_id;
  std::string source_camera_frame;
  std::string marker_frame;
  std::string target_frame;
  Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_GtoI = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
  bool hard_gate_passed = false;
  std::string hard_gate_status;
  bool stable = false;
  int stable_frames = 0;
  double stability_factor = 1.0;
  double reprojection_error_px = 0.0;
  double distance_m = 0.0;
  double view_angle_deg = 0.0;
  double area_px2 = 0.0;
  double side_mean_px = 0.0;
  double side_min_px = 0.0;
  double geometry_score = 0.0;
  double covariance_sigma_px = 0.0;
  double head_pose_match_dt_s = 0.0;
  std::string head_pose_match_mode;
  bool dynamic_covariance_fallback = false;
  bool head_covariance_fallback = false;
  std::string extrinsic_covariance_source;
};

struct DynamicArmPoseUpdateResult {
  bool accepted = false;
  bool state_updated = false;
  std::string reason = "not_run";
  double chi2 = -1.0;
  double translation_norm_m = 0.0;
  double rotation_deg = 0.0;
};

class UpdaterDynamicArmPose {
public:
  explicit UpdaterDynamicArmPose(const DynamicArmPoseUpdaterOptions &options);

  DynamicArmPoseUpdateResult try_update(std::shared_ptr<State> state, const DynamicArmPoseMeasurement &measurement);

  DynamicArmPoseUpdateResult innovation(std::shared_ptr<State> state, const DynamicArmPoseMeasurement &measurement) const;

  bool measurement_only() const { return _options.measurement_only; }

  static Eigen::Matrix<double, 6, 6> ros_covariance_to_update_order(const std::array<double, 36> &covariance_ros);

private:
  bool valid_measurement(const DynamicArmPoseMeasurement &measurement, std::string &reason) const;

  DynamicArmPoseUpdaterOptions _options;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_DYNAMIC_ARM_POSE_H
