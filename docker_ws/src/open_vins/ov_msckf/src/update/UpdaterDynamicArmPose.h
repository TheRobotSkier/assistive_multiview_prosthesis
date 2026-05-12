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
  bool allow_initial_lock = false;
  bool allow_reanchor = false;
  bool reanchor_measurement_only = true;
  int reanchor_min_samples = 5;
  double reanchor_window_s = 2.0;
  double reanchor_min_sample_dt_s = 0.50;
  double reanchor_max_velocity_mps = 2.0;
  double reanchor_max_sample_translation_std_m = 0.12;
  double reanchor_max_sample_rotation_std_deg = 8.0;
  double reanchor_trigger_translation_m = 0.75;
  double reanchor_trigger_rotation_deg = 20.0;
  double reanchor_cooldown_s = 5.0;
  double reanchor_skip_after_fixed_marker_s = 3.0;
  double reanchor_covariance_multiplier = 2.0;
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
  std::string head_pose_source_topic;
  std::string head_pose_source_type;
  double head_pose_time_offset_s = 0.0;
};

struct DynamicArmPoseUpdateResult {
  bool accepted = false;
  bool state_updated = false;
  std::string reason = "not_run";
  double chi2 = -1.0;
  double translation_norm_m = 0.0;
  double rotation_deg = 0.0;
  bool would_dynamic_initial_lock = false;
  bool dynamic_initial_lock_performed = false;
  bool would_dynamic_reanchor = false;
  bool dynamic_reanchor_performed = false;
  int reanchor_sample_count = 0;
  double reanchor_sample_span_s = 0.0;
  double reanchor_velocity_norm_mps = 0.0;
  double reanchor_sample_translation_std_m = 0.0;
  double reanchor_sample_rotation_std_deg = 0.0;
  bool reanchor_cooldown_active = false;
  bool reanchor_fixed_skip_active = false;
};

class UpdaterDynamicArmPose {
public:
  explicit UpdaterDynamicArmPose(const DynamicArmPoseUpdaterOptions &options);

  DynamicArmPoseUpdateResult try_update(std::shared_ptr<State> state, const DynamicArmPoseMeasurement &measurement);

  DynamicArmPoseUpdateResult innovation(std::shared_ptr<State> state, const DynamicArmPoseMeasurement &measurement) const;

  bool measurement_only() const { return _options.measurement_only; }

  bool valid_measurement(const DynamicArmPoseMeasurement &measurement, std::string &reason) const;

  static Eigen::Matrix<double, 6, 6> ros_covariance_to_update_order(const std::array<double, 36> &covariance_ros);

private:
  DynamicArmPoseUpdaterOptions _options;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_DYNAMIC_ARM_POSE_H
