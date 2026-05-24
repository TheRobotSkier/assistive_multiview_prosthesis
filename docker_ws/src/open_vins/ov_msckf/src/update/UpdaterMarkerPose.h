/*
 * Phase 2 marker-pose update path for the assistive multiview prosthesis
 * OpenVINS integration.
 */

#ifndef OV_MSCKF_UPDATER_MARKER_POSE_H
#define OV_MSCKF_UPDATER_MARKER_POSE_H

#include <Eigen/Eigen>
#include <memory>
#include <string>
#include <vector>

namespace ov_msckf {

class State;

struct MarkerPoseUpdaterOptions {
  bool enabled = false;
  std::string topic = "/head/marker_pose/observation";
  std::string status_topic = "/ov_msckf/marker_update/status";
  std::string global_frame_id = "marker_map";
  std::string target_frame = "imu";
  std::vector<int> fixed_marker_ids = {0};
  double time_tolerance_s = 0.05;
  double chi2_gate = 16.81;
  double noise_multiplier = 1.0;
  double max_update_translation_m = 0.25;
  double max_update_rotation_deg = 25.0;
  double reset_translation_m = 0.5;
  double reset_rotation_deg = 20.0;
  int reset_min_samples = 5;
  double reset_window_s = 0.5;
  double reset_min_sample_dt_s = 0.1;
  double reset_max_velocity_mps = 2.0;
  double reset_min_velocity_std_mps = 0.05;
  double reset_bias_gyro_std = 0.02;
  double reset_bias_accel_std = 0.20;
  bool marker_initial_lock_allow_zero_velocity = false;
  double marker_initial_lock_velocity_cov_std = 0.5;
  std::string marker_reset_bias_policy = "preserve";
};

struct MarkerPoseMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp = -1.0;
  int marker_id = -1;
  std::string frame_id;
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
};

struct MarkerPoseUpdateResult {
  bool accepted = false;
  bool state_updated = false;
  std::string reason = "not_run";
  double chi2 = -1.0;
  double translation_norm_m = 0.0;
  double rotation_deg = 0.0;
  bool reset_requested = false;
  bool reset_performed = false;
  bool reset_skipped_velocity_fit = false;
  std::string reset_reason;
  bool velocity_fit_passed = false;
  int velocity_fit_sample_count = 0;
  double velocity_fit_sample_span_s = 0.0;
  double velocity_fit_speed_mps = 0.0;
  bool marker_map_initialized = false;
  bool is_first_lock_attempt = false;
  bool initial_lock_zero_velocity_fallback = false;
  double bias_gyro_norm_before = 0.0;
  double bias_accel_norm_before = 0.0;
  double bias_gyro_norm_after = 0.0;
  double bias_accel_norm_after = 0.0;
  std::string active_bias_policy;
};

class UpdaterMarkerPose {
public:
  explicit UpdaterMarkerPose(const MarkerPoseUpdaterOptions &options);

  MarkerPoseUpdateResult try_update(std::shared_ptr<State> state, const MarkerPoseMeasurement &measurement);

  MarkerPoseUpdateResult innovation(std::shared_ptr<State> state, const MarkerPoseMeasurement &measurement) const;

  bool is_fixed_marker_id(int marker_id) const;

private:
  bool valid_measurement(const MarkerPoseMeasurement &measurement, std::string &reason) const;

  MarkerPoseUpdaterOptions _options;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_MARKER_POSE_H
