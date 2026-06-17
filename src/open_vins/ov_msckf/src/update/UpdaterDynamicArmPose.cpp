/*
 * Dynamic arm pose update path for head-observed arm-mounted markers.
 */

#include "UpdaterDynamicArmPose.h"

#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "types/Type.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <algorithm>
#include <cmath>

using namespace ov_core;
using namespace ov_msckf;
using namespace ov_type;

UpdaterDynamicArmPose::UpdaterDynamicArmPose(const DynamicArmPoseUpdaterOptions &options) : _options(options) {}

Eigen::Matrix<double, 6, 6> UpdaterDynamicArmPose::ros_covariance_to_update_order(const std::array<double, 36> &covariance_ros_array) {
  Eigen::Matrix<double, 6, 6> covariance_ros = Eigen::Matrix<double, 6, 6>::Zero();
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      covariance_ros(r, c) = covariance_ros_array.at(6 * r + c);
    }
  }
  const int reorder[6] = {3, 4, 5, 0, 1, 2};
  Eigen::Matrix<double, 6, 6> covariance_update = Eigen::Matrix<double, 6, 6>::Zero();
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      covariance_update(r, c) = covariance_ros(reorder[r], reorder[c]);
    }
  }
  return covariance_update;
}

bool UpdaterDynamicArmPose::valid_measurement(const DynamicArmPoseMeasurement &measurement, std::string &reason) const {
  if (!_options.enabled) {
    reason = "disabled";
    return false;
  }
  if (measurement.marker_id != _options.marker_id) {
    reason = "wrong_marker_id";
    return false;
  }
  if (!_options.global_frame_id.empty() && measurement.frame_id != _options.global_frame_id) {
    reason = "frame_mismatch";
    return false;
  }
  if (!_options.target_frame.empty() && measurement.target_frame != _options.target_frame) {
    reason = "target_frame_mismatch";
    return false;
  }
  if (!_options.source_camera_frame.empty() && measurement.source_camera_frame != _options.source_camera_frame) {
    reason = "source_camera_frame_mismatch";
    return false;
  }
  if (!_options.marker_frame.empty() && measurement.marker_frame != _options.marker_frame) {
    reason = "marker_frame_mismatch";
    return false;
  }
  if (!measurement.hard_gate_passed) {
    reason = "dynamic_hard_gate_failed:" + measurement.hard_gate_status;
    return false;
  }
  if (!measurement.stable) {
    reason = "unstable_dynamic_marker";
    return false;
  }
  if (!std::isfinite(measurement.timestamp)) {
    reason = "bad_timestamp";
    return false;
  }
  if (!measurement.p_IinG.allFinite() || !measurement.R_GtoI.allFinite() || !measurement.covariance.allFinite()) {
    reason = "nonfinite_measurement";
    return false;
  }
  if (std::abs(measurement.R_GtoI.determinant() - 1.0) > 1e-3) {
    reason = "bad_rotation_determinant";
    return false;
  }
  for (int i = 0; i < 6; i++) {
    if (measurement.covariance(i, i) <= 0.0) {
      reason = "nonpositive_covariance";
      return false;
    }
  }
  reason = "ok";
  return true;
}

DynamicArmPoseUpdateResult UpdaterDynamicArmPose::innovation(std::shared_ptr<State> state,
                                                             const DynamicArmPoseMeasurement &measurement) const {
  DynamicArmPoseUpdateResult result;

  std::string reason;
  if (!valid_measurement(measurement, reason)) {
    result.reason = reason;
    return result;
  }
  if (state->_timestamp >= 0.0 && std::abs(measurement.timestamp - state->_timestamp) > _options.time_tolerance_s) {
    result.reason = "time_tolerance_exceeded";
    return result;
  }

  Eigen::VectorXd res = Eigen::VectorXd::Zero(6);
  const Eigen::Matrix3d R_GtoI_hat = state->_imu->Rot();
  const Eigen::Vector3d p_IinG_hat = state->_imu->pos();

  res.block(0, 0, 3, 1) = -log_so3(measurement.R_GtoI * R_GtoI_hat.transpose());
  res.block(3, 0, 3, 1) = measurement.p_IinG - p_IinG_hat;

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, state->_imu->size());
  H.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
  H.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 6, 6> R = 0.5 * (measurement.covariance + measurement.covariance.transpose());
  R *= std::max(_options.noise_multiplier, 1e-6);
  for (int i = 0; i < 6; i++) {
    R(i, i) = std::max(R(i, i), 1e-12);
  }

  std::vector<std::shared_ptr<Type>> H_order;
  H_order.push_back(state->_imu);
  Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, H_order);
  Eigen::MatrixXd S = H * P_marg * H.transpose() + R;

  Eigen::LLT<Eigen::MatrixXd> llt(S);
  if (llt.info() != Eigen::Success) {
    result.reason = "innovation_covariance_not_pd";
    return result;
  }

  result.chi2 = res.dot(llt.solve(res));
  result.translation_norm_m = res.block(3, 0, 3, 1).norm();
  result.rotation_deg = 180.0 / M_PI * res.block(0, 0, 3, 1).norm();

  if (result.chi2 > _options.chi2_gate) {
    result.reason = "chi2_rejected";
    return result;
  }
  if (result.translation_norm_m > _options.max_update_translation_m) {
    result.reason = "translation_jump_rejected";
    return result;
  }
  if (result.rotation_deg > _options.max_update_rotation_deg) {
    result.reason = "rotation_jump_rejected";
    return result;
  }

  result.accepted = true;
  result.reason = "accepted";
  return result;
}

DynamicArmPoseUpdateResult UpdaterDynamicArmPose::try_update(std::shared_ptr<State> state,
                                                             const DynamicArmPoseMeasurement &measurement) {
  DynamicArmPoseUpdateResult result = innovation(state, measurement);
  if (!result.accepted) {
    PRINT_DEBUG(YELLOW "[DYNAMIC_ARM]: rejected update (%s, chi2 %.3f, dp %.3f m, dtheta %.2f deg)\n" RESET,
                result.reason.c_str(), result.chi2, result.translation_norm_m, result.rotation_deg);
    return result;
  }
  if (_options.measurement_only) {
    result.reason = "measurement_only";
    PRINT_INFO(CYAN "[DYNAMIC_ARM]: measurement-only accepted (chi2 %.3f, dp %.3f m, dtheta %.2f deg)\n" RESET, result.chi2,
               result.translation_norm_m, result.rotation_deg);
    return result;
  }

  Eigen::VectorXd res = Eigen::VectorXd::Zero(6);
  const Eigen::Matrix3d R_GtoI_hat = state->_imu->Rot();
  const Eigen::Vector3d p_IinG_hat = state->_imu->pos();
  res.block(0, 0, 3, 1) = -log_so3(measurement.R_GtoI * R_GtoI_hat.transpose());
  res.block(3, 0, 3, 1) = measurement.p_IinG - p_IinG_hat;

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, state->_imu->size());
  H.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
  H.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 6, 6> R = 0.5 * (measurement.covariance + measurement.covariance.transpose());
  R *= std::max(_options.noise_multiplier, 1e-6);
  for (int i = 0; i < 6; i++) {
    R(i, i) = std::max(R(i, i), 1e-12);
  }

  std::vector<std::shared_ptr<Type>> H_order;
  H_order.push_back(state->_imu);
  StateHelper::EKFUpdate(state, H_order, H, res, R);

  result.state_updated = true;
  PRINT_INFO(CYAN "[DYNAMIC_ARM]: accepted EKF update (chi2 %.3f, dp %.3f m, dtheta %.2f deg)\n" RESET, result.chi2,
             result.translation_norm_m, result.rotation_deg);
  return result;
}
