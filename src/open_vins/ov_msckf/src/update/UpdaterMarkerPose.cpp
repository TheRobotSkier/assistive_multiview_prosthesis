/*
 * Phase 2 marker-pose update path for the assistive multiview prosthesis
 * OpenVINS integration.
 */

#include "UpdaterMarkerPose.h"

#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "types/Type.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <algorithm>
#include <cmath>
#include <limits>

using namespace ov_core;
using namespace ov_msckf;
using namespace ov_type;

UpdaterMarkerPose::UpdaterMarkerPose(const MarkerPoseUpdaterOptions &options) : _options(options) {}

bool UpdaterMarkerPose::is_fixed_marker_id(int marker_id) const {
  return std::find(_options.fixed_marker_ids.begin(), _options.fixed_marker_ids.end(), marker_id) != _options.fixed_marker_ids.end();
}

bool UpdaterMarkerPose::valid_measurement(const MarkerPoseMeasurement &measurement, std::string &reason) const {
  if (!_options.enabled) {
    reason = "disabled";
    return false;
  }
  if (!is_fixed_marker_id(measurement.marker_id)) {
    reason = "non_fixed_marker_id";
    return false;
  }
  if (!measurement.hard_gate_passed) {
    reason = "phase1_hard_gate_failed:" + measurement.hard_gate_status;
    return false;
  }
  if (!measurement.stable) {
    reason = "unstable_marker";
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

MarkerPoseUpdateResult UpdaterMarkerPose::innovation(std::shared_ptr<State> state, const MarkerPoseMeasurement &measurement) const {
  MarkerPoseUpdateResult result;

  std::string reason;
  if (!valid_measurement(measurement, reason)) {
    result.reason = reason;
    return result;
  }

  Eigen::VectorXd res = Eigen::VectorXd::Zero(6);
  const Eigen::Matrix3d R_GtoI_hat = state->_imu->Rot();
  const Eigen::Vector3d p_IinG_hat = state->_imu->pos();

  // Residual order is [orientation, position]. The sign matches the JPL left
  // multiplicative update used by OpenVINS' IMU/Pose types.
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
    result.reason = "translation_jump_requires_reset";
    return result;
  }
  if (result.rotation_deg > _options.max_update_rotation_deg) {
    result.reason = "rotation_jump_requires_reset";
    return result;
  }

  result.accepted = true;
  result.reason = "accepted";
  return result;
}

MarkerPoseUpdateResult UpdaterMarkerPose::try_update(std::shared_ptr<State> state, const MarkerPoseMeasurement &measurement) {
  MarkerPoseUpdateResult result = innovation(state, measurement);
  if (!result.accepted) {
    PRINT_DEBUG(YELLOW "[MARKER]: rejected marker %d update (%s, chi2 %.3f, dp %.3f m, dtheta %.2f deg)\n" RESET,
                measurement.marker_id, result.reason.c_str(), result.chi2, result.translation_norm_m, result.rotation_deg);
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

  PRINT_INFO(CYAN "[MARKER]: accepted marker %d EKF update (chi2 %.3f, dp %.3f m, dtheta %.2f deg)\n" RESET,
             measurement.marker_id, result.chi2, result.translation_norm_m, result.rotation_deg);
  return result;
}
