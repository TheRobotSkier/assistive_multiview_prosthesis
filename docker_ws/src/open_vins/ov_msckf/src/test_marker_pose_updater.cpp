#include "update/UpdaterMarkerPose.h"

#include <cmath>
#include <iostream>
#include <string>

using namespace ov_msckf;

namespace {

bool expect(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << message << std::endl;
    return false;
  }
  return true;
}

MarkerPoseMeasurement make_measurement() {
  MarkerPoseMeasurement m;
  m.timestamp = 10.0;
  m.marker_id = 0;
  m.frame_id = "marker_map";
  m.marker_frame = "marker_0";
  m.target_frame = "imu";
  m.hard_gate_passed = true;
  m.hard_gate_status = "accepted";
  m.stable = true;
  m.R_GtoI = Eigen::Matrix3d::Identity();
  m.p_IinG = Eigen::Vector3d(0.0, 0.0, 0.0);
  m.covariance = 0.01 * Eigen::Matrix<double, 6, 6>::Identity();
  return m;
}

bool test_default_options_baseline() {
  MarkerPoseUpdaterOptions opts;
  return expect(!opts.enabled, "enabled should default false") &&
         expect(opts.fixed_marker_ids == std::vector<int>{0}, "fixed_marker_ids should default {0}") &&
         expect(opts.global_frame_id == "marker_map", "global_frame_id default mismatch") &&
         expect(opts.target_frame == "imu", "target_frame default mismatch") &&
         expect(std::abs(opts.chi2_gate - 16.81) < 1e-12, "chi2_gate default mismatch") &&
         expect(std::abs(opts.noise_multiplier - 1.0) < 1e-12, "noise_multiplier default mismatch") &&
         expect(std::abs(opts.max_update_translation_m - 0.25) < 1e-12, "max_update_translation default mismatch") &&
         expect(std::abs(opts.max_update_rotation_deg - 25.0) < 1e-12, "max_update_rotation default mismatch") &&
         expect(std::abs(opts.reset_translation_m - 0.5) < 1e-12, "reset_translation default mismatch") &&
         expect(std::abs(opts.reset_rotation_deg - 20.0) < 1e-12, "reset_rotation default mismatch") &&
         expect(opts.reset_min_samples == 5, "reset_min_samples default mismatch") &&
         expect(std::abs(opts.reset_window_s - 0.5) < 1e-12, "reset_window_s default mismatch") &&
         expect(std::abs(opts.reset_bias_gyro_std - 0.02) < 1e-12, "reset_bias_gyro_std default mismatch") &&
         expect(std::abs(opts.reset_bias_accel_std - 0.20) < 1e-12, "reset_bias_accel_std default mismatch");
}

bool test_valid_measurement_rejects_wrong_marker_id() {
  UpdaterMarkerPose updater(MarkerPoseUpdaterOptions{});
  std::string reason;

  MarkerPoseMeasurement mm = make_measurement();
  mm.marker_id = 99;
  bool valid = updater.valid_measurement(mm, reason);
  return expect(!valid, "measurement with wrong marker id should be rejected") &&
         expect(reason == "non_fixed_marker_id", "wrong marker id reason mismatch");
}

bool test_valid_measurement_rejects_wrong_frame() {
  UpdaterMarkerPose updater(MarkerPoseUpdaterOptions{});
  std::string reason;

  MarkerPoseMeasurement mm = make_measurement();
  mm.frame_id = "wrong_frame";
  bool valid = updater.valid_measurement(mm, reason);
  return expect(!valid, "measurement with wrong global_frame should be rejected") &&
         expect(reason == "frame_mismatch", "wrong frame reason mismatch");
}

bool test_valid_measurement_rejects_wrong_target_frame() {
  UpdaterMarkerPose updater(MarkerPoseUpdaterOptions{});
  std::string reason;

  MarkerPoseMeasurement mm = make_measurement();
  mm.target_frame = "head_imu";
  bool valid = updater.valid_measurement(mm, reason);
  return expect(!valid, "measurement with wrong target_frame should be rejected") &&
         expect(reason == "target_frame_mismatch", "wrong target frame reason mismatch");
}

bool test_valid_measurement_rejects_when_disabled() {
  MarkerPoseUpdaterOptions opts;
  opts.enabled = false;
  UpdaterMarkerPose updater(opts);
  std::string reason;

  bool valid = updater.valid_measurement(make_measurement(), reason);
  return expect(!valid, "measurement should be rejected when disabled") &&
         expect(reason == "disabled", "disabled reason mismatch");
}

bool test_is_fixed_marker_id_default() {
  MarkerPoseUpdaterOptions opts;
  UpdaterMarkerPose updater(opts);
  return expect(updater.is_fixed_marker_id(0), "marker 0 should be in default fixed list") &&
         expect(!updater.is_fixed_marker_id(99), "marker 99 should not be in default fixed list") &&
         expect(!updater.is_fixed_marker_id(2), "marker 2 should not be in default fixed list");
}

bool test_valid_measurement_rejects_unstable() {
  UpdaterMarkerPose updater(MarkerPoseUpdaterOptions{});
  std::string reason;

  MarkerPoseMeasurement mm = make_measurement();
  mm.stable = false;
  bool valid = updater.valid_measurement(mm, reason);
  return expect(!valid, "measurement with unstable marker should be rejected") &&
         expect(reason == "unstable_marker", "unstable reason mismatch");
}

bool test_valid_measurement_rejects_hard_gate() {
  UpdaterMarkerPose updater(MarkerPoseUpdaterOptions{});
  std::string reason;

  MarkerPoseMeasurement mm = make_measurement();
  mm.hard_gate_passed = false;
  mm.hard_gate_status = "low_geometry";
  bool valid = updater.valid_measurement(mm, reason);
  return expect(!valid, "measurement failing hard gate should be rejected") &&
         expect(reason == "phase1_hard_gate_failed:low_geometry", "hard gate reason mismatch");
}

bool test_valid_measurement_rejects_nonfinite() {
  UpdaterMarkerPose updater(MarkerPoseUpdaterOptions{});
  std::string reason;

  MarkerPoseMeasurement mm = make_measurement();
  mm.p_IinG(0) = std::numeric_limits<double>::quiet_NaN();
  bool valid = updater.valid_measurement(mm, reason);
  return expect(!valid, "measurement with NaN should be rejected") &&
         expect(reason == "nonfinite_measurement", "NaN reason mismatch");
}

bool test_valid_measurement_rejects_bad_rotation_determinant() {
  UpdaterMarkerPose updater(MarkerPoseUpdaterOptions{});
  std::string reason;

  MarkerPoseMeasurement mm = make_measurement();
  mm.R_GtoI = 0.5 * Eigen::Matrix3d::Identity();
  bool valid = updater.valid_measurement(mm, reason);
  return expect(!valid, "measurement with bad rotation det should be rejected") &&
         expect(reason == "bad_rotation_determinant", "bad rotation det reason mismatch");
}

bool test_valid_measurement_rejects_nonpositive_covariance() {
  UpdaterMarkerPose updater(MarkerPoseUpdaterOptions{});
  std::string reason;

  MarkerPoseMeasurement mm = make_measurement();
  mm.covariance(0, 0) = 0.0;
  bool valid = updater.valid_measurement(mm, reason);
  return expect(!valid, "measurement with zero covariance diagonal should be rejected") &&
         expect(reason == "nonpositive_covariance", "zero covariance reason mismatch");
}

bool test_valid_measurement_rejects_bad_timestamp() {
  UpdaterMarkerPose updater(MarkerPoseUpdaterOptions{});
  std::string reason;

  MarkerPoseMeasurement mm = make_measurement();
  mm.timestamp = std::numeric_limits<double>::quiet_NaN();
  bool valid = updater.valid_measurement(mm, reason);
  return expect(!valid, "measurement with NaN timestamp should be rejected") &&
         expect(reason == "bad_timestamp", "bad timestamp reason mismatch");
}

bool test_valid_measurement_accepts_good_measurement() {
  MarkerPoseUpdaterOptions opts;
  opts.enabled = true;
  UpdaterMarkerPose updater(opts);
  std::string reason;

  bool valid = updater.valid_measurement(make_measurement(), reason);
  return expect(valid, "good measurement should be accepted") &&
         expect(reason == "ok", "good measurement reason mismatch");
}

bool test_bias_policy_preserve_is_default() {
  MarkerPoseUpdaterOptions opts;
  return expect(opts.marker_reset_bias_policy == "preserve", "bias_policy default mismatch");
}

bool test_bias_policy_zero_can_be_set() {
  MarkerPoseUpdaterOptions opts;
  opts.marker_reset_bias_policy = "zero";
  return expect(opts.marker_reset_bias_policy == "zero", "bias_policy should be settable to zero");
}

bool test_bias_policy_zero_on_initial_lock_can_be_set() {
  MarkerPoseUpdaterOptions opts;
  opts.marker_reset_bias_policy = "zero_on_initial_lock";
  return expect(opts.marker_reset_bias_policy == "zero_on_initial_lock", "bias_policy should accept zero_on_initial_lock");
}

bool test_initial_lock_zero_velocity_defaults() {
  MarkerPoseUpdaterOptions opts;
  return expect(!opts.marker_initial_lock_allow_zero_velocity, "allow_zero_velocity should default false") &&
         expect(std::abs(opts.marker_initial_lock_velocity_cov_std - 0.5) < 1e-12, "velocity_cov_std default mismatch");
}

bool test_initial_lock_zero_velocity_can_be_enabled() {
  MarkerPoseUpdaterOptions opts;
  opts.marker_initial_lock_allow_zero_velocity = true;
  opts.marker_initial_lock_velocity_cov_std = 0.8;
  return expect(opts.marker_initial_lock_allow_zero_velocity, "allow_zero_velocity should be settable") &&
         expect(std::abs(opts.marker_initial_lock_velocity_cov_std - 0.8) < 1e-12, "velocity_cov_std should be settable");
}

} // namespace

int main() {
  bool ok = true;
  ok = test_default_options_baseline() && ok;
  ok = test_valid_measurement_rejects_wrong_marker_id() && ok;
  ok = test_valid_measurement_rejects_wrong_frame() && ok;
  ok = test_valid_measurement_rejects_wrong_target_frame() && ok;
  ok = test_valid_measurement_rejects_when_disabled() && ok;
  ok = test_is_fixed_marker_id_default() && ok;
  ok = test_valid_measurement_rejects_unstable() && ok;
  ok = test_valid_measurement_rejects_hard_gate() && ok;
  ok = test_valid_measurement_rejects_nonfinite() && ok;
  ok = test_valid_measurement_rejects_bad_rotation_determinant() && ok;
  ok = test_valid_measurement_rejects_nonpositive_covariance() && ok;
  ok = test_valid_measurement_rejects_bad_timestamp() && ok;
  ok = test_valid_measurement_accepts_good_measurement() && ok;
  ok = test_bias_policy_preserve_is_default() && ok;
  ok = test_bias_policy_zero_can_be_set() && ok;
  ok = test_bias_policy_zero_on_initial_lock_can_be_set() && ok;
  ok = test_initial_lock_zero_velocity_defaults() && ok;
  ok = test_initial_lock_zero_velocity_can_be_enabled() && ok;
  return ok ? 0 : 1;
}
