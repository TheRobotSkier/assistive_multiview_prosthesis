#include "state/State.h"
#include "update/UpdaterDynamicArmPose.h"
#include "utils/quat_ops.h"

#include <Eigen/Eigen>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

using namespace ov_core;
using namespace ov_msckf;

namespace {

bool expect(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << message << std::endl;
    return false;
  }
  return true;
}

std::shared_ptr<State> make_state() {
  StateOptions state_options;
  state_options.num_cameras = 1;
  auto state = std::make_shared<State>(state_options);
  state->_timestamp = 10.0;
  return state;
}

DynamicArmPoseUpdaterOptions make_options(bool measurement_only = false) {
  DynamicArmPoseUpdaterOptions options;
  options.enabled = true;
  options.measurement_only = measurement_only;
  options.global_frame_id = "marker_map";
  options.target_frame = "arm_imu";
  options.source_camera_frame = "head_d435i_head_color_optical_frame";
  options.marker_frame = "arm_marker_2";
  options.marker_id = 2;
  options.time_tolerance_s = 0.05;
  options.chi2_gate = 1e9;
  options.noise_multiplier = 1.0;
  options.max_update_translation_m = 10.0;
  options.max_update_rotation_deg = 180.0;
  return options;
}

DynamicArmPoseMeasurement make_measurement() {
  DynamicArmPoseMeasurement measurement;
  measurement.timestamp = 10.0;
  measurement.marker_id = 2;
  measurement.frame_id = "marker_map";
  measurement.source_camera_frame = "head_d435i_head_color_optical_frame";
  measurement.marker_frame = "arm_marker_2";
  measurement.target_frame = "arm_imu";
  measurement.p_IinG << 0.10, -0.02, 0.03;
  measurement.R_GtoI = exp_so3(Eigen::Vector3d(0.01, -0.02, 0.03));
  measurement.covariance = 1e-10 * Eigen::Matrix<double, 6, 6>::Identity();
  measurement.hard_gate_passed = true;
  measurement.hard_gate_status = "accepted";
  measurement.stable = true;
  measurement.stable_frames = 8;
  measurement.geometry_score = 0.9;
  measurement.reprojection_error_px = 0.3;
  measurement.distance_m = 0.5;
  measurement.view_angle_deg = 12.0;
  measurement.area_px2 = 10000.0;
  return measurement;
}

bool covariance_reorder_test() {
  std::array<double, 36> cov_ros{};
  cov_ros.at(0) = 1.0;   // x
  cov_ros.at(7) = 2.0;   // y
  cov_ros.at(14) = 3.0;  // z
  cov_ros.at(21) = 4.0;  // roll
  cov_ros.at(28) = 5.0;  // pitch
  cov_ros.at(35) = 6.0;  // yaw
  cov_ros.at(3) = 0.7;   // x-roll
  cov_ros.at(18) = 0.7;  // roll-x

  const Eigen::Matrix<double, 6, 6> cov_update = UpdaterDynamicArmPose::ros_covariance_to_update_order(cov_ros);
  return expect(std::abs(cov_update(0, 0) - 4.0) < 1e-12, "roll covariance not moved to update index 0") &&
         expect(std::abs(cov_update(1, 1) - 5.0) < 1e-12, "pitch covariance not moved to update index 1") &&
         expect(std::abs(cov_update(2, 2) - 6.0) < 1e-12, "yaw covariance not moved to update index 2") &&
         expect(std::abs(cov_update(3, 3) - 1.0) < 1e-12, "x covariance not moved to update index 3") &&
         expect(std::abs(cov_update(4, 4) - 2.0) < 1e-12, "y covariance not moved to update index 4") &&
         expect(std::abs(cov_update(5, 5) - 3.0) < 1e-12, "z covariance not moved to update index 5") &&
         expect(std::abs(cov_update(3, 0) - 0.7) < 1e-12, "x-roll cross covariance not reordered");
}

bool update_pulls_toward_measurement_test() {
  auto state = make_state();
  DynamicArmPoseMeasurement measurement = make_measurement();
  UpdaterDynamicArmPose updater(make_options(false));

  const double before = (state->_imu->pos() - measurement.p_IinG).norm();
  DynamicArmPoseUpdateResult result = updater.try_update(state, measurement);
  const double after = (state->_imu->pos() - measurement.p_IinG).norm();

  return expect(result.accepted, "dynamic update should be accepted") &&
         expect(result.state_updated, "dynamic update should mutate state when measurement_only is false") &&
         expect(after < before, "dynamic update moved state away from measurement");
}

bool rejection_does_not_mutate_test() {
  UpdaterDynamicArmPose updater(make_options(false));
  DynamicArmPoseMeasurement measurement = make_measurement();

  auto wrong_frame_state = make_state();
  const Eigen::Vector3d wrong_frame_before = wrong_frame_state->_imu->pos();
  DynamicArmPoseMeasurement wrong_frame = measurement;
  wrong_frame.target_frame = "head_imu";
  DynamicArmPoseUpdateResult wrong_frame_result = updater.try_update(wrong_frame_state, wrong_frame);

  auto wrong_id_state = make_state();
  const Eigen::Vector3d wrong_id_before = wrong_id_state->_imu->pos();
  DynamicArmPoseMeasurement wrong_id = measurement;
  wrong_id.marker_id = 9;
  DynamicArmPoseUpdateResult wrong_id_result = updater.try_update(wrong_id_state, wrong_id);

  auto bad_cov_state = make_state();
  const Eigen::Vector3d bad_cov_before = bad_cov_state->_imu->pos();
  DynamicArmPoseMeasurement bad_cov = measurement;
  bad_cov.covariance(0, 0) = std::numeric_limits<double>::quiet_NaN();
  DynamicArmPoseUpdateResult bad_cov_result = updater.try_update(bad_cov_state, bad_cov);

  auto stale_state = make_state();
  const Eigen::Vector3d stale_before = stale_state->_imu->pos();
  DynamicArmPoseMeasurement stale = measurement;
  stale.timestamp = 9.0;
  DynamicArmPoseUpdateResult stale_result = updater.try_update(stale_state, stale);

  return expect(!wrong_frame_result.accepted, "wrong target frame should reject") &&
         expect((wrong_frame_state->_imu->pos() - wrong_frame_before).norm() < 1e-12, "wrong frame mutated state") &&
         expect(!wrong_id_result.accepted, "wrong marker id should reject") &&
         expect((wrong_id_state->_imu->pos() - wrong_id_before).norm() < 1e-12, "wrong id mutated state") &&
         expect(!bad_cov_result.accepted, "non-finite covariance should reject") &&
         expect((bad_cov_state->_imu->pos() - bad_cov_before).norm() < 1e-12, "bad covariance mutated state") &&
         expect(!stale_result.accepted, "stale timestamp should reject") &&
         expect(stale_result.reason == "time_tolerance_exceeded", "stale timestamp should use time_tolerance_exceeded reason") &&
         expect((stale_state->_imu->pos() - stale_before).norm() < 1e-12, "stale measurement mutated state");
}

bool measurement_only_does_not_mutate_test() {
  auto state = make_state();
  const Eigen::Vector3d before_pos = state->_imu->pos();
  const Eigen::Vector4d before_quat = state->_imu->quat();
  UpdaterDynamicArmPose updater(make_options(true));
  DynamicArmPoseUpdateResult result = updater.try_update(state, make_measurement());

  return expect(result.accepted, "measurement-only should still gate as accepted") &&
         expect(!result.state_updated, "measurement-only must not report a state update") &&
         expect(result.reason == "measurement_only", "measurement-only reason mismatch") &&
         expect((state->_imu->pos() - before_pos).norm() < 1e-12, "measurement-only changed position") &&
         expect((state->_imu->quat() - before_quat).norm() < 1e-12, "measurement-only changed orientation");
}

} // namespace

int main() {
  bool ok = true;
  ok = covariance_reorder_test() && ok;
  ok = update_pulls_toward_measurement_test() && ok;
  ok = rejection_does_not_mutate_test() && ok;
  ok = measurement_only_does_not_mutate_test() && ok;
  return ok ? 0 : 1;
}
