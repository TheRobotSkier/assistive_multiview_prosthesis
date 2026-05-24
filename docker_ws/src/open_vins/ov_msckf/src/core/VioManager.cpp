/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "VioManager.h"

#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "feat/FeatureInitializer.h"
#include "track/TrackAruco.h"
#include "track/TrackDescriptor.h"
#include "track/TrackKLT.h"
#include "track/TrackSIM.h"
#include "types/Landmark.h"
#include "types/LandmarkRepresentation.h"
#include "types/Type.h"
#include "utils/opencv_lambda_body.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

#include "init/InertialInitializer.h"

#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "update/UpdaterDynamicArmPose.h"
#include "update/UpdaterMSCKF.h"
#include "update/UpdaterMarkerPose.h"
#include "update/UpdaterSLAM.h"
#include "update/UpdaterZeroVelocity.h"

#include <cmath>
#include <limits>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

VioManager::VioManager(VioManagerOptions &params_) : thread_init_running(false), thread_init_success(false) {

  // Nice startup message
  PRINT_DEBUG("=======================================\n");
  PRINT_DEBUG("OPENVINS ON-MANIFOLD EKF IS STARTING\n");
  PRINT_DEBUG("=======================================\n");

  // Nice debug
  this->params = params_;
  params.print_and_load_estimator();
  params.print_and_load_noise();
  params.print_and_load_state();
  params.print_and_load_trackers();

  // This will globally set the thread count we will use
  // -1 will reset to the system default threading (usually the num of cores)
  cv::setNumThreads(params.num_opencv_threads);
  cv::setRNGSeed(0);

  // Create the state!!
  state = std::make_shared<State>(params.state_options);

  // Set the IMU intrinsics
  state->_calib_imu_dw->set_value(params.vec_dw);
  state->_calib_imu_dw->set_fej(params.vec_dw);
  state->_calib_imu_da->set_value(params.vec_da);
  state->_calib_imu_da->set_fej(params.vec_da);
  state->_calib_imu_tg->set_value(params.vec_tg);
  state->_calib_imu_tg->set_fej(params.vec_tg);
  state->_calib_imu_GYROtoIMU->set_value(params.q_GYROtoIMU);
  state->_calib_imu_GYROtoIMU->set_fej(params.q_GYROtoIMU);
  state->_calib_imu_ACCtoIMU->set_value(params.q_ACCtoIMU);
  state->_calib_imu_ACCtoIMU->set_fej(params.q_ACCtoIMU);

  // Timeoffset from camera to IMU
  Eigen::VectorXd temp_camimu_dt;
  temp_camimu_dt.resize(1);
  temp_camimu_dt(0) = params.calib_camimu_dt;
  state->_calib_dt_CAMtoIMU->set_value(temp_camimu_dt);
  state->_calib_dt_CAMtoIMU->set_fej(temp_camimu_dt);

  // Loop through and load each of the cameras
  state->_cam_intrinsics_cameras = params.camera_intrinsics;
  for (int i = 0; i < state->_options.num_cameras; i++) {
    state->_cam_intrinsics.at(i)->set_value(params.camera_intrinsics.at(i)->get_value());
    state->_cam_intrinsics.at(i)->set_fej(params.camera_intrinsics.at(i)->get_value());
    state->_calib_IMUtoCAM.at(i)->set_value(params.camera_extrinsics.at(i));
    state->_calib_IMUtoCAM.at(i)->set_fej(params.camera_extrinsics.at(i));
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // If we are recording statistics, then open our file
  if (params.record_timing_information) {
    // If the file exists, then delete it
    if (boost::filesystem::exists(params.record_timing_filepath)) {
      boost::filesystem::remove(params.record_timing_filepath);
      PRINT_INFO(YELLOW "[STATS]: found old file found, deleted...\n" RESET);
    }
    // Create the directory that we will open the file in
    boost::filesystem::path p(params.record_timing_filepath);
    boost::filesystem::create_directories(p.parent_path());
    // Open our statistics file!
    of_statistics.open(params.record_timing_filepath, std::ofstream::out | std::ofstream::app);
    // Write the header information into it
    of_statistics << "# timestamp (sec),tracking,propagation,msckf update,";
    if (state->_options.max_slam_features > 0) {
      of_statistics << "slam update,slam delayed,";
    }
    of_statistics << "re-tri & marg,total" << std::endl;
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Let's make a feature extractor
  // NOTE: after we initialize we will increase the total number of feature tracks
  // NOTE: we will split the total number of features over all cameras uniformly
  int init_max_features = std::floor((double)params.init_options.init_max_features / (double)params.state_options.num_cameras);
  if (params.use_klt) {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackKLT(state->_cam_intrinsics_cameras, init_max_features,
                                                         state->_options.max_aruco_features, params.use_stereo, params.histogram_method,
                                                         params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist));
  } else {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackDescriptor(
        state->_cam_intrinsics_cameras, init_max_features, state->_options.max_aruco_features, params.use_stereo, params.histogram_method,
        params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist, params.knn_ratio));
  }

  // Initialize our aruco tag extractor
  if (params.use_aruco) {
    trackARUCO = std::shared_ptr<TrackBase>(new TrackAruco(state->_cam_intrinsics_cameras, state->_options.max_aruco_features,
                                                           params.use_stereo, params.histogram_method, params.downsize_aruco));
  }

  // Initialize our state propagator
  propagator = std::make_shared<Propagator>(params.imu_noises, params.gravity_mag);

  // Our state initialize
  initializer = std::make_shared<ov_init::InertialInitializer>(params.init_options, trackFEATS->get_feature_database());

  // Make the updater!
  updaterMSCKF = std::make_shared<UpdaterMSCKF>(params.msckf_options, params.featinit_options);
  updaterSLAM = std::make_shared<UpdaterSLAM>(params.slam_options, params.aruco_options, params.featinit_options);

  // If we are using zero velocity updates, then create the updater
  if (params.try_zupt) {
    updaterZUPT = std::make_shared<UpdaterZeroVelocity>(params.zupt_options, params.imu_noises, trackFEATS->get_feature_database(),
                                                        propagator, params.gravity_mag, params.zupt_max_velocity,
                                                        params.zupt_noise_multiplier, params.zupt_max_disparity);
  }

  // Phase 2 marker updates are deliberately opt-in so the original OpenVINS path is unchanged.
  if (params.marker_pose_options.enabled) {
    updaterMarkerPose = std::make_shared<UpdaterMarkerPose>(params.marker_pose_options);
  }
  if (params.dynamic_arm_pose_options.enabled) {
    updaterDynamicArmPose = std::make_shared<UpdaterDynamicArmPose>(params.dynamic_arm_pose_options);
  }
}

void VioManager::feed_measurement_imu(const ov_core::ImuData &message) {

  // The oldest time we need IMU with is the last clone
  // We shouldn't really need the whole window, but if we go backwards in time we will
  double oldest_time = state->margtimestep();
  if (oldest_time > state->_timestamp) {
    oldest_time = -1;
  }
  if (!is_initialized_vio) {
    oldest_time = message.timestamp - params.init_options.init_window_time + state->_calib_dt_CAMtoIMU->value()(0) - 0.10;
  }
  propagator->feed_imu(message, oldest_time);

  // Push back to our initializer
  if (!is_initialized_vio) {
    initializer->feed_imu(message, oldest_time);
  }

  // Push back to the zero velocity updater if it is enabled
  // No need to push back if we are just doing the zv-update at the begining and we have moved
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    updaterZUPT->feed_imu(message, oldest_time);
  }
}

MarkerPoseUpdateResult VioManager::feed_measurement_marker(const MarkerPoseMeasurement &message) {

  MarkerPoseUpdateResult result;
  result.marker_map_initialized = is_marker_global_initialized;

  if (updaterMarkerPose == nullptr || !params.marker_pose_options.enabled) {
    result.reason = "disabled";
    return result;
  }
  if (!is_initialized_vio || state->_timestamp < 0.0) {
    result.reason = "not_initialized";
    PRINT_DEBUG(YELLOW "[MARKER]: dropping marker %d before VIO initialization\n" RESET, message.marker_id);
    return result;
  }
  if (!updaterMarkerPose->is_fixed_marker_id(message.marker_id)) {
    result.reason = "non_fixed_marker_id";
    PRINT_DEBUG(YELLOW "[MARKER]: rejecting non-fixed marker id %d\n" RESET, message.marker_id);
    return result;
  }
  if (last_marker_timestamp >= 0.0 && message.timestamp <= last_marker_timestamp + 1e-9) {
    result.reason = "stale_or_duplicate";
    PRINT_DEBUG(YELLOW "[MARKER]: rejecting stale/duplicate marker %.6f <= %.6f\n" RESET, message.timestamp, last_marker_timestamp);
    return result;
  }

  const double dt = message.timestamp - state->_timestamp;
  if (std::abs(dt) > params.marker_pose_options.time_tolerance_s) {
    result.reason = "time_tolerance_exceeded";
    PRINT_DEBUG(YELLOW "[MARKER]: rejecting marker timestamp %.6f, state %.6f, dt %.3f s\n" RESET, message.timestamp, state->_timestamp,
                dt);
    return result;
  }

  last_marker_timestamp = message.timestamp;
  record_marker_measurement(message);

  MarkerPoseUpdateResult innovation = updaterMarkerPose->innovation(state, message);
  result.chi2 = innovation.chi2;
  result.translation_norm_m = innovation.translation_norm_m;
  result.rotation_deg = innovation.rotation_deg;

  if (should_marker_reset(message, innovation)) {
    result.reset_requested = true;
    result.reset_reason = innovation.reason;

    if (!recent_marker_measurements.empty()) {
      result.velocity_fit_sample_count = (int)recent_marker_measurements.size();
      result.velocity_fit_sample_span_s = recent_marker_measurements.back().timestamp - recent_marker_measurements.front().timestamp;
    }

    const bool is_first_lock = !is_marker_global_initialized;
    result.is_first_lock_attempt = is_first_lock;

    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d velocity_covariance = Eigen::Matrix3d::Identity();
    if (!marker_velocity_fit(velocity, velocity_covariance)) {
      if (is_first_lock && params.marker_pose_options.marker_initial_lock_allow_zero_velocity) {
        velocity.setZero();
        const double vel_std = params.marker_pose_options.marker_initial_lock_velocity_cov_std;
        velocity_covariance = vel_std * vel_std * Eigen::Matrix3d::Identity();
        result.initial_lock_zero_velocity_fallback = true;
        result.reset_skipped_velocity_fit = true;
        result.velocity_fit_speed_mps = 0.0;
        PRINT_WARNING(CYAN "[MARKER]: first lock using zero-velocity fallback (cov std %.3f m/s) for marker %d\n" RESET,
                      vel_std, message.marker_id);
      } else {
        result.reset_skipped_velocity_fit = true;
        result.reason = "reset_skipped_velocity_fit_unreliable";
        PRINT_WARNING(YELLOW "[MARKER]: reset requested for marker %d but velocity fit is not reliable yet (%s)\n" RESET, message.marker_id,
                      innovation.reason.c_str());
        return result;
      }
    } else {
      result.velocity_fit_passed = true;
      result.velocity_fit_speed_mps = velocity.norm();
    }
    std::string reset_reason;
    if (!is_marker_global_initialized) {
      reset_reason = result.initial_lock_zero_velocity_fallback ? "first_marker_map_lock_zero_velocity" : "first_marker_map_lock";
    } else {
      reset_reason = innovation.reason;
    }
    result.bias_gyro_norm_before = state->_imu->bias_g().norm();
    result.bias_accel_norm_before = state->_imu->bias_a().norm();
    result.active_bias_policy = params.marker_pose_options.marker_reset_bias_policy;

    if (reset_to_marker_map(message, velocity, velocity_covariance, reset_reason)) {
      result.reset_performed = true;
      result.state_updated = true;
      result.accepted = true;
      result.reason = reset_reason;
      result.marker_map_initialized = true;
      result.bias_gyro_norm_after = state->_imu->bias_g().norm();
      result.bias_accel_norm_after = state->_imu->bias_a().norm();
      last_fixed_marker_update_timestamp = message.timestamp;
      return result;
    }
    result.reason = "reset_failed";
    return result;
  }

  if (!is_marker_global_initialized) {
    result.reason = "marker_map_not_locked";
    PRINT_DEBUG(YELLOW "[MARKER]: waiting for explicit marker_map lock before EKF marker updates\n" RESET);
    return result;
  }

  MarkerPoseUpdateResult update_result = updaterMarkerPose->try_update(state, message);
  result.accepted = update_result.accepted;
  result.reason = update_result.reason;
  result.chi2 = update_result.chi2;
  result.translation_norm_m = update_result.translation_norm_m;
  result.rotation_deg = update_result.rotation_deg;
  if (update_result.accepted) {
    result.state_updated = true;
    last_fixed_marker_update_timestamp = message.timestamp;
  }
  return result;
}

DynamicArmPoseUpdateResult VioManager::feed_measurement_dynamic_arm_pose(const DynamicArmPoseMeasurement &message) {

  DynamicArmPoseUpdateResult result;
  if (updaterDynamicArmPose == nullptr || !params.dynamic_arm_pose_options.enabled) {
    result.reason = "disabled";
    return result;
  }
  if (!is_initialized_vio || state->_timestamp < 0.0) {
    result.reason = "not_initialized";
    PRINT_DEBUG(YELLOW "[DYNAMIC_ARM]: dropping before VIO initialization\n" RESET);
    return result;
  }
  if (last_dynamic_arm_timestamp >= 0.0 && message.timestamp <= last_dynamic_arm_timestamp + 1e-9) {
    result.reason = "stale_or_duplicate";
    PRINT_DEBUG(YELLOW "[DYNAMIC_ARM]: rejecting stale/duplicate %.6f <= %.6f\n" RESET, message.timestamp, last_dynamic_arm_timestamp);
    return result;
  }

  const double dt = message.timestamp - state->_timestamp;
  if (std::abs(dt) > params.dynamic_arm_pose_options.time_tolerance_s) {
    result.reason = "time_tolerance_exceeded";
    PRINT_DEBUG(YELLOW "[DYNAMIC_ARM]: rejecting timestamp %.6f, state %.6f, dt %.3f s\n" RESET, message.timestamp,
                state->_timestamp, dt);
    return result;
  }

  std::string validation_reason;
  if (!updaterDynamicArmPose->valid_measurement(message, validation_reason)) {
    result.reason = validation_reason;
    PRINT_DEBUG(YELLOW "[DYNAMIC_ARM]: rejecting invalid dynamic measurement (%s)\n" RESET, validation_reason.c_str());
    return result;
  }
  last_dynamic_arm_timestamp = message.timestamp;
  record_dynamic_arm_measurement(message);

  if (!is_marker_global_initialized) {
    if (params.dynamic_arm_pose_options.allow_initial_lock) {
      return try_dynamic_arm_reanchor(message, true, "marker_map_not_locked");
    }
    result.reason = "marker_map_not_locked";
    PRINT_DEBUG(YELLOW "[DYNAMIC_ARM]: waiting for fixed marker_map lock before dynamic updates\n" RESET);
    return result;
  }

  if (last_fixed_marker_update_timestamp >= 0.0 &&
      message.timestamp - last_fixed_marker_update_timestamp < params.dynamic_arm_pose_options.skip_after_fixed_marker_s) {
    result.reason = "skipped_recent_fixed_marker_update";
    PRINT_DEBUG(YELLOW "[DYNAMIC_ARM]: skipping after recent fixed marker update, dt %.3f s\n" RESET,
                message.timestamp - last_fixed_marker_update_timestamp);
    return result;
  }

  if (last_dynamic_arm_update_timestamp >= 0.0 &&
      message.timestamp - last_dynamic_arm_update_timestamp < params.dynamic_arm_pose_options.min_update_interval_s) {
    result.reason = "skipped_min_update_interval";
    PRINT_DEBUG(YELLOW "[DYNAMIC_ARM]: skipping min update interval, dt %.3f s\n" RESET,
                message.timestamp - last_dynamic_arm_update_timestamp);
    return result;
  }

  result = updaterDynamicArmPose->try_update(state, message);
  if (result.accepted) {
    last_dynamic_arm_update_timestamp = message.timestamp;
  } else if (params.dynamic_arm_pose_options.allow_reanchor &&
             (result.translation_norm_m > params.dynamic_arm_pose_options.reanchor_trigger_translation_m ||
              result.rotation_deg > params.dynamic_arm_pose_options.reanchor_trigger_rotation_deg ||
              result.reason == "chi2_rejected" || result.reason == "translation_jump_rejected" ||
              result.reason == "rotation_jump_rejected")) {
    result = try_dynamic_arm_reanchor(message, false, result.reason);
  }
  std::vector<std::shared_ptr<Type>> check_order;
  check_order.push_back(state->_imu);
  Eigen::MatrixXd imu_covariance = StateHelper::get_marginal_covariance(state, check_order);
  if (!state->_imu->value().allFinite() || !imu_covariance.allFinite()) {
    PRINT_ERROR(RED "[DYNAMIC_ARM]: non-finite state/covariance after dynamic update\n" RESET);
    result.accepted = false;
    result.state_updated = false;
    result.reason = "nonfinite_state_after_update";
  }
  return result;
}

void VioManager::record_dynamic_arm_measurement(const DynamicArmPoseMeasurement &measurement) {
  recent_dynamic_arm_measurements.push_back(measurement);
  const double oldest_allowed = measurement.timestamp - std::max(params.dynamic_arm_pose_options.reanchor_window_s, 0.0);
  while (!recent_dynamic_arm_measurements.empty() && recent_dynamic_arm_measurements.front().timestamp < oldest_allowed) {
    recent_dynamic_arm_measurements.pop_front();
  }
  while ((int)recent_dynamic_arm_measurements.size() > std::max(2 * params.dynamic_arm_pose_options.reanchor_min_samples, 20)) {
    recent_dynamic_arm_measurements.pop_front();
  }
}

bool VioManager::dynamic_arm_velocity_fit(Eigen::Vector3d &velocity, Eigen::Matrix3d &velocity_covariance,
                                          DynamicArmPoseUpdateResult &result) const {

  std::vector<DynamicArmPoseMeasurement, Eigen::aligned_allocator<DynamicArmPoseMeasurement>> usable;
  for (const auto &measurement : recent_dynamic_arm_measurements) {
    if (!measurement.hard_gate_passed || !measurement.stable || !measurement.p_IinG.allFinite() || !measurement.R_GtoI.allFinite() ||
        !measurement.covariance.allFinite()) {
      continue;
    }
    usable.push_back(measurement);
  }

  result.reanchor_sample_count = (int)usable.size();
  if ((int)usable.size() < params.dynamic_arm_pose_options.reanchor_min_samples) {
    result.reason = "dynamic_reanchor_insufficient_samples";
    return false;
  }

  const double t0 = usable.front().timestamp;
  const double t1 = usable.back().timestamp;
  result.reanchor_sample_span_s = t1 - t0;
  if (t1 - t0 < params.dynamic_arm_pose_options.reanchor_min_sample_dt_s) {
    result.reason = "dynamic_reanchor_sample_span_too_short";
    return false;
  }

  double sum_w = 0.0;
  double sum_wt = 0.0;
  Eigen::Vector3d sum_wp = Eigen::Vector3d::Zero();
  for (const auto &measurement : usable) {
    const double pos_var = std::max(measurement.covariance.block<3, 3>(3, 3).trace() / 3.0, 1e-8);
    const double w = 1.0 / pos_var;
    const double t = measurement.timestamp - t0;
    sum_w += w;
    sum_wt += w * t;
    sum_wp += w * measurement.p_IinG;
  }
  if (sum_w <= 0.0) {
    result.reason = "dynamic_reanchor_bad_weights";
    return false;
  }

  const double t_mean = sum_wt / sum_w;
  const Eigen::Vector3d p_mean = sum_wp / sum_w;
  double denom = 0.0;
  Eigen::Vector3d numer = Eigen::Vector3d::Zero();
  for (const auto &measurement : usable) {
    const double pos_var = std::max(measurement.covariance.block<3, 3>(3, 3).trace() / 3.0, 1e-8);
    const double w = 1.0 / pos_var;
    const double dt = measurement.timestamp - t0 - t_mean;
    denom += w * dt * dt;
    numer += w * dt * (measurement.p_IinG - p_mean);
  }
  if (denom <= 1e-12) {
    result.reason = "dynamic_reanchor_velocity_fit_degenerate";
    return false;
  }

  velocity = numer / denom;
  result.reanchor_velocity_norm_mps = velocity.norm();
  if (!velocity.allFinite() || velocity.norm() > params.dynamic_arm_pose_options.reanchor_max_velocity_mps) {
    result.reason = "dynamic_reanchor_velocity_too_large";
    return false;
  }

  double translation_residual_sq = 0.0;
  double rotation_residual_sq = 0.0;
  const Eigen::Matrix3d R_ref = usable.back().R_GtoI;
  for (const auto &measurement : usable) {
    const double dt = measurement.timestamp - t0 - t_mean;
    const Eigen::Vector3d residual = measurement.p_IinG - (p_mean + velocity * dt);
    translation_residual_sq += residual.squaredNorm();
    rotation_residual_sq += std::pow(log_so3(measurement.R_GtoI * R_ref.transpose()).norm(), 2);
  }
  result.reanchor_sample_translation_std_m = std::sqrt(translation_residual_sq / std::max((int)usable.size(), 1));
  result.reanchor_sample_rotation_std_deg =
      180.0 / M_PI * std::sqrt(rotation_residual_sq / std::max((int)usable.size(), 1));
  if (result.reanchor_sample_translation_std_m > params.dynamic_arm_pose_options.reanchor_max_sample_translation_std_m) {
    result.reason = "dynamic_reanchor_translation_scatter_too_large";
    return false;
  }
  if (result.reanchor_sample_rotation_std_deg > params.dynamic_arm_pose_options.reanchor_max_sample_rotation_std_deg) {
    result.reason = "dynamic_reanchor_rotation_scatter_too_large";
    return false;
  }

  const double min_var = std::pow(params.marker_pose_options.reset_min_velocity_std_mps, 2);
  velocity_covariance = Eigen::Matrix3d::Identity() * min_var;
  if ((int)usable.size() > 2 && denom > 0.0) {
    const double var = std::max(min_var, result.reanchor_sample_translation_std_m * result.reanchor_sample_translation_std_m /
                                             std::max(denom / std::max(sum_w, 1e-12), 1e-6));
    velocity_covariance = Eigen::Matrix3d::Identity() * var;
  }
  if (!velocity_covariance.allFinite()) {
    result.reason = "dynamic_reanchor_velocity_covariance_nonfinite";
    return false;
  }
  return true;
}

DynamicArmPoseUpdateResult VioManager::try_dynamic_arm_reanchor(const DynamicArmPoseMeasurement &measurement, bool initial_lock,
                                                                const std::string &trigger_reason) {
  DynamicArmPoseUpdateResult result;
  result.reason = trigger_reason;
  if (is_marker_global_initialized) {
    result = updaterDynamicArmPose->innovation(state, measurement);
  }
  if (initial_lock) {
    result.would_dynamic_initial_lock = true;
  } else {
    result.would_dynamic_reanchor = true;
  }

  const double fixed_dt =
      (last_fixed_marker_update_timestamp >= 0.0) ? measurement.timestamp - last_fixed_marker_update_timestamp : -1.0;
  if (last_fixed_marker_update_timestamp >= 0.0 &&
      fixed_dt < params.dynamic_arm_pose_options.reanchor_skip_after_fixed_marker_s) {
    result.reanchor_fixed_skip_active = true;
    result.reason = "dynamic_reanchor_skipped_recent_fixed_marker_update";
    return result;
  }

  if (last_dynamic_arm_reanchor_timestamp >= 0.0 &&
      measurement.timestamp - last_dynamic_arm_reanchor_timestamp < params.dynamic_arm_pose_options.reanchor_cooldown_s) {
    result.reanchor_cooldown_active = true;
    result.reason = "dynamic_reanchor_cooldown";
    return result;
  }

  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Matrix3d velocity_covariance = Eigen::Matrix3d::Identity();
  if (!dynamic_arm_velocity_fit(velocity, velocity_covariance, result)) {
    return result;
  }

  result.accepted = true;
  if (params.dynamic_arm_pose_options.reanchor_measurement_only) {
    result.reason = initial_lock ? "would_dynamic_initial_lock" : "would_dynamic_reanchor";
    return result;
  }

  if (reset_to_dynamic_arm_pose(measurement, velocity, velocity_covariance,
                                initial_lock ? "dynamic_initial_lock" : "dynamic_reanchor")) {
    result.state_updated = true;
    last_dynamic_arm_update_timestamp = measurement.timestamp;
    last_dynamic_arm_reanchor_timestamp = measurement.timestamp;
    if (initial_lock) {
      result.dynamic_initial_lock_performed = true;
      result.reason = "dynamic_initial_lock_performed";
    } else {
      result.dynamic_reanchor_performed = true;
      result.reason = "dynamic_reanchor_performed";
    }
    return result;
  }

  result.accepted = false;
  result.reason = initial_lock ? "dynamic_initial_lock_reset_failed" : "dynamic_reanchor_reset_failed";
  return result;
}

void VioManager::record_marker_measurement(const MarkerPoseMeasurement &measurement) {
  recent_marker_measurements.push_back(measurement);
  const double oldest_allowed = measurement.timestamp - std::max(params.marker_pose_options.reset_window_s, 0.0);
  while (!recent_marker_measurements.empty() && recent_marker_measurements.front().timestamp < oldest_allowed) {
    recent_marker_measurements.pop_front();
  }
  while ((int)recent_marker_measurements.size() > std::max(2 * params.marker_pose_options.reset_min_samples, 20)) {
    recent_marker_measurements.pop_front();
  }
}

bool VioManager::marker_velocity_fit(Eigen::Vector3d &velocity, Eigen::Matrix3d &velocity_covariance) const {

  std::vector<MarkerPoseMeasurement, Eigen::aligned_allocator<MarkerPoseMeasurement>> usable;
  for (const auto &measurement : recent_marker_measurements) {
    if (!updaterMarkerPose->is_fixed_marker_id(measurement.marker_id)) {
      continue;
    }
    if (!measurement.hard_gate_passed || !measurement.stable || !measurement.p_IinG.allFinite() || !measurement.covariance.allFinite()) {
      continue;
    }
    usable.push_back(measurement);
  }

  if ((int)usable.size() < params.marker_pose_options.reset_min_samples) {
    return false;
  }

  const double t0 = usable.front().timestamp;
  const double t1 = usable.back().timestamp;
  if (t1 - t0 < params.marker_pose_options.reset_min_sample_dt_s) {
    return false;
  }

  double sum_w = 0.0;
  double sum_wt = 0.0;
  Eigen::Vector3d sum_wp = Eigen::Vector3d::Zero();
  for (const auto &measurement : usable) {
    const double pos_var = std::max(measurement.covariance.block<3, 3>(3, 3).trace() / 3.0, 1e-8);
    const double w = 1.0 / pos_var;
    const double t = measurement.timestamp - t0;
    sum_w += w;
    sum_wt += w * t;
    sum_wp += w * measurement.p_IinG;
  }
  if (sum_w <= 0.0) {
    return false;
  }

  const double t_mean = sum_wt / sum_w;
  const Eigen::Vector3d p_mean = sum_wp / sum_w;
  double denom = 0.0;
  Eigen::Vector3d numer = Eigen::Vector3d::Zero();
  for (const auto &measurement : usable) {
    const double pos_var = std::max(measurement.covariance.block<3, 3>(3, 3).trace() / 3.0, 1e-8);
    const double w = 1.0 / pos_var;
    const double dt = measurement.timestamp - t0 - t_mean;
    denom += w * dt * dt;
    numer += w * dt * (measurement.p_IinG - p_mean);
  }
  if (denom <= 1e-12) {
    return false;
  }

  velocity = numer / denom;
  if (!velocity.allFinite() || velocity.norm() > params.marker_pose_options.reset_max_velocity_mps) {
    return false;
  }

  Eigen::Vector3d weighted_sse = Eigen::Vector3d::Zero();
  double weight_sum = 0.0;
  for (const auto &measurement : usable) {
    const double pos_var = std::max(measurement.covariance.block<3, 3>(3, 3).trace() / 3.0, 1e-8);
    const double w = 1.0 / pos_var;
    const double dt = measurement.timestamp - t0 - t_mean;
    const Eigen::Vector3d residual = measurement.p_IinG - (p_mean + velocity * dt);
    weighted_sse += w * residual.cwiseAbs2();
    weight_sum += w;
  }

  const double min_var = std::pow(params.marker_pose_options.reset_min_velocity_std_mps, 2);
  velocity_covariance = Eigen::Matrix3d::Identity() * min_var;
  if ((int)usable.size() > 2 && weight_sum > 0.0) {
    const Eigen::Vector3d residual_var = weighted_sse / std::max(weight_sum * ((int)usable.size() - 2), 1.0);
    for (int i = 0; i < 3; i++) {
      velocity_covariance(i, i) = std::max(min_var, residual_var(i) / std::max(denom / weight_sum, 1e-6));
    }
  }
  return velocity_covariance.allFinite();
}

bool VioManager::should_marker_reset(const MarkerPoseMeasurement &, const MarkerPoseUpdateResult &innovation) const {
  if (!is_marker_global_initialized) {
    return true;
  }
  if (innovation.translation_norm_m > params.marker_pose_options.reset_translation_m) {
    return true;
  }
  if (innovation.rotation_deg > params.marker_pose_options.reset_rotation_deg) {
    return true;
  }
  return innovation.reason == "translation_jump_requires_reset" || innovation.reason == "rotation_jump_requires_reset";
}

bool VioManager::reset_to_marker_map(const MarkerPoseMeasurement &measurement, const Eigen::Vector3d &velocity,
                                     const Eigen::Matrix3d &velocity_covariance, const std::string &reason) {

  if (!measurement.p_IinG.allFinite() || !measurement.R_GtoI.allFinite() || !measurement.covariance.allFinite() || !velocity.allFinite() ||
      !velocity_covariance.allFinite()) {
    PRINT_WARNING(YELLOW "[MARKER]: refusing reset because pose/velocity fit contains non-finite values\n" RESET);
    return false;
  }

  clear_marker_reset_state();

  std::vector<std::shared_ptr<Type>> reset_order;
  reset_order.push_back(state->_imu);
  if (state->_options.do_calib_imu_intrinsics) {
    reset_order.push_back(state->_calib_imu_dw);
    reset_order.push_back(state->_calib_imu_da);
    if (state->_options.do_calib_imu_g_sensitivity) {
      reset_order.push_back(state->_calib_imu_tg);
    }
    if (state->_options.imu_model == StateOptions::ImuModel::KALIBR) {
      reset_order.push_back(state->_calib_imu_GYROtoIMU);
    } else {
      reset_order.push_back(state->_calib_imu_ACCtoIMU);
    }
  }
  if (state->_options.do_calib_camera_timeoffset) {
    reset_order.push_back(state->_calib_dt_CAMtoIMU);
  }
  if (state->_options.do_calib_camera_pose) {
    for (const auto &calib : state->_calib_IMUtoCAM) {
      reset_order.push_back(calib.second);
    }
  }
  if (state->_options.do_calib_camera_intrinsics) {
    for (const auto &intrin : state->_cam_intrinsics) {
      reset_order.push_back(intrin.second);
    }
  }

  Eigen::MatrixXd reset_cov = StateHelper::get_marginal_covariance(state, reset_order);

  // Capture original bias covariance before zeroing (for inflate_only policy)
  Eigen::Matrix3d original_bias_g_cov = reset_cov.block<3, 3>(9, 9);
  Eigen::Matrix3d original_bias_a_cov = reset_cov.block<3, 3>(12, 12);

  reset_cov.block(0, 0, state->_imu->size(), reset_cov.cols()).setZero();
  reset_cov.block(0, 0, reset_cov.rows(), state->_imu->size()).setZero();
  reset_cov.block<3, 3>(0, 0) = measurement.covariance.block<3, 3>(0, 0);
  reset_cov.block<3, 3>(3, 3) = measurement.covariance.block<3, 3>(3, 3);
  reset_cov.block<3, 3>(6, 6) = velocity_covariance;

  // Bias covariance: config-based default or inflated from original
  if (params.marker_pose_options.marker_reset_bias_policy == "inflate_only") {
    constexpr double kInflateMultiplier = 4.0;
    Eigen::Matrix3d cov_g = original_bias_g_cov.allFinite() ? original_bias_g_cov : Eigen::Matrix3d::Identity();
    Eigen::Matrix3d cov_a = original_bias_a_cov.allFinite() ? original_bias_a_cov : Eigen::Matrix3d::Identity();
    reset_cov.block<3, 3>(9, 9) = kInflateMultiplier * cov_g;
    reset_cov.block<3, 3>(12, 12) = kInflateMultiplier * cov_a;
  } else {
    reset_cov.block<3, 3>(9, 9) = std::pow(params.marker_pose_options.reset_bias_gyro_std, 2) * Eigen::Matrix3d::Identity();
    reset_cov.block<3, 3>(12, 12) = std::pow(params.marker_pose_options.reset_bias_accel_std, 2) * Eigen::Matrix3d::Identity();
  }

  // Bias value handling based on policy
  const std::string &policy = params.marker_pose_options.marker_reset_bias_policy;
  bool is_first_lock = !is_marker_global_initialized;
  bool zero_bias = (policy == "zero") || (policy == "zero_on_initial_lock" && is_first_lock);

  Eigen::Matrix<double, 16, 1> imu_value = state->_imu->value();
  imu_value.block<4, 1>(0, 0) = rot_2_quat(measurement.R_GtoI);
  imu_value.block<3, 1>(4, 0) = measurement.p_IinG;
  imu_value.block<3, 1>(7, 0) = velocity;
  imu_value.block<3, 1>(10, 0) = zero_bias ? Eigen::Vector3d::Zero() : state->_imu->bias_g();
  imu_value.block<3, 1>(13, 0) = zero_bias ? Eigen::Vector3d::Zero() : state->_imu->bias_a();
  state->_imu->set_value(imu_value);
  state->_imu->set_fej(imu_value);

  StateHelper::set_initial_covariance(state, reset_cov, reset_order);

  startup_time = state->_timestamp;
  timelastupdate = state->_timestamp;
  distance = 0.0;
  is_marker_global_initialized = true;
  propagator->invalidate_cache();

  PRINT_WARNING(CYAN "[MARKER]: reset OpenVINS global gauge to marker_map using marker %d (%s, %s bias), v=[%.3f %.3f %.3f] m/s\n" RESET,
                measurement.marker_id, reason.c_str(), policy.c_str(), velocity(0), velocity(1), velocity(2));
  return true;
}

bool VioManager::reset_to_dynamic_arm_pose(const DynamicArmPoseMeasurement &measurement, const Eigen::Vector3d &velocity,
                                           const Eigen::Matrix3d &velocity_covariance, const std::string &reason) {
  MarkerPoseMeasurement reset_measurement;
  reset_measurement.timestamp = measurement.timestamp;
  reset_measurement.marker_id = measurement.marker_id;
  reset_measurement.frame_id = measurement.frame_id;
  reset_measurement.marker_frame = measurement.marker_frame;
  reset_measurement.target_frame = measurement.target_frame;
  reset_measurement.p_IinG = measurement.p_IinG;
  reset_measurement.R_GtoI = measurement.R_GtoI;
  reset_measurement.covariance =
      std::max(params.dynamic_arm_pose_options.reanchor_covariance_multiplier, 1.0) * measurement.covariance;
  reset_measurement.hard_gate_passed = measurement.hard_gate_passed;
  reset_measurement.hard_gate_status = measurement.hard_gate_status;
  reset_measurement.stable = measurement.stable;
  reset_measurement.stable_frames = measurement.stable_frames;
  reset_measurement.stability_factor = measurement.stability_factor;
  reset_measurement.reprojection_error_px = measurement.reprojection_error_px;
  reset_measurement.distance_m = measurement.distance_m;
  reset_measurement.view_angle_deg = measurement.view_angle_deg;
  reset_measurement.area_px2 = measurement.area_px2;
  reset_measurement.side_mean_px = measurement.side_mean_px;
  reset_measurement.side_min_px = measurement.side_min_px;
  reset_measurement.geometry_score = measurement.geometry_score;
  reset_measurement.covariance_sigma_px = measurement.covariance_sigma_px;
  Eigen::Matrix3d reset_velocity_covariance =
      std::max(params.dynamic_arm_pose_options.reanchor_covariance_multiplier, 1.0) * velocity_covariance;
  return reset_to_marker_map(reset_measurement, velocity, reset_velocity_covariance, reason);
}

void VioManager::clear_marker_reset_state() {

  while (!state->_clones_IMU.empty()) {
    const double clone_time = state->_clones_IMU.begin()->first;
    StateHelper::marginalize(state, state->_clones_IMU.begin()->second);
    state->_clones_IMU.erase(clone_time);
  }

  auto it_slam = state->_features_SLAM.begin();
  while (it_slam != state->_features_SLAM.end()) {
    StateHelper::marginalize(state, it_slam->second);
    it_slam = state->_features_SLAM.erase(it_slam);
  }

  if (trackFEATS != nullptr && trackFEATS->get_feature_database() != nullptr) {
    trackFEATS->get_feature_database()->cleanup_measurements(std::numeric_limits<double>::infinity());
  }
  if (trackARUCO != nullptr && trackARUCO->get_feature_database() != nullptr) {
    trackARUCO->get_feature_database()->cleanup_measurements(std::numeric_limits<double>::infinity());
  }
  initializer = std::make_shared<ov_init::InertialInitializer>(params.init_options, trackFEATS->get_feature_database());

  good_features_MSCKF.clear();
  active_tracks_time = -1;
  active_tracks_posinG.clear();
  active_tracks_uvd.clear();
  active_image.release();
  active_feat_linsys_A.clear();
  active_feat_linsys_b.clear();
  active_feat_linsys_count.clear();
}

void VioManager::feed_measurement_simulation(double timestamp, const std::vector<int> &camids,
                                             const std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Check if we actually have a simulated tracker
  // If not, recreate and re-cast the tracker to our simulation tracker
  std::shared_ptr<TrackSIM> trackSIM = std::dynamic_pointer_cast<TrackSIM>(trackFEATS);
  if (trackSIM == nullptr) {
    // Replace with the simulated tracker
    trackSIM = std::make_shared<TrackSIM>(state->_cam_intrinsics_cameras, state->_options.max_aruco_features);
    trackFEATS = trackSIM;
    // Need to also replace it in init and zv-upt since it points to the trackFEATS db pointer
    initializer = std::make_shared<ov_init::InertialInitializer>(params.init_options, trackFEATS->get_feature_database());
    if (params.try_zupt) {
      updaterZUPT = std::make_shared<UpdaterZeroVelocity>(params.zupt_options, params.imu_noises, trackFEATS->get_feature_database(),
                                                          propagator, params.gravity_mag, params.zupt_max_velocity,
                                                          params.zupt_noise_multiplier, params.zupt_max_disparity);
    }
    PRINT_WARNING(RED "[SIM]: casting our tracker to a TrackSIM object!\n" RESET);
  }

  // Feed our simulation tracker
  trackSIM->feed_measurement_simulation(timestamp, camids, feats);
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Check if we should do zero-velocity, if so update the state with it
  // Note that in the case that we only use in the beginning initialization phase
  // If we have since moved, then we should never try to do a zero velocity update!
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    // If the same state time, use the previous timestep decision
    if (state->_timestamp != timestamp) {
      did_zupt_update = updaterZUPT->try_update(state, timestamp);
    }
    if (did_zupt_update) {
      assert(state->_timestamp == timestamp);
      propagator->clean_old_imu_measurements(timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      updaterZUPT->clean_old_imu_measurements(timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      propagator->invalidate_cache();
      return;
    }
  }

  // If we do not have VIO initialization, then return an error
  if (!is_initialized_vio) {
    PRINT_ERROR(RED "[SIM]: your vio system should already be initialized before simulating features!!!\n" RESET);
    PRINT_ERROR(RED "[SIM]: initialize your system first before calling feed_measurement_simulation()!!!!\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Call on our propagate and update function
  // Simulation is either all sync, or single camera...
  ov_core::CameraData message;
  message.timestamp = timestamp;
  for (auto const &camid : camids) {
    int width = state->_cam_intrinsics_cameras.at(camid)->w();
    int height = state->_cam_intrinsics_cameras.at(camid)->h();
    message.sensor_ids.push_back(camid);
    message.images.push_back(cv::Mat::zeros(cv::Size(width, height), CV_8UC1));
    message.masks.push_back(cv::Mat::zeros(cv::Size(width, height), CV_8UC1));
  }
  do_feature_propagate_update(message);
}

void VioManager::track_image_and_update(const ov_core::CameraData &message_const) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Assert we have valid measurement data and ids
  assert(!message_const.sensor_ids.empty());
  assert(message_const.sensor_ids.size() == message_const.images.size());
  for (size_t i = 0; i < message_const.sensor_ids.size() - 1; i++) {
    assert(message_const.sensor_ids.at(i) != message_const.sensor_ids.at(i + 1));
  }

  // Downsample if we are downsampling
  ov_core::CameraData message = message_const;
  for (size_t i = 0; i < message.sensor_ids.size() && params.downsample_cameras; i++) {
    cv::Mat img = message.images.at(i);
    cv::Mat mask = message.masks.at(i);
    cv::Mat img_temp, mask_temp;
    cv::pyrDown(img, img_temp, cv::Size(img.cols / 2.0, img.rows / 2.0));
    message.images.at(i) = img_temp;
    cv::pyrDown(mask, mask_temp, cv::Size(mask.cols / 2.0, mask.rows / 2.0));
    message.masks.at(i) = mask_temp;
  }

  // Perform our feature tracking!
  trackFEATS->feed_new_camera(message);

  // If the aruco tracker is available, the also pass to it
  // NOTE: binocular tracking for aruco doesn't make sense as we by default have the ids
  // NOTE: thus we just call the stereo tracking if we are doing binocular!
  if (is_initialized_vio && trackARUCO != nullptr) {
    trackARUCO->feed_new_camera(message);
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Check if we should do zero-velocity, if so update the state with it
  // Note that in the case that we only use in the beginning initialization phase
  // If we have since moved, then we should never try to do a zero velocity update!
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    // If the same state time, use the previous timestep decision
    if (state->_timestamp != message.timestamp) {
      did_zupt_update = updaterZUPT->try_update(state, message.timestamp);
    }
    if (did_zupt_update) {
      assert(state->_timestamp == message.timestamp);
      propagator->clean_old_imu_measurements(message.timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      updaterZUPT->clean_old_imu_measurements(message.timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      propagator->invalidate_cache();
      return;
    }
  }

  // If we do not have VIO initialization, then try to initialize
  // TODO: Or if we are trying to reset the system, then do that here!
  if (!is_initialized_vio) {
    is_initialized_vio = try_to_initialize(message);
    if (!is_initialized_vio) {
      double time_track = (rT2 - rT1).total_microseconds() * 1e-6;
      PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for tracking\n" RESET, time_track);
      return;
    }
  }

  // Call on our propagate and update function
  do_feature_propagate_update(message);
}

void VioManager::do_feature_propagate_update(const ov_core::CameraData &message) {

  //===================================================================================
  // State propagation, and clone augmentation
  //===================================================================================

  // Return if the camera measurement is out of order
  if (state->_timestamp > message.timestamp) {
    PRINT_WARNING(YELLOW "image received out of order, unable to do anything (prop dt = %3f)\n" RESET,
                  (message.timestamp - state->_timestamp));
    return;
  }

  // Propagate the state forward to the current update time
  // Also augment it with a new clone!
  // NOTE: if the state is already at the given time (can happen in sim)
  // NOTE: then no need to prop since we already are at the desired timestep
  if (state->_timestamp != message.timestamp) {
    propagator->propagate_and_clone(state, message.timestamp);
  }
  rT3 = boost::posix_time::microsec_clock::local_time();

  // If we have not reached max clones, we should just return...
  // This isn't super ideal, but it keeps the logic after this easier...
  // We can start processing things when we have at least 5 clones since we can start triangulating things...
  if ((int)state->_clones_IMU.size() < std::min(state->_options.max_clone_size, 5)) {
    PRINT_DEBUG("waiting for enough clone states (%d of %d)....\n", (int)state->_clones_IMU.size(),
                std::min(state->_options.max_clone_size, 5));
    return;
  }

  // Return if we where unable to propagate
  if (state->_timestamp != message.timestamp) {
    PRINT_WARNING(RED "[PROP]: Propagator unable to propagate the state forward in time!\n" RESET);
    PRINT_WARNING(RED "[PROP]: It has been %.3f since last time we propagated\n" RESET, message.timestamp - state->_timestamp);
    return;
  }
  has_moved_since_zupt = true;

  //===================================================================================
  // MSCKF features and KLT tracks that are SLAM features
  //===================================================================================

  // Now, lets get all features that should be used for an update that are lost in the newest frame
  // We explicitly request features that have not been deleted (used) in another update step
  std::vector<std::shared_ptr<Feature>> feats_lost, feats_marg, feats_slam;
  feats_lost = trackFEATS->get_feature_database()->features_not_containing_newer(state->_timestamp, false, true);

  // Don't need to get the oldest features until we reach our max number of clones
  if ((int)state->_clones_IMU.size() > state->_options.max_clone_size || (int)state->_clones_IMU.size() > 5) {
    feats_marg = trackFEATS->get_feature_database()->features_containing(state->margtimestep(), false, true);
    if (trackARUCO != nullptr && message.timestamp - startup_time >= params.dt_slam_delay) {
      feats_slam = trackARUCO->get_feature_database()->features_containing(state->margtimestep(), false, true);
    }
  }

  // Remove any lost features that were from other image streams
  // E.g: if we are cam1 and cam0 has not processed yet, we don't want to try to use those in the update yet
  // E.g: thus we wait until cam0 process its newest image to remove features which were seen from that camera
  auto it1 = feats_lost.begin();
  while (it1 != feats_lost.end()) {
    bool found_current_message_camid = false;
    for (const auto &camuvpair : (*it1)->uvs) {
      if (std::find(message.sensor_ids.begin(), message.sensor_ids.end(), camuvpair.first) != message.sensor_ids.end()) {
        found_current_message_camid = true;
        break;
      }
    }
    if (found_current_message_camid) {
      it1++;
    } else {
      it1 = feats_lost.erase(it1);
    }
  }

  // We also need to make sure that the max tracks does not contain any lost features
  // This could happen if the feature was lost in the last frame, but has a measurement at the marg timestep
  it1 = feats_lost.begin();
  while (it1 != feats_lost.end()) {
    if (std::find(feats_marg.begin(), feats_marg.end(), (*it1)) != feats_marg.end()) {
      // PRINT_WARNING(YELLOW "FOUND FEATURE THAT WAS IN BOTH feats_lost and feats_marg!!!!!!\n" RESET);
      it1 = feats_lost.erase(it1);
    } else {
      it1++;
    }
  }

  // Find tracks that have reached max length, these can be made into SLAM features
  std::vector<std::shared_ptr<Feature>> feats_maxtracks;
  auto it2 = feats_marg.begin();
  while (it2 != feats_marg.end()) {
    // See if any of our camera's reached max track
    bool reached_max = false;
    for (const auto &cams : (*it2)->timestamps) {
      if ((int)cams.second.size() > state->_options.max_clone_size) {
        reached_max = true;
        break;
      }
    }
    // If max track, then add it to our possible slam feature list
    if (reached_max) {
      feats_maxtracks.push_back(*it2);
      it2 = feats_marg.erase(it2);
    } else {
      it2++;
    }
  }

  // Count how many aruco tags we have in our state
  int curr_aruco_tags = 0;
  auto it0 = state->_features_SLAM.begin();
  while (it0 != state->_features_SLAM.end()) {
    if ((int)(*it0).second->_featid <= 4 * state->_options.max_aruco_features)
      curr_aruco_tags++;
    it0++;
  }

  // Append a new SLAM feature if we have the room to do so
  // Also check that we have waited our delay amount (normally prevents bad first set of slam points)
  if (state->_options.max_slam_features > 0 && message.timestamp - startup_time >= params.dt_slam_delay &&
      (int)state->_features_SLAM.size() < state->_options.max_slam_features + curr_aruco_tags) {
    // Get the total amount to add, then the max amount that we can add given our marginalize feature array
    int amount_to_add = (state->_options.max_slam_features + curr_aruco_tags) - (int)state->_features_SLAM.size();
    int valid_amount = (amount_to_add > (int)feats_maxtracks.size()) ? (int)feats_maxtracks.size() : amount_to_add;
    // If we have at least 1 that we can add, lets add it!
    // Note: we remove them from the feat_marg array since we don't want to reuse information...
    if (valid_amount > 0) {
      feats_slam.insert(feats_slam.end(), feats_maxtracks.end() - valid_amount, feats_maxtracks.end());
      feats_maxtracks.erase(feats_maxtracks.end() - valid_amount, feats_maxtracks.end());
    }
  }

  // Loop through current SLAM features, we have tracks of them, grab them for this update!
  // NOTE: if we have a slam feature that has lost tracking, then we should marginalize it out
  // NOTE: we only enforce this if the current camera message is where the feature was seen from
  // NOTE: if you do not use FEJ, these types of slam features *degrade* the estimator performance....
  // NOTE: we will also marginalize SLAM features if they have failed their update a couple times in a row
  for (std::pair<const size_t, std::shared_ptr<Landmark>> &landmark : state->_features_SLAM) {
    if (trackARUCO != nullptr) {
      std::shared_ptr<Feature> feat1 = trackARUCO->get_feature_database()->get_feature(landmark.second->_featid);
      if (feat1 != nullptr)
        feats_slam.push_back(feat1);
    }
    std::shared_ptr<Feature> feat2 = trackFEATS->get_feature_database()->get_feature(landmark.second->_featid);
    if (feat2 != nullptr)
      feats_slam.push_back(feat2);
    assert(landmark.second->_unique_camera_id != -1);
    bool current_unique_cam =
        std::find(message.sensor_ids.begin(), message.sensor_ids.end(), landmark.second->_unique_camera_id) != message.sensor_ids.end();
    if (feat2 == nullptr && current_unique_cam)
      landmark.second->should_marg = true;
    if (landmark.second->update_fail_count > 1)
      landmark.second->should_marg = true;
  }

  // Lets marginalize out all old SLAM features here
  // These are ones that where not successfully tracked into the current frame
  // We do *NOT* marginalize out our aruco tags landmarks
  StateHelper::marginalize_slam(state);

  // Separate our SLAM features into new ones, and old ones
  std::vector<std::shared_ptr<Feature>> feats_slam_DELAYED, feats_slam_UPDATE;
  for (size_t i = 0; i < feats_slam.size(); i++) {
    if (state->_features_SLAM.find(feats_slam.at(i)->featid) != state->_features_SLAM.end()) {
      feats_slam_UPDATE.push_back(feats_slam.at(i));
      // PRINT_DEBUG("[UPDATE-SLAM]: found old feature %d (%d
      // measurements)\n",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
    } else {
      feats_slam_DELAYED.push_back(feats_slam.at(i));
      // PRINT_DEBUG("[UPDATE-SLAM]: new feature ready %d (%d
      // measurements)\n",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
    }
  }

  // Concatenate our MSCKF feature arrays (i.e., ones not being used for slam updates)
  std::vector<std::shared_ptr<Feature>> featsup_MSCKF = feats_lost;
  featsup_MSCKF.insert(featsup_MSCKF.end(), feats_marg.begin(), feats_marg.end());
  featsup_MSCKF.insert(featsup_MSCKF.end(), feats_maxtracks.begin(), feats_maxtracks.end());

  //===================================================================================
  // Now that we have a list of features, lets do the EKF update for MSCKF and SLAM!
  //===================================================================================

  // Sort based on track length
  // TODO: we should have better selection logic here (i.e. even feature distribution in the FOV etc..)
  // TODO: right now features that are "lost" are at the front of this vector, while ones at the end are long-tracks
  auto compare_feat = [](const std::shared_ptr<Feature> &a, const std::shared_ptr<Feature> &b) -> bool {
    size_t asize = 0;
    size_t bsize = 0;
    for (const auto &pair : a->timestamps)
      asize += pair.second.size();
    for (const auto &pair : b->timestamps)
      bsize += pair.second.size();
    return asize < bsize;
  };
  std::sort(featsup_MSCKF.begin(), featsup_MSCKF.end(), compare_feat);

  // Pass them to our MSCKF updater
  // NOTE: if we have more then the max, we select the "best" ones (i.e. max tracks) for this update
  // NOTE: this should only really be used if you want to track a lot of features, or have limited computational resources
  if ((int)featsup_MSCKF.size() > state->_options.max_msckf_in_update)
    featsup_MSCKF.erase(featsup_MSCKF.begin(), featsup_MSCKF.end() - state->_options.max_msckf_in_update);
  updaterMSCKF->update(state, featsup_MSCKF);
  propagator->invalidate_cache();
  rT4 = boost::posix_time::microsec_clock::local_time();

  // Perform SLAM delay init and update
  // NOTE: that we provide the option here to do a *sequential* update
  // NOTE: this will be a lot faster but won't be as accurate.
  std::vector<std::shared_ptr<Feature>> feats_slam_UPDATE_TEMP;
  while (!feats_slam_UPDATE.empty()) {
    // Get sub vector of the features we will update with
    std::vector<std::shared_ptr<Feature>> featsup_TEMP;
    featsup_TEMP.insert(featsup_TEMP.begin(), feats_slam_UPDATE.begin(),
                        feats_slam_UPDATE.begin() + std::min(state->_options.max_slam_in_update, (int)feats_slam_UPDATE.size()));
    feats_slam_UPDATE.erase(feats_slam_UPDATE.begin(),
                            feats_slam_UPDATE.begin() + std::min(state->_options.max_slam_in_update, (int)feats_slam_UPDATE.size()));
    // Do the update
    updaterSLAM->update(state, featsup_TEMP);
    feats_slam_UPDATE_TEMP.insert(feats_slam_UPDATE_TEMP.end(), featsup_TEMP.begin(), featsup_TEMP.end());
    propagator->invalidate_cache();
  }
  feats_slam_UPDATE = feats_slam_UPDATE_TEMP;
  rT5 = boost::posix_time::microsec_clock::local_time();
  updaterSLAM->delayed_init(state, feats_slam_DELAYED);
  rT6 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  // Update our visualization feature set, and clean up the old features
  //===================================================================================

  // Re-triangulate all current tracks in the current frame
  if (message.sensor_ids.at(0) == 0) {

    // Re-triangulate features
    retriangulate_active_tracks(message);

    // Clear the MSCKF features only on the base camera
    // Thus we should be able to visualize the other unique camera stream
    // MSCKF features as they will also be appended to the vector
    good_features_MSCKF.clear();
  }

  // Save all the MSCKF features used in the update
  for (auto const &feat : featsup_MSCKF) {
    good_features_MSCKF.push_back(feat->p_FinG);
    feat->to_delete = true;
  }

  //===================================================================================
  // Cleanup, marginalize out what we don't need any more...
  //===================================================================================

  // Remove features that where used for the update from our extractors at the last timestep
  // This allows for measurements to be used in the future if they failed to be used this time
  // Note we need to do this before we feed a new image, as we want all new measurements to NOT be deleted
  trackFEATS->get_feature_database()->cleanup();
  if (trackARUCO != nullptr) {
    trackARUCO->get_feature_database()->cleanup();
  }

  // First do anchor change if we are about to lose an anchor pose
  updaterSLAM->change_anchors(state);

  // Cleanup any features older than the marginalization time
  if ((int)state->_clones_IMU.size() > state->_options.max_clone_size) {
    trackFEATS->get_feature_database()->cleanup_measurements(state->margtimestep());
    if (trackARUCO != nullptr) {
      trackARUCO->get_feature_database()->cleanup_measurements(state->margtimestep());
    }
  }

  // Finally marginalize the oldest clone if needed
  StateHelper::marginalize_old_clone(state);
  rT7 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  // Debug info, and stats tracking
  //===================================================================================

  // Get timing statitics information
  double time_track = (rT2 - rT1).total_microseconds() * 1e-6;
  double time_prop = (rT3 - rT2).total_microseconds() * 1e-6;
  double time_msckf = (rT4 - rT3).total_microseconds() * 1e-6;
  double time_slam_update = (rT5 - rT4).total_microseconds() * 1e-6;
  double time_slam_delay = (rT6 - rT5).total_microseconds() * 1e-6;
  double time_marg = (rT7 - rT6).total_microseconds() * 1e-6;
  double time_total = (rT7 - rT1).total_microseconds() * 1e-6;

  // Timing information
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for tracking\n" RESET, time_track);
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for propagation\n" RESET, time_prop);
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for MSCKF update (%d feats)\n" RESET, time_msckf, (int)featsup_MSCKF.size());
  if (state->_options.max_slam_features > 0) {
    PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for SLAM update (%d feats)\n" RESET, time_slam_update, (int)state->_features_SLAM.size());
    PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for SLAM delayed init (%d feats)\n" RESET, time_slam_delay, (int)feats_slam_DELAYED.size());
  }
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for re-tri & marg (%d clones in state)\n" RESET, time_marg, (int)state->_clones_IMU.size());

  std::stringstream ss;
  ss << "[TIME]: " << std::setprecision(4) << time_total << " seconds for total (camera";
  for (const auto &id : message.sensor_ids) {
    ss << " " << id;
  }
  ss << ")" << std::endl;
  PRINT_DEBUG(BLUE "%s" RESET, ss.str().c_str());

  // Finally if we are saving stats to file, lets save it to file
  if (params.record_timing_information && of_statistics.is_open()) {
    // We want to publish in the IMU clock frame
    // The timestamp in the state will be the last camera time
    double t_ItoC = state->_calib_dt_CAMtoIMU->value()(0);
    double timestamp_inI = state->_timestamp + t_ItoC;
    // Append to the file
    of_statistics << std::fixed << std::setprecision(15) << timestamp_inI << "," << std::fixed << std::setprecision(5) << time_track << ","
                  << time_prop << "," << time_msckf << ",";
    if (state->_options.max_slam_features > 0) {
      of_statistics << time_slam_update << "," << time_slam_delay << ",";
    }
    of_statistics << time_marg << "," << time_total << std::endl;
    of_statistics.flush();
  }

  // Update our distance traveled
  if (timelastupdate != -1 && state->_clones_IMU.find(timelastupdate) != state->_clones_IMU.end()) {
    Eigen::Matrix<double, 3, 1> dx = state->_imu->pos() - state->_clones_IMU.at(timelastupdate)->pos();
    distance += dx.norm();
  }
  timelastupdate = message.timestamp;

  // Debug, print our current state
  PRINT_INFO("q_GtoI = %.3f,%.3f,%.3f,%.3f | p_IinG = %.3f,%.3f,%.3f | dist = %.2f (meters)\n", state->_imu->quat()(0),
             state->_imu->quat()(1), state->_imu->quat()(2), state->_imu->quat()(3), state->_imu->pos()(0), state->_imu->pos()(1),
             state->_imu->pos()(2), distance);
  PRINT_INFO("bg = %.4f,%.4f,%.4f | ba = %.4f,%.4f,%.4f\n", state->_imu->bias_g()(0), state->_imu->bias_g()(1), state->_imu->bias_g()(2),
             state->_imu->bias_a()(0), state->_imu->bias_a()(1), state->_imu->bias_a()(2));

  // Debug for camera imu offset
  if (state->_options.do_calib_camera_timeoffset) {
    PRINT_INFO("camera-imu timeoffset = %.5f\n", state->_calib_dt_CAMtoIMU->value()(0));
  }

  // Debug for camera intrinsics
  if (state->_options.do_calib_camera_intrinsics) {
    for (int i = 0; i < state->_options.num_cameras; i++) {
      std::shared_ptr<Vec> calib = state->_cam_intrinsics.at(i);
      PRINT_INFO("cam%d intrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f,%.3f\n", (int)i, calib->value()(0), calib->value()(1),
                 calib->value()(2), calib->value()(3), calib->value()(4), calib->value()(5), calib->value()(6), calib->value()(7));
    }
  }

  // Debug for camera extrinsics
  if (state->_options.do_calib_camera_pose) {
    for (int i = 0; i < state->_options.num_cameras; i++) {
      std::shared_ptr<PoseJPL> calib = state->_calib_IMUtoCAM.at(i);
      PRINT_INFO("cam%d extrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f\n", (int)i, calib->quat()(0), calib->quat()(1), calib->quat()(2),
                 calib->quat()(3), calib->pos()(0), calib->pos()(1), calib->pos()(2));
    }
  }

  // Debug for imu intrinsics
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::KALIBR) {
    PRINT_INFO("q_GYROtoI = %.3f,%.3f,%.3f,%.3f\n", state->_calib_imu_GYROtoIMU->value()(0), state->_calib_imu_GYROtoIMU->value()(1),
               state->_calib_imu_GYROtoIMU->value()(2), state->_calib_imu_GYROtoIMU->value()(3));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::RPNG) {
    PRINT_INFO("q_ACCtoI = %.3f,%.3f,%.3f,%.3f\n", state->_calib_imu_ACCtoIMU->value()(0), state->_calib_imu_ACCtoIMU->value()(1),
               state->_calib_imu_ACCtoIMU->value()(2), state->_calib_imu_ACCtoIMU->value()(3));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::KALIBR) {
    PRINT_INFO("Dw = | %.4f,%.4f,%.4f | %.4f,%.4f | %.4f |\n", state->_calib_imu_dw->value()(0), state->_calib_imu_dw->value()(1),
               state->_calib_imu_dw->value()(2), state->_calib_imu_dw->value()(3), state->_calib_imu_dw->value()(4),
               state->_calib_imu_dw->value()(5));
    PRINT_INFO("Da = | %.4f,%.4f,%.4f | %.4f,%.4f | %.4f |\n", state->_calib_imu_da->value()(0), state->_calib_imu_da->value()(1),
               state->_calib_imu_da->value()(2), state->_calib_imu_da->value()(3), state->_calib_imu_da->value()(4),
               state->_calib_imu_da->value()(5));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::RPNG) {
    PRINT_INFO("Dw = | %.4f | %.4f,%.4f | %.4f,%.4f,%.4f |\n", state->_calib_imu_dw->value()(0), state->_calib_imu_dw->value()(1),
               state->_calib_imu_dw->value()(2), state->_calib_imu_dw->value()(3), state->_calib_imu_dw->value()(4),
               state->_calib_imu_dw->value()(5));
    PRINT_INFO("Da = | %.4f | %.4f,%.4f | %.4f,%.4f,%.4f |\n", state->_calib_imu_da->value()(0), state->_calib_imu_da->value()(1),
               state->_calib_imu_da->value()(2), state->_calib_imu_da->value()(3), state->_calib_imu_da->value()(4),
               state->_calib_imu_da->value()(5));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.do_calib_imu_g_sensitivity) {
    PRINT_INFO("Tg = | %.4f,%.4f,%.4f |  %.4f,%.4f,%.4f | %.4f,%.4f,%.4f |\n", state->_calib_imu_tg->value()(0),
               state->_calib_imu_tg->value()(1), state->_calib_imu_tg->value()(2), state->_calib_imu_tg->value()(3),
               state->_calib_imu_tg->value()(4), state->_calib_imu_tg->value()(5), state->_calib_imu_tg->value()(6),
               state->_calib_imu_tg->value()(7), state->_calib_imu_tg->value()(8));
  }
}
