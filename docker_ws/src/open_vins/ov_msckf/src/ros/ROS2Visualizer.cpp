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

#include "ROS2Visualizer.h"

#include "core/VioManager.h"
#include "ros/ROSVisualizerHelper.h"
#include "sim/Simulator.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/dataset_reader.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

namespace {

Eigen::Matrix3d quat_xyzw_to_rot(const geometry_msgs::msg::Quaternion &quat_msg) {
  Eigen::Vector4d q(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
  const double norm = q.norm();
  if (norm <= 0.0 || !std::isfinite(norm)) {
    return Eigen::Matrix3d::Identity();
  }
  q /= norm;
  const double x = q(0);
  const double y = q(1);
  const double z = q(2);
  const double w = q(3);
  Eigen::Matrix3d R;
  R << 1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
      2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
      2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y);
  return R;
}

std::string json_escape(const std::string &value) {
  std::ostringstream out;
  for (char ch : value) {
    if (ch == '"' || ch == '\\') {
      out << '\\' << ch;
    } else if (ch == '\n') {
      out << "\\n";
    } else {
      out << ch;
    }
  }
  return out.str();
}

double covariance_std(const Eigen::Matrix<double, 6, 6> &covariance, int index) {
  return std::sqrt(std::max(covariance(index, index), 0.0));
}

} // namespace

ROS2Visualizer::ROS2Visualizer(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<VioManager> app, std::shared_ptr<Simulator> sim)
    : _node(node), _app(app), _sim(sim), thread_update_running(false), run_visualize_thread(true) {

  // Setup our transform broadcaster
  mTfBr = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  if (_app->get_params().marker_pose_options.enabled) {
    global_frame_id = _app->get_params().marker_pose_options.global_frame_id;
  }
  if (node->has_parameter("global_frame_id")) {
    node->get_parameter<std::string>("global_frame_id", global_frame_id);
  }
  if (node->has_parameter("imu_frame_id")) {
    node->get_parameter<std::string>("imu_frame_id", imu_frame_id);
  }
  if (node->has_parameter("camera_frame_prefix")) {
    node->get_parameter<std::string>("camera_frame_prefix", camera_frame_prefix);
  }
  if (node->has_parameter("truth_frame_id")) {
    node->get_parameter<std::string>("truth_frame_id", truth_frame_id);
  }

  // Create image transport
  image_transport::ImageTransport it(node);

  // Setup pose and path publisher
  pub_poseimu = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("poseimu", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_poseimu->get_topic_name());
  pub_odomimu = node->create_publisher<nav_msgs::msg::Odometry>("odomimu", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_odomimu->get_topic_name());
  pub_pathimu = node->create_publisher<nav_msgs::msg::Path>("pathimu", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_pathimu->get_topic_name());
  pub_marker_map_locked = node->create_publisher<std_msgs::msg::Bool>("marker_map_locked", rclcpp::QoS(1).transient_local().reliable());
  PRINT_DEBUG("Publishing: %s\n", pub_marker_map_locked->get_topic_name());
  std_msgs::msg::Bool initial_marker_map_locked;
  initial_marker_map_locked.data = false;
  pub_marker_map_locked->publish(initial_marker_map_locked);

  // 3D points publishing
  pub_points_msckf = node->create_publisher<sensor_msgs::msg::PointCloud2>("points_msckf", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_points_msckf->get_topic_name());
  pub_points_slam = node->create_publisher<sensor_msgs::msg::PointCloud2>("points_slam", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_points_msckf->get_topic_name());
  pub_points_aruco = node->create_publisher<sensor_msgs::msg::PointCloud2>("points_aruco", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_points_aruco->get_topic_name());
  pub_points_sim = node->create_publisher<sensor_msgs::msg::PointCloud2>("points_sim", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_points_sim->get_topic_name());

  // Our tracking image
  it_pub_tracks = it.advertise("trackhist", 2);
  PRINT_DEBUG("Publishing: %s\n", it_pub_tracks.getTopic().c_str());

  // Groundtruth publishers
  pub_posegt = node->create_publisher<geometry_msgs::msg::PoseStamped>("posegt", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_posegt->get_topic_name());
  pub_pathgt = node->create_publisher<nav_msgs::msg::Path>("pathgt", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_pathgt->get_topic_name());

  // Loop closure publishers
  pub_loop_pose = node->create_publisher<nav_msgs::msg::Odometry>("loop_pose", 2);
  pub_loop_point = node->create_publisher<sensor_msgs::msg::PointCloud>("loop_feats", 2);
  pub_loop_extrinsic = node->create_publisher<nav_msgs::msg::Odometry>("loop_extrinsic", 2);
  pub_loop_intrinsics = node->create_publisher<sensor_msgs::msg::CameraInfo>("loop_intrinsics", 2);
  it_pub_loop_img_depth = it.advertise("loop_depth", 2);
  it_pub_loop_img_depth_color = it.advertise("loop_depth_colored", 2);
  if (_app->get_params().dynamic_arm_pose_options.enabled) {
    pub_dynamic_arm_status =
        node->create_publisher<std_msgs::msg::String>(_app->get_params().dynamic_arm_pose_options.status_topic, 10);
    PRINT_DEBUG("Publishing: %s\n", pub_dynamic_arm_status->get_topic_name());
  }
  if (_app->get_params().marker_pose_options.enabled) {
    pub_marker_status =
        node->create_publisher<std_msgs::msg::String>(_app->get_params().marker_pose_options.status_topic, 10);
    PRINT_DEBUG("Publishing: %s\n", pub_marker_status->get_topic_name());
  }

  // option to enable publishing of global to IMU transformation
  if (node->has_parameter("publish_global_to_imu_tf")) {
    node->get_parameter<bool>("publish_global_to_imu_tf", publish_global2imu_tf);
  }
  if (node->has_parameter("publish_calibration_tf")) {
    node->get_parameter<bool>("publish_calibration_tf", publish_calibration_tf);
  }

  // Load groundtruth if we have it and are not doing simulation
  // NOTE: needs to be a csv ASL format file
  std::string path_to_gt;
  bool has_gt = node->get_parameter("path_gt", path_to_gt);
  if (has_gt && _sim == nullptr && !path_to_gt.empty()) {
    DatasetReader::load_gt_file(path_to_gt, gt_states);
    PRINT_DEBUG("gt file path is: %s\n", path_to_gt.c_str());
  }

  // Load if we should save the total state to file
  // If so, then open the file and create folders as needed
  if (node->has_parameter("save_total_state")) {
    node->get_parameter<bool>("save_total_state", save_total_state);
  }
  if (save_total_state) {

    // files we will open
    std::string filepath_est = "state_estimate.txt";
    std::string filepath_std = "state_deviation.txt";
    std::string filepath_gt = "state_groundtruth.txt";
    if (node->has_parameter("filepath_est")) {
      node->get_parameter<std::string>("filepath_est", filepath_est);
    }
    if (node->has_parameter("filepath_std")) {
      node->get_parameter<std::string>("filepath_std", filepath_std);
    }
    if (node->has_parameter("filepath_gt")) {
      node->get_parameter<std::string>("filepath_gt", filepath_gt);
    }

    // If it exists, then delete it
    if (boost::filesystem::exists(filepath_est))
      boost::filesystem::remove(filepath_est);
    if (boost::filesystem::exists(filepath_std))
      boost::filesystem::remove(filepath_std);

    // Create folder path to this location if not exists
    boost::filesystem::create_directories(boost::filesystem::path(filepath_est.c_str()).parent_path());
    boost::filesystem::create_directories(boost::filesystem::path(filepath_std.c_str()).parent_path());

    // Open the files
    of_state_est.open(filepath_est.c_str());
    of_state_std.open(filepath_std.c_str());
    of_state_est << "# timestamp(s) q p v bg ba cam_imu_dt num_cam cam0_k cam0_d cam0_rot cam0_trans ... imu_model dw da tg wtoI atoI etc"
                 << std::endl;
    of_state_std << "# timestamp(s) q p v bg ba cam_imu_dt num_cam cam0_k cam0_d cam0_rot cam0_trans ... imu_model dw da tg wtoI atoI etc"
                 << std::endl;

    // Groundtruth if we are simulating
    if (_sim != nullptr) {
      if (boost::filesystem::exists(filepath_gt))
        boost::filesystem::remove(filepath_gt);
      boost::filesystem::create_directories(boost::filesystem::path(filepath_gt.c_str()).parent_path());
      of_state_gt.open(filepath_gt.c_str());
      of_state_gt << "# timestamp(s) q p v bg ba cam_imu_dt num_cam cam0_k cam0_d cam0_rot cam0_trans ... imu_model dw da tg wtoI atoI etc"
                  << std::endl;
    }
  }

  // Start thread for the image publishing
  if (_app->get_params().use_multi_threading_pubs) {
    thread_visualize = std::make_shared<std::thread>([this] {
      rclcpp::Rate loop_rate(20);
      while (rclcpp::ok() && run_visualize_thread) {
        publish_images();
        loop_rate.sleep();
      }
    });
  }
}

ROS2Visualizer::~ROS2Visualizer() {
  // Signal and join the visualize thread
  run_visualize_thread = false;
  if (thread_visualize && thread_visualize->joinable()) {
    thread_visualize->join();
  }

  // Manually reset all publishers and subscriptions
  it_pub_tracks.shutdown();
  it_pub_loop_img_depth.shutdown();
  it_pub_loop_img_depth_color.shutdown();
  pub_poseimu.reset();
  pub_odomimu.reset();
  pub_pathimu.reset();
  pub_points_msckf.reset();
  pub_points_slam.reset();
  pub_points_aruco.reset();
  pub_points_sim.reset();
  pub_loop_pose.reset();
  pub_loop_extrinsic.reset();
  pub_loop_point.reset();
  pub_loop_intrinsics.reset();
  pub_dynamic_arm_status.reset();
  pub_marker_status.reset();
  mTfBr.reset();
  sub_imu.reset();
  sub_marker_pose.reset();
  sub_dynamic_arm_pose.reset();
  for (auto &sub : subs_cam)
    sub.reset();
  for (auto &sync : sync_cam)
    sync.reset();
  for (auto &sub : sync_subs_cam)
    sub.reset();
  pub_pathgt.reset();
  pub_posegt.reset();
}

void ROS2Visualizer::setup_subscribers(std::shared_ptr<ov_core::YamlParser> parser) {

  // We need a valid parser
  assert(parser != nullptr);

  // Create imu subscriber (handle legacy ros param info)
  std::string topic_imu;
  _node->declare_parameter<std::string>("topic_imu", "/imu0");
  _node->get_parameter("topic_imu", topic_imu);
  parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);
  sub_imu = _node->create_subscription<sensor_msgs::msg::Imu>(topic_imu, rclcpp::SensorDataQoS(),
                                                              std::bind(&ROS2Visualizer::callback_inertial, this, std::placeholders::_1));
  PRINT_INFO("subscribing to IMU: %s\n", topic_imu.c_str());

  if (_app->get_params().marker_pose_options.enabled) {
    const std::string marker_topic = _app->get_params().marker_pose_options.topic;
    sub_marker_pose = _node->create_subscription<sensor_fusion_msgs::msg::MarkerPoseObservation>(
        marker_topic, rclcpp::QoS(10), std::bind(&ROS2Visualizer::callback_marker_pose, this, std::placeholders::_1));
    PRINT_INFO("subscribing to marker observations: %s\n", marker_topic.c_str());
  }
  if (_app->get_params().dynamic_arm_pose_options.enabled) {
    const std::string dynamic_arm_topic = _app->get_params().dynamic_arm_pose_options.topic;
    sub_dynamic_arm_pose = _node->create_subscription<sensor_fusion_msgs::msg::DynamicArmPoseObservation>(
        dynamic_arm_topic, rclcpp::QoS(10), std::bind(&ROS2Visualizer::callback_dynamic_arm_pose, this, std::placeholders::_1));
    PRINT_INFO("subscribing to dynamic arm observations: %s\n", dynamic_arm_topic.c_str());
  }

  // Logic for sync stereo subscriber
  // https://answers.ros.org/question/96346/subscribe-to-two-image_raws-with-one-function/?answer=96491#post-id-96491
  if (_app->get_params().state_options.num_cameras == 2) {
    // Read in the topics
    std::string cam_topic0, cam_topic1;
    _node->declare_parameter<std::string>("topic_camera" + std::to_string(0), "/cam" + std::to_string(0) + "/image_raw");
    _node->get_parameter("topic_camera" + std::to_string(0), cam_topic0);
    _node->declare_parameter<std::string>("topic_camera" + std::to_string(1), "/cam" + std::to_string(1) + "/image_raw");
    _node->get_parameter("topic_camera" + std::to_string(1), cam_topic1);
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(0), "rostopic", cam_topic0);
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(1), "rostopic", cam_topic1);
    // Create sync filter (they have unique pointers internally, so we have to use move logic here...)
    auto image_sub0 = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(_node, cam_topic0);
    auto image_sub1 = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(_node, cam_topic1);
    auto sync = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(10), *image_sub0, *image_sub1);
    sync->registerCallback(std::bind(&ROS2Visualizer::callback_stereo, this, std::placeholders::_1, std::placeholders::_2, 0, 1));
    // sync->registerCallback([](const sensor_msgs::msg::Image::SharedPtr msg0, const sensor_msgs::msg::Image::SharedPtr msg1)
    // {callback_stereo(msg0, msg1, 0, 1);});
    // sync->registerCallback(&callback_stereo2); // since the above two alternatives fail to compile for some reason
    // Append to our vector of subscribers
    sync_cam.push_back(sync);
    sync_subs_cam.push_back(image_sub0);
    sync_subs_cam.push_back(image_sub1);
    PRINT_INFO("subscribing to cam (stereo): %s\n", cam_topic0.c_str());
    PRINT_INFO("subscribing to cam (stereo): %s\n", cam_topic1.c_str());
  } else {
    // Now we should add any non-stereo callbacks here
    for (int i = 0; i < _app->get_params().state_options.num_cameras; i++) {
      // read in the topic
      std::string cam_topic;
      _node->declare_parameter<std::string>("topic_camera" + std::to_string(i), "/cam" + std::to_string(i) + "/image_raw");
      _node->get_parameter("topic_camera" + std::to_string(i), cam_topic);
      parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", cam_topic);
      // create subscriber
      // auto sub = _node->create_subscription<sensor_msgs::msg::Image>(
      //    cam_topic, rclcpp::SensorDataQoS(), std::bind(&ROS2Visualizer::callback_monocular, this, std::placeholders::_1, i));
      auto sub = _node->create_subscription<sensor_msgs::msg::Image>(
          cam_topic, 10, [this, i](const sensor_msgs::msg::Image::SharedPtr msg0) { callback_monocular(msg0, i); });
      subs_cam.push_back(sub);
      PRINT_INFO("subscribing to cam (mono): %s\n", cam_topic.c_str());
    }
  }
}

void ROS2Visualizer::visualize() {

  // Return if we have already visualized
  if (last_visualization_timestamp == _app->get_state()->_timestamp && _app->initialized())
    return;
  last_visualization_timestamp = _app->get_state()->_timestamp;

  // Start timing
  // boost::posix_time::ptime rT0_1, rT0_2;
  // rT0_1 = boost::posix_time::microsec_clock::local_time();

  // publish current image (only if not multi-threaded)
  if (!_app->get_params().use_multi_threading_pubs)
    publish_images();

  // Return if we have not inited
  if (!_app->initialized())
    return;

  // Save the start time of this dataset
  if (!start_time_set) {
    rT1 = boost::posix_time::microsec_clock::local_time();
    start_time_set = true;
  }

  // publish state
  publish_state();

  // publish points
  publish_features();

  // Publish gt if we have it
  publish_groundtruth();

  // Publish keyframe information
  publish_loopclosure_information();

  // Save total state
  if (save_total_state) {
    ROSVisualizerHelper::sim_save_total_state_to_file(_app->get_state(), _sim, of_state_est, of_state_std, of_state_gt);
  }

  // Print how much time it took to publish / displaying things
  // rT0_2 = boost::posix_time::microsec_clock::local_time();
  // double time_total = (rT0_2 - rT0_1).total_microseconds() * 1e-6;
  // PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for visualization\n" RESET, time_total);
}

void ROS2Visualizer::visualize_odometry(double timestamp) {

  // Return if we have not inited and a second has passes
  if (!_app->initialized() || (timestamp - _app->initialized_time()) < 1)
    return;

  // Get fast propagate state at the desired timestamp
  std::shared_ptr<State> state = _app->get_state();
  Eigen::Matrix<double, 13, 1> state_plus = Eigen::Matrix<double, 13, 1>::Zero();
  Eigen::Matrix<double, 12, 12> cov_plus = Eigen::Matrix<double, 12, 12>::Zero();
  if (!_app->get_propagator()->fast_state_propagate(state, timestamp, state_plus, cov_plus))
    return;

  // Publish our odometry message if requested
  if (pub_odomimu->get_subscription_count() != 0) {

    // Our odometry message
    nav_msgs::msg::Odometry odomIinM;
    odomIinM.header.stamp = ROSVisualizerHelper::get_time_from_seconds(timestamp);
    odomIinM.header.frame_id = global_frame_id;

    // The POSE component (orientation and position)
    odomIinM.pose.pose.orientation.x = state_plus(0);
    odomIinM.pose.pose.orientation.y = state_plus(1);
    odomIinM.pose.pose.orientation.z = state_plus(2);
    odomIinM.pose.pose.orientation.w = state_plus(3);
    odomIinM.pose.pose.position.x = state_plus(4);
    odomIinM.pose.pose.position.y = state_plus(5);
    odomIinM.pose.pose.position.z = state_plus(6);

    // The TWIST component (angular and linear velocities)
    odomIinM.child_frame_id = imu_frame_id;
    odomIinM.twist.twist.linear.x = state_plus(7);   // vel in local frame
    odomIinM.twist.twist.linear.y = state_plus(8);   // vel in local frame
    odomIinM.twist.twist.linear.z = state_plus(9);   // vel in local frame
    odomIinM.twist.twist.angular.x = state_plus(10); // we do not estimate this...
    odomIinM.twist.twist.angular.y = state_plus(11); // we do not estimate this...
    odomIinM.twist.twist.angular.z = state_plus(12); // we do not estimate this...

    // Finally set the covariance in the message (in the order position then orientation as per ros convention)
    Eigen::Matrix<double, 12, 12> Phi = Eigen::Matrix<double, 12, 12>::Zero();
    Phi.block(0, 3, 3, 3).setIdentity();
    Phi.block(3, 0, 3, 3).setIdentity();
    Phi.block(6, 6, 6, 6).setIdentity();
    cov_plus = Phi * cov_plus * Phi.transpose();
    for (int r = 0; r < 6; r++) {
      for (int c = 0; c < 6; c++) {
        odomIinM.pose.covariance[6 * r + c] = cov_plus(r, c);
      }
    }
    for (int r = 0; r < 6; r++) {
      for (int c = 0; c < 6; c++) {
        odomIinM.twist.covariance[6 * r + c] = cov_plus(r + 6, c + 6);
      }
    }
    pub_odomimu->publish(odomIinM);
  }

  // Publish our transform on TF
  // NOTE: since we use JPL we have an implicit conversion to Hamilton when we publish
  // NOTE: a rotation from GtoI in JPL has the same xyzw as a ItoG Hamilton rotation
  auto odom_pose = std::make_shared<ov_type::PoseJPL>();
  odom_pose->set_value(state_plus.block(0, 0, 7, 1));
  geometry_msgs::msg::TransformStamped trans = ROSVisualizerHelper::get_stamped_transform_from_pose(_node, odom_pose, false);
  trans.header.stamp = _node->now();
  trans.header.frame_id = global_frame_id;
  trans.child_frame_id = imu_frame_id;
  if (publish_global2imu_tf) {
    mTfBr->sendTransform(trans);
  }

  // Loop through each camera calibration and publish it
  for (const auto &calib : state->_calib_IMUtoCAM) {
    geometry_msgs::msg::TransformStamped trans_calib = ROSVisualizerHelper::get_stamped_transform_from_pose(_node, calib.second, true);
    trans_calib.header.stamp = _node->now();
    trans_calib.header.frame_id = imu_frame_id;
    trans_calib.child_frame_id = camera_frame_prefix + std::to_string(calib.first);
    if (publish_calibration_tf) {
      mTfBr->sendTransform(trans_calib);
    }
  }
}

void ROS2Visualizer::visualize_final() {

  // Final time offset value
  if (_app->get_state()->_options.do_calib_camera_timeoffset) {
    PRINT_INFO(REDPURPLE "camera-imu timeoffset = %.5f\n\n" RESET, _app->get_state()->_calib_dt_CAMtoIMU->value()(0));
  }

  // Final camera intrinsics
  if (_app->get_state()->_options.do_calib_camera_intrinsics) {
    for (int i = 0; i < _app->get_state()->_options.num_cameras; i++) {
      std::shared_ptr<Vec> calib = _app->get_state()->_cam_intrinsics.at(i);
      PRINT_INFO(REDPURPLE "cam%d intrinsics:\n" RESET, (int)i);
      PRINT_INFO(REDPURPLE "%.3f,%.3f,%.3f,%.3f\n" RESET, calib->value()(0), calib->value()(1), calib->value()(2), calib->value()(3));
      PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,%.5f\n\n" RESET, calib->value()(4), calib->value()(5), calib->value()(6), calib->value()(7));
    }
  }

  // Final camera extrinsics
  if (_app->get_state()->_options.do_calib_camera_pose) {
    for (int i = 0; i < _app->get_state()->_options.num_cameras; i++) {
      std::shared_ptr<PoseJPL> calib = _app->get_state()->_calib_IMUtoCAM.at(i);
      Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
      T_CtoI.block(0, 0, 3, 3) = quat_2_Rot(calib->quat()).transpose();
      T_CtoI.block(0, 3, 3, 1) = -T_CtoI.block(0, 0, 3, 3) * calib->pos();
      PRINT_INFO(REDPURPLE "T_C%dtoI:\n" RESET, i);
      PRINT_INFO(REDPURPLE "%.3f,%.3f,%.3f,%.3f,\n" RESET, T_CtoI(0, 0), T_CtoI(0, 1), T_CtoI(0, 2), T_CtoI(0, 3));
      PRINT_INFO(REDPURPLE "%.3f,%.3f,%.3f,%.3f,\n" RESET, T_CtoI(1, 0), T_CtoI(1, 1), T_CtoI(1, 2), T_CtoI(1, 3));
      PRINT_INFO(REDPURPLE "%.3f,%.3f,%.3f,%.3f,\n" RESET, T_CtoI(2, 0), T_CtoI(2, 1), T_CtoI(2, 2), T_CtoI(2, 3));
      PRINT_INFO(REDPURPLE "%.3f,%.3f,%.3f,%.3f\n\n" RESET, T_CtoI(3, 0), T_CtoI(3, 1), T_CtoI(3, 2), T_CtoI(3, 3));
    }
  }

  // IMU intrinsics
  if (_app->get_state()->_options.do_calib_imu_intrinsics) {
    Eigen::Matrix3d Dw = State::Dm(_app->get_state()->_options.imu_model, _app->get_state()->_calib_imu_dw->value());
    Eigen::Matrix3d Da = State::Dm(_app->get_state()->_options.imu_model, _app->get_state()->_calib_imu_da->value());
    Eigen::Matrix3d Tw = Dw.colPivHouseholderQr().solve(Eigen::Matrix3d::Identity());
    Eigen::Matrix3d Ta = Da.colPivHouseholderQr().solve(Eigen::Matrix3d::Identity());
    Eigen::Matrix3d R_IMUtoACC = _app->get_state()->_calib_imu_ACCtoIMU->Rot().transpose();
    Eigen::Matrix3d R_IMUtoGYRO = _app->get_state()->_calib_imu_GYROtoIMU->Rot().transpose();
    PRINT_INFO(REDPURPLE "Tw:\n" RESET);
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, Tw(0, 0), Tw(0, 1), Tw(0, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, Tw(1, 0), Tw(1, 1), Tw(1, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f\n\n" RESET, Tw(2, 0), Tw(2, 1), Tw(2, 2));
    PRINT_INFO(REDPURPLE "Ta:\n" RESET);
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, Ta(0, 0), Ta(0, 1), Ta(0, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, Ta(1, 0), Ta(1, 1), Ta(1, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f\n\n" RESET, Ta(2, 0), Ta(2, 1), Ta(2, 2));
    PRINT_INFO(REDPURPLE "R_IMUtoACC:\n" RESET);
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, R_IMUtoACC(0, 0), R_IMUtoACC(0, 1), R_IMUtoACC(0, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, R_IMUtoACC(1, 0), R_IMUtoACC(1, 1), R_IMUtoACC(1, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f\n\n" RESET, R_IMUtoACC(2, 0), R_IMUtoACC(2, 1), R_IMUtoACC(2, 2));
    PRINT_INFO(REDPURPLE "R_IMUtoGYRO:\n" RESET);
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, R_IMUtoGYRO(0, 0), R_IMUtoGYRO(0, 1), R_IMUtoGYRO(0, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, R_IMUtoGYRO(1, 0), R_IMUtoGYRO(1, 1), R_IMUtoGYRO(1, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f\n\n" RESET, R_IMUtoGYRO(2, 0), R_IMUtoGYRO(2, 1), R_IMUtoGYRO(2, 2));
  }

  // IMU intrinsics gravity sensitivity
  if (_app->get_state()->_options.do_calib_imu_g_sensitivity) {
    Eigen::Matrix3d Tg = State::Tg(_app->get_state()->_calib_imu_tg->value());
    PRINT_INFO(REDPURPLE "Tg:\n" RESET);
    PRINT_INFO(REDPURPLE "%.6f,%.6f,%.6f,\n" RESET, Tg(0, 0), Tg(0, 1), Tg(0, 2));
    PRINT_INFO(REDPURPLE "%.6f,%.6f,%.6f,\n" RESET, Tg(1, 0), Tg(1, 1), Tg(1, 2));
    PRINT_INFO(REDPURPLE "%.6f,%.6f,%.6f\n\n" RESET, Tg(2, 0), Tg(2, 1), Tg(2, 2));
  }

  // Publish RMSE if we have it
  if (!gt_states.empty()) {
    PRINT_INFO(REDPURPLE "RMSE: %.3f (deg) orientation\n" RESET, std::sqrt(summed_mse_ori / summed_number));
    PRINT_INFO(REDPURPLE "RMSE: %.3f (m) position\n\n" RESET, std::sqrt(summed_mse_pos / summed_number));
  }

  // Publish RMSE and NEES if doing simulation
  if (_sim != nullptr) {
    PRINT_INFO(REDPURPLE "RMSE: %.3f (deg) orientation\n" RESET, std::sqrt(summed_mse_ori / summed_number));
    PRINT_INFO(REDPURPLE "RMSE: %.3f (m) position\n\n" RESET, std::sqrt(summed_mse_pos / summed_number));
    PRINT_INFO(REDPURPLE "NEES: %.3f (deg) orientation\n" RESET, summed_nees_ori / summed_number);
    PRINT_INFO(REDPURPLE "NEES: %.3f (m) position\n\n" RESET, summed_nees_pos / summed_number);
  }

  // Print the total time
  rT2 = boost::posix_time::microsec_clock::local_time();
  PRINT_INFO(REDPURPLE "TIME: %.3f seconds\n\n" RESET, (rT2 - rT1).total_microseconds() * 1e-6);
}

void ROS2Visualizer::callback_inertial(const sensor_msgs::msg::Imu::SharedPtr msg) {

  // convert into correct format
  ov_core::ImuData message;
  message.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  message.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  message.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  // send it to our VIO system
  _app->feed_measurement_imu(message);
  visualize_odometry(message.timestamp);

  // If the processing queue is currently active / running just return so we can keep getting measurements
  // Otherwise create a second thread to do our update in an async manor
  // The visualization of the state, images, and features will be synchronous with the update!
  bool expected = false;
  if (!thread_update_running.compare_exchange_strong(expected, true))
    return;
  const double imu_timestamp = message.timestamp;
  std::thread thread([this, imu_timestamp] {
    // Lock on the queue (prevents new images from appending)
    std::lock_guard<std::mutex> lck(camera_queue_mtx);

    // Count how many unique image streams
    std::map<int, bool> unique_cam_ids;
    for (const auto &cam_msg : camera_queue) {
      unique_cam_ids[cam_msg.sensor_ids.at(0)] = true;
    }

    // If we do not have enough unique cameras then we need to wait
    // We should wait till we have one of each camera to ensure we propagate in the correct order
    const auto &params = _app->get_params();
    size_t num_unique_cameras = (params.state_options.num_cameras == 2) ? 1 : params.state_options.num_cameras;
    if (unique_cam_ids.size() == num_unique_cameras) {

      // Loop through our queue and see if we are able to process any of our camera measurements
      // We are able to process if we have at least one IMU measurement greater than the camera time
      double timestamp_imu_inC = imu_timestamp - _app->get_state()->_calib_dt_CAMtoIMU->value()(0);
      while (!camera_queue.empty() && camera_queue.at(0).timestamp < timestamp_imu_inC) {
        auto rT0_1 = boost::posix_time::microsec_clock::local_time();
        double update_dt = 100.0 * (timestamp_imu_inC - camera_queue.at(0).timestamp);
        _app->feed_measurement_camera(camera_queue.at(0));
        process_marker_queue();
        process_dynamic_arm_queue();
        visualize();
        camera_queue.pop_front();
        auto rT0_2 = boost::posix_time::microsec_clock::local_time();
        double time_total = (rT0_2 - rT0_1).total_microseconds() * 1e-6;
        PRINT_INFO(BLUE "[TIME]: %.4f seconds total (%.1f hz, %.2f ms behind)\n" RESET, time_total, 1.0 / time_total, update_dt);
      }
    }
    thread_update_running = false;
  });

  // If we are single threaded, then run single threaded
  // Otherwise detach this thread so it runs in the background!
  if (!_app->get_params().use_multi_threading_subs) {
    thread.join();
  } else {
    thread.detach();
  }
}

void ROS2Visualizer::callback_monocular(const sensor_msgs::msg::Image::SharedPtr msg0, int cam_id0) {

  // Check if we should drop this image
  double timestamp = msg0->header.stamp.sec + msg0->header.stamp.nanosec * 1e-9;
  double time_delta = 1.0 / _app->get_params().track_frequency;
  if (camera_last_timestamp.find(cam_id0) != camera_last_timestamp.end() && timestamp < camera_last_timestamp.at(cam_id0) + time_delta) {
    return;
  }
  camera_last_timestamp[cam_id0] = timestamp;

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    PRINT_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Create the measurement
  ov_core::CameraData message;
  message.timestamp = cv_ptr->header.stamp.sec + cv_ptr->header.stamp.nanosec * 1e-9;
  message.sensor_ids.push_back(cam_id0);
  message.images.push_back(cv_ptr->image.clone());

  // Load the mask if we are using it, else it is empty
  // TODO: in the future we should get this from external pixel segmentation
  if (_app->get_params().use_mask) {
    message.masks.push_back(_app->get_params().masks.at(cam_id0));
  } else {
    message.masks.push_back(cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1));
  }

  // append it to our queue of images
  std::lock_guard<std::mutex> lck(camera_queue_mtx);
  camera_queue.push_back(message);
  std::sort(camera_queue.begin(), camera_queue.end());
  const size_t max_camera_queue_size = 3;
  while (camera_queue.size() > max_camera_queue_size) {
    PRINT_WARNING(YELLOW "[QUEUE]: dropping stale camera frame at %.6f (queue size %zu > %zu)\n" RESET, camera_queue.front().timestamp,
                  camera_queue.size(), max_camera_queue_size);
    camera_queue.pop_front();
  }
}

void ROS2Visualizer::callback_stereo(const sensor_msgs::msg::Image::ConstSharedPtr msg0, const sensor_msgs::msg::Image::ConstSharedPtr msg1,
                                     int cam_id0, int cam_id1) {

  // Check if we should drop this image
  double timestamp = msg0->header.stamp.sec + msg0->header.stamp.nanosec * 1e-9;
  double time_delta = 1.0 / _app->get_params().track_frequency;
  if (camera_last_timestamp.find(cam_id0) != camera_last_timestamp.end() && timestamp < camera_last_timestamp.at(cam_id0) + time_delta) {
    return;
  }
  camera_last_timestamp[cam_id0] = timestamp;

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr0;
  try {
    cv_ptr0 = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    PRINT_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr1;
  try {
    cv_ptr1 = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    PRINT_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Create the measurement
  ov_core::CameraData message;
  message.timestamp = cv_ptr0->header.stamp.sec + cv_ptr0->header.stamp.nanosec * 1e-9;
  message.sensor_ids.push_back(cam_id0);
  message.sensor_ids.push_back(cam_id1);
  message.images.push_back(cv_ptr0->image.clone());
  message.images.push_back(cv_ptr1->image.clone());

  // Load the mask if we are using it, else it is empty
  // TODO: in the future we should get this from external pixel segmentation
  if (_app->get_params().use_mask) {
    message.masks.push_back(_app->get_params().masks.at(cam_id0));
    message.masks.push_back(_app->get_params().masks.at(cam_id1));
  } else {
    // message.masks.push_back(cv::Mat(cv_ptr0->image.rows, cv_ptr0->image.cols, CV_8UC1, cv::Scalar(255)));
    message.masks.push_back(cv::Mat::zeros(cv_ptr0->image.rows, cv_ptr0->image.cols, CV_8UC1));
    message.masks.push_back(cv::Mat::zeros(cv_ptr1->image.rows, cv_ptr1->image.cols, CV_8UC1));
  }

  // append it to our queue of images
  std::lock_guard<std::mutex> lck(camera_queue_mtx);
  camera_queue.push_back(message);
  std::sort(camera_queue.begin(), camera_queue.end());
  const size_t max_camera_queue_size = 3;
  while (camera_queue.size() > max_camera_queue_size) {
    PRINT_WARNING(YELLOW "[QUEUE]: dropping stale stereo camera frame at %.6f (queue size %zu > %zu)\n" RESET,
                  camera_queue.front().timestamp, camera_queue.size(), max_camera_queue_size);
    camera_queue.pop_front();
  }
}

void ROS2Visualizer::callback_marker_pose(const sensor_fusion_msgs::msg::MarkerPoseObservation::SharedPtr msg) {

  MarkerPoseMeasurement measurement;
  measurement.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  measurement.marker_id = msg->marker_id;
  measurement.frame_id = msg->header.frame_id;
  measurement.marker_frame = msg->marker_frame;
  measurement.target_frame = msg->target_frame;
  measurement.p_IinG << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

  const Eigen::Matrix3d R_GtoI_pose = quat_xyzw_to_rot(msg->pose.pose.orientation);
  measurement.R_GtoI = R_GtoI_pose.transpose();

  Eigen::Matrix<double, 6, 6> covariance_ros = Eigen::Matrix<double, 6, 6>::Zero();
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      covariance_ros(r, c) = msg->pose.covariance[6 * r + c];
    }
  }
  const int reorder[6] = {3, 4, 5, 0, 1, 2};
  measurement.covariance.setZero();
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      measurement.covariance(r, c) = covariance_ros(reorder[r], reorder[c]);
    }
  }

  measurement.hard_gate_passed = msg->hard_gate_passed;
  measurement.hard_gate_status = msg->hard_gate_status;
  measurement.stable = msg->stable;
  measurement.stable_frames = msg->stable_frames;
  measurement.stability_factor = msg->stability_factor;
  measurement.reprojection_error_px = msg->reprojection_error_px;
  measurement.distance_m = msg->distance_m;
  measurement.view_angle_deg = msg->view_angle_deg;
  measurement.area_px2 = msg->area_px2;
  measurement.side_mean_px = msg->side_mean_px;
  measurement.side_min_px = msg->side_min_px;
  measurement.geometry_score = msg->geometry_score;
  measurement.covariance_sigma_px = msg->covariance_sigma_px;

  std::lock_guard<std::mutex> lck(marker_queue_mtx);
  marker_queue.push_back(measurement);
  std::sort(marker_queue.begin(), marker_queue.end(), [](const MarkerPoseMeasurement &a, const MarkerPoseMeasurement &b) {
    return a.timestamp < b.timestamp;
  });
}

void ROS2Visualizer::callback_dynamic_arm_pose(const sensor_fusion_msgs::msg::DynamicArmPoseObservation::SharedPtr msg) {

  DynamicArmPoseMeasurement measurement;
  measurement.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  measurement.marker_id = msg->marker_id;
  measurement.frame_id = msg->header.frame_id;
  measurement.source_camera_frame = msg->source_camera_frame;
  measurement.marker_frame = msg->marker_frame;
  measurement.target_frame = msg->target_frame;
  measurement.p_IinG << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

  const Eigen::Matrix3d R_GtoI_pose = quat_xyzw_to_rot(msg->pose.pose.orientation);
  measurement.R_GtoI = R_GtoI_pose.transpose();

  std::array<double, 36> covariance_ros{};
  for (int i = 0; i < 36; i++) {
    covariance_ros.at(i) = msg->pose.covariance[i];
  }
  measurement.covariance = UpdaterDynamicArmPose::ros_covariance_to_update_order(covariance_ros);

  measurement.hard_gate_passed = msg->hard_gate_passed;
  measurement.hard_gate_status = msg->hard_gate_status;
  measurement.stable = msg->stable;
  measurement.stable_frames = msg->stable_frames;
  measurement.stability_factor = msg->stability_factor;
  measurement.reprojection_error_px = msg->reprojection_error_px;
  measurement.distance_m = msg->distance_m;
  measurement.view_angle_deg = msg->view_angle_deg;
  measurement.area_px2 = msg->area_px2;
  measurement.side_mean_px = msg->side_mean_px;
  measurement.side_min_px = msg->side_min_px;
  measurement.geometry_score = msg->geometry_score;
  measurement.covariance_sigma_px = msg->covariance_sigma_px;
  measurement.head_pose_match_dt_s = msg->head_pose_match_dt_s;
  measurement.head_pose_match_mode = msg->head_pose_match_mode;
  measurement.head_pose_source_topic = msg->head_pose_source_topic;
  measurement.head_pose_source_type = msg->head_pose_source_type;
  measurement.head_pose_time_offset_s = msg->head_pose_time_offset_s;
  measurement.dynamic_covariance_fallback = msg->dynamic_covariance_fallback;
  measurement.head_covariance_fallback = msg->head_covariance_fallback;
  measurement.extrinsic_covariance_source = msg->extrinsic_covariance_source;

  std::lock_guard<std::mutex> lck(dynamic_arm_queue_mtx);
  dynamic_arm_queue.push_back(measurement);
  std::sort(dynamic_arm_queue.begin(), dynamic_arm_queue.end(),
            [](const DynamicArmPoseMeasurement &a, const DynamicArmPoseMeasurement &b) { return a.timestamp < b.timestamp; });
}

void ROS2Visualizer::process_marker_queue() {

  if (!_app->get_params().marker_pose_options.enabled) {
    return;
  }

  std::lock_guard<std::mutex> lck(marker_queue_mtx);
  if (!_app->initialized()) {
    const double newest_to_keep = _node->now().seconds() - 2.0;
    while (!marker_queue.empty() && marker_queue.front().timestamp < newest_to_keep) {
      MarkerPoseUpdateResult result;
      result.reason = "not_initialized_queue_drop";
      publish_marker_status(marker_queue.front(), result, "not_initialized_queue_drop");
      marker_queue.pop_front();
    }
    return;
  }

  const double state_timestamp = _app->get_state()->_timestamp;
  const double tolerance = _app->get_params().marker_pose_options.time_tolerance_s;
  while (!marker_queue.empty()) {
    const MarkerPoseMeasurement measurement = marker_queue.front();
    if (measurement.timestamp < state_timestamp - tolerance) {
      PRINT_DEBUG(YELLOW "[MARKER]: visualizer dropping stale marker %.6f for state %.6f\n" RESET, measurement.timestamp, state_timestamp);
      MarkerPoseUpdateResult result;
      result.reason = "visualizer_stale";
      publish_marker_status(measurement, result, "visualizer_stale");
      marker_queue.pop_front();
      continue;
    }
    if (measurement.timestamp > state_timestamp + tolerance) {
      break;
    }
    marker_queue.pop_front();
    MarkerPoseUpdateResult result = _app->feed_measurement_marker(measurement);
    publish_marker_status(measurement, result);
  }
}

void ROS2Visualizer::process_dynamic_arm_queue() {

  if (!_app->get_params().dynamic_arm_pose_options.enabled) {
    return;
  }

  std::lock_guard<std::mutex> lck(dynamic_arm_queue_mtx);
  if (!_app->initialized()) {
    const double newest_to_keep = _node->now().seconds() - 2.0;
    while (!dynamic_arm_queue.empty() && dynamic_arm_queue.front().timestamp < newest_to_keep) {
      DynamicArmPoseUpdateResult result;
      result.reason = "not_initialized_queue_drop";
      publish_dynamic_arm_status(dynamic_arm_queue.front(), result, "not_initialized_queue_drop");
      dynamic_arm_queue.pop_front();
    }
    return;
  }

  const double state_timestamp = _app->get_state()->_timestamp;
  const double tolerance = _app->get_params().dynamic_arm_pose_options.time_tolerance_s;
  while (!dynamic_arm_queue.empty()) {
    const DynamicArmPoseMeasurement measurement = dynamic_arm_queue.front();
    if (measurement.timestamp < state_timestamp - tolerance) {
      DynamicArmPoseUpdateResult result;
      result.reason = "visualizer_stale";
      publish_dynamic_arm_status(measurement, result, "visualizer_stale");
      PRINT_DEBUG(YELLOW "[DYNAMIC_ARM]: visualizer dropping stale measurement %.6f for state %.6f\n" RESET, measurement.timestamp,
                  state_timestamp);
      dynamic_arm_queue.pop_front();
      continue;
    }
    if (measurement.timestamp > state_timestamp + tolerance) {
      break;
    }
    dynamic_arm_queue.pop_front();
    DynamicArmPoseUpdateResult result = _app->feed_measurement_dynamic_arm_pose(measurement);
    publish_dynamic_arm_status(measurement, result);
  }
}

void ROS2Visualizer::publish_dynamic_arm_status(const DynamicArmPoseMeasurement &measurement,
                                                const DynamicArmPoseUpdateResult &result,
                                                const std::string &queue_reason) {
  if (pub_dynamic_arm_status == nullptr) {
    return;
  }
  const auto options = _app->get_params().dynamic_arm_pose_options;
  const double state_timestamp = (_app->get_state() != nullptr) ? _app->get_state()->_timestamp : -1.0;
  const double fixed_dt =
      (_app->last_fixed_marker_update_time() >= 0.0) ? measurement.timestamp - _app->last_fixed_marker_update_time() : -1.0;
  const bool fixed_skip_active =
      _app->last_fixed_marker_update_time() >= 0.0 && fixed_dt < options.skip_after_fixed_marker_s;
  const bool reanchor_fixed_skip_active =
      _app->last_fixed_marker_update_time() >= 0.0 && fixed_dt < options.reanchor_skip_after_fixed_marker_s;

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6);
  ss << "{";
  ss << "\"event_type\":\"dynamic_arm_pose_update\"";
  ss << ",\"stamp\":" << measurement.timestamp;
  ss << ",\"state_timestamp\":" << state_timestamp;
  ss << ",\"accepted\":" << (result.accepted ? "true" : "false");
  ss << ",\"state_updated\":" << (result.state_updated ? "true" : "false");
  ss << ",\"reason\":\"" << json_escape(queue_reason.empty() ? result.reason : queue_reason) << "\"";
  ss << ",\"marker_id\":" << measurement.marker_id;
  ss << ",\"frame_id\":\"" << json_escape(measurement.frame_id) << "\"";
  ss << ",\"source_camera_frame\":\"" << json_escape(measurement.source_camera_frame) << "\"";
  ss << ",\"marker_frame\":\"" << json_escape(measurement.marker_frame) << "\"";
  ss << ",\"target_frame\":\"" << json_escape(measurement.target_frame) << "\"";
  ss << ",\"chi2\":" << result.chi2;
  ss << ",\"innovation_translation_m\":" << result.translation_norm_m;
  ss << ",\"innovation_rotation_deg\":" << result.rotation_deg;
  ss << ",\"noise_multiplier\":" << options.noise_multiplier;
  ss << ",\"measurement_only\":" << (options.measurement_only ? "true" : "false");
  ss << ",\"last_fixed_marker_update_dt_s\":" << fixed_dt;
  ss << ",\"fixed_marker_skip_active\":" << (fixed_skip_active ? "true" : "false");
  ss << ",\"reanchor_fixed_skip_active\":" << ((result.reanchor_fixed_skip_active || reanchor_fixed_skip_active) ? "true" : "false");
  ss << ",\"would_dynamic_initial_lock\":" << (result.would_dynamic_initial_lock ? "true" : "false");
  ss << ",\"dynamic_initial_lock_performed\":" << (result.dynamic_initial_lock_performed ? "true" : "false");
  ss << ",\"would_dynamic_reanchor\":" << (result.would_dynamic_reanchor ? "true" : "false");
  ss << ",\"dynamic_reanchor_performed\":" << (result.dynamic_reanchor_performed ? "true" : "false");
  ss << ",\"reanchor_sample_count\":" << result.reanchor_sample_count;
  ss << ",\"reanchor_sample_span_s\":" << result.reanchor_sample_span_s;
  ss << ",\"reanchor_velocity_norm_mps\":" << result.reanchor_velocity_norm_mps;
  ss << ",\"reanchor_sample_translation_std_m\":" << result.reanchor_sample_translation_std_m;
  ss << ",\"reanchor_sample_rotation_std_deg\":" << result.reanchor_sample_rotation_std_deg;
  ss << ",\"reanchor_cooldown_active\":" << (result.reanchor_cooldown_active ? "true" : "false");
  ss << ",\"dynamic_covariance_fallback\":" << (measurement.dynamic_covariance_fallback ? "true" : "false");
  ss << ",\"head_covariance_fallback\":" << (measurement.head_covariance_fallback ? "true" : "false");
  ss << ",\"extrinsic_covariance_source\":\"" << json_escape(measurement.extrinsic_covariance_source) << "\"";
  ss << ",\"head_pose_match_dt_s\":" << measurement.head_pose_match_dt_s;
  ss << ",\"head_pose_match_mode\":\"" << json_escape(measurement.head_pose_match_mode) << "\"";
  ss << ",\"head_pose_source_topic\":\"" << json_escape(measurement.head_pose_source_topic) << "\"";
  ss << ",\"head_pose_source_type\":\"" << json_escape(measurement.head_pose_source_type) << "\"";
  ss << ",\"head_pose_time_offset_s\":" << measurement.head_pose_time_offset_s;
  ss << ",\"std_roll_deg\":" << 180.0 / M_PI * covariance_std(measurement.covariance, 0);
  ss << ",\"std_pitch_deg\":" << 180.0 / M_PI * covariance_std(measurement.covariance, 1);
  ss << ",\"std_yaw_deg\":" << 180.0 / M_PI * covariance_std(measurement.covariance, 2);
  ss << ",\"std_x_m\":" << covariance_std(measurement.covariance, 3);
  ss << ",\"std_y_m\":" << covariance_std(measurement.covariance, 4);
  ss << ",\"std_z_m\":" << covariance_std(measurement.covariance, 5);
  ss << "}";

  std_msgs::msg::String msg;
  msg.data = ss.str();
  pub_dynamic_arm_status->publish(msg);
}

void ROS2Visualizer::publish_marker_status(const MarkerPoseMeasurement &measurement,
                                           const MarkerPoseUpdateResult &result,
                                           const std::string &queue_reason) {
  if (pub_marker_status == nullptr) {
    return;
  }
  const double state_timestamp = (_app->get_state() != nullptr) ? _app->get_state()->_timestamp : -1.0;
  const double dt = (state_timestamp >= 0.0) ? measurement.timestamp - state_timestamp : 0.0;
  const double fixed_dt =
      (_app->last_fixed_marker_update_time() >= 0.0) ? measurement.timestamp - _app->last_fixed_marker_update_time() : -1.0;

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6);
  ss << "{";
  ss << "\"event_type\":\"marker_pose_update\"";
  ss << ",\"stamp\":" << measurement.timestamp;
  ss << ",\"state_timestamp\":" << state_timestamp;
  ss << ",\"dt\":" << dt;
  ss << ",\"accepted\":" << (result.accepted ? "true" : "false");
  ss << ",\"state_updated\":" << (result.state_updated ? "true" : "false");
  ss << ",\"reason\":\"" << json_escape(queue_reason.empty() ? result.reason : queue_reason) << "\"";
  ss << ",\"marker_id\":" << measurement.marker_id;
  ss << ",\"frame_id\":\"" << json_escape(measurement.frame_id) << "\"";
  ss << ",\"marker_frame\":\"" << json_escape(measurement.marker_frame) << "\"";
  ss << ",\"target_frame\":\"" << json_escape(measurement.target_frame) << "\"";
  ss << ",\"stable\":" << (measurement.stable ? "true" : "false");
  ss << ",\"stable_frames\":" << measurement.stable_frames;
  ss << ",\"hard_gate_passed\":" << (measurement.hard_gate_passed ? "true" : "false");
  ss << ",\"hard_gate_status\":\"" << json_escape(measurement.hard_gate_status) << "\"";
  ss << ",\"chi2\":" << result.chi2;
  ss << ",\"innovation_translation_m\":" << result.translation_norm_m;
  ss << ",\"innovation_rotation_deg\":" << result.rotation_deg;
  ss << ",\"noise_multiplier\":" << _app->get_params().marker_pose_options.noise_multiplier;
  ss << ",\"chi2_gate\":" << _app->get_params().marker_pose_options.chi2_gate;
  ss << ",\"max_update_translation_m\":" << _app->get_params().marker_pose_options.max_update_translation_m;
  ss << ",\"max_update_rotation_deg\":" << _app->get_params().marker_pose_options.max_update_rotation_deg;
  ss << ",\"reset_requested\":" << (result.reset_requested ? "true" : "false");
  ss << ",\"reset_reason\":\"" << json_escape(result.reset_reason) << "\"";
  ss << ",\"reset_performed\":" << (result.reset_performed ? "true" : "false");
  ss << ",\"reset_skipped_velocity_fit\":" << (result.reset_skipped_velocity_fit ? "true" : "false");
  ss << ",\"velocity_fit_passed\":" << (result.velocity_fit_passed ? "true" : "false");
  ss << ",\"velocity_fit_sample_count\":" << result.velocity_fit_sample_count;
  ss << ",\"velocity_fit_sample_span_s\":" << result.velocity_fit_sample_span_s;
  ss << ",\"velocity_fit_speed_mps\":" << result.velocity_fit_speed_mps;
  ss << ",\"marker_map_initialized\":" << (result.marker_map_initialized ? "true" : "false");
  ss << ",\"last_fixed_marker_update_dt_s\":" << fixed_dt;
  ss << ",\"reprojection_error_px\":" << measurement.reprojection_error_px;
  ss << ",\"distance_m\":" << measurement.distance_m;
  ss << ",\"view_angle_deg\":" << measurement.view_angle_deg;
  ss << ",\"area_px2\":" << measurement.area_px2;
  ss << ",\"side_mean_px\":" << measurement.side_mean_px;
  ss << ",\"side_min_px\":" << measurement.side_min_px;
  ss << ",\"geometry_score\":" << measurement.geometry_score;
  ss << ",\"covariance_sigma_px\":" << measurement.covariance_sigma_px;
  ss << ",\"stability_factor\":" << measurement.stability_factor;
  ss << ",\"std_roll_deg\":" << 180.0 / M_PI * covariance_std(measurement.covariance, 0);
  ss << ",\"std_pitch_deg\":" << 180.0 / M_PI * covariance_std(measurement.covariance, 1);
  ss << ",\"std_yaw_deg\":" << 180.0 / M_PI * covariance_std(measurement.covariance, 2);
  ss << ",\"std_x_m\":" << covariance_std(measurement.covariance, 3);
  ss << ",\"std_y_m\":" << covariance_std(measurement.covariance, 4);
  ss << ",\"std_z_m\":" << covariance_std(measurement.covariance, 5);
  ss << "}";

  std_msgs::msg::String msg;
  msg.data = ss.str();
  pub_marker_status->publish(msg);
}

void ROS2Visualizer::publish_state() {

  // Get the current state
  std::shared_ptr<State> state = _app->get_state();
  if (_app->get_params().marker_pose_options.enabled && _app->marker_global_initialized() && !marker_path_has_locked) {
    poses_imu.clear();
    marker_path_has_locked = true;
  }
  if (pub_marker_map_locked != nullptr) {
    std_msgs::msg::Bool marker_map_locked_msg;
    marker_map_locked_msg.data = _app->marker_global_initialized();
    pub_marker_map_locked->publish(marker_map_locked_msg);
  }

  // We want to publish in the IMU clock frame
  // The timestamp in the state will be the last camera time
  double t_ItoC = state->_calib_dt_CAMtoIMU->value()(0);
  double timestamp_inI = state->_timestamp + t_ItoC;

  // Create pose of IMU (note we use the bag time)
  geometry_msgs::msg::PoseWithCovarianceStamped poseIinM;
  poseIinM.header.stamp = ROSVisualizerHelper::get_time_from_seconds(timestamp_inI);
  poseIinM.header.frame_id = global_frame_id;
  poseIinM.pose.pose.orientation.x = state->_imu->quat()(0);
  poseIinM.pose.pose.orientation.y = state->_imu->quat()(1);
  poseIinM.pose.pose.orientation.z = state->_imu->quat()(2);
  poseIinM.pose.pose.orientation.w = state->_imu->quat()(3);
  poseIinM.pose.pose.position.x = state->_imu->pos()(0);
  poseIinM.pose.pose.position.y = state->_imu->pos()(1);
  poseIinM.pose.pose.position.z = state->_imu->pos()(2);

  // Finally set the covariance in the message (in the order position then orientation as per ros convention)
  std::vector<std::shared_ptr<Type>> statevars;
  statevars.push_back(state->_imu->pose()->p());
  statevars.push_back(state->_imu->pose()->q());
  Eigen::Matrix<double, 6, 6> covariance_posori = StateHelper::get_marginal_covariance(_app->get_state(), statevars);
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      poseIinM.pose.covariance[6 * r + c] = covariance_posori(r, c);
    }
  }
  pub_poseimu->publish(poseIinM);

  //=========================================================
  //=========================================================

  // Append to our pose vector
  geometry_msgs::msg::PoseStamped posetemp;
  posetemp.header = poseIinM.header;
  posetemp.pose = poseIinM.pose.pose;
  poses_imu.push_back(posetemp);

  // Create our path (imu)
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  // NOTE: https://github.com/ros-visualization/rviz/issues/1107
  nav_msgs::msg::Path arrIMU;
  arrIMU.header.stamp = _node->now();
  arrIMU.header.frame_id = global_frame_id;
  for (size_t i = 0; i < poses_imu.size(); i += std::floor((double)poses_imu.size() / 16384.0) + 1) {
    arrIMU.poses.push_back(poses_imu.at(i));
  }
  pub_pathimu->publish(arrIMU);
}

void ROS2Visualizer::publish_images() {

  // Return if we have already visualized
  if (_app->get_state() == nullptr)
    return;
  if (last_visualization_timestamp_image == _app->get_state()->_timestamp && _app->initialized())
    return;
  last_visualization_timestamp_image = _app->get_state()->_timestamp;

  // Check if we have subscribers
  if (it_pub_tracks.getNumSubscribers() == 0)
    return;

  // Get our image of history tracks
  cv::Mat img_history = _app->get_historical_viz_image();
  if (img_history.empty())
    return;

  // Create our message
  std_msgs::msg::Header header;
  header.stamp = _node->now();
  header.frame_id = camera_frame_prefix + "0";
  sensor_msgs::msg::Image::SharedPtr exl_msg = cv_bridge::CvImage(header, "bgr8", img_history).toImageMsg();

  // Publish
  it_pub_tracks.publish(exl_msg);
}

void ROS2Visualizer::publish_features() {

  // Check if we have subscribers
  if (pub_points_msckf->get_subscription_count() == 0 && pub_points_slam->get_subscription_count() == 0 &&
      pub_points_aruco->get_subscription_count() == 0 && pub_points_sim->get_subscription_count() == 0)
    return;

  // Get our good MSCKF features
  std::vector<Eigen::Vector3d> feats_msckf = _app->get_good_features_MSCKF();
  sensor_msgs::msg::PointCloud2 cloud = ROSVisualizerHelper::get_ros_pointcloud(_node, feats_msckf);
  cloud.header.frame_id = global_frame_id;
  pub_points_msckf->publish(cloud);

  // Get our good SLAM features
  std::vector<Eigen::Vector3d> feats_slam = _app->get_features_SLAM();
  sensor_msgs::msg::PointCloud2 cloud_SLAM = ROSVisualizerHelper::get_ros_pointcloud(_node, feats_slam);
  cloud_SLAM.header.frame_id = global_frame_id;
  pub_points_slam->publish(cloud_SLAM);

  // Get our good ARUCO features
  std::vector<Eigen::Vector3d> feats_aruco = _app->get_features_ARUCO();
  sensor_msgs::msg::PointCloud2 cloud_ARUCO = ROSVisualizerHelper::get_ros_pointcloud(_node, feats_aruco);
  cloud_ARUCO.header.frame_id = global_frame_id;
  pub_points_aruco->publish(cloud_ARUCO);

  // Skip the rest of we are not doing simulation
  if (_sim == nullptr)
    return;

  // Get our good SIMULATION features
  std::vector<Eigen::Vector3d> feats_sim = _sim->get_map_vec();
  sensor_msgs::msg::PointCloud2 cloud_SIM = ROSVisualizerHelper::get_ros_pointcloud(_node, feats_sim);
  cloud_SIM.header.frame_id = global_frame_id;
  pub_points_sim->publish(cloud_SIM);
}

void ROS2Visualizer::publish_groundtruth() {

  // Our groundtruth state
  Eigen::Matrix<double, 17, 1> state_gt;

  // We want to publish in the IMU clock frame
  // The timestamp in the state will be the last camera time
  double t_ItoC = _app->get_state()->_calib_dt_CAMtoIMU->value()(0);
  double timestamp_inI = _app->get_state()->_timestamp + t_ItoC;

  // Check that we have the timestamp in our GT file [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
  if (_sim == nullptr && (gt_states.empty() || !DatasetReader::get_gt_state(timestamp_inI, state_gt, gt_states))) {
    return;
  }

  // Get the simulated groundtruth
  // NOTE: we get the true time in the IMU clock frame
  if (_sim != nullptr) {
    timestamp_inI = _app->get_state()->_timestamp + _sim->get_true_parameters().calib_camimu_dt;
    if (!_sim->get_state(timestamp_inI, state_gt))
      return;
  }

  // Get the GT and system state state
  Eigen::Matrix<double, 16, 1> state_ekf = _app->get_state()->_imu->value();

  // Create pose of IMU
  geometry_msgs::msg::PoseStamped poseIinM;
  poseIinM.header.stamp = ROSVisualizerHelper::get_time_from_seconds(timestamp_inI);
  poseIinM.header.frame_id = global_frame_id;
  poseIinM.pose.orientation.x = state_gt(1, 0);
  poseIinM.pose.orientation.y = state_gt(2, 0);
  poseIinM.pose.orientation.z = state_gt(3, 0);
  poseIinM.pose.orientation.w = state_gt(4, 0);
  poseIinM.pose.position.x = state_gt(5, 0);
  poseIinM.pose.position.y = state_gt(6, 0);
  poseIinM.pose.position.z = state_gt(7, 0);
  pub_posegt->publish(poseIinM);

  // Append to our pose vector
  poses_gt.push_back(poseIinM);

  // Create our path (imu)
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  // NOTE: https://github.com/ros-visualization/rviz/issues/1107
  nav_msgs::msg::Path arrIMU;
  arrIMU.header.stamp = _node->now();
  arrIMU.header.frame_id = global_frame_id;
  for (size_t i = 0; i < poses_gt.size(); i += std::floor((double)poses_gt.size() / 16384.0) + 1) {
    arrIMU.poses.push_back(poses_gt.at(i));
  }
  pub_pathgt->publish(arrIMU);

  // Publish our transform on TF
  geometry_msgs::msg::TransformStamped trans;
  trans.header.stamp = _node->now();
  trans.header.frame_id = global_frame_id;
  trans.child_frame_id = truth_frame_id;
  trans.transform.rotation.x = state_gt(1, 0);
  trans.transform.rotation.y = state_gt(2, 0);
  trans.transform.rotation.z = state_gt(3, 0);
  trans.transform.rotation.w = state_gt(4, 0);
  trans.transform.translation.x = state_gt(5, 0);
  trans.transform.translation.y = state_gt(6, 0);
  trans.transform.translation.z = state_gt(7, 0);
  if (publish_global2imu_tf) {
    mTfBr->sendTransform(trans);
  }

  //==========================================================================
  //==========================================================================

  // Difference between positions
  double dx = state_ekf(4, 0) - state_gt(5, 0);
  double dy = state_ekf(5, 0) - state_gt(6, 0);
  double dz = state_ekf(6, 0) - state_gt(7, 0);
  double err_pos = std::sqrt(dx * dx + dy * dy + dz * dz);

  // Quaternion error
  Eigen::Matrix<double, 4, 1> quat_gt, quat_st, quat_diff;
  quat_gt << state_gt(1, 0), state_gt(2, 0), state_gt(3, 0), state_gt(4, 0);
  quat_st << state_ekf(0, 0), state_ekf(1, 0), state_ekf(2, 0), state_ekf(3, 0);
  quat_diff = quat_multiply(quat_st, Inv(quat_gt));
  double err_ori = (180 / M_PI) * 2 * quat_diff.block(0, 0, 3, 1).norm();

  //==========================================================================
  //==========================================================================

  // Get covariance of pose
  std::vector<std::shared_ptr<Type>> statevars;
  statevars.push_back(_app->get_state()->_imu->q());
  statevars.push_back(_app->get_state()->_imu->p());
  Eigen::Matrix<double, 6, 6> covariance = StateHelper::get_marginal_covariance(_app->get_state(), statevars);

  // Calculate NEES values
  // NOTE: need to manually multiply things out to make static asserts work
  // NOTE: https://github.com/rpng/open_vins/pull/226
  // NOTE: https://github.com/rpng/open_vins/issues/236
  // NOTE: https://gitlab.com/libeigen/eigen/-/issues/1664
  Eigen::Vector3d quat_diff_vec = quat_diff.block(0, 0, 3, 1);
  Eigen::Vector3d cov_vec = covariance.block(0, 0, 3, 3).inverse() * 2 * quat_diff.block(0, 0, 3, 1);
  double ori_nees = 2 * quat_diff_vec.dot(cov_vec);
  Eigen::Vector3d errpos = state_ekf.block(4, 0, 3, 1) - state_gt.block(5, 0, 3, 1);
  double pos_nees = errpos.transpose() * covariance.block(3, 3, 3, 3).inverse() * errpos;

  //==========================================================================
  //==========================================================================

  // Update our average variables
  if (!std::isnan(ori_nees) && !std::isnan(pos_nees)) {
    summed_mse_ori += err_ori * err_ori;
    summed_mse_pos += err_pos * err_pos;
    summed_nees_ori += ori_nees;
    summed_nees_pos += pos_nees;
    summed_number++;
  }

  // Nice display for the user
  PRINT_INFO(REDPURPLE "error to gt => %.3f, %.3f (deg,m) | rmse => %.3f, %.3f (deg,m) | called %d times\n" RESET, err_ori, err_pos,
             std::sqrt(summed_mse_ori / summed_number), std::sqrt(summed_mse_pos / summed_number), (int)summed_number);
  PRINT_INFO(REDPURPLE "nees => %.1f, %.1f (ori,pos) | avg nees = %.1f, %.1f (ori,pos)\n" RESET, ori_nees, pos_nees,
             summed_nees_ori / summed_number, summed_nees_pos / summed_number);

  //==========================================================================
  //==========================================================================
}

void ROS2Visualizer::publish_loopclosure_information() {

  // Get the current tracks in this frame
  double active_tracks_time1 = -1;
  double active_tracks_time2 = -1;
  std::unordered_map<size_t, Eigen::Vector3d> active_tracks_posinG;
  std::unordered_map<size_t, Eigen::Vector3d> active_tracks_uvd;
  cv::Mat active_cam0_image;
  _app->get_active_tracks(active_tracks_time1, active_tracks_posinG, active_tracks_uvd);
  _app->get_active_image(active_tracks_time2, active_cam0_image);
  if (active_tracks_time1 == -1)
    return;
  if (_app->get_state()->_clones_IMU.find(active_tracks_time1) == _app->get_state()->_clones_IMU.end())
    return;
  if (active_tracks_time1 != active_tracks_time2)
    return;

  // Default header
  std_msgs::msg::Header header;
  header.stamp = ROSVisualizerHelper::get_time_from_seconds(active_tracks_time1);

  //======================================================
  // Check if we have subscribers for the pose odometry, camera intrinsics, or extrinsics
  if (pub_loop_pose->get_subscription_count() != 0 || pub_loop_extrinsic->get_subscription_count() != 0 ||
      pub_loop_intrinsics->get_subscription_count() != 0) {

    // PUBLISH HISTORICAL POSE ESTIMATE
    nav_msgs::msg::Odometry odometry_pose;
    odometry_pose.header = header;
    odometry_pose.header.frame_id = global_frame_id;
    odometry_pose.pose.pose.position.x = _app->get_state()->_clones_IMU.at(active_tracks_time1)->pos()(0);
    odometry_pose.pose.pose.position.y = _app->get_state()->_clones_IMU.at(active_tracks_time1)->pos()(1);
    odometry_pose.pose.pose.position.z = _app->get_state()->_clones_IMU.at(active_tracks_time1)->pos()(2);
    odometry_pose.pose.pose.orientation.x = _app->get_state()->_clones_IMU.at(active_tracks_time1)->quat()(0);
    odometry_pose.pose.pose.orientation.y = _app->get_state()->_clones_IMU.at(active_tracks_time1)->quat()(1);
    odometry_pose.pose.pose.orientation.z = _app->get_state()->_clones_IMU.at(active_tracks_time1)->quat()(2);
    odometry_pose.pose.pose.orientation.w = _app->get_state()->_clones_IMU.at(active_tracks_time1)->quat()(3);
    pub_loop_pose->publish(odometry_pose);

    // PUBLISH IMU TO CAMERA0 EXTRINSIC
    // need to flip the transform to the IMU frame
    Eigen::Vector4d q_ItoC = _app->get_state()->_calib_IMUtoCAM.at(0)->quat();
    Eigen::Vector3d p_CinI = -_app->get_state()->_calib_IMUtoCAM.at(0)->Rot().transpose() * _app->get_state()->_calib_IMUtoCAM.at(0)->pos();
    nav_msgs::msg::Odometry odometry_calib;
    odometry_calib.header = header;
    odometry_calib.header.frame_id = imu_frame_id;
    odometry_calib.pose.pose.position.x = p_CinI(0);
    odometry_calib.pose.pose.position.y = p_CinI(1);
    odometry_calib.pose.pose.position.z = p_CinI(2);
    odometry_calib.pose.pose.orientation.x = q_ItoC(0);
    odometry_calib.pose.pose.orientation.y = q_ItoC(1);
    odometry_calib.pose.pose.orientation.z = q_ItoC(2);
    odometry_calib.pose.pose.orientation.w = q_ItoC(3);
    pub_loop_extrinsic->publish(odometry_calib);

    // PUBLISH CAMERA0 INTRINSICS
    bool is_fisheye = (std::dynamic_pointer_cast<ov_core::CamEqui>(_app->get_params().camera_intrinsics.at(0)) != nullptr);
    sensor_msgs::msg::CameraInfo cameraparams;
    cameraparams.header = header;
    cameraparams.header.frame_id = camera_frame_prefix + "0";
    cameraparams.distortion_model = is_fisheye ? "equidistant" : "plumb_bob";
    Eigen::VectorXd cparams = _app->get_state()->_cam_intrinsics.at(0)->value();
    cameraparams.d = {cparams(4), cparams(5), cparams(6), cparams(7)};
    cameraparams.k = {cparams(0), 0, cparams(2), 0, cparams(1), cparams(3), 0, 0, 1};
    pub_loop_intrinsics->publish(cameraparams);
  }

  //======================================================
  // PUBLISH FEATURE TRACKS IN THE GLOBAL FRAME OF REFERENCE
  if (pub_loop_point->get_subscription_count() != 0) {

    // Construct the message
    sensor_msgs::msg::PointCloud point_cloud;
    point_cloud.header = header;
    point_cloud.header.frame_id = global_frame_id;
    for (const auto &feattimes : active_tracks_posinG) {

      // Get this feature information
      size_t featid = feattimes.first;
      Eigen::Vector3d uvd = Eigen::Vector3d::Zero();
      if (active_tracks_uvd.find(featid) != active_tracks_uvd.end()) {
        uvd = active_tracks_uvd.at(featid);
      }
      Eigen::Vector3d pFinG = active_tracks_posinG.at(featid);

      // Push back 3d point
      geometry_msgs::msg::Point32 p;
      p.x = pFinG(0);
      p.y = pFinG(1);
      p.z = pFinG(2);
      point_cloud.points.push_back(p);

      // Push back the uv_norm, uv_raw, and feature id
      // NOTE: we don't use the normalized coordinates to save time here
      // NOTE: they will have to be re-normalized in the loop closure code
      sensor_msgs::msg::ChannelFloat32 p_2d;
      p_2d.values.push_back(0);
      p_2d.values.push_back(0);
      p_2d.values.push_back(uvd(0));
      p_2d.values.push_back(uvd(1));
      p_2d.values.push_back(featid);
      point_cloud.channels.push_back(p_2d);
    }
    pub_loop_point->publish(point_cloud);
  }

  //======================================================
  // Depth images of sparse points and its colorized version
  if (it_pub_loop_img_depth.getNumSubscribers() != 0 || it_pub_loop_img_depth_color.getNumSubscribers() != 0) {

    // Create the images we will populate with the depths
    std::pair<int, int> wh_pair = {active_cam0_image.cols, active_cam0_image.rows};
    cv::Mat depthmap = cv::Mat::zeros(wh_pair.second, wh_pair.first, CV_16UC1);
    cv::Mat depthmap_viz = active_cam0_image;

    // Loop through all points and append
    for (const auto &feattimes : active_tracks_uvd) {

      // Get this feature information
      size_t featid = feattimes.first;
      Eigen::Vector3d uvd = active_tracks_uvd.at(featid);

      // Skip invalid points
      double dw = 4;
      if (uvd(0) < dw || uvd(0) > wh_pair.first - dw || uvd(1) < dw || uvd(1) > wh_pair.second - dw) {
        continue;
      }

      // Append the depth
      // NOTE: scaled by 1000 to fit the 16U
      // NOTE: access order is y,x (stupid opencv convention stuff)
      depthmap.at<uint16_t>((int)uvd(1), (int)uvd(0)) = (uint16_t)(1000 * uvd(2));

      // Taken from LSD-SLAM codebase segment into 0-4 meter segments:
      // https://github.com/tum-vision/lsd_slam/blob/d1e6f0e1a027889985d2e6b4c0fe7a90b0c75067/lsd_slam_core/src/util/globalFuncs.cpp#L87-L96
      float id = 1.0f / (float)uvd(2);
      float r = (0.0f - id) * 255 / 1.0f;
      if (r < 0)
        r = -r;
      float g = (1.0f - id) * 255 / 1.0f;
      if (g < 0)
        g = -g;
      float b = (2.0f - id) * 255 / 1.0f;
      if (b < 0)
        b = -b;
      uchar rc = r < 0 ? 0 : (r > 255 ? 255 : r);
      uchar gc = g < 0 ? 0 : (g > 255 ? 255 : g);
      uchar bc = b < 0 ? 0 : (b > 255 ? 255 : b);
      cv::Scalar color(255 - rc, 255 - gc, 255 - bc);

      // Small square around the point (note the above bound check needs to take into account this width)
      cv::Point p0(uvd(0) - dw, uvd(1) - dw);
      cv::Point p1(uvd(0) + dw, uvd(1) + dw);
      cv::rectangle(depthmap_viz, p0, p1, color, -1);
    }

    // Create our messages
    header.frame_id = camera_frame_prefix + "0";
    sensor_msgs::msg::Image::SharedPtr exl_msg1 =
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, depthmap).toImageMsg();
    it_pub_loop_img_depth.publish(exl_msg1);
    header.stamp = _node->now();
    header.frame_id = camera_frame_prefix + "0";
    sensor_msgs::msg::Image::SharedPtr exl_msg2 = cv_bridge::CvImage(header, "bgr8", depthmap_viz).toImageMsg();
    it_pub_loop_img_depth_color.publish(exl_msg2);
  }
}
