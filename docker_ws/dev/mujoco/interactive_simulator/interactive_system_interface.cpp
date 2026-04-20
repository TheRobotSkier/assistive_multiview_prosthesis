#include "interactive_system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <future>
#include <functional>

#include "builtin_interfaces/msg/time.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
namespace mia_hand_mujoco
{
InteractiveSystemInterface::InteractiveSystemInterface()
: preshaping_call_running_(false),
  pose_pub_counter_(0),
  imu_pub_counter_(0)
{
  hand_twist_prev_pos_[0] = 0.0;
  hand_twist_prev_pos_[1] = 0.0;
  hand_twist_prev_pos_[2] = 0.0;
  hand_twist_prev_quat_[0] = 1.0;
  hand_twist_prev_quat_[1] = 0.0;
  hand_twist_prev_quat_[2] = 0.0;
  hand_twist_prev_quat_[3] = 0.0;
  hand_twist_prev_sim_time_ = 0.0;
  hand_twist_initialized_ = false;
}

hardware_interface::CallbackReturn InteractiveSystemInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams& params)
{
  hardware_interface::CallbackReturn result =
    hardware_interface::CallbackReturn::SUCCESS;

  if (hardware_interface::CallbackReturn::SUCCESS ==
      hardware_interface::SystemInterface::on_init(params))
  {
    logger_ = std::make_unique<rclcpp::Logger>(
      rclcpp::get_logger("mia_hand_interactive_system_interface"));

    RCLCPP_INFO(*logger_, "Initializing interactive simulator...");

    if (params.hardware_info.hardware_parameters.find("xml_model_path") !=
        params.hardware_info.hardware_parameters.end())
    {
      xml_model_path_ = params.hardware_info.hardware_parameters.at("xml_model_path");
    }
    else
    {
      RCLCPP_FATAL(*logger_, "'xml_model_path' parameter not specified.");
      result = hardware_interface::CallbackReturn::ERROR;
    }
  }
  else
  {
    result = hardware_interface::CallbackReturn::ERROR;
  }

  if (hardware_interface::CallbackReturn::ERROR != result)
  {
    if (!read_joints_info(params.hardware_info.joints))
    {
      result = hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (hardware_interface::CallbackReturn::ERROR != result)
  {
    read_rviz2_joints_info(params.hardware_info.joints);
  }

  return result;
}

hardware_interface::CallbackReturn InteractiveSystemInterface::on_configure(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  hardware_interface::CallbackReturn result =
    hardware_interface::CallbackReturn::SUCCESS;

  RCLCPP_INFO(*logger_, "Configuring and starting MuJoCo interactive simulation...");

  for (std::size_t i = 0; i < 3; ++i) {
    jnt_pos_state_[i] = 0.0;
    jnt_vel_state_[i] = 0.0;
    jnt_pos_cmd_[i] = 0.0;
    jnt_vel_cmd_[i] = 0.0;
  }

  jnt_cmd_modes_[0] = CommandMode::kNone;
  jnt_cmd_modes_[1] = CommandMode::kNone;
  jnt_cmd_modes_[2] = CommandMode::kNone;

  std::promise<bool> sim_start_ok_prms;
  std::future<bool> sim_start_ok_ftr = sim_start_ok_prms.get_future();

  sim_trd_ = std::thread(
    InteractiveSimulator::simulate,
    xml_model_path_.c_str(),
    std::move(sim_start_ok_prms));
  sim_trd_.detach();

  if (sim_start_ok_ftr.get())
  {
    RCLCPP_INFO(*logger_, "MuJoCo interactive simulation started.");
  }
  else
  {
    result = hardware_interface::CallbackReturn::ERROR;
    RCLCPP_ERROR(*logger_,
      "Failed to start MuJoCo interactive simulation: %s",
      InteractiveSimulator::get_instance().get_error_msg());
  }

  return result;
}

hardware_interface::CallbackReturn InteractiveSystemInterface::on_cleanup(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  InteractiveSimulator::get_instance().stop_simulation();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn InteractiveSystemInterface::on_activate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  RCLCPP_INFO(*logger_, "Activating interactive simulator...");

  using Pose = geometry_msgs::msg::Pose;
  auto node = get_node();

  hand_pose_sub_ = node->create_subscription<Pose>(
    "/mujoco/set_hand_pose", 10,
    std::bind(&InteractiveSystemInterface::on_hand_pose_msg, this, std::placeholders::_1));
  object_pose_sub_ = node->create_subscription<Pose>(
    "/mujoco/set_object_pose", 10,
    std::bind(&InteractiveSystemInterface::on_object_pose_msg, this, std::placeholders::_1));
  camera_pose_sub_ = node->create_subscription<Pose>(
    "/mujoco/set_camera_pose", 10,
    std::bind(&InteractiveSystemInterface::on_camera_pose_msg, this, std::placeholders::_1));

  hand_pose_pub_   = node->create_publisher<Pose>("/mujoco/hand_pose",   10);
  hand_pose_alias_pub_ = node->create_publisher<Pose>("/hand_pose", 10);
  hand_twist_pub_  = node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/hand_twist", 10);
  object_pose_pub_ = node->create_publisher<Pose>("/mujoco/object_pose", 10);
  camera_pose_pub_ = node->create_publisher<Pose>("/mujoco/camera_pose", 10);

  using PoseStamped = geometry_msgs::msg::PoseStamped;
  motion_hand_sub_ = node->create_subscription<PoseStamped>(
    "/mujoco/move_hand", 10,
    std::bind(&InteractiveSystemInterface::on_hand_motion_msg, this, std::placeholders::_1));
  motion_obj_sub_ = node->create_subscription<PoseStamped>(
    "/mujoco/move_object", 10,
    std::bind(&InteractiveSystemInterface::on_object_motion_msg, this, std::placeholders::_1));
  motion_cam_sub_ = node->create_subscription<PoseStamped>(
    "/mujoco/move_camera", 10,
    std::bind(&InteractiveSystemInterface::on_camera_motion_msg, this, std::placeholders::_1));

  pose_pub_counter_ = 0;
  imu_pub_counter_  = 0;

  imu_pub_      = node->create_publisher<sensor_msgs::msg::Imu>(
    "/mujoco/front_cam/imu", 10);
  mag_pub_      = node->create_publisher<sensor_msgs::msg::MagneticField>(
    "/mujoco/front_cam/imu/magnetic_field", 10);
  sim_time_pub_ = node->create_publisher<std_msgs::msg::Float64>(
    "/mujoco/sim_time", 10);

  imu2_pub_ = node->create_publisher<sensor_msgs::msg::Imu>(
    "/mujoco/wrist_cam/imu", 10);
  mag2_pub_ = node->create_publisher<sensor_msgs::msg::MagneticField>(
    "/mujoco/wrist_cam/imu/magnetic_field", 10);

  preshaping_trigger_client_ = node->create_client<std_srvs::srv::Trigger>(
    "/grasp_preshaping/compute_grasp");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn InteractiveSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  RCLCPP_INFO(*logger_, "Deactivating interactive simulator...");
  hand_pose_sub_.reset();
  object_pose_sub_.reset();
  camera_pose_sub_.reset();
  hand_pose_pub_.reset();
  hand_pose_alias_pub_.reset();
  hand_twist_pub_.reset();
  object_pose_pub_.reset();
  camera_pose_pub_.reset();
  motion_hand_sub_.reset();
  motion_obj_sub_.reset();
  motion_cam_sub_.reset();
  imu_pub_.reset();
  mag_pub_.reset();
  sim_time_pub_.reset();
  imu2_pub_.reset();
  mag2_pub_.reset();
  preshaping_trigger_client_.reset();
  preshaping_call_running_.store(false);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
InteractiveSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> jnt_state_interfaces;

  for (std::size_t i = 0; i < 3; ++i) {
    if (b_jnt_pos_state_defined_[i]) {
      jnt_state_interfaces.emplace_back(hardware_interface::StateInterface(
        jnt_names_[i], hardware_interface::HW_IF_POSITION,
        &jnt_pos_state_[i]));
    }

    if (b_jnt_vel_state_defined_[i]) {
      jnt_state_interfaces.emplace_back(hardware_interface::StateInterface(
        jnt_names_[i], hardware_interface::HW_IF_VELOCITY,
        &jnt_vel_state_[i]));
    }

    if (!rviz2_joints_[i].name.empty()) {
      jnt_state_interfaces.emplace_back(hardware_interface::StateInterface(
        rviz2_joints_[i].name, hardware_interface::HW_IF_POSITION,
        &rviz2_joints_[i].pos));
      jnt_state_interfaces.emplace_back(hardware_interface::StateInterface(
        rviz2_joints_[i].name, hardware_interface::HW_IF_VELOCITY,
        &rviz2_joints_[i].vel));
    }
  }

  return jnt_state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
InteractiveSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> jnt_cmd_interfaces;

  for (std::size_t i = 0; i < 3; ++i) {
    if (b_jnt_pos_cmd_defined_[i]) {
      jnt_cmd_interfaces.emplace_back(hardware_interface::CommandInterface(
        jnt_names_[i], hardware_interface::HW_IF_POSITION,
        &jnt_pos_cmd_[i]));
    }

    if (b_jnt_vel_cmd_defined_[i]) {
      jnt_cmd_interfaces.emplace_back(hardware_interface::CommandInterface(
        jnt_names_[i], hardware_interface::HW_IF_VELOCITY,
        &jnt_vel_cmd_[i]));
    }
  }

  return jnt_cmd_interfaces;
}

hardware_interface::return_type
InteractiveSystemInterface::prepare_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type result = hardware_interface::return_type::OK;

  for (const auto& iface : stop_interfaces) {
    for (std::size_t jnt_it = 0; jnt_it < 3; ++jnt_it) {
      if (std::string::npos != iface.find(jnt_names_[jnt_it])) {
        jnt_cmd_modes_[jnt_it] = CommandMode::kNone;
        InteractiveSimulator::get_instance().stop_jnt(jnt_it);
      }
    }
  }

  for (const auto& iface : start_interfaces) {
    for (std::size_t jnt_it = 0; jnt_it < 3; ++jnt_it) {
      if (iface == jnt_names_[jnt_it] + "/" + hardware_interface::HW_IF_POSITION) {
        if (CommandMode::kNone == jnt_cmd_modes_[jnt_it]) {
          jnt_cmd_modes_[jnt_it] = CommandMode::kPosition;
        } else {
          RCLCPP_ERROR(*logger_,
            "New command mode specified for %s while another one is active.",
            jnt_names_[jnt_it].c_str());
          result = hardware_interface::return_type::ERROR;
        }
      } else if (iface == jnt_names_[jnt_it] + "/" + hardware_interface::HW_IF_VELOCITY) {
        if (CommandMode::kNone == jnt_cmd_modes_[jnt_it]) {
          jnt_cmd_modes_[jnt_it] = CommandMode::kVelocity;
        } else {
          RCLCPP_ERROR(*logger_,
            "New command mode specified for %s while another one is active.",
            jnt_names_[jnt_it].c_str());
          result = hardware_interface::return_type::ERROR;
        }
      }
    }
  }

  return result;
}

hardware_interface::return_type InteractiveSystemInterface::read(
  const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  if (InteractiveSimulator::get_instance().consume_planner_request()) {
    trigger_preshaping_service();
  }

  InteractiveSimulator::get_instance().read_jnt_vel(
    jnt_vel_state_[0], jnt_vel_state_[1], jnt_vel_state_[2]);

  for (std::size_t i = 0; i < 3; ++i) {
    if (std::abs(jnt_vel_state_[i]) < 0.001) {
      jnt_vel_state_[i] = 0.0;
    }
  }

  InteractiveSimulator::get_instance().read_jnt_pos(
    jnt_pos_state_[0], jnt_pos_state_[1], jnt_pos_state_[2]);

  for (std::size_t i = 0; i < 3; ++i) {
    if (std::abs(jnt_pos_state_[i]) < 0.01) {
      jnt_pos_state_[i] = 0.0;
    }
  }

  // Publish current scene poses at ~10 Hz (throttled from 1 kHz read loop)
  if (++pose_pub_counter_ >= 100) {
    pose_pub_counter_ = 0;
    auto publish_pose = [](
      const rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr& pub,
      const double pos[3], const double quat_wxyz[4])
    {
      if (!pub) return;
      geometry_msgs::msg::Pose msg;
      msg.position.x = pos[0];
      msg.position.y = pos[1];
      msg.position.z = pos[2];
      msg.orientation.w = quat_wxyz[0];
      msg.orientation.x = quat_wxyz[1];
      msg.orientation.y = quat_wxyz[2];
      msg.orientation.z = quat_wxyz[3];
      pub->publish(msg);
    };
    double pos[3], quat[4];
    InteractiveSimulator::get_instance().get_hand_pose(pos, quat);
    publish_pose(hand_pose_pub_, pos, quat);
    publish_pose(hand_pose_alias_pub_, pos, quat);

    if (hand_twist_pub_) {
      geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
      twist_msg.header.frame_id = "palm_r";
      twist_msg.header.stamp = rclcpp::Clock().now();

      double hand_ang_vel[3], hand_lin_acc[3], hand_mag_field[3], hand_orientation[4], sim_time;
      InteractiveSimulator::get_instance().get_imu_data(
        hand_ang_vel, hand_lin_acc, hand_mag_field, hand_orientation, sim_time);

      if (!hand_twist_initialized_) {
        for (int i = 0; i < 3; ++i) {
          hand_twist_prev_pos_[i] = pos[i];
        }
        for (int i = 0; i < 4; ++i) {
          hand_twist_prev_quat_[i] = quat[i];
        }
        hand_twist_prev_sim_time_ = sim_time;
        hand_twist_initialized_ = true;
      } else {
        const double dt = sim_time - hand_twist_prev_sim_time_;
        if (dt > 1e-6) {
          double delta_q[4] = {
            hand_twist_prev_quat_[0] * quat[0] + hand_twist_prev_quat_[1] * quat[1]
              + hand_twist_prev_quat_[2] * quat[2] + hand_twist_prev_quat_[3] * quat[3],
            -hand_twist_prev_quat_[1] * quat[0] + hand_twist_prev_quat_[0] * quat[1]
              - hand_twist_prev_quat_[3] * quat[2] + hand_twist_prev_quat_[2] * quat[3],
            -hand_twist_prev_quat_[2] * quat[0] + hand_twist_prev_quat_[3] * quat[1]
              + hand_twist_prev_quat_[0] * quat[2] - hand_twist_prev_quat_[1] * quat[3],
            -hand_twist_prev_quat_[3] * quat[0] - hand_twist_prev_quat_[2] * quat[1]
              + hand_twist_prev_quat_[1] * quat[2] + hand_twist_prev_quat_[0] * quat[3],
          };
          if (delta_q[0] < 0.0) {
            for (double& value : delta_q) {
              value = -value;
            }
          }

          const double world_lin_vel[3] = {
            (pos[0] - hand_twist_prev_pos_[0]) / dt,
            (pos[1] - hand_twist_prev_pos_[1]) / dt,
            (pos[2] - hand_twist_prev_pos_[2]) / dt,
          };

          const double q_w = quat[0];
          const double q_x = quat[1];
          const double q_y = quat[2];
          const double q_z = quat[3];
          const double r00 = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);
          const double r01 = 2.0 * (q_x * q_y - q_z * q_w);
          const double r02 = 2.0 * (q_x * q_z + q_y * q_w);
          const double r10 = 2.0 * (q_x * q_y + q_z * q_w);
          const double r11 = 1.0 - 2.0 * (q_x * q_x + q_z * q_z);
          const double r12 = 2.0 * (q_y * q_z - q_x * q_w);
          const double r20 = 2.0 * (q_x * q_z - q_y * q_w);
          const double r21 = 2.0 * (q_y * q_z + q_x * q_w);
          const double r22 = 1.0 - 2.0 * (q_x * q_x + q_y * q_y);

          twist_msg.twist.twist.linear.x = r00 * world_lin_vel[0] + r10 * world_lin_vel[1] + r20 * world_lin_vel[2];
          twist_msg.twist.twist.linear.y = r01 * world_lin_vel[0] + r11 * world_lin_vel[1] + r21 * world_lin_vel[2];
          twist_msg.twist.twist.linear.z = r02 * world_lin_vel[0] + r12 * world_lin_vel[1] + r22 * world_lin_vel[2];

          twist_msg.twist.twist.angular.x = 2.0 * delta_q[1] / dt;
          twist_msg.twist.twist.angular.y = 2.0 * delta_q[2] / dt;
          twist_msg.twist.twist.angular.z = 2.0 * delta_q[3] / dt;
        }
        hand_twist_prev_sim_time_ = sim_time;
        for (int i = 0; i < 3; ++i) {
          hand_twist_prev_pos_[i] = pos[i];
        }
        for (int i = 0; i < 4; ++i) {
          hand_twist_prev_quat_[i] = quat[i];
        }
      }

      for (double& covariance_value : twist_msg.twist.covariance) {
        covariance_value = 0.0;
      }
      twist_msg.twist.covariance[0] = 1e-6;
      twist_msg.twist.covariance[7] = 1e-6;
      twist_msg.twist.covariance[14] = 1e-6;
      twist_msg.twist.covariance[21] = 1e-6;
      twist_msg.twist.covariance[28] = 1e-6;
      twist_msg.twist.covariance[35] = 1e-6;
      hand_twist_pub_->publish(twist_msg);
    }

    InteractiveSimulator::get_instance().get_object_pose(pos, quat);
    publish_pose(object_pose_pub_, pos, quat);
    InteractiveSimulator::get_instance().get_camera_pose(pos, quat);
    publish_pose(camera_pose_pub_, pos, quat);

    // Publish simulation time at same ~10 Hz rate
    double ang_vel[3], lin_acc[3], mag_field[3], orientation[4], sim_time;
    InteractiveSimulator::get_instance().get_imu_data(
      ang_vel, lin_acc, mag_field, orientation, sim_time);
    if (sim_time_pub_) {
      std_msgs::msg::Float64 t_msg;
      t_msg.data = sim_time;
      sim_time_pub_->publish(t_msg);
    }
  }

  // Publish IMU at ~100 Hz (every 10 calls)
  if (++imu_pub_counter_ >= 10) {
    imu_pub_counter_ = 0;
    double ang_vel[3], lin_acc[3], mag_field[3], orientation[4], sim_time;
    InteractiveSimulator::get_instance().get_imu_data(
      ang_vel, lin_acc, mag_field, orientation, sim_time);

    if (imu_pub_) {
      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header.frame_id = "mujoco_front_depth_cam";
      imu_msg.header.stamp    = rclcpp::Clock().now();
      imu_msg.angular_velocity.x = ang_vel[0];
      imu_msg.angular_velocity.y = ang_vel[1];
      imu_msg.angular_velocity.z = ang_vel[2];
      imu_msg.linear_acceleration.x = lin_acc[0];
      imu_msg.linear_acceleration.y = lin_acc[1];
      imu_msg.linear_acceleration.z = lin_acc[2];
      imu_msg.orientation.w = orientation[0];
      imu_msg.orientation.x = orientation[1];
      imu_msg.orientation.y = orientation[2];
      imu_msg.orientation.z = orientation[3];
      // -1 = covariance unknown (we publish perfect sim values)
      imu_msg.orientation_covariance[0]         = -1.0;
      imu_msg.angular_velocity_covariance[0]    = -1.0;
      imu_msg.linear_acceleration_covariance[0] = -1.0;
      imu_pub_->publish(imu_msg);
    }

    if (mag_pub_) {
      sensor_msgs::msg::MagneticField mag_msg;
      mag_msg.header.frame_id = "mujoco_front_depth_cam";
      mag_msg.header.stamp    = rclcpp::Clock().now();
      mag_msg.magnetic_field.x = mag_field[0];
      mag_msg.magnetic_field.y = mag_field[1];
      mag_msg.magnetic_field.z = mag_field[2];
      mag_msg.magnetic_field_covariance[0] = -1.0;
      mag_pub_->publish(mag_msg);
    }

    // Wrist camera IMU
    double ang_vel2[3], lin_acc2[3], mag_field2[3], orientation2[4];
    InteractiveSimulator::get_instance().get_wrist_cam_imu_data(
      ang_vel2, lin_acc2, mag_field2, orientation2);

    if (imu2_pub_) {
      sensor_msgs::msg::Imu imu2_msg;
      imu2_msg.header.frame_id = "mujoco_wrist_cam";
      imu2_msg.header.stamp    = rclcpp::Clock().now();
      imu2_msg.angular_velocity.x = ang_vel2[0];
      imu2_msg.angular_velocity.y = ang_vel2[1];
      imu2_msg.angular_velocity.z = ang_vel2[2];
      imu2_msg.linear_acceleration.x = lin_acc2[0];
      imu2_msg.linear_acceleration.y = lin_acc2[1];
      imu2_msg.linear_acceleration.z = lin_acc2[2];
      imu2_msg.orientation.w = orientation2[0];
      imu2_msg.orientation.x = orientation2[1];
      imu2_msg.orientation.y = orientation2[2];
      imu2_msg.orientation.z = orientation2[3];
      imu2_msg.orientation_covariance[0]         = -1.0;
      imu2_msg.angular_velocity_covariance[0]    = -1.0;
      imu2_msg.linear_acceleration_covariance[0] = -1.0;
      imu2_pub_->publish(imu2_msg);
    }

    if (mag2_pub_) {
      sensor_msgs::msg::MagneticField mag2_msg;
      mag2_msg.header.frame_id = "mujoco_wrist_cam";
      mag2_msg.header.stamp    = rclcpp::Clock().now();
      mag2_msg.magnetic_field.x = mag_field2[0];
      mag2_msg.magnetic_field.y = mag_field2[1];
      mag2_msg.magnetic_field.z = mag_field2[2];
      mag2_msg.magnetic_field_covariance[0] = -1.0;
      mag2_pub_->publish(mag2_msg);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type InteractiveSystemInterface::write(
  const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  for (std::size_t jnt_it = 0; jnt_it < 3; ++jnt_it) {
    if (CommandMode::kPosition == jnt_cmd_modes_[jnt_it]) {
      InteractiveSimulator::get_instance().set_jnt_pos(jnt_it, jnt_pos_cmd_[jnt_it]);
    } else if (CommandMode::kVelocity == jnt_cmd_modes_[jnt_it]) {
      InteractiveSimulator::get_instance().set_jnt_vel(jnt_it, jnt_vel_cmd_[jnt_it]);
    }
  }

  return hardware_interface::return_type::OK;
}

bool InteractiveSystemInterface::read_joints_info(
  const std::vector<hardware_interface::ComponentInfo>& jnt_info)
{
  bool success = true;

  if (4 == jnt_info.size()) {
    std::array<std::string, 3> jnt_roles = {
      "j_thumb_fle", "j_index_fle", "j_mrl_fle"};

    for (std::size_t jnt_roles_it = 0; jnt_roles_it < 3; ++jnt_roles_it) {
      const std::string& jnt_role = jnt_roles[jnt_roles_it];
      const auto role_match_it = std::find_if(
        jnt_info.begin(), jnt_info.end(),
        [&jnt_role](const hardware_interface::ComponentInfo& jnt)
        { return std::string::npos != jnt.name.find(jnt_role); });

      if (role_match_it != jnt_info.end()) {
        jnt_names_[jnt_roles_it] = role_match_it->name;

        if (has_cmd_interface(*role_match_it, hardware_interface::HW_IF_POSITION)) {
          b_jnt_pos_cmd_defined_[jnt_roles_it] = true;
        }
        if (has_state_interface(*role_match_it, hardware_interface::HW_IF_POSITION)) {
          b_jnt_pos_state_defined_[jnt_roles_it] = true;
        }
        if (has_cmd_interface(*role_match_it, hardware_interface::HW_IF_VELOCITY)) {
          b_jnt_vel_cmd_defined_[jnt_roles_it] = true;
        }
        if (has_state_interface(*role_match_it, hardware_interface::HW_IF_VELOCITY)) {
          b_jnt_vel_state_defined_[jnt_roles_it] = true;
        }
      } else {
        RCLCPP_FATAL(*logger_,
          "Could not find a joint with %s role.", jnt_role.c_str());
        success = false;
      }
    }
  } else {
    RCLCPP_FATAL(*logger_, "4 joints expected, but %ld provided.", jnt_info.size());
    success = false;
  }

  return success;
}

bool InteractiveSystemInterface::has_cmd_interface(
  const hardware_interface::ComponentInfo& jnt_info,
  const std::string& interface_name)
{
  return (
    jnt_info.command_interfaces.end() !=
    std::find_if(
      jnt_info.command_interfaces.begin(), jnt_info.command_interfaces.end(),
      [&interface_name](const hardware_interface::InterfaceInfo& iface)
      { return interface_name == iface.name; }));
}

bool InteractiveSystemInterface::has_state_interface(
  const hardware_interface::ComponentInfo& jnt_info,
  const std::string& interface_name)
{
  return (
    jnt_info.state_interfaces.end() !=
    std::find_if(
      jnt_info.state_interfaces.begin(), jnt_info.state_interfaces.end(),
      [&interface_name](const hardware_interface::InterfaceInfo& iface)
      { return interface_name == iface.name; }));
}

void InteractiveSystemInterface::read_rviz2_joints_info(
  const std::vector<hardware_interface::ComponentInfo>& jnt_info)
{
  std::array<std::string, 1> rviz2_jnt_names = {"j_thumb_opp"};

  for (std::size_t jnt_it = 0; jnt_it < 1; ++jnt_it) {
    const std::string& jnt_name_to_find = rviz2_jnt_names[jnt_it];
    const auto jnt_info_match = std::find_if(
      jnt_info.begin(), jnt_info.end(),
      [&jnt_name_to_find](const hardware_interface::ComponentInfo& jnt)
      { return std::string::npos != jnt.name.find(jnt_name_to_find); });

    if (jnt_info_match != jnt_info.end()) {
      rviz2_joints_[jnt_it].name = jnt_info_match->name;
    } else {
      RCLCPP_ERROR(*logger_,
        "%s not found among the joint names provided.",
        jnt_name_to_find.c_str());
    }
  }
}

void InteractiveSystemInterface::pose_msg_to_sim(
  const geometry_msgs::msg::Pose& msg,
  double pos[3], double quat_wxyz[4])
{
  pos[0] = msg.position.x;
  pos[1] = msg.position.y;
  pos[2] = msg.position.z;
  // geometry_msgs uses xyzw; InteractiveSimulator expects wxyz
  quat_wxyz[0] = msg.orientation.w;
  quat_wxyz[1] = msg.orientation.x;
  quat_wxyz[2] = msg.orientation.y;
  quat_wxyz[3] = msg.orientation.z;
}

void InteractiveSystemInterface::on_hand_pose_msg(
  const geometry_msgs::msg::Pose::SharedPtr msg)
{
  double pos[3], quat[4];
  pose_msg_to_sim(*msg, pos, quat);
  InteractiveSimulator::get_instance().set_hand_pose(pos, quat);
}

void InteractiveSystemInterface::on_object_pose_msg(
  const geometry_msgs::msg::Pose::SharedPtr msg)
{
  double pos[3], quat[4];
  pose_msg_to_sim(*msg, pos, quat);
  InteractiveSimulator::get_instance().set_object_pose(pos, quat);
}

void InteractiveSystemInterface::on_camera_pose_msg(
  const geometry_msgs::msg::Pose::SharedPtr msg)
{
  double pos[3], quat[4];
  pose_msg_to_sim(*msg, pos, quat);
  InteractiveSimulator::get_instance().set_camera_pose(pos, quat);
}

// Smooth motion callbacks. The stamp field encodes the motion duration:
//   duration_s = stamp.sec + stamp.nanosec / 1e9
// If stamp is zero, a default of 1.0 s is used.
static double duration_from_stamp(const builtin_interfaces::msg::Time& stamp)
{
  const double d = static_cast<double>(stamp.sec)
                 + static_cast<double>(stamp.nanosec) * 1e-9;
  return (d > 1e-6) ? d : 1.0;
}

void InteractiveSystemInterface::on_hand_motion_msg(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  double pos[3], quat[4];
  pose_msg_to_sim(msg->pose, pos, quat);
  InteractiveSimulator::get_instance().request_hand_move(
    pos, quat, duration_from_stamp(msg->header.stamp));
}

void InteractiveSystemInterface::on_object_motion_msg(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  double pos[3], quat[4];
  pose_msg_to_sim(msg->pose, pos, quat);
  InteractiveSimulator::get_instance().request_object_move(
    pos, quat, duration_from_stamp(msg->header.stamp));
}

void InteractiveSystemInterface::on_camera_motion_msg(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  double pos[3], quat[4];
  pose_msg_to_sim(msg->pose, pos, quat);
  InteractiveSimulator::get_instance().request_camera_move(
    pos, quat, duration_from_stamp(msg->header.stamp));
}

void InteractiveSystemInterface::trigger_preshaping_service()
{
  if (!preshaping_trigger_client_) {
    InteractiveSimulator::get_instance().report_planner_result(
      false, "Preshaping service client not initialized");
    return;
  }

  bool expected = false;
  if (!preshaping_call_running_.compare_exchange_strong(expected, true)) {
    InteractiveSimulator::get_instance().report_planner_result(
      false, "Preshaping call already in flight");
    return;
  }

  auto client = preshaping_trigger_client_;
  auto logger = logger_.get();

  std::thread([this, client, logger]() {
    using namespace std::chrono_literals;

    auto finish = [this](bool success, const std::string& message) {
      InteractiveSimulator::get_instance().report_planner_result(success, message);
      preshaping_call_running_.store(false);
    };

    if (!client->wait_for_service(2s)) {
      if (logger) {
        RCLCPP_ERROR(*logger, "Preshaping service /grasp_preshaping/compute_grasp unavailable");
      }
      finish(false, "Preshaping service unavailable");
      return;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(req);
    constexpr auto kTotalTimeout = 15s;
    if (future.wait_for(kTotalTimeout) != std::future_status::ready) {
      if (logger) {
        RCLCPP_ERROR(*logger, "Preshaping service timeout");
      }
      finish(false, "Preshaping timeout");
      return;
    }

    const auto response = future.get();
    finish(response->success, response->message);
  }).detach();
}

}  // namespace mia_hand_mujoco

PLUGINLIB_EXPORT_CLASS(
  mia_hand_mujoco::InteractiveSystemInterface,
  hardware_interface::SystemInterface)
