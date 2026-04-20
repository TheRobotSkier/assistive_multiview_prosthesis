#include "planner_gui_system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <future>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace mia_hand_mujoco
{
PlannerGuiSystemInterface::PlannerGuiSystemInterface()
: pose_pub_counter_(0),
  preshaping_call_running_(false)
{
}

hardware_interface::CallbackReturn PlannerGuiSystemInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams& params)
{
  hardware_interface::CallbackReturn result =
    hardware_interface::CallbackReturn::SUCCESS;

  if (hardware_interface::CallbackReturn::SUCCESS ==
      hardware_interface::SystemInterface::on_init(params))
  {
    logger_ = std::make_unique<rclcpp::Logger>(
      rclcpp::get_logger("mia_hand_system_interface_sim_gui"));

    RCLCPP_INFO(*logger_, "Initializing planner GUI simulator...");

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

hardware_interface::CallbackReturn PlannerGuiSystemInterface::on_configure(
  const rclcpp_lifecycle::State& /* previous state */)
{
  hardware_interface::CallbackReturn result =
    hardware_interface::CallbackReturn::SUCCESS;

  RCLCPP_INFO(*logger_, "Configuring and starting MuJoCo planner GUI simulation...");

  for (std::size_t data_it = 0; data_it < 3; ++data_it)
  {
    jnt_pos_state_[data_it] = 0.0;
    jnt_vel_state_[data_it] = 0.0;

    jnt_pos_cmd_[data_it] = 0.0;
    jnt_vel_cmd_[data_it] = 0.0;
  }

  jnt_cmd_modes_[0] = CommandMode::kNone;
  jnt_cmd_modes_[1] = CommandMode::kNone;
  jnt_cmd_modes_[2] = CommandMode::kNone;

  std::promise<bool> sim_start_ok_prms;
  std::future<bool> sim_start_ok_ftr = sim_start_ok_prms.get_future();

  sim_trd_ = std::thread(
    PlannerGuiSimulator::simulate, xml_model_path_.c_str(), std::move(sim_start_ok_prms));
  sim_trd_.detach();

  if (sim_start_ok_ftr.get())
  {
    RCLCPP_INFO(*logger_, "MuJoCo planner GUI simulation started.");
  }
  else
  {
    result = hardware_interface::CallbackReturn::ERROR;
    RCLCPP_ERROR(*logger_,
      "Failed to start MuJoCo planner GUI simulation: %s",
      PlannerGuiSimulator::get_instance().get_error_msg());
  }

  return result;
}

hardware_interface::CallbackReturn PlannerGuiSystemInterface::on_cleanup(
  const rclcpp_lifecycle::State& /* previous state */)
{
  PlannerGuiSimulator::get_instance().stop_simulation();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PlannerGuiSystemInterface::on_activate(
  const rclcpp_lifecycle::State& /* previous state */)
{
  RCLCPP_INFO(*logger_, "Activating planner GUI simulator...");

  auto node = get_node();
  hand_pose_pub_ = node->create_publisher<geometry_msgs::msg::Pose>("/mujoco/hand_pose", 10);
  hand_pose_alias_pub_ = node->create_publisher<geometry_msgs::msg::Pose>("/hand_pose", 10);
  hand_twist_pub_ = node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/hand_twist", 10);
  preshaping_trigger_client_ = node->create_client<std_srvs::srv::Trigger>(
    "/grasp_preshaping/compute_grasp");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PlannerGuiSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State& /* previous state */)
{
  RCLCPP_INFO(*logger_, "Deactivating planner GUI simulator...");
  hand_pose_pub_.reset();
  hand_pose_alias_pub_.reset();
  hand_twist_pub_.reset();
  preshaping_trigger_client_.reset();
  preshaping_call_running_.store(false);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PlannerGuiSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> jnt_state_interfaces;

  for (std::size_t data_it = 0; data_it < 3; ++data_it)
  {
    if (b_jnt_pos_state_defined_[data_it])
    {
      jnt_state_interfaces.emplace_back(hardware_interface::StateInterface(
        jnt_names_[data_it], hardware_interface::HW_IF_POSITION,
        &jnt_pos_state_[data_it]));
    }

    if (b_jnt_vel_state_defined_[data_it])
    {
      jnt_state_interfaces.emplace_back(hardware_interface::StateInterface(
        jnt_names_[data_it], hardware_interface::HW_IF_VELOCITY,
        &jnt_vel_state_[data_it]));
    }

    if (!rviz2_joints_[data_it].name.empty())
    {
      jnt_state_interfaces.emplace_back(hardware_interface::StateInterface(
        rviz2_joints_[data_it].name, hardware_interface::HW_IF_POSITION,
        &rviz2_joints_[data_it].pos));

      jnt_state_interfaces.emplace_back(hardware_interface::StateInterface(
        rviz2_joints_[data_it].name, hardware_interface::HW_IF_VELOCITY,
        &rviz2_joints_[data_it].vel));
    }
  }

  return jnt_state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PlannerGuiSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> jnt_cmd_interfaces;

  for (std::size_t data_it = 0; data_it < 3; ++data_it)
  {
    if (b_jnt_pos_cmd_defined_[data_it])
    {
      jnt_cmd_interfaces.emplace_back(hardware_interface::CommandInterface(
        jnt_names_[data_it], hardware_interface::HW_IF_POSITION,
        &jnt_pos_cmd_[data_it]));
    }

    if (b_jnt_vel_cmd_defined_[data_it])
    {
      jnt_cmd_interfaces.emplace_back(hardware_interface::CommandInterface(
        jnt_names_[data_it], hardware_interface::HW_IF_VELOCITY,
        &jnt_vel_cmd_[data_it]));
    }
  }

  return jnt_cmd_interfaces;
}

hardware_interface::return_type
PlannerGuiSystemInterface::prepare_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type result = hardware_interface::return_type::OK;

  for (std::size_t stop_it = 0; stop_it < stop_interfaces.size(); ++stop_it)
  {
    for (std::size_t jnt_it = 0; jnt_it < 3; ++jnt_it)
    {
      if (std::string::npos != stop_interfaces[stop_it].find(jnt_names_[jnt_it]))
      {
        jnt_cmd_modes_[jnt_it] = CommandMode::kNone;
        PlannerGuiSimulator::get_instance().stop_jnt(jnt_it);
      }
    }
  }

  for (std::size_t start_it = 0; start_it < start_interfaces.size(); ++start_it)
  {
    for (std::size_t jnt_it = 0; jnt_it < 3; ++jnt_it)
    {
      if (start_interfaces[start_it] ==
          jnt_names_[jnt_it] + "/" + hardware_interface::HW_IF_POSITION)
      {
        if (CommandMode::kNone == jnt_cmd_modes_[jnt_it])
        {
          jnt_cmd_modes_[jnt_it] = CommandMode::kPosition;
        }
        else
        {
          RCLCPP_ERROR(*logger_,
            "New command mode specified for %s while another one is active.",
            jnt_names_[jnt_it].c_str());
          result = hardware_interface::return_type::ERROR;
        }
      }
      else if (start_interfaces[start_it] ==
               jnt_names_[jnt_it] + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        if (CommandMode::kNone == jnt_cmd_modes_[jnt_it])
        {
          jnt_cmd_modes_[jnt_it] = CommandMode::kVelocity;
        }
        else
        {
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

hardware_interface::return_type PlannerGuiSystemInterface::read(
  const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  if (++pose_pub_counter_ >= 10)
  {
    pose_pub_counter_ = 0;

    double pos[3], quat[4];
    PlannerGuiSimulator::get_instance().get_hand_pose(pos, quat);

    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = pos[0];
    pose_msg.position.y = pos[1];
    pose_msg.position.z = pos[2];
    pose_msg.orientation.w = quat[0];
    pose_msg.orientation.x = quat[1];
    pose_msg.orientation.y = quat[2];
    pose_msg.orientation.z = quat[3];

    if (hand_pose_pub_)
    {
      hand_pose_pub_->publish(pose_msg);
    }
    if (hand_pose_alias_pub_)
    {
      hand_pose_alias_pub_->publish(pose_msg);
    }

    if (hand_twist_pub_)
    {
      geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
      twist_msg.header.frame_id = "palm_r";
      twist_msg.header.stamp = rclcpp::Clock().now();
      twist_msg.twist.twist.linear.x = 0.0;
      twist_msg.twist.twist.linear.y = 0.0;
      twist_msg.twist.twist.linear.z = 0.0;
      twist_msg.twist.twist.angular.x = 0.0;
      twist_msg.twist.twist.angular.y = 0.0;
      twist_msg.twist.twist.angular.z = 0.0;
      for (double& covariance_value : twist_msg.twist.covariance)
      {
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
  }

  if (PlannerGuiSimulator::get_instance().consume_planner_request())
  {
    trigger_preshaping_service();
  }

  PlannerGuiSimulator::get_instance().read_jnt_vel(
    jnt_vel_state_[0], jnt_vel_state_[1], jnt_vel_state_[2]);

  if (std::abs(jnt_vel_state_[0]) < 0.001)
  {
    jnt_vel_state_[0] = 0.0;
  }
  if (std::abs(jnt_vel_state_[1]) < 0.001)
  {
    jnt_vel_state_[1] = 0.0;
  }
  if (std::abs(jnt_vel_state_[2]) < 0.001)
  {
    jnt_vel_state_[2] = 0.0;
  }

  PlannerGuiSimulator::get_instance().read_jnt_pos(
    jnt_pos_state_[0], jnt_pos_state_[1], jnt_pos_state_[2]);

  if (std::abs(jnt_pos_state_[0]) < 0.01)
  {
    jnt_pos_state_[0] = 0.0;
  }
  if (std::abs(jnt_pos_state_[1]) < 0.01)
  {
    jnt_pos_state_[1] = 0.0;
  }
  if (std::abs(jnt_pos_state_[2]) < 0.01)
  {
    jnt_pos_state_[2] = 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PlannerGuiSystemInterface::write(
  const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  for (std::size_t jnt_it = 0; jnt_it < 3; ++jnt_it)
  {
    if (CommandMode::kPosition == jnt_cmd_modes_[jnt_it])
    {
      PlannerGuiSimulator::get_instance().set_jnt_pos(jnt_it, jnt_pos_cmd_[jnt_it]);
    }
    else if (CommandMode::kVelocity == jnt_cmd_modes_[jnt_it])
    {
      PlannerGuiSimulator::get_instance().set_jnt_vel(jnt_it, jnt_vel_cmd_[jnt_it]);
    }
  }

  return hardware_interface::return_type::OK;
}

bool PlannerGuiSystemInterface::read_joints_info(
  const std::vector<hardware_interface::ComponentInfo>& jnt_info)
{
  bool success = true;

  if (4 == jnt_info.size())
  {
    std::array<std::string, 3> jnt_roles = {
      "j_thumb_fle", "j_index_fle", "j_mrl_fle"};

    for (std::size_t jnt_roles_it = 0; jnt_roles_it < 3; ++jnt_roles_it)
    {
      const std::string& jnt_role = jnt_roles[jnt_roles_it];
      const auto role_match_it = std::find_if(
        jnt_info.begin(), jnt_info.end(),
        [&jnt_role](const hardware_interface::ComponentInfo& jnt)
        { return std::string::npos != jnt.name.find(jnt_role); });

      if (role_match_it != jnt_info.end())
      {
        jnt_names_[jnt_roles_it] = role_match_it->name;

        if (has_cmd_interface(*role_match_it, hardware_interface::HW_IF_POSITION))
        {
          b_jnt_pos_cmd_defined_[jnt_roles_it] = true;
        }
        if (has_state_interface(*role_match_it, hardware_interface::HW_IF_POSITION))
        {
          b_jnt_pos_state_defined_[jnt_roles_it] = true;
        }
        if (has_cmd_interface(*role_match_it, hardware_interface::HW_IF_VELOCITY))
        {
          b_jnt_vel_cmd_defined_[jnt_roles_it] = true;
        }
        if (has_state_interface(*role_match_it, hardware_interface::HW_IF_VELOCITY))
        {
          b_jnt_vel_state_defined_[jnt_roles_it] = true;
        }
      }
      else
      {
        RCLCPP_FATAL(*logger_,
          "Could not find a joint with %s role.", jnt_role.c_str());
        success = false;
      }
    }
  }
  else
  {
    RCLCPP_FATAL(*logger_, "4 joints expected, but %ld provided.", jnt_info.size());
    success = false;
  }

  return success;
}

bool PlannerGuiSystemInterface::has_cmd_interface(
  const hardware_interface::ComponentInfo& jnt_info,
  const std::string& interface_name)
{
  return (
    jnt_info.command_interfaces.end() !=
    std::find_if(
      jnt_info.command_interfaces.begin(), jnt_info.command_interfaces.end(),
      [&interface_name](const hardware_interface::InterfaceInfo& interface)
      { return interface_name == interface.name; }));
}

bool PlannerGuiSystemInterface::has_state_interface(
  const hardware_interface::ComponentInfo& jnt_info,
  const std::string& interface_name)
{
  return (
    jnt_info.state_interfaces.end() !=
    std::find_if(
      jnt_info.state_interfaces.begin(), jnt_info.state_interfaces.end(),
      [&interface_name](const hardware_interface::InterfaceInfo& interface)
      { return interface_name == interface.name; }));
}

void PlannerGuiSystemInterface::read_rviz2_joints_info(
  const std::vector<hardware_interface::ComponentInfo>& jnt_info)
{
  std::array<std::string, 1> rviz2_jnt_names = {
    "j_thumb_opp"};

  for (std::size_t jnt_it = 0; jnt_it < 1; ++jnt_it)
  {
    const std::string& jnt_name_to_find = rviz2_jnt_names[jnt_it];
    const auto jnt_info_match = std::find_if(
      jnt_info.begin(), jnt_info.end(),
      [&jnt_name_to_find](const hardware_interface::ComponentInfo& jnt)
      { return std::string::npos != jnt.name.find(jnt_name_to_find); });

    if (jnt_info_match != jnt_info.end())
    {
      rviz2_joints_[jnt_it].name = jnt_info_match->name;
    }
    else
    {
      RCLCPP_ERROR(*logger_, "%s not found among the joint names provided. Make "
        "sure that the related joint name contains such part.",
        jnt_name_to_find.c_str());
    }
  }
}

void PlannerGuiSystemInterface::trigger_preshaping_service()
{
  if (!preshaping_trigger_client_)
  {
    PlannerGuiSimulator::get_instance().report_planner_result(
      false, "Preshaping service client not initialized");
    return;
  }

  bool expected = false;
  if (!preshaping_call_running_.compare_exchange_strong(expected, true))
  {
    PlannerGuiSimulator::get_instance().report_planner_result(
      false, "Preshaping call already in flight");
    return;
  }

  auto client = preshaping_trigger_client_;
  auto logger = logger_.get();

  std::thread([this, client, logger]() {
    using namespace std::chrono_literals;

    auto finish = [this](bool success, const std::string& message) {
      PlannerGuiSimulator::get_instance().report_planner_result(success, message);
      preshaping_call_running_.store(false);
    };

    if (!client->wait_for_service(2s))
    {
      if (logger)
      {
        RCLCPP_ERROR(*logger, "Preshaping service /grasp_preshaping/compute_grasp unavailable");
      }
      finish(false, "Preshaping service unavailable");
      return;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(req);
    constexpr auto kTotalTimeout = 15s;
    if (future.wait_for(kTotalTimeout) != std::future_status::ready)
    {
      if (logger)
      {
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
  mia_hand_mujoco::PlannerGuiSystemInterface,
  hardware_interface::SystemInterface)
