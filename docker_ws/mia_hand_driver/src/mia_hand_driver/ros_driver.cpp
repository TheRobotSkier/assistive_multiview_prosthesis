#include "mia_hand_driver/ros_driver.hpp"

#include <cstdlib>

#include <chrono>

namespace mia_hand_driver
{
std::unique_ptr<RosDriver> RosDriver::create(std::shared_ptr<rclcpp::Node>& node)
{
  std::unique_ptr<RosDriver> ptr = std::unique_ptr<RosDriver>(new RosDriver());

  if (false == ptr->init(node))
  {
    ptr = nullptr;
  }

  return ptr;
}

RosDriver::RosDriver():
  is_connected_(false),
  mot_pos_stream_on_(false),
  mot_spe_stream_on_(false),
  mot_cur_stream_on_(false),
  fin_for_stream_on_(false),
  jnt_pos_stream_on_(false),
  jnt_spe_stream_on_(false),
  n_streams_(0)
{
}

bool RosDriver::init(std::shared_ptr<rclcpp::Node>& node)
{
  bool init_success = true;
  node_= node;

  mia_hand_ = CppDriver::create();

  if (nullptr != mia_hand_)
  {
    std::string serial_port = node_->declare_parameter("serial_port", "/dev/ttyUSB0");

    init_topics();
    init_srvs();
    init_actions();
    
    if (mia_hand_->open_serial_port(serial_port))
    {
      RCLCPP_INFO(node_->get_logger(), 
                    "%s serial port opened.", serial_port.c_str());

      /* Reading grasp references, also to check proper connection with Mia Hand.
       */
      if (get_grasps_refs())
      {
        is_connected_ = true;
        RCLCPP_INFO(node_->get_logger(), "Mia Hand connected.");
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(),
            "Failed to read Mia Hand current settings. Please verify that a Mia "
            "Hand device is properly connected to the specified port and "
            "re-connect again through the \"connect\" ROS service.");
      }
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(),
                    "Failed to open %s serial port", serial_port.c_str());
    }

    connection_tmr_ = node->create_wall_timer(
        std::chrono::seconds(1), 
        std::bind(&RosDriver::connection_tmr_fun, this));
    stream_tmr_ = node_->create_wall_timer(
        std::chrono::milliseconds(10), 
        std::bind(&RosDriver::stream_tmr_fun, this));
    stream_tmr_->cancel();
  }
  else
  {
    init_success = false;
    RCLCPP_FATAL(node->get_logger(), 
                    "Failed to create a mia_hand_driver::CppDriver object.");
  }

  return init_success;
}

template<typename ActionMsg>
RosDriver::Action<ActionMsg>::Action(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionMsg>>
    goal_hnd) : 
  goal_handle(goal_hnd)
{
  result = std::make_shared<typename ActionMsg::Result>();
}

void RosDriver::init_topics()
{
  mot_pos_pub_ = node_->create_publisher<mia_hand_msgs::msg::MotorData>(
      "data_streams/motors/positions/data", 1);
  mot_spe_pub_ = node_->create_publisher<mia_hand_msgs::msg::MotorData>(
      "data_streams/motors/speeds/data", 1);
  mot_cur_pub_ = node_->create_publisher<mia_hand_msgs::msg::MotorData>(
      "data_streams/motors/currents/data", 1);

  jnt_pos_pub_ = node_->create_publisher<mia_hand_msgs::msg::JointData>(
      "data_streams/joints/positions/data", 1);
  jnt_spe_pub_ = node_->create_publisher<mia_hand_msgs::msg::JointData>(
      "data_streams/joints/speeds/data", 1);

  fin_for_pub_ = node_->create_publisher<mia_hand_msgs::msg::ForceData>(
      "data_streams/fingers/forces/data", 1);

  return;
}

void RosDriver::init_srvs()
{
  connect_srv_ =
    node_->create_service<mia_hand_msgs::srv::Connect>("connect",
        std::bind(&RosDriver::connect_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  disconnect_srv_ =
    node_->create_service<std_srvs::srv::Trigger>("disconnect",
        std::bind(&RosDriver::disconnect_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  stop_srv_ =
    node_->create_service<std_srvs::srv::Trigger>("stop",
        std::bind(&RosDriver::stop_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  play_srv_ =
    node_->create_service<std_srvs::srv::Trigger>("play",
        std::bind(&RosDriver::play_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  calibrate_mot_pos_srv_ =
    node_->create_service<std_srvs::srv::Trigger>("motors/calibrate_positions",
        std::bind(&RosDriver::calibrate_mot_pos_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  store_current_sets_srv_ =
    node_->create_service<std_srvs::srv::Trigger>("store_current_settings",
        std::bind(&RosDriver::store_current_sets_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  get_fw_version_srv_ =
    node_->create_service<std_srvs::srv::Trigger>("get_firmware_version",
        std::bind(&RosDriver::get_fw_version_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  get_mot_pos_srv_ =
    node_->create_service<mia_hand_msgs::srv::GetMotorData>("motors/get_positions",
        std::bind(&RosDriver::get_mot_pos_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  get_mot_spe_srv_ =
    node_->create_service<mia_hand_msgs::srv::GetMotorData>("motors/get_speeds",
        std::bind(&RosDriver::get_mot_spe_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  get_mot_cur_srv_ =
    node_->create_service<mia_hand_msgs::srv::GetMotorData>("motors/get_currents",
        std::bind(&RosDriver::get_mot_cur_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  get_fin_for_srv_ =
    node_->create_service<mia_hand_msgs::srv::GetForceData>("fingers/get_forces",
        std::bind(&RosDriver::get_fin_for_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  get_jnt_pos_srv_ =
    node_->create_service<mia_hand_msgs::srv::GetJointData>("joints/get_positions",
        std::bind(&RosDriver::get_jnt_pos_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  get_jnt_spe_srv_ =
    node_->create_service<mia_hand_msgs::srv::GetJointData>("joints/get_speeds",
        std::bind(&RosDriver::get_jnt_spe_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  set_thumb_mot_spe_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetMotorSpeed>("motors/thumb/set_speed",
        std::bind(&RosDriver::set_thumb_mot_spe_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  set_index_mot_spe_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetMotorSpeed>("motors/index/set_speed",
        std::bind(&RosDriver::set_index_mot_spe_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  set_mrl_mot_spe_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetMotorSpeed>("motors/mrl/set_speed",
        std::bind(&RosDriver::set_mrl_mot_spe_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  set_thumb_mot_traj_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetMotorTraj>("motors/thumb/set_trajectory",
        std::bind(&RosDriver::set_thumb_mot_traj_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  set_index_mot_traj_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetMotorTraj>("motors/index/set_trajectory",
        std::bind(&RosDriver::set_index_mot_traj_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  set_mrl_mot_traj_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetMotorTraj>("motors/mrl/set_trajectory",
        std::bind(&RosDriver::set_mrl_mot_traj_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  set_thumb_jnt_spe_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetJointSpeed>("joints/thumb/set_speed",
        std::bind(&RosDriver::set_thumb_jnt_spe_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  set_index_jnt_spe_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetJointSpeed>("joints/index/set_speed",
        std::bind(&RosDriver::set_index_jnt_spe_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  set_mrl_jnt_spe_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetJointSpeed>("joints/mrl/set_speed",
        std::bind(&RosDriver::set_mrl_jnt_spe_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  set_thumb_jnt_traj_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetJointTraj>("joints/thumb/set_trajectory",
        std::bind(&RosDriver::set_thumb_jnt_traj_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  set_index_jnt_traj_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetJointTraj>("joints/index/set_trajectory",
        std::bind(&RosDriver::set_index_jnt_traj_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  set_mrl_jnt_traj_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetJointTraj>("joints/mrl/set_trajectory",
        std::bind(&RosDriver::set_mrl_jnt_traj_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  get_cyl_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::GetGraspReferences>("grasps/cylindrical/get_references",
        std::bind(&RosDriver::get_cyl_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  get_pin_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::GetGraspReferences>("grasps/pinch/get_references",
        std::bind(&RosDriver::get_pin_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  get_lat_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::GetGraspReferences>("grasps/lateral/get_references",
        std::bind(&RosDriver::get_lat_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  get_pup_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::GetGraspReferences>("grasps/point_up/get_references",
        std::bind(&RosDriver::get_pup_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  get_pdw_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::GetGraspReferences>("grasps/point_down/get_references",
        std::bind(&RosDriver::get_pdw_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  get_sph_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::GetGraspReferences>("grasps/spherical/get_references",
        std::bind(&RosDriver::get_sph_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  get_tri_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::GetGraspReferences>("grasps/tri_digital/get_references",
        std::bind(&RosDriver::get_tri_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  get_rlx_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::GetGraspReferences>("grasps/relax/get_references",
        std::bind(&RosDriver::get_rlx_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  set_cyl_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetGraspReferences>("grasps/cylindrical/set_references",
        std::bind(&RosDriver::set_cyl_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  set_pin_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetGraspReferences>("grasps/pinch/set_references",
        std::bind(&RosDriver::set_pin_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  set_lat_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetGraspReferences>("grasps/lateral/set_references",
        std::bind(&RosDriver::set_lat_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  set_pup_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetGraspReferences>("grasps/point_up/set_references",
        std::bind(&RosDriver::set_pup_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  set_pdw_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetGraspReferences>("grasps/point_down/set_references",
        std::bind(&RosDriver::set_pdw_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  set_sph_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetGraspReferences>("grasps/spherical/set_references",
        std::bind(&RosDriver::set_sph_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  set_tri_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetGraspReferences>("grasps/tridigital/set_references",
        std::bind(&RosDriver::set_tri_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  set_rlx_refs_srv_ = 
    node_->create_service<mia_hand_msgs::srv::SetGraspReferences>("grasps/relax/set_references",
        std::bind(&RosDriver::set_rlx_refs_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  execute_cyl_srv_ = 
    node_->create_service<mia_hand_msgs::srv::ExecuteGrasp>("grasps/cylindrical/execute",
        std::bind(&RosDriver::execute_cyl_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  execute_pin_srv_ = 
    node_->create_service<mia_hand_msgs::srv::ExecuteGrasp>("grasps/pinch/execute",
        std::bind(&RosDriver::execute_pin_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  execute_lat_srv_ = 
    node_->create_service<mia_hand_msgs::srv::ExecuteGrasp>("grasps/lateral/execute",
        std::bind(&RosDriver::execute_lat_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  execute_pup_srv_ = 
    node_->create_service<mia_hand_msgs::srv::ExecuteGrasp>("grasps/point_up/execute",
        std::bind(&RosDriver::execute_pup_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  execute_pdw_srv_ = 
    node_->create_service<mia_hand_msgs::srv::ExecuteGrasp>("grasps/point_down/execute",
        std::bind(&RosDriver::execute_pdw_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  execute_sph_srv_ = 
    node_->create_service<mia_hand_msgs::srv::ExecuteGrasp>("grasps/spherical/execute",
        std::bind(&RosDriver::execute_sph_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  execute_tri_srv_ = 
    node_->create_service<mia_hand_msgs::srv::ExecuteGrasp>("grasps/tridigital/execute",
        std::bind(&RosDriver::execute_tri_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  execute_rlx_srv_ = 
    node_->create_service<mia_hand_msgs::srv::ExecuteGrasp>("grasps/relax/execute",
        std::bind(&RosDriver::execute_rlx_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  switch_mot_pos_stream_srv_ = 
    node_->create_service<std_srvs::srv::SetBool>("data_streams/motors/positions/switch",
        std::bind(&RosDriver::switch_mot_pos_stream_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  switch_mot_spe_stream_srv_ = 
    node_->create_service<std_srvs::srv::SetBool>("data_streams/motors/speeds/switch",
        std::bind(&RosDriver::switch_mot_spe_stream_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  switch_mot_cur_stream_srv_ = 
    node_->create_service<std_srvs::srv::SetBool>("data_streams/motors/currents/switch",
        std::bind(&RosDriver::switch_mot_cur_stream_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  switch_jnt_pos_stream_srv_ = 
    node_->create_service<std_srvs::srv::SetBool>("data_streams/joints/positions/switch",
        std::bind(&RosDriver::switch_jnt_pos_stream_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  switch_jnt_spe_stream_srv_ = 
    node_->create_service<std_srvs::srv::SetBool>("data_streams/joints/speeds/switch",
        std::bind(&RosDriver::switch_jnt_spe_stream_srv_fun, this, std::placeholders::_1, std::placeholders::_2));
  switch_fin_for_stream_srv_ = 
    node_->create_service<std_srvs::srv::SetBool>("data_streams/fingers/forces/switch",
        std::bind(&RosDriver::switch_fin_for_stream_srv_fun, this, std::placeholders::_1, std::placeholders::_2));

  return;
}

void RosDriver::init_actions()
{
  thumb_mot_traj_act_ = 
    rclcpp_action::create_server<mia_hand_msgs::action::MotorTraj>(
      node_, "motors/thumb/trajectory", 
      std::bind(&RosDriver::thumb_mot_traj_act_goal_fun, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RosDriver::thumb_mot_traj_act_canc_fun, this, std::placeholders::_1),
      std::bind(&RosDriver::thumb_mot_traj_act_accept_fun, this, std::placeholders::_1));
  index_mot_traj_act_ = 
    rclcpp_action::create_server<mia_hand_msgs::action::MotorTraj>(
      node_, "motors/index/trajectory", 
      std::bind(&RosDriver::index_mot_traj_act_goal_fun, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RosDriver::index_mot_traj_act_canc_fun, this, std::placeholders::_1),
      std::bind(&RosDriver::index_mot_traj_act_accept_fun, this, std::placeholders::_1));
  mrl_mot_traj_act_ = 
    rclcpp_action::create_server<mia_hand_msgs::action::MotorTraj>(
      node_, "motors/mrl/trajectory", 
      std::bind(&RosDriver::mrl_mot_traj_act_goal_fun, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RosDriver::mrl_mot_traj_act_canc_fun, this, std::placeholders::_1),
      std::bind(&RosDriver::mrl_mot_traj_act_accept_fun, this, std::placeholders::_1));

  thumb_jnt_traj_act_ = 
    rclcpp_action::create_server<mia_hand_msgs::action::JointTraj>(
      node_, "joints/thumb/trajectory", 
      std::bind(&RosDriver::thumb_jnt_traj_act_goal_fun, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RosDriver::thumb_jnt_traj_act_canc_fun, this, std::placeholders::_1),
      std::bind(&RosDriver::thumb_jnt_traj_act_accept_fun, this, std::placeholders::_1));
  index_jnt_traj_act_ = 
    rclcpp_action::create_server<mia_hand_msgs::action::JointTraj>(
      node_, "joints/index/trajectory", 
      std::bind(&RosDriver::index_jnt_traj_act_goal_fun, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RosDriver::index_jnt_traj_act_canc_fun, this, std::placeholders::_1),
      std::bind(&RosDriver::index_jnt_traj_act_accept_fun, this, std::placeholders::_1));
  mrl_jnt_traj_act_ = 
    rclcpp_action::create_server<mia_hand_msgs::action::JointTraj>(
      node_, "joints/mrl/trajectory", 
      std::bind(&RosDriver::mrl_jnt_traj_act_goal_fun, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RosDriver::mrl_jnt_traj_act_canc_fun, this, std::placeholders::_1),
      std::bind(&RosDriver::mrl_jnt_traj_act_accept_fun, this, std::placeholders::_1));

  cyl_act_ =
    rclcpp_action::create_server<mia_hand_msgs::action::Grasp>(
      node_, "grasps/cylindrical/action", 
      std::bind(&RosDriver::cyl_act_goal_fun, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RosDriver::cyl_act_canc_fun, this, std::placeholders::_1),
      std::bind(&RosDriver::cyl_act_accept_fun, this, std::placeholders::_1));
  pin_act_ =
    rclcpp_action::create_server<mia_hand_msgs::action::Grasp>(
      node_, "grasps/pinch/action", 
      std::bind(&RosDriver::pin_act_goal_fun, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RosDriver::pin_act_canc_fun, this, std::placeholders::_1),
      std::bind(&RosDriver::pin_act_accept_fun, this, std::placeholders::_1));
  lat_act_ =
    rclcpp_action::create_server<mia_hand_msgs::action::Grasp>(
      node_, "grasps/lateral/action", 
      std::bind(&RosDriver::lat_act_goal_fun, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RosDriver::lat_act_canc_fun, this, std::placeholders::_1),
      std::bind(&RosDriver::lat_act_accept_fun, this, std::placeholders::_1));
  pup_act_ =
    rclcpp_action::create_server<mia_hand_msgs::action::Grasp>(
      node_, "grasps/point_up/action", 
      std::bind(&RosDriver::pup_act_goal_fun, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RosDriver::pup_act_canc_fun, this, std::placeholders::_1),
      std::bind(&RosDriver::pup_act_accept_fun, this, std::placeholders::_1));
  pdw_act_ =
    rclcpp_action::create_server<mia_hand_msgs::action::Grasp>(
      node_, "grasps/point_down/action", 
      std::bind(&RosDriver::pdw_act_goal_fun, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RosDriver::pdw_act_canc_fun, this, std::placeholders::_1),
      std::bind(&RosDriver::pdw_act_accept_fun, this, std::placeholders::_1));
  sph_act_ =
    rclcpp_action::create_server<mia_hand_msgs::action::Grasp>(
      node_, "grasps/spherical/action", 
      std::bind(&RosDriver::sph_act_goal_fun, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RosDriver::sph_act_canc_fun, this, std::placeholders::_1),
      std::bind(&RosDriver::sph_act_accept_fun, this, std::placeholders::_1));
  tri_act_ =
    rclcpp_action::create_server<mia_hand_msgs::action::Grasp>(
      node_, "grasps/tridigital/action", 
      std::bind(&RosDriver::tri_act_goal_fun, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RosDriver::tri_act_canc_fun, this, std::placeholders::_1),
      std::bind(&RosDriver::tri_act_accept_fun, this, std::placeholders::_1));
  rlx_act_ =
    rclcpp_action::create_server<mia_hand_msgs::action::Grasp>(
      node_, "grasps/relax/action", 
      std::bind(&RosDriver::rlx_act_goal_fun, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RosDriver::rlx_act_canc_fun, this, std::placeholders::_1),
      std::bind(&RosDriver::rlx_act_accept_fun, this, std::placeholders::_1));

  return;
}

bool RosDriver::get_grasps_refs()
{
  bool success = 
    mia_hand_->get_grasp_refs('C', 0, cyl_open_pos_[0], cyl_close_pos_[0]) &&
    mia_hand_->get_grasp_refs('C', 1, cyl_open_pos_[1], cyl_close_pos_[1]) &&
    mia_hand_->get_grasp_refs('C', 2, cyl_open_pos_[2], cyl_close_pos_[2]) &&
    mia_hand_->get_grasp_refs('P', 0, pin_open_pos_[0], pin_close_pos_[0]) &&
    mia_hand_->get_grasp_refs('P', 1, pin_open_pos_[1], pin_close_pos_[1]) &&
    mia_hand_->get_grasp_refs('P', 2, pin_open_pos_[2], pin_close_pos_[2]) &&
    mia_hand_->get_grasp_refs('L', 0, lat_open_pos_[0], lat_close_pos_[0]) &&
    mia_hand_->get_grasp_refs('L', 1, lat_open_pos_[1], lat_close_pos_[1]) &&
    mia_hand_->get_grasp_refs('L', 2, lat_open_pos_[2], lat_close_pos_[2]) &&
    mia_hand_->get_grasp_refs('U', 0, pup_open_pos_[0], pup_close_pos_[0]) &&
    mia_hand_->get_grasp_refs('U', 1, pup_open_pos_[1], pup_close_pos_[1]) &&
    mia_hand_->get_grasp_refs('U', 2, pup_open_pos_[2], pup_close_pos_[2]) &&
    mia_hand_->get_grasp_refs('D', 0, pdw_open_pos_[0], pdw_close_pos_[0]) &&
    mia_hand_->get_grasp_refs('D', 1, pdw_open_pos_[1], pdw_close_pos_[1]) &&
    mia_hand_->get_grasp_refs('D', 2, pdw_open_pos_[2], pdw_close_pos_[2]) &&
    mia_hand_->get_grasp_refs('S', 0, sph_open_pos_[0], sph_close_pos_[0]) &&
    mia_hand_->get_grasp_refs('S', 1, sph_open_pos_[1], sph_close_pos_[1]) &&
    mia_hand_->get_grasp_refs('S', 2, sph_open_pos_[2], sph_close_pos_[2]) &&
    mia_hand_->get_grasp_refs('T', 0, tri_open_pos_[0], tri_close_pos_[0]) &&
    mia_hand_->get_grasp_refs('T', 1, tri_open_pos_[1], tri_close_pos_[1]) &&
    mia_hand_->get_grasp_refs('T', 2, tri_open_pos_[2], tri_close_pos_[2]) &&
    mia_hand_->get_grasp_refs('R', 0, rlx_open_pos_[0], rlx_close_pos_[0]) &&
    mia_hand_->get_grasp_refs('R', 1, rlx_open_pos_[1], rlx_close_pos_[1]) &&
    mia_hand_->get_grasp_refs('R', 2, rlx_open_pos_[2], rlx_close_pos_[2]);

  return success;
}

void RosDriver::update_stream_period()
{
  if (!stream_tmr_->is_canceled())
  {
    stream_tmr_->cancel();
  }

  if (n_streams_ > 0)
  {
    stream_tmr_ = node_->create_wall_timer(
        std::chrono::milliseconds(10 * n_streams_), 
        std::bind(&RosDriver::stream_tmr_fun, this));
  }

  return;
}

void RosDriver::connection_tmr_fun()
{
  if (mia_hand_->is_connected())
  {
    if (!is_connected_)
    {
      /* Checking again connection by reading grasp references.
       */
      is_connected_ = get_grasps_refs();
      
      if (is_connected_)
      {
        RCLCPP_INFO(node_->get_logger(), "Mia Hand connected.");
      }
    }
  }
  else
  {
    if (is_connected_)
    {
      is_connected_ = false;
      RCLCPP_INFO(node_->get_logger(), "Mia Hand disconnected.");
    }
  }

  return;
}

void RosDriver::connect_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::Connect::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::Connect::Response> rsp)
{
  if (mia_hand_->open_serial_port(req->serial_port_dev))
  {
    /* Checking actual connection with Mia Hand by reading the grasp references.
     */
    rsp->success = get_grasps_refs();

    if (rsp->success)
    {
      if (!is_connected_)  // Mia Hand not already connected
      {
        is_connected_ = true;
        RCLCPP_INFO(node_->get_logger(), "Mia Hand connected.");
      }

      rsp->message = "Mia Hand connected.";
    }
    else
    {
      rsp->message = 
        "Failed to read Mia Hand current settings. Please verify that a Mia "
        "Hand device is properly connected to the specified port and "
        "re-connect again through the \"connect\" ROS service.";
    }
  }
  else
  {
    rsp->success = false;
    rsp->message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::disconnect_srv_fun(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> rsp)
{
  (void)req;  // Explicitly ignoring empty request, to suppress compiler warning.

  rsp->success = mia_hand_->close_serial_port();

  if (rsp->success)
  {
    rsp->message = "Mia Hand serial port closed.";
  }
  else
  {
    rsp->message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::stop_srv_fun(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> rsp)
{
  (void)req;  // Explicitly ignoring empty request, to suppress compiler warning.

  rsp->success = mia_hand_->emergency_stop();

  if (rsp->success)
  {
    rsp->message = "Mia Hand emergency stop triggered.";
  }
  else
  {
    rsp->message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::play_srv_fun(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> rsp)
{
  (void)req;  // Explicitly ignoring empty request, to suppress compiler warning.

  mia_hand_->play();
  rsp->success = true;
  rsp->message = "Mia Hand normal operation restored.";

  return;
}

void RosDriver::calibrate_mot_pos_srv_fun(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> rsp)
{
  (void)req;  // Explicitly ignoring empty request, to suppress compiler warning.

  rsp->success = mia_hand_->calibrate_motor_positions();

  if (rsp->success)
  {
    rsp->message = "Mia Hand motor positions calibration started successfully.";
  }
  else
  {
    rsp->message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::store_current_sets_srv_fun(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> rsp)
{
  (void)req;  // Explicitly ignoring empty request, to suppress compiler warning.

  rsp->success = mia_hand_->store_current_settings();

  if (rsp->success)
  {
    rsp->message = "Mia Hand current settings stored successfully.";
  }
  else
  {
    rsp->message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::get_fw_version_srv_fun(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> rsp)
{
  (void)req;  // Explicitly ignoring empty request, to suppress compiler warning.

  rsp->message = mia_hand_->get_fw_version();

  if (!rsp->message.empty())
  {
    rsp->success = true;
  }
  else
  {
    rsp->success = false;
    rsp->message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::get_mot_pos_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::GetMotorData::Request> /* req */, 
    std::shared_ptr<mia_hand_msgs::srv::GetMotorData::Response> rsp)
{
  rsp->success = mia_hand_->get_motor_positions(
      rsp->thumb_data, rsp->index_data, rsp->mrl_data);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::get_mot_spe_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::GetMotorData::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::GetMotorData::Response> rsp)
{
  (void)req;  // Explicitly ignoring empty request, to suppress compiler warning.

  rsp->success = mia_hand_->get_motor_speeds(
      rsp->thumb_data, rsp->index_data, rsp->mrl_data);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::get_mot_cur_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::GetMotorData::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::GetMotorData::Response> rsp)
{
  (void)req;  // Explicitly ignoring empty request, to suppress compiler warning.

  rsp->success = mia_hand_->get_motor_currents(
      rsp->thumb_data, rsp->index_data, rsp->mrl_data);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::get_fin_for_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::GetForceData::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::GetForceData::Response> rsp)
{
  (void)req;  // Explicitly ignoring empty request, to suppress compiler warning.

  rsp->success = mia_hand_->get_finger_forces(
      rsp->thumb_nfor, rsp->index_nfor, rsp->mrl_nfor, 
      rsp->thumb_tfor, rsp->index_tfor, rsp->mrl_tfor);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::get_jnt_pos_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::GetJointData::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::GetJointData::Response> rsp)
{
  (void)req;  // Explicitly ignoring empty request, to suppress compiler warning.

  rsp->success = mia_hand_->get_joint_positions(
      rsp->thumb_data, rsp->index_data, rsp->mrl_data);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::get_jnt_spe_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::GetJointData::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::GetJointData::Response> rsp)
{
  (void)req;  // Explicitly ignoring empty request, to suppress compiler warning.

  rsp->success = mia_hand_->get_joint_speeds(
      rsp->thumb_data, rsp->index_data, rsp->mrl_data);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_thumb_mot_spe_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetMotorSpeed::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::SetMotorSpeed::Response> rsp)
{
  /* Canceling any previous thumb trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->set_motor_speed(
                              0, req->target_speed, req->max_current_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_index_mot_spe_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetMotorSpeed::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::SetMotorSpeed::Response> rsp)
{
  /* Canceling any previous index trajectory or grasp goal.
   */
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->set_motor_speed(
                              1, req->target_speed, req->max_current_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_mrl_mot_spe_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetMotorSpeed::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::SetMotorSpeed::Response> rsp)
{
  /* Canceling any previous mrl trajectory or grasp goal.
   */
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->set_motor_speed(
                              2, req->target_speed, req->max_current_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_thumb_mot_traj_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetMotorTraj::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::SetMotorTraj::Response> rsp)
{
  /* Canceling any previous thumb trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->set_motor_trajectory(
                              0, req->target_position, req->spe_for_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_index_mot_traj_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetMotorTraj::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::SetMotorTraj::Response> rsp)
{
  /* Canceling any previous index trajectory or grasp goal.
   */
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->set_motor_trajectory(
                              1, req->target_position, req->spe_for_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_mrl_mot_traj_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetMotorTraj::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::SetMotorTraj::Response> rsp)
{
  /* Canceling any previous mrl trajectory or grasp goal.
   */
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->set_motor_trajectory(
                              2, req->target_position, req->spe_for_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_thumb_jnt_spe_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetJointSpeed::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::SetJointSpeed::Response> rsp)
{
  /* Canceling any previous thumb trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->set_joint_speed(
                              0, req->target_speed, req->max_current_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_index_jnt_spe_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetJointSpeed::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::SetJointSpeed::Response> rsp)
{
  /* Canceling any previous index trajectory or grasp goal.
   */
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->set_joint_speed(
                              1, req->target_speed, req->max_current_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_mrl_jnt_spe_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetJointSpeed::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::SetJointSpeed::Response> rsp)
{
  /* Canceling any previous mrl trajectory or grasp goal.
   */
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->set_joint_speed(
                              2, req->target_speed, req->max_current_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_thumb_jnt_traj_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetJointTraj::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::SetJointTraj::Response> rsp)
{
  /* Canceling any previous thumb trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->set_joint_trajectory(
                              0, req->target_angle, req->spe_for_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_index_jnt_traj_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetJointTraj::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::SetJointTraj::Response> rsp)
{
  /* Canceling any previous index trajectory or grasp goal.
   */
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->set_joint_trajectory(
                              1, req->target_angle, req->spe_for_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_mrl_jnt_traj_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetJointTraj::Request> req, 
    std::shared_ptr<mia_hand_msgs::srv::SetJointTraj::Response> rsp)
{
  /* Canceling any previous mrl trajectory or grasp goal.
   */
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->set_joint_trajectory(
                              2, req->target_angle, req->spe_for_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

rclcpp_action::GoalResponse RosDriver::thumb_mot_traj_act_goal_fun(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mia_hand_msgs::action::MotorTraj::Goal> goal)
{
  (void)uuid;  // Explicitly discarding unused goal ID, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(), 
      "Received new thumb motor trajectory goal with target position %d.", goal->target_position);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosDriver::thumb_mot_traj_act_canc_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::MotorTraj>> goal_handle)
{
  (void)goal_handle;  // Explicitly discarding unused goal handle, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(),
      "Received request to cancel thumb motor trajectory goal.");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosDriver::thumb_mot_traj_act_accept_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::MotorTraj>> goal_handle)
{
  /* Canceling any previous thumb trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  /* Creating new action.
   */
  std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>> action =
    std::make_shared<Action<mia_hand_msgs::action::MotorTraj>>(goal_handle);

  /* Adding new action to the queue.
   */
  {
    std::lock_guard<std::mutex> lock(actions_mtx_);
    thumb_mot_traj_actions_.emplace_back(action);
  }

  /* Spinning new background thread to execute the action.
   */
  std::thread{std::bind(
    &RosDriver::thumb_mot_traj_act_execute, this, std::placeholders::_1), 
    action}.detach();

  return;
}

void RosDriver::thumb_mot_traj_act_execute(
  std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>> action)
{
  RCLCPP_INFO(node_->get_logger(),
      "Executing thumb motor trajectory goal...");

  const std::shared_ptr<const mia_hand_msgs::action::MotorTraj::Goal> 
    goal = action->goal_handle->get_goal();
  std::shared_ptr<mia_hand_msgs::action::MotorTraj::Feedback>
    feedback = std::make_shared<mia_hand_msgs::action::MotorTraj::Feedback>();

  int32_t ignored_val = 0;  // Used to discard unused feedback values.

  rclcpp::Rate rate(2.0);  // Running action loop at 2 Hz.

  if (mia_hand_->set_motor_trajectory(0, goal->target_position, goal->spe_for_percent))
  {
    while (action->goal_handle->is_active() && rclcpp::ok())
    {
      if (
        (mia_hand_->get_motor_positions(
          feedback->current_position, ignored_val, ignored_val)) &&
        (mia_hand_->get_motor_speeds(
          feedback->current_speed, ignored_val, ignored_val)))
      {
        if (!action->goal_handle->is_canceling())
        {
          if (feedback->current_position == goal->target_position)
          {
            RCLCPP_INFO(node_->get_logger(), "Thumb motor trajectory goal succeded.");

            std::lock_guard<std::mutex> lock(actions_mtx_);
            action->result->position = feedback->current_position;
            action->goal_handle->succeed(action->result);
          }

          action->goal_handle->publish_feedback(feedback);
        }
        else
        {
          mia_hand_->stop();

          RCLCPP_INFO(node_->get_logger(), 
              "Thumb motor trajectory goal canceled.");

          std::lock_guard<std::mutex> lock(actions_mtx_);
          action->result->position = feedback->current_position;
          action->goal_handle->canceled(action->result);
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), 
            "Thumb motor trajectory action aborted.");

        std::lock_guard<std::mutex> lock(actions_mtx_);
        action->result->err_message = mia_hand_->get_error_msg();
        action->goal_handle->abort(action->result);
      }

      rate.sleep();
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Thumb motor trajectory action aborted.");

    std::lock_guard<std::mutex> lock(actions_mtx_);
    action->result->err_message = mia_hand_->get_error_msg();
    action->goal_handle->abort(action->result);
  }

  /* Removing ended action from queue.
   */
  remove_by_goal_id(
    thumb_mot_traj_actions_, action->goal_handle->get_goal_id());

  return;
}

rclcpp_action::GoalResponse RosDriver::index_mot_traj_act_goal_fun(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mia_hand_msgs::action::MotorTraj::Goal> goal)
{
  (void)uuid;  // Explicitly discarding unused goal ID, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(), 
      "Received new index motor trajectory goal with target position %d.", goal->target_position);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosDriver::index_mot_traj_act_canc_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::MotorTraj>> goal_handle)
{
  (void)goal_handle;  // Explicitly discarding unused goal handle, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(),
      "Received request to cancel index motor trajectory goal.");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosDriver::index_mot_traj_act_accept_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::MotorTraj>> goal_handle)
{
  /* Canceling any previous index trajectory or grasp goal.
   */
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  /* Creating new action.
   */
  std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>> action =
    std::make_shared<Action<mia_hand_msgs::action::MotorTraj>>(goal_handle);

  /* Adding new action to the queue.
   */
  {
    std::lock_guard<std::mutex> lock(actions_mtx_);
    index_mot_traj_actions_.emplace_back(action);
  }

  /* Spinning new background thread to execute the action.
   */
  std::thread{std::bind(
    &RosDriver::index_mot_traj_act_execute, this, std::placeholders::_1), 
    action}.detach();

  return;
}

void RosDriver::index_mot_traj_act_execute(
  std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>> action)
{
  RCLCPP_INFO(node_->get_logger(),
      "Executing index motor trajectory goal...");

  const std::shared_ptr<const mia_hand_msgs::action::MotorTraj::Goal> 
    goal = action->goal_handle->get_goal();
  std::shared_ptr<mia_hand_msgs::action::MotorTraj::Feedback>
    feedback = std::make_shared<mia_hand_msgs::action::MotorTraj::Feedback>();

  int32_t ignored_val = 0;  // Used to discard unused feedback values.

  rclcpp::Rate rate(2.0);  // Running action loop at 2 Hz.

  if (mia_hand_->set_motor_trajectory(1, goal->target_position, goal->spe_for_percent))
  {
    while (action->goal_handle->is_active() && rclcpp::ok())
    {
      if (
        (mia_hand_->get_motor_positions(
          ignored_val, feedback->current_position, ignored_val)) &&
        (mia_hand_->get_motor_speeds(
          ignored_val, feedback->current_speed, ignored_val)))
      {
        if (!action->goal_handle->is_canceling())
        {
          if (feedback->current_position == goal->target_position)
          {
            RCLCPP_INFO(node_->get_logger(), "Index motor trajectory goal succeded.");

            std::lock_guard<std::mutex> lock(actions_mtx_);
            action->result->position = feedback->current_position;
            action->goal_handle->succeed(action->result);
          }

          action->goal_handle->publish_feedback(feedback);
        }
        else
        {
          mia_hand_->stop();

          RCLCPP_INFO(node_->get_logger(), 
              "Index motor trajectory goal canceled.");

          std::lock_guard<std::mutex> lock(actions_mtx_);
          action->result->position = feedback->current_position;
          action->goal_handle->canceled(action->result);
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), 
            "Index motor trajectory action aborted.");

        std::lock_guard<std::mutex> lock(actions_mtx_);
        action->result->err_message = mia_hand_->get_error_msg();
        action->goal_handle->abort(action->result);
      }

      rate.sleep();
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Index motor trajectory action aborted.");

    std::lock_guard<std::mutex> lock(actions_mtx_);
    action->result->err_message = mia_hand_->get_error_msg();
    action->goal_handle->abort(action->result);
  }

  /* Removing ended action from queue.
   */
  remove_by_goal_id(
    index_mot_traj_actions_, action->goal_handle->get_goal_id());

  return;
}

rclcpp_action::GoalResponse RosDriver::mrl_mot_traj_act_goal_fun(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mia_hand_msgs::action::MotorTraj::Goal> goal)
{
  (void)uuid;  // Explicitly discarding unused goal ID, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(), 
      "Received new mrl motor trajectory goal with target position %d.", goal->target_position);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosDriver::mrl_mot_traj_act_canc_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::MotorTraj>> goal_handle)
{
  (void)goal_handle;  // Explicitly discarding unused goal handle, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(),
      "Received request to cancel mrl motor trajectory goal.");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosDriver::mrl_mot_traj_act_accept_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::MotorTraj>> goal_handle)
{
  /* Canceling any previous mrl trajectory or grasp goal.
   */
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  /* Creating new action.
   */
  std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>> action =
    std::make_shared<Action<mia_hand_msgs::action::MotorTraj>>(goal_handle);

  /* Adding new action to the queue.
   */
  {
    std::lock_guard<std::mutex> lock(actions_mtx_);
    mrl_mot_traj_actions_.emplace_back(action);
  }

  /* Spinning new background thread to execute the action.
   */
  std::thread{std::bind(
    &RosDriver::mrl_mot_traj_act_execute, this, std::placeholders::_1), 
    action}.detach();

  return;
}

void RosDriver::mrl_mot_traj_act_execute(
  std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>> action)
{
  RCLCPP_INFO(node_->get_logger(),
      "Executing mrl motor trajectory goal...");

  const std::shared_ptr<const mia_hand_msgs::action::MotorTraj::Goal> 
    goal = action->goal_handle->get_goal();
  std::shared_ptr<mia_hand_msgs::action::MotorTraj::Feedback>
    feedback = std::make_shared<mia_hand_msgs::action::MotorTraj::Feedback>();

  int32_t ignored_val = 0;  // Used to discard unused feedback values.

  rclcpp::Rate rate(2.0);  // Running action loop at 2 Hz.

  if (mia_hand_->set_motor_trajectory(2, goal->target_position, goal->spe_for_percent))
  {
    while (action->goal_handle->is_active() && rclcpp::ok())
    {
      if (
        (mia_hand_->get_motor_positions(
          ignored_val, ignored_val, feedback->current_position)) &&
        (mia_hand_->get_motor_speeds(
          ignored_val, ignored_val, feedback->current_speed)))
      {
        if (!action->goal_handle->is_canceling())
        {
          if (feedback->current_position == goal->target_position)
          {
            RCLCPP_INFO(node_->get_logger(), "Mrl motor trajectory goal succeded.");

            std::lock_guard<std::mutex> lock(actions_mtx_);
            action->result->position = feedback->current_position;
            action->goal_handle->succeed(action->result);
          }

          action->goal_handle->publish_feedback(feedback);
        }
        else
        {
          mia_hand_->stop();

          RCLCPP_INFO(node_->get_logger(), 
              "Mrl motor trajectory goal canceled.");

          std::lock_guard<std::mutex> lock(actions_mtx_);
          action->result->position = feedback->current_position;
          action->goal_handle->canceled(action->result);
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), 
            "Mrl motor trajectory action aborted.");

        std::lock_guard<std::mutex> lock(actions_mtx_);
        action->result->err_message = mia_hand_->get_error_msg();
        action->goal_handle->abort(action->result);
      }

      rate.sleep();
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Mrl motor trajectory action aborted.");

    std::lock_guard<std::mutex> lock(actions_mtx_);
    action->result->err_message = mia_hand_->get_error_msg();
    action->goal_handle->abort(action->result);
  }

  /* Removing ended action from queue.
   */
  remove_by_goal_id(
    mrl_mot_traj_actions_, action->goal_handle->get_goal_id());

  return;
}

rclcpp_action::GoalResponse RosDriver::thumb_jnt_traj_act_goal_fun(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mia_hand_msgs::action::JointTraj::Goal> goal)
{
  (void)uuid;  // Explicitly discarding unused goal ID, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(), 
      "Received new thumb joint trajectory goal with target angle of %.2f rad.", goal->target_angle);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosDriver::thumb_jnt_traj_act_canc_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::JointTraj>> goal_handle)
{
  (void)goal_handle;  // Explicitly discarding unused goal handle, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(),
      "Received request to cancel thumb joint trajectory goal.");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosDriver::thumb_jnt_traj_act_accept_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::JointTraj>> goal_handle)
{
  /* Canceling any previous thumb trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  /* Creating new action.
   */
  std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>> action =
    std::make_shared<Action<mia_hand_msgs::action::JointTraj>>(goal_handle);

  /* Adding new action to the queue.
   */
  {
    std::lock_guard<std::mutex> lock(actions_mtx_);
    thumb_jnt_traj_actions_.emplace_back(action);
  }

  /* Spinning new background thread to execute the action.
   */
  std::thread{std::bind(
    &RosDriver::thumb_jnt_traj_act_execute, this, std::placeholders::_1), 
    action}.detach();

  return;
}

void RosDriver::thumb_jnt_traj_act_execute(
  std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>> action)
{
  RCLCPP_INFO(node_->get_logger(),
      "Executing thumb joint trajectory goal...");

  const std::shared_ptr<const mia_hand_msgs::action::JointTraj::Goal> 
    goal = action->goal_handle->get_goal();
  std::shared_ptr<mia_hand_msgs::action::JointTraj::Feedback>
    feedback = std::make_shared<mia_hand_msgs::action::JointTraj::Feedback>();

  double ignored_ang = 0.0f;  // Used to discard unused angles.
  int32_t ignored_spe = 0;   // Used to discard unused speeds.

  rclcpp::Rate rate(2.0);  // Running action loop at 2 Hz.

  if (mia_hand_->set_joint_trajectory(0, goal->target_angle, goal->spe_for_percent))
  {
    while (action->goal_handle->is_active() && rclcpp::ok())
    {
      if (
        (mia_hand_->get_joint_positions(
          feedback->current_angle, ignored_ang, ignored_ang)) &&
        (mia_hand_->get_motor_speeds(
          feedback->current_speed, ignored_spe, ignored_spe)))
      {
        if (!action->goal_handle->is_canceling())
        {
          if (std::abs(goal->target_angle - feedback->current_angle) < 0.01f)
          {
            RCLCPP_INFO(node_->get_logger(), "Thumb joint trajectory goal succeded.");

            std::lock_guard<std::mutex> lock(actions_mtx_);
            action->result->angle = feedback->current_angle;
            action->goal_handle->succeed(action->result);
          }

          action->goal_handle->publish_feedback(feedback);
        }
        else
        {
          mia_hand_->stop();

          RCLCPP_INFO(node_->get_logger(), 
              "Thumb joint trajectory goal canceled.");

          std::lock_guard<std::mutex> lock(actions_mtx_);
          action->result->angle = feedback->current_angle;
          action->goal_handle->canceled(action->result);
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), 
            "Thumb joint trajectory action aborted.");

        std::lock_guard<std::mutex> lock(actions_mtx_);
        action->result->err_message = mia_hand_->get_error_msg();
        action->goal_handle->abort(action->result);
      }

      rate.sleep();
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Thumb joint trajectory action aborted.");

    std::lock_guard<std::mutex> lock(actions_mtx_);
    action->result->err_message = mia_hand_->get_error_msg();
    action->goal_handle->abort(action->result);
  }

  /* Removing ended action from queue.
   */
  remove_by_goal_id(
    thumb_jnt_traj_actions_, action->goal_handle->get_goal_id());

  return;
}

rclcpp_action::GoalResponse RosDriver::index_jnt_traj_act_goal_fun(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mia_hand_msgs::action::JointTraj::Goal> goal)
{
  (void)uuid;  // Explicitly discarding unused goal ID, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(), 
      "Received new index joint trajectory goal with target angle of %.2f rad.", goal->target_angle);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosDriver::index_jnt_traj_act_canc_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::JointTraj>> goal_handle)
{
  (void)goal_handle;  // Explicitly discarding unused goal handle, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(),
      "Received request to cancel index joint trajectory goal.");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosDriver::index_jnt_traj_act_accept_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::JointTraj>> goal_handle)
{
  /* Canceling any previous index trajectory or grasp goal.
   */
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  /* Creating new action.
   */
  std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>> action =
    std::make_shared<Action<mia_hand_msgs::action::JointTraj>>(goal_handle);

  /* Adding new action to the queue.
   */
  {
    std::lock_guard<std::mutex> lock(actions_mtx_);
    index_jnt_traj_actions_.emplace_back(action);
  }

  /* Spinning new background thread to execute the action.
   */
  std::thread{std::bind(
    &RosDriver::index_jnt_traj_act_execute, this, std::placeholders::_1), 
    action}.detach();

  return;
}

void RosDriver::index_jnt_traj_act_execute(
  std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>> action)
{
  RCLCPP_INFO(node_->get_logger(),
      "Executing index joint trajectory goal...");

  const std::shared_ptr<const mia_hand_msgs::action::JointTraj::Goal> 
    goal = action->goal_handle->get_goal();
  std::shared_ptr<mia_hand_msgs::action::JointTraj::Feedback>
    feedback = std::make_shared<mia_hand_msgs::action::JointTraj::Feedback>();

  double ignored_ang = 0.0f;  // Used to discard unused angles.
  int32_t ignored_spe = 0;   // Used to discard unused speeds.

  rclcpp::Rate rate(2.0);  // Running action loop at 2 Hz.

  if (mia_hand_->set_joint_trajectory(1, goal->target_angle, goal->spe_for_percent))
  {
    while (action->goal_handle->is_active() && rclcpp::ok())
    {
      if (
        (mia_hand_->get_joint_positions(
          ignored_ang, feedback->current_angle, ignored_ang)) &&
        (mia_hand_->get_motor_speeds(
          ignored_spe, feedback->current_speed, ignored_spe)))
      {
        if (!action->goal_handle->is_canceling())
        {
          if (std::abs(goal->target_angle - feedback->current_angle) < 0.01f)
          {
            RCLCPP_INFO(node_->get_logger(), "Index joint trajectory goal succeded.");

            std::lock_guard<std::mutex> lock(actions_mtx_);
            action->result->angle = feedback->current_angle;
            action->goal_handle->succeed(action->result);
          }

          action->goal_handle->publish_feedback(feedback);
        }
        else
        {
          mia_hand_->stop();

          RCLCPP_INFO(node_->get_logger(), 
              "Index joint trajectory goal canceled.");

          std::lock_guard<std::mutex> lock(actions_mtx_);
          action->result->angle = feedback->current_angle;
          action->goal_handle->canceled(action->result);
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), 
            "Index joint trajectory action aborted.");

        std::lock_guard<std::mutex> lock(actions_mtx_);
        action->result->err_message = mia_hand_->get_error_msg();
        action->goal_handle->abort(action->result);
      }

      rate.sleep();
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Index joint trajectory action aborted.");

    std::lock_guard<std::mutex> lock(actions_mtx_);
    action->result->err_message = mia_hand_->get_error_msg();
    action->goal_handle->abort(action->result);
  }

  /* Removing ended action from queue.
   */
  remove_by_goal_id(
    index_jnt_traj_actions_, action->goal_handle->get_goal_id());

  return;
}

rclcpp_action::GoalResponse RosDriver::mrl_jnt_traj_act_goal_fun(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mia_hand_msgs::action::JointTraj::Goal> goal)
{
  (void)uuid;  // Explicitly discarding unused goal ID, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(), 
      "Received new mrl joint trajectory goal with target angle of %.2f rad.", goal->target_angle);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosDriver::mrl_jnt_traj_act_canc_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::JointTraj>> goal_handle)
{
  (void)goal_handle;  // Explicitly discarding unused goal handle, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(),
      "Received request to cancel mrl joint trajectory goal.");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosDriver::mrl_jnt_traj_act_accept_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::JointTraj>> goal_handle)
{
  /* Canceling any previous mrl trajectory or grasp goal.
   */
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  /* Creating new action.
   */
  std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>> action =
    std::make_shared<Action<mia_hand_msgs::action::JointTraj>>(goal_handle);

  /* Adding new action to the queue.
   */
  {
    std::lock_guard<std::mutex> lock(actions_mtx_);
    mrl_jnt_traj_actions_.emplace_back(action);
  }

  /* Spinning new background thread to execute the action.
   */
  std::thread{std::bind(
    &RosDriver::mrl_jnt_traj_act_execute, this, std::placeholders::_1), 
    action}.detach();

  return;
}

void RosDriver::mrl_jnt_traj_act_execute(
  std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>> action)
{
  RCLCPP_INFO(node_->get_logger(),
      "Executing mrl joint trajectory goal...");

  const std::shared_ptr<const mia_hand_msgs::action::JointTraj::Goal> 
    goal = action->goal_handle->get_goal();
  std::shared_ptr<mia_hand_msgs::action::JointTraj::Feedback>
    feedback = std::make_shared<mia_hand_msgs::action::JointTraj::Feedback>();

  double ignored_ang = 0.0f;  // Used to discard unused angles.
  int32_t ignored_spe = 0;   // Used to discard unused speeds.

  rclcpp::Rate rate(2.0);  // Running action loop at 2 Hz.

  if (mia_hand_->set_joint_trajectory(2, goal->target_angle, goal->spe_for_percent))
  {
    while (action->goal_handle->is_active() && rclcpp::ok())
    {
      if (
        (mia_hand_->get_joint_positions(
          ignored_ang, ignored_ang, feedback->current_angle)) &&
        (mia_hand_->get_motor_speeds(
          ignored_spe, ignored_spe, feedback->current_speed)))
      {
        if (!action->goal_handle->is_canceling())
        {
          if (std::abs(goal->target_angle - feedback->current_angle) < 0.01f)
          {
            RCLCPP_INFO(node_->get_logger(), "Mrl joint trajectory goal succeded.");

            std::lock_guard<std::mutex> lock(actions_mtx_);
            action->result->angle = feedback->current_angle;
            action->goal_handle->succeed(action->result);
          }

          action->goal_handle->publish_feedback(feedback);
        }
        else
        {
          mia_hand_->stop();

          RCLCPP_INFO(node_->get_logger(), 
              "Mrl joint trajectory goal canceled.");

          std::lock_guard<std::mutex> lock(actions_mtx_);
          action->result->angle = feedback->current_angle;
          action->goal_handle->canceled(action->result);
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), 
            "Mrl joint trajectory action aborted.");

        std::lock_guard<std::mutex> lock(actions_mtx_);
        action->result->err_message = mia_hand_->get_error_msg();
        action->goal_handle->abort(action->result);
      }

      rate.sleep();
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Mrl joint trajectory action aborted.");

    std::lock_guard<std::mutex> lock(actions_mtx_);
    action->result->err_message = mia_hand_->get_error_msg();
    action->goal_handle->abort(action->result);
  }

  /* Removing ended action from queue.
   */
  remove_by_goal_id(
    mrl_jnt_traj_actions_, action->goal_handle->get_goal_id());

  return;
}

void RosDriver::get_cyl_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp)
{
  rsp->success = mia_hand_->get_grasp_refs('C', req->motor, 
                                            rsp->open_mpos, rsp->close_mpos);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::get_pin_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp)
{
  rsp->success = mia_hand_->get_grasp_refs('P', req->motor, 
                                            rsp->open_mpos, rsp->close_mpos);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::get_lat_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp)
{
  rsp->success = mia_hand_->get_grasp_refs('L', req->motor, 
                                            rsp->open_mpos, rsp->close_mpos);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::get_pup_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp)
{
  rsp->success = mia_hand_->get_grasp_refs('U', req->motor, 
                                            rsp->open_mpos, rsp->close_mpos);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::get_pdw_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp)
{
  rsp->success = mia_hand_->get_grasp_refs('D', req->motor, 
                                            rsp->open_mpos, rsp->close_mpos);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::get_sph_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp)
{
  rsp->success = mia_hand_->get_grasp_refs('S', req->motor, 
                                            rsp->open_mpos, rsp->close_mpos);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::get_tri_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp)
{
  rsp->success = mia_hand_->get_grasp_refs('T', req->motor, 
                                            rsp->open_mpos, rsp->close_mpos);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::get_rlx_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp)
{
  rsp->success = mia_hand_->get_grasp_refs('R', req->motor, 
                                            rsp->open_mpos, rsp->close_mpos);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_cyl_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp)
{
  if (
    mia_hand_->set_grasp_refs(
      'C', req->motor, req->open_mpos, req->close_mpos) &&
    mia_hand_->get_grasp_refs(
      'C', req->motor, cyl_open_pos_[req->motor], cyl_close_pos_[req->motor]))
  {
    /* Checking if the grasp references read from Mia Hand after the setting 
     * actually correspond to the desired ones.
     */
    if ((cyl_open_pos_[req->motor] == req->open_mpos) &&
        (cyl_close_pos_[req->motor] == req->close_mpos))
    {
      rsp->success = true;
    }
    else
    {
      rsp->success = false;
      rsp->err_message = "Failed to set the target grasp references.";
    }
  }
  else
  {
    rsp->success = false;
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_pin_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp)
{
  if (
    mia_hand_->set_grasp_refs(
      'P', req->motor, req->open_mpos, req->close_mpos) &&
    mia_hand_->get_grasp_refs(
      'P', req->motor, pin_open_pos_[req->motor], pin_close_pos_[req->motor]))
  {
    /* Checking if the grasp references read from Mia Hand after the setting 
     * actually correspond to the desired ones.
     */
    if ((pin_open_pos_[req->motor] == req->open_mpos) &&
        (pin_close_pos_[req->motor] == req->close_mpos))
    {
      rsp->success = true;
    }
    else
    {
      rsp->success = false;
      rsp->err_message = "Failed to set the target grasp references.";
    }
  }
  else
  {
    rsp->success = false;
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_lat_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp)
{
  if (
    mia_hand_->set_grasp_refs(
      'L', req->motor, req->open_mpos, req->close_mpos) &&
    mia_hand_->get_grasp_refs(
      'L', req->motor, lat_open_pos_[req->motor], lat_close_pos_[req->motor]))
  {
    /* Checking if the grasp references read from Mia Hand after the setting 
     * actually correspond to the desired ones.
     */
    if ((lat_open_pos_[req->motor] == req->open_mpos) &&
        (lat_close_pos_[req->motor] == req->close_mpos))
    {
      rsp->success = true;
    }
    else
    {
      rsp->success = false;
      rsp->err_message = "Failed to set the target grasp references.";
    }
  }
  else
  {
    rsp->success = false;
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_pup_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp)
{
  if (
    mia_hand_->set_grasp_refs(
      'U', req->motor, req->open_mpos, req->close_mpos) &&
    mia_hand_->get_grasp_refs(
      'U', req->motor, pup_open_pos_[req->motor], pup_close_pos_[req->motor]))
  {
    /* Checking if the grasp references read from Mia Hand after the setting 
     * actually correspond to the desired ones.
     */
    if ((pup_open_pos_[req->motor] == req->open_mpos) &&
        (pup_close_pos_[req->motor] == req->close_mpos))
    {
      rsp->success = true;
    }
    else
    {
      rsp->success = false;
      rsp->err_message = "Failed to set the target grasp references.";
    }
  }
  else
  {
    rsp->success = false;
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_pdw_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp)
{
  if (
    mia_hand_->set_grasp_refs(
      'D', req->motor, req->open_mpos, req->close_mpos) &&
    mia_hand_->get_grasp_refs(
      'D', req->motor, pdw_open_pos_[req->motor], pdw_close_pos_[req->motor]))
  {
    /* Checking if the grasp references read from Mia Hand after the setting 
     * actually correspond to the desired ones.
     */
    if ((pdw_open_pos_[req->motor] == req->open_mpos) &&
        (pdw_close_pos_[req->motor] == req->close_mpos))
    {
      rsp->success = true;
    }
    else
    {
      rsp->success = false;
      rsp->err_message = "Failed to set the target grasp references.";
    }
  }
  else
  {
    rsp->success = false;
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_sph_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp)
{
  if (
    mia_hand_->set_grasp_refs(
      'S', req->motor, req->open_mpos, req->close_mpos) &&
    mia_hand_->get_grasp_refs(
      'S', req->motor, sph_open_pos_[req->motor], sph_close_pos_[req->motor]))
  {
    /* Checking if the grasp references read from Mia Hand after the setting 
     * actually correspond to the desired ones.
     */
    if ((sph_open_pos_[req->motor] == req->open_mpos) &&
        (sph_close_pos_[req->motor] == req->close_mpos))
    {
      rsp->success = true;
    }
    else
    {
      rsp->success = false;
      rsp->err_message = "Failed to set the target grasp references.";
    }
  }
  else
  {
    rsp->success = false;
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_tri_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp)
{
  if (
    mia_hand_->set_grasp_refs(
      'T', req->motor, req->open_mpos, req->close_mpos) &&
    mia_hand_->get_grasp_refs(
      'T', req->motor, tri_open_pos_[req->motor], tri_close_pos_[req->motor]))
  {
    /* Checking if the grasp references read from Mia Hand after the setting 
     * actually correspond to the desired ones.
     */
    if ((tri_open_pos_[req->motor] == req->open_mpos) &&
        (tri_close_pos_[req->motor] == req->close_mpos))
    {
      rsp->success = true;
    }
    else
    {
      rsp->success = false;
      rsp->err_message = "Failed to set the target grasp references.";
    }
  }
  else
  {
    rsp->success = false;
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::set_rlx_refs_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp)
{
  if (
    mia_hand_->set_grasp_refs(
      'R', req->motor, req->open_mpos, req->close_mpos) &&
    mia_hand_->get_grasp_refs(
      'R', req->motor, rlx_open_pos_[req->motor], rlx_close_pos_[req->motor]))
  {
    /* Checking if the grasp references read from Mia Hand after the setting 
     * actually correspond to the desired ones.
     */
    if ((rlx_open_pos_[req->motor] == req->open_mpos) &&
        (rlx_close_pos_[req->motor] == req->close_mpos))
    {
      rsp->success = true;
    }
    else
    {
      rsp->success = false;
      rsp->err_message = "Failed to set the target grasp references.";
    }
  }
  else
  {
    rsp->success = false;
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::execute_cyl_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->execute_grasp('C', req->close_percent,
                                            req->spe_for_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::execute_pin_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->execute_grasp('P', req->close_percent,
                                            req->spe_for_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::execute_lat_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->execute_grasp('L', req->close_percent,
                                            req->spe_for_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::execute_pup_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->execute_grasp('U', req->close_percent,
                                            req->spe_for_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::execute_pdw_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->execute_grasp('D', req->close_percent,
                                            req->spe_for_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::execute_sph_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->execute_grasp('S', req->close_percent,
                                            req->spe_for_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::execute_tri_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->execute_grasp('T', req->close_percent,
                                            req->spe_for_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

void RosDriver::execute_rlx_srv_fun(
    const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
    std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  rsp->success = mia_hand_->execute_grasp('R', req->close_percent,
                                            req->spe_for_percent);

  if (!rsp->success)
  {
    rsp->err_message = mia_hand_->get_error_msg();
  }

  return;
}

rclcpp_action::GoalResponse RosDriver::cyl_act_goal_fun(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal)
{
  (void)uuid;  // Explicitly discarding unused goal ID, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(), 
      "Received new cylindrical grasp goal with %d%% target closure.", goal->target_closure_percent);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosDriver::cyl_act_canc_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  (void)goal_handle;  // Explicitly discarding unused goal handle, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(),
      "Received request to cancel cylindrical grasp goal.");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosDriver::cyl_act_accept_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  /* Creating new action.
   */
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action =
    std::make_shared<Action<mia_hand_msgs::action::Grasp>>(goal_handle);

  /* Adding new action to the queue.
   */
  {
    std::lock_guard<std::mutex> lock(actions_mtx_);
    grasp_actions_.emplace_back(action);
  }

  /* Spinning new background thread to execute the action.
   */
  std::thread{std::bind(
    &RosDriver::cyl_act_execute, this, std::placeholders::_1), 
    action}.detach();

  return;
}

void RosDriver::cyl_act_execute(
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action)
{
  RCLCPP_INFO(node_->get_logger(),
      "Executing cylindrical grasp goal...");

  const std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> 
    goal = action->goal_handle->get_goal();
  std::shared_ptr<mia_hand_msgs::action::Grasp::Feedback>
    feedback = std::make_shared<mia_hand_msgs::action::Grasp::Feedback>();

  /* Computing target motor positions from the target closure percentage.
   */
  int32_t target_mpos[3] = {0, 0, 0};

  target_mpos[0] = cyl_open_pos_[0] 
                  + ((cyl_close_pos_[0] - cyl_open_pos_[0]) * goal->target_closure_percent) / 100;
  target_mpos[1] = cyl_open_pos_[1] 
                  + ((cyl_close_pos_[1] - cyl_open_pos_[1]) * goal->target_closure_percent) / 100;
  target_mpos[2] = cyl_open_pos_[2] 
                  + ((cyl_close_pos_[2] - cyl_open_pos_[2]) * goal->target_closure_percent) / 100;

  rclcpp::Rate rate(2.0);  // Running action loop at 2 Hz.

  if (mia_hand_->execute_grasp('C', goal->target_closure_percent, goal->spe_for_percent))
  {
    while (action->goal_handle->is_active() && rclcpp::ok())
    {
      if (
        mia_hand_->get_motor_positions(
          feedback->current_mpos[0], feedback->current_mpos[1], feedback->current_mpos[2]) &&
        mia_hand_->get_motor_speeds(
          feedback->current_mspe[0], feedback->current_mspe[1], feedback->current_mspe[2]) && 
        mia_hand_->get_finger_forces(
          feedback->current_nfor[0], feedback->current_nfor[1], feedback->current_nfor[2], 
          feedback->current_tfor[0], feedback->current_tfor[1], feedback->current_tfor[2]))
      {
        if (!action->goal_handle->is_canceling())
        {
          if (
            (target_mpos[0] == feedback->current_mpos[0]) &&
            (target_mpos[1] == feedback->current_mpos[1]) &&
            (target_mpos[2] == feedback->current_mpos[2]))
          {
            RCLCPP_INFO(node_->get_logger(), "Cylindrical grasp goal succeded.");

            std::lock_guard<std::mutex> lock(actions_mtx_);

            action->result->mpos[0] = feedback->current_mpos[0];
            action->result->mpos[1] = feedback->current_mpos[1];
            action->result->mpos[2] = feedback->current_mpos[2];

            action->goal_handle->succeed(action->result);
          }

          action->goal_handle->publish_feedback(feedback);
        }
        else
        {
          mia_hand_->stop();

          RCLCPP_INFO(node_->get_logger(), 
              "Cylindrical grasp goal canceled.");

          std::lock_guard<std::mutex> lock(actions_mtx_);
          action->result->mpos[0] = feedback->current_mpos[0];
          action->result->mpos[1] = feedback->current_mpos[1];
          action->result->mpos[2] = feedback->current_mpos[2];
          action->goal_handle->canceled(action->result);
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), 
            "Cylindrical grasp action aborted.");

        std::lock_guard<std::mutex> lock(actions_mtx_);
        action->result->err_message = mia_hand_->get_error_msg();
        action->goal_handle->abort(action->result);
      }

      rate.sleep();
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Cylindrical grasp action aborted.");

    std::lock_guard<std::mutex> lock(actions_mtx_);
    action->result->err_message = mia_hand_->get_error_msg();
    action->goal_handle->abort(action->result);
  }

  /* Removing ended action from queue.
   */
  remove_by_goal_id(
    grasp_actions_, action->goal_handle->get_goal_id());

  return;
}

rclcpp_action::GoalResponse RosDriver::pin_act_goal_fun(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal)
{
  (void)uuid;  // Explicitly discarding unused goal ID, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(), 
      "Received new pinch grasp goal with %d%% target closure.", goal->target_closure_percent);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosDriver::pin_act_canc_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  (void)goal_handle;  // Explicitly discarding unused goal handle, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(),
      "Received request to cancel pinch grasp goal.");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosDriver::pin_act_accept_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  /* Creating new action.
   */
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action =
    std::make_shared<Action<mia_hand_msgs::action::Grasp>>(goal_handle);

  /* Adding new action to the queue.
   */
  {
    std::lock_guard<std::mutex> lock(actions_mtx_);
    grasp_actions_.emplace_back(action);
  }

  /* Spinning new background thread to execute the action.
   */
  std::thread{std::bind(
    &RosDriver::pin_act_execute, this, std::placeholders::_1), 
    action}.detach();

  return;
}

void RosDriver::pin_act_execute(
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action)
{
  RCLCPP_INFO(node_->get_logger(),
      "Executing pinch grasp goal...");

  const std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> 
    goal = action->goal_handle->get_goal();
  std::shared_ptr<mia_hand_msgs::action::Grasp::Feedback>
    feedback = std::make_shared<mia_hand_msgs::action::Grasp::Feedback>();

  /* Computing target motor positions from the target closure percentage.
   */
  int32_t target_mpos[3] = {0, 0, 0};

  target_mpos[0] = pin_open_pos_[0] 
                  + ((pin_close_pos_[0] - pin_open_pos_[0]) * goal->target_closure_percent) / 100;
  target_mpos[1] = pin_open_pos_[1] 
                  + ((pin_close_pos_[1] - pin_open_pos_[1]) * goal->target_closure_percent) / 100;
  target_mpos[2] = pin_open_pos_[2] 
                  + ((pin_close_pos_[2] - pin_open_pos_[2]) * goal->target_closure_percent) / 100;

  rclcpp::Rate rate(2.0);  // Running action loop at 2 Hz.

  if (mia_hand_->execute_grasp('P', goal->target_closure_percent, goal->spe_for_percent))
  {
    while (action->goal_handle->is_active() && rclcpp::ok())
    {
      if (
        mia_hand_->get_motor_positions(
          feedback->current_mpos[0], feedback->current_mpos[1], feedback->current_mpos[2]) &&
        mia_hand_->get_motor_speeds(
          feedback->current_mspe[0], feedback->current_mspe[1], feedback->current_mspe[2]) && 
        mia_hand_->get_finger_forces(
          feedback->current_nfor[0], feedback->current_nfor[1], feedback->current_nfor[2], 
          feedback->current_tfor[0], feedback->current_tfor[1], feedback->current_tfor[2]))
      {
        if (!action->goal_handle->is_canceling())
        {
          if (
            (target_mpos[0] == feedback->current_mpos[0]) &&
            (target_mpos[1] == feedback->current_mpos[1]) &&
            (target_mpos[2] == feedback->current_mpos[2]))
          {
            RCLCPP_INFO(node_->get_logger(), "Pinch grasp goal succeded.");

            std::lock_guard<std::mutex> lock(actions_mtx_);
            action->result->mpos[0] = feedback->current_mpos[0];
            action->result->mpos[1] = feedback->current_mpos[1];
            action->result->mpos[2] = feedback->current_mpos[2];

            action->goal_handle->succeed(action->result);
          }

          action->goal_handle->publish_feedback(feedback);
        }
        else
        {
          mia_hand_->stop();

          RCLCPP_INFO(node_->get_logger(), 
              "Pinch grasp goal canceled.");

          std::lock_guard<std::mutex> lock(actions_mtx_);
          action->result->mpos[0] = feedback->current_mpos[0];
          action->result->mpos[1] = feedback->current_mpos[1];
          action->result->mpos[2] = feedback->current_mpos[2];
          action->goal_handle->canceled(action->result);
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), 
            "Pinch grasp action aborted.");

        std::lock_guard<std::mutex> lock(actions_mtx_);
        action->result->err_message = mia_hand_->get_error_msg();
        action->goal_handle->abort(action->result);
      }

      rate.sleep();
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Pinch grasp action aborted.");

    std::lock_guard<std::mutex> lock(actions_mtx_);
    action->result->err_message = mia_hand_->get_error_msg();
    action->goal_handle->abort(action->result);
  }

  /* Removing ended action from queue.
   */
  remove_by_goal_id(
    grasp_actions_, action->goal_handle->get_goal_id());

  return;
}

rclcpp_action::GoalResponse RosDriver::lat_act_goal_fun(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal)
{
  (void)uuid;  // Explicitly discarding unused goal ID, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(), 
      "Received new lateral grasp goal with %d%% target closure.", goal->target_closure_percent);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosDriver::lat_act_canc_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  (void)goal_handle;  // Explicitly discarding unused goal handle, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(),
      "Received request to cancel lateral grasp goal.");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosDriver::lat_act_accept_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  /* Creating new action.
   */
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action =
    std::make_shared<Action<mia_hand_msgs::action::Grasp>>(goal_handle);

  /* Adding new action to the queue.
   */
  {
    std::lock_guard<std::mutex> lock(actions_mtx_);
    grasp_actions_.emplace_back(action);
  }

  /* Spinning new background thread to execute the action.
   */
  std::thread{std::bind(
    &RosDriver::lat_act_execute, this, std::placeholders::_1), 
    action}.detach();

  return;
}

void RosDriver::lat_act_execute(
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action)
{
  RCLCPP_INFO(node_->get_logger(),
      "Executing lateral grasp goal...");

  const std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> 
    goal = action->goal_handle->get_goal();
  std::shared_ptr<mia_hand_msgs::action::Grasp::Feedback>
    feedback = std::make_shared<mia_hand_msgs::action::Grasp::Feedback>();

  /* Computing target motor positions from the target closure percentage.
   */
  int32_t target_mpos[3] = {0, 0, 0};

  target_mpos[0] = lat_open_pos_[0] 
                  + ((lat_close_pos_[0] - lat_open_pos_[0]) * goal->target_closure_percent) / 100;
  target_mpos[1] = lat_open_pos_[1] 
                  + ((lat_close_pos_[1] - lat_open_pos_[1]) * goal->target_closure_percent) / 100;
  target_mpos[2] = lat_open_pos_[2] 
                  + ((lat_close_pos_[2] - lat_open_pos_[2]) * goal->target_closure_percent) / 100;

  rclcpp::Rate rate(2.0);  // Running action loop at 2 Hz.

  if (mia_hand_->execute_grasp('L', goal->target_closure_percent, goal->spe_for_percent))
  {
    while (action->goal_handle->is_active() && rclcpp::ok())
    {
      if (
        mia_hand_->get_motor_positions(
          feedback->current_mpos[0], feedback->current_mpos[1], feedback->current_mpos[2]) &&
        mia_hand_->get_motor_speeds(
          feedback->current_mspe[0], feedback->current_mspe[1], feedback->current_mspe[2]) && 
        mia_hand_->get_finger_forces(
          feedback->current_nfor[0], feedback->current_nfor[1], feedback->current_nfor[2], 
          feedback->current_tfor[0], feedback->current_tfor[1], feedback->current_tfor[2]))
      {
        if (!action->goal_handle->is_canceling())
        {
          if (
            (target_mpos[0] == feedback->current_mpos[0]) &&
            (target_mpos[1] == feedback->current_mpos[1]) &&
            (target_mpos[2] == feedback->current_mpos[2]))
          {
            RCLCPP_INFO(node_->get_logger(), "Lateral grasp goal succeded.");

            std::lock_guard<std::mutex> lock(actions_mtx_);
            action->result->mpos[0] = feedback->current_mpos[0];
            action->result->mpos[1] = feedback->current_mpos[1];
            action->result->mpos[2] = feedback->current_mpos[2];

            action->goal_handle->succeed(action->result);
          }

          action->goal_handle->publish_feedback(feedback);
        }
        else
        {
          mia_hand_->stop();

          RCLCPP_INFO(node_->get_logger(), 
              "Lateral grasp goal canceled.");

          std::lock_guard<std::mutex> lock(actions_mtx_);
          action->result->mpos[0] = feedback->current_mpos[0];
          action->result->mpos[1] = feedback->current_mpos[1];
          action->result->mpos[2] = feedback->current_mpos[2];
          action->goal_handle->canceled(action->result);
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), 
            "Lateral grasp action aborted.");

        std::lock_guard<std::mutex> lock(actions_mtx_);
        action->result->err_message = mia_hand_->get_error_msg();
        action->goal_handle->abort(action->result);
      }

      rate.sleep();
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Lateral grasp action aborted.");

    std::lock_guard<std::mutex> lock(actions_mtx_);
    action->result->err_message = mia_hand_->get_error_msg();
    action->goal_handle->abort(action->result);
  }

  /* Removing ended action from queue.
   */
  remove_by_goal_id(
    grasp_actions_, action->goal_handle->get_goal_id());

  return;
}

rclcpp_action::GoalResponse RosDriver::pup_act_goal_fun(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal)
{
  (void)uuid;  // Explicitly discarding unused goal ID, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(), 
      "Received new point-up grasp goal with %d%% target closure.", goal->target_closure_percent);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosDriver::pup_act_canc_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  (void)goal_handle;  // Explicitly discarding unused goal handle, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(),
      "Received request to cancel point-up grasp goal.");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosDriver::pup_act_accept_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  /* Creating new action.
   */
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action =
    std::make_shared<Action<mia_hand_msgs::action::Grasp>>(goal_handle);

  /* Adding new action to the queue.
   */
  {
    std::lock_guard<std::mutex> lock(actions_mtx_);
    grasp_actions_.emplace_back(action);
  }

  /* Spinning new background thread to execute the action.
   */
  std::thread{std::bind(
    &RosDriver::pup_act_execute, this, std::placeholders::_1), 
    action}.detach();

  return;
}

void RosDriver::pup_act_execute(
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action)
{
  RCLCPP_INFO(node_->get_logger(),
      "Executing point-up grasp goal...");

  const std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> 
    goal = action->goal_handle->get_goal();
  std::shared_ptr<mia_hand_msgs::action::Grasp::Feedback>
    feedback = std::make_shared<mia_hand_msgs::action::Grasp::Feedback>();

  /* Computing target motor positions from the target closure percentage.
   */
  int32_t target_mpos[3] = {0, 0, 0};

  target_mpos[0] = pup_open_pos_[0] 
                  + ((pup_close_pos_[0] - pup_open_pos_[0]) * goal->target_closure_percent) / 100;
  target_mpos[1] = pup_open_pos_[1] 
                  + ((pup_close_pos_[1] - pup_open_pos_[1]) * goal->target_closure_percent) / 100;
  target_mpos[2] = pup_open_pos_[2] 
                  + ((pup_close_pos_[2] - pup_open_pos_[2]) * goal->target_closure_percent) / 100;

  rclcpp::Rate rate(2.0);  // Running action loop at 2 Hz.

  if (mia_hand_->execute_grasp('U', goal->target_closure_percent, goal->spe_for_percent))
  {
    while (action->goal_handle->is_active() && rclcpp::ok())
    {
      if (
        mia_hand_->get_motor_positions(
          feedback->current_mpos[0], feedback->current_mpos[1], feedback->current_mpos[2]) &&
        mia_hand_->get_motor_speeds(
          feedback->current_mspe[0], feedback->current_mspe[1], feedback->current_mspe[2]) && 
        mia_hand_->get_finger_forces(
          feedback->current_nfor[0], feedback->current_nfor[1], feedback->current_nfor[2], 
          feedback->current_tfor[0], feedback->current_tfor[1], feedback->current_tfor[2]))
      {
        if (!action->goal_handle->is_canceling())
        {
          if (
            (target_mpos[0] == feedback->current_mpos[0]) &&
            (target_mpos[1] == feedback->current_mpos[1]) &&
            (target_mpos[2] == feedback->current_mpos[2]))
          {
            RCLCPP_INFO(node_->get_logger(), "Point-up grasp goal succeded.");

            std::lock_guard<std::mutex> lock(actions_mtx_);
            action->result->mpos[0] = feedback->current_mpos[0];
            action->result->mpos[1] = feedback->current_mpos[1];
            action->result->mpos[2] = feedback->current_mpos[2];

            action->goal_handle->succeed(action->result);
          }

          action->goal_handle->publish_feedback(feedback);
        }
        else
        {
          mia_hand_->stop();

          RCLCPP_INFO(node_->get_logger(), 
              "Point-up grasp goal canceled.");

          std::lock_guard<std::mutex> lock(actions_mtx_);
          action->result->mpos[0] = feedback->current_mpos[0];
          action->result->mpos[1] = feedback->current_mpos[1];
          action->result->mpos[2] = feedback->current_mpos[2];
          action->goal_handle->canceled(action->result);
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), 
            "Point-up grasp action aborted.");

        std::lock_guard<std::mutex> lock(actions_mtx_);
        action->result->err_message = mia_hand_->get_error_msg();
        action->goal_handle->abort(action->result);
      }

      rate.sleep();
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Point-up grasp action aborted.");

    std::lock_guard<std::mutex> lock(actions_mtx_);
    action->result->err_message = mia_hand_->get_error_msg();
    action->goal_handle->abort(action->result);
  }

  /* Removing ended action from queue.
   */
  remove_by_goal_id(
    grasp_actions_, action->goal_handle->get_goal_id());

  return;
}

rclcpp_action::GoalResponse RosDriver::pdw_act_goal_fun(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal)
{
  (void)uuid;  // Explicitly discarding unused goal ID, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(), 
      "Received new point-down grasp goal with %d%% target closure.", goal->target_closure_percent);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosDriver::pdw_act_canc_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  (void)goal_handle;  // Explicitly discarding unused goal handle, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(),
      "Received request to cancel point-down grasp goal.");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosDriver::pdw_act_accept_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  /* Creating new action.
   */
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action =
    std::make_shared<Action<mia_hand_msgs::action::Grasp>>(goal_handle);

  /* Adding new action to the queue.
   */
  {
    std::lock_guard<std::mutex> lock(actions_mtx_);
    grasp_actions_.emplace_back(action);
  }

  /* Spinning new background thread to execute the action.
   */
  std::thread{std::bind(
    &RosDriver::pdw_act_execute, this, std::placeholders::_1), 
    action}.detach();

  return;
}

void RosDriver::pdw_act_execute(
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action)
{
  RCLCPP_INFO(node_->get_logger(),
      "Executing point-down grasp goal...");

  const std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> 
    goal = action->goal_handle->get_goal();
  std::shared_ptr<mia_hand_msgs::action::Grasp::Feedback>
    feedback = std::make_shared<mia_hand_msgs::action::Grasp::Feedback>();

  /* Computing target motor positions from the target closure percentage.
   */
  int32_t target_mpos[3] = {0, 0, 0};

  target_mpos[0] = pdw_open_pos_[0] 
                  + ((pdw_close_pos_[0] - pdw_open_pos_[0]) * goal->target_closure_percent) / 100;
  target_mpos[1] = pdw_open_pos_[1] 
                  + ((pdw_close_pos_[1] - pdw_open_pos_[1]) * goal->target_closure_percent) / 100;
  target_mpos[2] = pdw_open_pos_[2] 
                  + ((pdw_close_pos_[2] - pdw_open_pos_[2]) * goal->target_closure_percent) / 100;

  rclcpp::Rate rate(2.0);  // Running action loop at 2 Hz.

  if (mia_hand_->execute_grasp('D', goal->target_closure_percent, goal->spe_for_percent))
  {
    while (action->goal_handle->is_active() && rclcpp::ok())
    {
      if (
        mia_hand_->get_motor_positions(
          feedback->current_mpos[0], feedback->current_mpos[1], feedback->current_mpos[2]) &&
        mia_hand_->get_motor_speeds(
          feedback->current_mspe[0], feedback->current_mspe[1], feedback->current_mspe[2]) && 
        mia_hand_->get_finger_forces(
          feedback->current_nfor[0], feedback->current_nfor[1], feedback->current_nfor[2], 
          feedback->current_tfor[0], feedback->current_tfor[1], feedback->current_tfor[2]))
      {
        if (!action->goal_handle->is_canceling())
        {
          if (
            (target_mpos[0] == feedback->current_mpos[0]) &&
            (target_mpos[1] == feedback->current_mpos[1]) &&
            (target_mpos[2] == feedback->current_mpos[2]))
          {
            RCLCPP_INFO(node_->get_logger(), "Point-down grasp goal succeded.");

            std::lock_guard<std::mutex> lock(actions_mtx_);
            action->result->mpos[0] = feedback->current_mpos[0];
            action->result->mpos[1] = feedback->current_mpos[1];
            action->result->mpos[2] = feedback->current_mpos[2];

            action->goal_handle->succeed(action->result);
          }

          action->goal_handle->publish_feedback(feedback);
        }
        else
        {
          mia_hand_->stop();

          RCLCPP_INFO(node_->get_logger(), 
              "Point-down grasp goal canceled.");

          std::lock_guard<std::mutex> lock(actions_mtx_);
          action->result->mpos[0] = feedback->current_mpos[0];
          action->result->mpos[1] = feedback->current_mpos[1];
          action->result->mpos[2] = feedback->current_mpos[2];
          action->goal_handle->canceled(action->result);
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), 
            "Point-down grasp action aborted.");

        std::lock_guard<std::mutex> lock(actions_mtx_);
        action->result->err_message = mia_hand_->get_error_msg();
        action->goal_handle->abort(action->result);
      }

      rate.sleep();
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Point-down grasp action aborted.");

    std::lock_guard<std::mutex> lock(actions_mtx_);
    action->result->err_message = mia_hand_->get_error_msg();
    action->goal_handle->abort(action->result);
  }

  /* Removing ended action from queue.
   */
  remove_by_goal_id(
    grasp_actions_, action->goal_handle->get_goal_id());

  return;
}

rclcpp_action::GoalResponse RosDriver::sph_act_goal_fun(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal)
{
  (void)uuid;  // Explicitly discarding unused goal ID, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(), 
      "Received new spherical grasp goal with %d%% target closure.", goal->target_closure_percent);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosDriver::sph_act_canc_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  (void)goal_handle;  // Explicitly discarding unused goal handle, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(),
      "Received request to cancel spherical grasp goal.");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosDriver::sph_act_accept_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  /* Creating new action.
   */
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action =
    std::make_shared<Action<mia_hand_msgs::action::Grasp>>(goal_handle);

  /* Adding new action to the queue.
   */
  {
    std::lock_guard<std::mutex> lock(actions_mtx_);
    grasp_actions_.emplace_back(action);
  }

  /* Spinning new background thread to execute the action.
   */
  std::thread{std::bind(
    &RosDriver::sph_act_execute, this, std::placeholders::_1), 
    action}.detach();

  return;
}

void RosDriver::sph_act_execute(
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action)
{
  RCLCPP_INFO(node_->get_logger(),
      "Executing spherical grasp goal...");

  const std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> 
    goal = action->goal_handle->get_goal();
  std::shared_ptr<mia_hand_msgs::action::Grasp::Feedback>
    feedback = std::make_shared<mia_hand_msgs::action::Grasp::Feedback>();

  /* Computing target motor positions from the target closure percentage.
   */
  int32_t target_mpos[3] = {0, 0, 0};

  target_mpos[0] = sph_open_pos_[0] 
                  + ((sph_close_pos_[0] - sph_open_pos_[0]) * goal->target_closure_percent) / 100;
  target_mpos[1] = sph_open_pos_[1] 
                  + ((sph_close_pos_[1] - sph_open_pos_[1]) * goal->target_closure_percent) / 100;
  target_mpos[2] = sph_open_pos_[2] 
                  + ((sph_close_pos_[2] - sph_open_pos_[2]) * goal->target_closure_percent) / 100;

  rclcpp::Rate rate(2.0);  // Running action loop at 2 Hz.

  if (mia_hand_->execute_grasp('S', goal->target_closure_percent, goal->spe_for_percent))
  {
    while (action->goal_handle->is_active() && rclcpp::ok())
    {
      if (
        mia_hand_->get_motor_positions(
          feedback->current_mpos[0], feedback->current_mpos[1], feedback->current_mpos[2]) &&
        mia_hand_->get_motor_speeds(
          feedback->current_mspe[0], feedback->current_mspe[1], feedback->current_mspe[2]) && 
        mia_hand_->get_finger_forces(
          feedback->current_nfor[0], feedback->current_nfor[1], feedback->current_nfor[2], 
          feedback->current_tfor[0], feedback->current_tfor[1], feedback->current_tfor[2]))
      {
        if (!action->goal_handle->is_canceling())
        {
          if (
            (target_mpos[0] == feedback->current_mpos[0]) &&
            (target_mpos[1] == feedback->current_mpos[1]) &&
            (target_mpos[2] == feedback->current_mpos[2]))
          {
            RCLCPP_INFO(node_->get_logger(), "Spherical grasp goal succeded.");

            std::lock_guard<std::mutex> lock(actions_mtx_);
            action->result->mpos[0] = feedback->current_mpos[0];
            action->result->mpos[1] = feedback->current_mpos[1];
            action->result->mpos[2] = feedback->current_mpos[2];

            action->goal_handle->succeed(action->result);
          }

          action->goal_handle->publish_feedback(feedback);
        }
        else
        {
          mia_hand_->stop();

          RCLCPP_INFO(node_->get_logger(), 
              "Spherical grasp goal canceled.");

          std::lock_guard<std::mutex> lock(actions_mtx_);
          action->result->mpos[0] = feedback->current_mpos[0];
          action->result->mpos[1] = feedback->current_mpos[1];
          action->result->mpos[2] = feedback->current_mpos[2];
          action->goal_handle->canceled(action->result);
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), 
            "Spherical grasp action aborted.");

        std::lock_guard<std::mutex> lock(actions_mtx_);
        action->result->err_message = mia_hand_->get_error_msg();
        action->goal_handle->abort(action->result);
      }

      rate.sleep();
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Spherical grasp action aborted.");

    std::lock_guard<std::mutex> lock(actions_mtx_);
    action->result->err_message = mia_hand_->get_error_msg();
    action->goal_handle->abort(action->result);
  }

  /* Removing ended action from queue.
   */
  remove_by_goal_id(
    grasp_actions_, action->goal_handle->get_goal_id());

  return;
}

rclcpp_action::GoalResponse RosDriver::tri_act_goal_fun(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal)
{
  (void)uuid;  // Explicitly discarding unused goal ID, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(), 
      "Received new tri-digital grasp goal with %d%% target closure.", goal->target_closure_percent);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosDriver::tri_act_canc_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  (void)goal_handle;  // Explicitly discarding unused goal handle, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(),
      "Received request to cancel tri-digital grasp goal.");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosDriver::tri_act_accept_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  /* Creating new action.
   */
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action =
    std::make_shared<Action<mia_hand_msgs::action::Grasp>>(goal_handle);

  /* Adding new action to the queue.
   */
  {
    std::lock_guard<std::mutex> lock(actions_mtx_);
    grasp_actions_.emplace_back(action);
  }

  /* Spinning new background thread to execute the action.
   */
  std::thread{std::bind(
    &RosDriver::tri_act_execute, this, std::placeholders::_1), 
    action}.detach();

  return;
}

void RosDriver::tri_act_execute(
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action)
{
  RCLCPP_INFO(node_->get_logger(),
      "Executing tri-digital grasp goal...");

  const std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> 
    goal = action->goal_handle->get_goal();
  std::shared_ptr<mia_hand_msgs::action::Grasp::Feedback>
    feedback = std::make_shared<mia_hand_msgs::action::Grasp::Feedback>();

  /* Computing target motor positions from the target closure percentage.
   */
  int32_t target_mpos[3] = {0, 0, 0};

  target_mpos[0] = tri_open_pos_[0] 
                  + ((tri_close_pos_[0] - tri_open_pos_[0]) * goal->target_closure_percent) / 100;
  target_mpos[1] = tri_open_pos_[1] 
                  + ((tri_close_pos_[1] - tri_open_pos_[1]) * goal->target_closure_percent) / 100;
  target_mpos[2] = tri_open_pos_[2] 
                  + ((tri_close_pos_[2] - tri_open_pos_[2]) * goal->target_closure_percent) / 100;

  rclcpp::Rate rate(2.0);  // Running action loop at 2 Hz.

  if (mia_hand_->execute_grasp('T', goal->target_closure_percent, goal->spe_for_percent))
  {
    while (action->goal_handle->is_active() && rclcpp::ok())
    {
      if (
        mia_hand_->get_motor_positions(
          feedback->current_mpos[0], feedback->current_mpos[1], feedback->current_mpos[2]) &&
        mia_hand_->get_motor_speeds(
          feedback->current_mspe[0], feedback->current_mspe[1], feedback->current_mspe[2]) && 
        mia_hand_->get_finger_forces(
          feedback->current_nfor[0], feedback->current_nfor[1], feedback->current_nfor[2], 
          feedback->current_tfor[0], feedback->current_tfor[1], feedback->current_tfor[2]))
      {
        if (!action->goal_handle->is_canceling())
        {
          if (
            (target_mpos[0] == feedback->current_mpos[0]) &&
            (target_mpos[1] == feedback->current_mpos[1]) &&
            (target_mpos[2] == feedback->current_mpos[2]))
          {
            RCLCPP_INFO(node_->get_logger(), "Tri-digital grasp goal succeded.");

            std::lock_guard<std::mutex> lock(actions_mtx_);
            action->result->mpos[0] = feedback->current_mpos[0];
            action->result->mpos[1] = feedback->current_mpos[1];
            action->result->mpos[2] = feedback->current_mpos[2];

            action->goal_handle->succeed(action->result);
          }

          action->goal_handle->publish_feedback(feedback);
        }
        else
        {
          mia_hand_->stop();

          RCLCPP_INFO(node_->get_logger(), 
              "Tri-digital grasp goal canceled.");

          std::lock_guard<std::mutex> lock(actions_mtx_);
          action->result->mpos[0] = feedback->current_mpos[0];
          action->result->mpos[1] = feedback->current_mpos[1];
          action->result->mpos[2] = feedback->current_mpos[2];
          action->goal_handle->canceled(action->result);
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), 
            "Tri-digital grasp action aborted.");

        std::lock_guard<std::mutex> lock(actions_mtx_);
        action->result->err_message = mia_hand_->get_error_msg();
        action->goal_handle->abort(action->result);
      }

      rate.sleep();
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Tri-digital grasp action aborted.");

    std::lock_guard<std::mutex> lock(actions_mtx_);
    action->result->err_message = mia_hand_->get_error_msg();
    action->goal_handle->abort(action->result);
  }

  /* Removing ended action from queue.
   */
  remove_by_goal_id(
    grasp_actions_, action->goal_handle->get_goal_id());

  return;
}

rclcpp_action::GoalResponse RosDriver::rlx_act_goal_fun(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal)
{
  (void)uuid;  // Explicitly discarding unused goal ID, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(), 
      "Received new relax grasp goal with %d%% target closure.", goal->target_closure_percent);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RosDriver::rlx_act_canc_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  (void)goal_handle;  // Explicitly discarding unused goal handle, to suppress compiler warning.

  RCLCPP_INFO(node_->get_logger(),
      "Received request to cancel relax grasp goal.");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RosDriver::rlx_act_accept_fun(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle)
{
  /* Canceling any previous trajectory or grasp goal.
   */
  abort_active_goals(thumb_mot_traj_actions_);
  abort_active_goals(thumb_jnt_traj_actions_);
  abort_active_goals(index_mot_traj_actions_);
  abort_active_goals(index_jnt_traj_actions_);
  abort_active_goals(mrl_mot_traj_actions_);
  abort_active_goals(mrl_jnt_traj_actions_);
  abort_active_goals(grasp_actions_);

  /* Creating new action.
   */
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action =
    std::make_shared<Action<mia_hand_msgs::action::Grasp>>(goal_handle);

  /* Adding new action to the queue.
   */
  {
    std::lock_guard<std::mutex> lock(actions_mtx_);
    grasp_actions_.emplace_back(action);
  }

  /* Spinning new background thread to execute the action.
   */
  std::thread{std::bind(
    &RosDriver::rlx_act_execute, this, std::placeholders::_1), 
    action}.detach();

  return;
}

void RosDriver::rlx_act_execute(
  std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action)
{
  RCLCPP_INFO(node_->get_logger(),
      "Executing relax grasp goal...");

  const std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> 
    goal = action->goal_handle->get_goal();
  std::shared_ptr<mia_hand_msgs::action::Grasp::Feedback>
    feedback = std::make_shared<mia_hand_msgs::action::Grasp::Feedback>();

  /* Computing target motor positions from the target closure percentage.
   */
  int32_t target_mpos[3] = {0, 0, 0};

  target_mpos[0] = rlx_open_pos_[0] 
                  + ((rlx_close_pos_[0] - rlx_open_pos_[0]) * goal->target_closure_percent) / 100;
  target_mpos[1] = rlx_open_pos_[1] 
                  + ((rlx_close_pos_[1] - rlx_open_pos_[1]) * goal->target_closure_percent) / 100;
  target_mpos[2] = rlx_open_pos_[2] 
                  + ((rlx_close_pos_[2] - rlx_open_pos_[2]) * goal->target_closure_percent) / 100;

  rclcpp::Rate rate(2.0);  // Running action loop at 2 Hz.

  if (mia_hand_->execute_grasp('R', goal->target_closure_percent, goal->spe_for_percent))
  {
    while (action->goal_handle->is_active() && rclcpp::ok())
    {
      if (
        mia_hand_->get_motor_positions(
          feedback->current_mpos[0], feedback->current_mpos[1], feedback->current_mpos[2]) &&
        mia_hand_->get_motor_speeds(
          feedback->current_mspe[0], feedback->current_mspe[1], feedback->current_mspe[2]) && 
        mia_hand_->get_finger_forces(
          feedback->current_nfor[0], feedback->current_nfor[1], feedback->current_nfor[2], 
          feedback->current_tfor[0], feedback->current_tfor[1], feedback->current_tfor[2]))
      {
        if (!action->goal_handle->is_canceling())
        {
          if (
            (target_mpos[0] == feedback->current_mpos[0]) &&
            (target_mpos[1] == feedback->current_mpos[1]) &&
            (target_mpos[2] == feedback->current_mpos[2]))
          {
            RCLCPP_INFO(node_->get_logger(), "Relax grasp goal succeded.");

            std::lock_guard<std::mutex> lock(actions_mtx_);
            action->result->mpos[0] = feedback->current_mpos[0];
            action->result->mpos[1] = feedback->current_mpos[1];
            action->result->mpos[2] = feedback->current_mpos[2];

            action->goal_handle->succeed(action->result);
          }

          action->goal_handle->publish_feedback(feedback);
        }
        else
        {
          mia_hand_->stop();

          RCLCPP_INFO(node_->get_logger(), 
              "Relax grasp goal canceled.");

          std::lock_guard<std::mutex> lock(actions_mtx_);
          action->result->mpos[0] = feedback->current_mpos[0];
          action->result->mpos[1] = feedback->current_mpos[1];
          action->result->mpos[2] = feedback->current_mpos[2];
          action->goal_handle->canceled(action->result);
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), 
            "Relax grasp action aborted.");

        std::lock_guard<std::mutex> lock(actions_mtx_);
        action->result->err_message = mia_hand_->get_error_msg();
        action->goal_handle->abort(action->result);
      }

      rate.sleep();
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Relax grasp action aborted.");

    std::lock_guard<std::mutex> lock(actions_mtx_);
    action->result->err_message = mia_hand_->get_error_msg();
    action->goal_handle->abort(action->result);
  }

  /* Removing ended action from queue.
   */
  remove_by_goal_id(
    grasp_actions_, action->goal_handle->get_goal_id());

  return;
}

void RosDriver::stream_tmr_fun()
{
  if (mot_pos_stream_on_)
  {
    if (mia_hand_->get_motor_positions(
          mot_data_msg_.thumb_data, mot_data_msg_.index_data,
          mot_data_msg_.mrl_data))
    {
      mot_pos_pub_->publish(mot_data_msg_);
    }
  }

  if (mot_spe_stream_on_)
  {
    if (mia_hand_->get_motor_speeds(
          mot_data_msg_.thumb_data, mot_data_msg_.index_data,
          mot_data_msg_.mrl_data))
    {
      mot_spe_pub_->publish(mot_data_msg_);
    }
  }

  if (mot_cur_stream_on_)
  {
    if (mia_hand_->get_motor_currents(
          mot_data_msg_.thumb_data, mot_data_msg_.index_data,
          mot_data_msg_.mrl_data))
    {
      mot_cur_pub_->publish(mot_data_msg_);
    }
  }

  if (jnt_pos_stream_on_)
  {
    if (mia_hand_->get_joint_positions(
          jnt_data_msg_.thumb_data, jnt_data_msg_.index_data,
          jnt_data_msg_.mrl_data))
    {
      jnt_pos_pub_->publish(jnt_data_msg_);
    }
  }

  if (jnt_spe_stream_on_)
  {
    if (mia_hand_->get_joint_speeds(
          jnt_data_msg_.thumb_data, jnt_data_msg_.index_data,
          jnt_data_msg_.mrl_data))
    {
      jnt_spe_pub_->publish(jnt_data_msg_);
    }
  }

  if (fin_for_stream_on_)
  {
    if (mia_hand_->get_finger_forces(
          for_data_msg_.thumb_nfor, for_data_msg_.index_nfor, for_data_msg_.mrl_nfor, 
          for_data_msg_.thumb_tfor, for_data_msg_.index_tfor, for_data_msg_.mrl_tfor))
    {
      fin_for_pub_->publish(for_data_msg_);
    }
  }

  return;
}

void RosDriver::switch_mot_pos_stream_srv_fun(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> rsp)
{
  if (true == req->data)
  {
    if (!mot_pos_stream_on_)
    {
      ++n_streams_;
      update_stream_period();
      mot_pos_stream_on_ = true;

      rsp->success = true;
      rsp->message = "Motor positions streaming switched on.";
    }
    else
    {
      rsp->success = false;
      rsp->message = "Motor positions streaming already on.";
    }
  }
  else
  {
    if (mot_pos_stream_on_)
    {
      --n_streams_;
      update_stream_period();
      mot_pos_stream_on_ = false;

      rsp->success = true;
      rsp->message = "Motor positions streaming switched off.";
    }
    else
    {
      rsp->success = false;
      rsp->message = "Motor positions streaming already off.";
    }
  }

  return;
}

void RosDriver::switch_mot_spe_stream_srv_fun(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> rsp)
{
  if (true == req->data)
  {
    if (!mot_spe_stream_on_)
    {
      ++n_streams_;
      update_stream_period();
      mot_spe_stream_on_ = true;

      rsp->success = true;
      rsp->message = "Motor speeds streaming switched on.";
    }
    else
    {
      rsp->success = false;
      rsp->message = "Motor speeds streaming already on.";
    }
  }
  else
  {
    if (mot_spe_stream_on_)
    {
      --n_streams_;
      update_stream_period();
      mot_spe_stream_on_ = false;

      rsp->success = true;
      rsp->message = "Motor speeds streaming switched off.";
    }
    else
    {
      rsp->success = false;
      rsp->message = "Motor speeds streaming already off.";
    }
  }

  return;
}

void RosDriver::switch_mot_cur_stream_srv_fun(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> rsp)
{
  if (true == req->data)
  {
    if (!mot_cur_stream_on_)
    {
      ++n_streams_;
      update_stream_period();
      mot_cur_stream_on_ = true;

      rsp->success = true;
      rsp->message = "Motor currents streaming switched on.";
    }
    else
    {
      rsp->success = false;
      rsp->message = "Motor currents streaming already on.";
    }
  }
  else
  {
    if (mot_cur_stream_on_)
    {
      --n_streams_;
      update_stream_period();
      mot_cur_stream_on_ = false;

      rsp->success = true;
      rsp->message = "Motor currents streaming switched off.";
    }
    else
    {
      rsp->success = false;
      rsp->message = "Motor currents streaming already off.";
    }
  }

  return;
}

void RosDriver::switch_jnt_pos_stream_srv_fun(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> rsp)
{
  if (true == req->data)
  {
    if (!jnt_pos_stream_on_)
    {
      ++n_streams_;
      update_stream_period();
      jnt_pos_stream_on_ = true;

      rsp->success = true;
      rsp->message = "Joint positions streaming switched on.";
    }
    else
    {
      rsp->success = false;
      rsp->message = "Joint positions streaming already on.";
    }
  }
  else
  {
    if (jnt_pos_stream_on_)
    {
      --n_streams_;
      update_stream_period();
      jnt_pos_stream_on_ = false;

      rsp->success = true;
      rsp->message = "Joint positions streaming switched off.";
    }
    else
    {
      rsp->success = false;
      rsp->message = "Joint positions streaming already off.";
    }
  }

  return;
}

void RosDriver::switch_jnt_spe_stream_srv_fun(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> rsp)
{
  if (true == req->data)
  {
    if (!jnt_spe_stream_on_)
    {
      ++n_streams_;
      update_stream_period();
      jnt_spe_stream_on_ = true;

      rsp->success = true;
      rsp->message = "Joint speeds streaming switched on.";
    }
    else
    {
      rsp->success = false;
      rsp->message = "Joint speeds streaming already on.";
    }
  }
  else
  {
    if (jnt_spe_stream_on_)
    {
      --n_streams_;
      update_stream_period();
      jnt_spe_stream_on_ = false;

      rsp->success = true;
      rsp->message = "Joint speeds streaming switched off.";
    }
    else
    {
      rsp->success = false;
      rsp->message = "Joint speeds streaming already off.";
    }
  }

  return;
}

void RosDriver::switch_fin_for_stream_srv_fun(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> rsp)
{
  if (true == req->data)
  {
    if (!fin_for_stream_on_)
    {
      ++n_streams_;
      update_stream_period();
      fin_for_stream_on_ = true;

      rsp->success = true;
      rsp->message = "Finger forces streaming switched on.";
    }
    else
    {
      rsp->success = false;
      rsp->message = "Finger forces streaming already on.";
    }
  }
  else
  {
    if (fin_for_stream_on_)
    {
      --n_streams_;
      update_stream_period();
      fin_for_stream_on_ = false;

      rsp->success = true;
      rsp->message = "Finger forces streaming switched off.";
    }
    else
    {
      rsp->success = false;
      rsp->message = "Finger forces streaming already off.";
    }
  }

  return;
}

void RosDriver::abort_active_goals(
  const std::vector<std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>>>& actions)
{
  std::lock_guard<std::mutex> lock(actions_mtx_);

  if (!actions.empty())
  {
    std::for_each(actions.begin(), actions.end(),
      [](const std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>>& action)
      {
        if (action->goal_handle->is_active())
        {
          action->goal_handle->abort(action->result);
        }

        return;
      });
  }

  return;
}

void RosDriver::abort_active_goals(
  const std::vector<std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>>>& actions)
{
  std::lock_guard<std::mutex> lock(actions_mtx_);

  if (!actions.empty())
  {
    std::for_each(actions.begin(), actions.end(),
      [](const std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>>& action)
      {
        if (action->goal_handle->is_active())
        {
          action->goal_handle->abort(action->result);
        }

        return;
      });
  }

  return;
}

void RosDriver::abort_active_goals(
  const std::vector<std::shared_ptr<Action<mia_hand_msgs::action::Grasp>>>& actions)
{
  std::lock_guard<std::mutex> lock(actions_mtx_);

  if (!actions.empty())
  {
    std::for_each(actions.begin(), actions.end(),
      [](const std::shared_ptr<Action<mia_hand_msgs::action::Grasp>>& action)
      {
        if (action->goal_handle->is_active())
        {
          action->goal_handle->abort(action->result);
        }

        return;
      });
  }

  return;
}

void RosDriver::remove_by_goal_id(
  std::vector<std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>>>& actions, 
  const rclcpp_action::GoalUUID& uuid)
{
  std::lock_guard<std::mutex> lock(actions_mtx_);

  std::vector<std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>>>::const_iterator
    actions_it = std::find_if(
      actions.begin(), actions.end(),
      [&uuid](const std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>>& action)
      {
        return uuid == action->goal_handle->get_goal_id();
      });

  if (actions_it != actions.end())
  {
    actions.erase(actions_it);
  }

  return;
}

void RosDriver::remove_by_goal_id(
  std::vector<std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>>>& actions, 
  const rclcpp_action::GoalUUID& uuid)
{
  std::lock_guard<std::mutex> lock(actions_mtx_);

  std::vector<std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>>>::const_iterator
    actions_it = std::find_if(
      actions.begin(), actions.end(),
      [&uuid](const std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>>& action)
      {
        return uuid == action->goal_handle->get_goal_id();
      });

  if (actions_it != actions.end())
  {
    actions.erase(actions_it);
  }

  return;
}

void RosDriver::remove_by_goal_id(
  std::vector<std::shared_ptr<Action<mia_hand_msgs::action::Grasp>>>& actions, 
  const rclcpp_action::GoalUUID& uuid)
{
  std::lock_guard<std::mutex> lock(actions_mtx_);

  std::vector<std::shared_ptr<Action<mia_hand_msgs::action::Grasp>>>::const_iterator
    actions_it = std::find_if(
      actions.begin(), actions.end(),
      [&uuid](const std::shared_ptr<Action<mia_hand_msgs::action::Grasp>>& action)
      {
        return uuid == action->goal_handle->get_goal_id();
      });

  if (actions_it != actions.end())
  {
    actions.erase(actions_it);
  }

  return;
}
}  // namespace
