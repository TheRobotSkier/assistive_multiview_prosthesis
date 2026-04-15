#ifndef MIA_HAND_DRIVER_ROS_DRIVER_HPP
#define MIA_HAND_DRIVER_ROS_DRIVER_HPP

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "mia_hand_msgs/msg/motor_data.hpp"
#include "mia_hand_msgs/msg/joint_data.hpp"
#include "mia_hand_msgs/msg/force_data.hpp"

#include "mia_hand_msgs/srv/connect.hpp"
#include "mia_hand_msgs/srv/execute_grasp.hpp"
#include "mia_hand_msgs/srv/get_grasp_references.hpp"
#include "mia_hand_msgs/srv/get_joint_data.hpp"
#include "mia_hand_msgs/srv/get_motor_data.hpp"
#include "mia_hand_msgs/srv/get_force_data.hpp"
#include "mia_hand_msgs/srv/set_grasp_references.hpp"
#include "mia_hand_msgs/srv/set_joint_speed.hpp"
#include "mia_hand_msgs/srv/set_joint_traj.hpp"
#include "mia_hand_msgs/srv/set_motor_speed.hpp"
#include "mia_hand_msgs/srv/set_motor_traj.hpp"

#include "mia_hand_msgs/action/motor_traj.hpp"
#include "mia_hand_msgs/action/joint_traj.hpp"
#include "mia_hand_msgs/action/grasp.hpp"

#include "mia_hand_driver/cpp_driver.hpp"

namespace mia_hand_driver
{
/**
 * \brief Class implementing a ROS2 interface for Mia Hand devices, wrapping the
 *        #CppDriver one.
 */
class RosDriver
{
public:

  /**
   * \brief Factory function for creating a new CppDriver object.
   *
   * A factory function is used for avoiding to throw exceptions in the class
   * constructor, in case of errors during object initialization.
   *
   * @param node Shared pointer to Mia Hand node.
   * @return \code std::unique_ptr \endcode to the new RosDriver object.
   */
  static std::unique_ptr<RosDriver> create(std::shared_ptr<rclcpp::Node>& node);

  /* Preventing RosDriver objects to be copyable, either through the copy
   * constructor and the assignment operator, to ensure the unique access to
   * them through the std::unique_ptr.
   */
  RosDriver(RosDriver&) = delete;
  RosDriver& operator=(RosDriver&) = delete;

private:

  /**
   * \brief Class constructor.
   *
   * The class constructor is private to allow instantiations of new RosDriver
   * objects only through #create() factory function.
   */
  RosDriver();

  /**
   * \brief Function for initializing a RosDriver object.
   *
   * This function is necessary for accessing all the non-static members through
   * the static factory function #create().
   *
   * @param node Shared pointer to Mia Hand node. 
   * @return true, if object initialization was successful, false if not. 
   */ 
  bool init(std::shared_ptr<rclcpp::Node>& node);

  /**
   * \brief Class used for keeping trace of the actions under execution. 
   */
  template<typename ActionMsg>
  struct Action
  {
    Action(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionMsg>> goal_hnd);

    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionMsg>> goal_handle;
    std::shared_ptr<typename ActionMsg::Result> result;
  };

  /**
   * \brief Function for initializing the ROS topics.
   */
  void init_topics();

  /**
   * \brief Function for initializing the ROS services.
   */
  void init_srvs();

  /**
   * \brief Function for initializing the ROS actions.
   */
  void init_actions();

  /**
   * \brief Function for updating the streaming period.
   *
   * This function is issued every time a data stream is switched on or off, to
   * adjust the #stream_tmr_ period, depending on the residual number od active
   * streamings.
   */
  void update_stream_period();

  /**
   * \brief Function for reading all the grasp OPEN and CLOSE motor positions
   *        from Mia Hand.
   *
   * @return \code true \endcode, if grasp references reading was successful, 
   *         \code false \endcode if not.
   */
  bool get_grasps_refs();

  /**
   * \brief Callback function of #connection_tmr_ timer.
   *
   * Here, connection with Mia Hand is checked, through the appropriate
   * CppDriver method.
   */
  void connection_tmr_fun();

  /**
   * \brief Callback function of #connect_srv_ service.
   */
  void connect_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::Connect::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::Connect::Response> rsp);

  /**
   * \brief Callback function of #disconnect_srv_ service.
   */
  void disconnect_srv_fun(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
      std::shared_ptr<std_srvs::srv::Trigger::Response> rsp);

  /**
   * \brief Callback function of #stop_srv_ service.
   */
  void stop_srv_fun(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
      std::shared_ptr<std_srvs::srv::Trigger::Response> rsp);

  /**
   * \brief Callback function of #play_srv_ service.
   */
  void play_srv_fun(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
      std::shared_ptr<std_srvs::srv::Trigger::Response> rsp);

  /**
   * \brief Callback function of #calibrate_mot_pos_srv_ service.
   */
  void calibrate_mot_pos_srv_fun(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
      std::shared_ptr<std_srvs::srv::Trigger::Response> rsp);

  /**
   * \brief Callback function of #store_current_sets_srv_ service.
   */
  void store_current_sets_srv_fun(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
      std::shared_ptr<std_srvs::srv::Trigger::Response> rsp);

  /**
   * \brief Callback function of #get_firmware_srv_ service.
   */
  void get_fw_version_srv_fun(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
      std::shared_ptr<std_srvs::srv::Trigger::Response> rsp);

  /**
   * \brief Callback function of #get_mot_pos_srv_ service.
   */
  void get_mot_pos_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::GetMotorData::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::GetMotorData::Response> rsp);

  /**
   * \brief Callback function of #get_mot_spe_srv_ service.
   */
  void get_mot_spe_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::GetMotorData::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::GetMotorData::Response> rsp);

  /**
   * \brief Callback function of #get_mot_cur_srv_ service.
   */
  void get_mot_cur_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::GetMotorData::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::GetMotorData::Response> rsp);

  /**
   * \brief Callback function of #get_fin_for_srv_ service.
   */
  void get_fin_for_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::GetForceData::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::GetForceData::Response> rsp);

  /**
   * \brief Callback function of #get_jnt_pos_srv_ service.
   */
  void get_jnt_pos_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::GetJointData::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::GetJointData::Response> rsp);

  /**
   * \brief Callback function of #get_jnt_spe_srv_ service.
   */
  void get_jnt_spe_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::GetJointData::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::GetJointData::Response> rsp);

  /**
   * \brief Callback function of #set_thumb_mot_spe_srv_ service.
   */
  void set_thumb_mot_spe_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetMotorSpeed::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::SetMotorSpeed::Response> rsp);

  /**
   * \brief Callback function of #set_index_mot_spe_srv_ service.
   */
  void set_index_mot_spe_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetMotorSpeed::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::SetMotorSpeed::Response> rsp);

  /**
   * \brief Callback function of #set_mrl_mot_spe_srv_ service.
   */
  void set_mrl_mot_spe_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetMotorSpeed::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::SetMotorSpeed::Response> rsp);

  /**
   * \brief Callback function of #set_thumb_mot_traj_srv_ service.
   */
  void set_thumb_mot_traj_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetMotorTraj::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::SetMotorTraj::Response> rsp);

  /**
   * \brief Callback function of #set_index_mot_traj_srv_ service.
   */
  void set_index_mot_traj_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetMotorTraj::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::SetMotorTraj::Response> rsp);

  /**
   * \brief Callback function of #set_mrl_mot_traj_srv_ service.
   */
  void set_mrl_mot_traj_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetMotorTraj::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::SetMotorTraj::Response> rsp);

  /**
   * \brief Callback function of #set_thumb_jnt_spe_srv_ service.
   */
  void set_thumb_jnt_spe_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetJointSpeed::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::SetJointSpeed::Response> rsp);

  /**
   * \brief Callback function of #set_index_jnt_spe_srv_ service.
   */
  void set_index_jnt_spe_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetJointSpeed::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::SetJointSpeed::Response> rsp);

  /**
   * \brief Callback function of #set_mrl_jnt_spe_srv_ service.
   */
  void set_mrl_jnt_spe_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetJointSpeed::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::SetJointSpeed::Response> rsp);

  /**
   * \brief Callback function of #set_thumb_jnt_traj_srv_ service.
   */
  void set_thumb_jnt_traj_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetJointTraj::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::SetJointTraj::Response> rsp);

  /**
   * \brief Callback function of #set_index_jnt_traj_srv_ service.
   */
  void set_index_jnt_traj_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetJointTraj::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::SetJointTraj::Response> rsp);

  /**
   * \brief Callback function of #set_mrl_jnt_traj_srv_ service.
   */
  void set_mrl_jnt_traj_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetJointTraj::Request> req, 
      std::shared_ptr<mia_hand_msgs::srv::SetJointTraj::Response> rsp);

  /**
   * \brief Callback function for a #thumb_mot_traj_act_ goal request.
   */
  rclcpp_action::GoalResponse thumb_mot_traj_act_goal_fun(
      const rclcpp_action::GoalUUID& uuid, 
      std::shared_ptr<const mia_hand_msgs::action::MotorTraj::Goal> goal);

  /**
   * \brief Callback function for a #thumb_mot_traj_act_ cancel request.
   */
  rclcpp_action::CancelResponse thumb_mot_traj_act_canc_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::MotorTraj>> goal_handle);

  /**
   * \brief Callback function for accepting a #thumb_mot_traj_act_ goal.
   */
  void thumb_mot_traj_act_accept_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::MotorTraj>> goal_handle);

  /**
   * \brief Function for executing #thumb_mot_traj_act_ action.
   */
  void thumb_mot_traj_act_execute(
    std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>> action);

  /**
   * \brief Callback function for a #index_mot_traj_act_ goal request.
   */
  rclcpp_action::GoalResponse index_mot_traj_act_goal_fun(
      const rclcpp_action::GoalUUID& uuid, 
      std::shared_ptr<const mia_hand_msgs::action::MotorTraj::Goal> goal);

  /**
   * \brief Callback function for a #index_mot_traj_act_ cancel request.
   */
  rclcpp_action::CancelResponse index_mot_traj_act_canc_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::MotorTraj>> goal_handle);

  /**
   * \brief Callback function for accepting a #index_mot_traj_act_ goal.
   */
  void index_mot_traj_act_accept_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::MotorTraj>> goal_handle);

  /**
   * \brief Function for executing #index_mot_traj_act_ action.
   */
  void index_mot_traj_act_execute(
    std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>> action);

  /**
   * \brief Callback function for a #mrl_mot_traj_act_ goal request.
   */
  rclcpp_action::GoalResponse mrl_mot_traj_act_goal_fun(
      const rclcpp_action::GoalUUID& uuid, 
      std::shared_ptr<const mia_hand_msgs::action::MotorTraj::Goal> goal);

  /**
   * \brief Callback function for a #mrl_mot_traj_act_ cancel request.
   */
  rclcpp_action::CancelResponse mrl_mot_traj_act_canc_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::MotorTraj>> goal_handle);

  /**
   * \brief Callback function for accepting a #mrl_mot_traj_act_ goal.
   */
  void mrl_mot_traj_act_accept_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::MotorTraj>> goal_handle);

  /**
   * \brief Function for executing #mrl_mot_traj_act_ action.
   */
  void mrl_mot_traj_act_execute(
    std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>> action);

  /**
   * \brief Callback function for a #thumb_jnt_traj_act_ goal request.
   */
  rclcpp_action::GoalResponse thumb_jnt_traj_act_goal_fun(
      const rclcpp_action::GoalUUID& uuid, 
      std::shared_ptr<const mia_hand_msgs::action::JointTraj::Goal> goal);

  /**
   * \brief Callback function for a #thumb_jnt_traj_act_ cancel request.
   */
  rclcpp_action::CancelResponse thumb_jnt_traj_act_canc_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::JointTraj>> goal_handle);

  /**
   * \brief Callback function for accepting a #thumb_jnt_traj_act_ goal.
   */
  void thumb_jnt_traj_act_accept_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::JointTraj>> goal_handle);

  /**
   * \brief Function for executing #thumb_jnt_traj_act_ action.
   */
  void thumb_jnt_traj_act_execute(
    std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>> action);

  /**
   * \brief Callback function for a #index_jnt_traj_act_ goal request.
   */
  rclcpp_action::GoalResponse index_jnt_traj_act_goal_fun(
      const rclcpp_action::GoalUUID& uuid, 
      std::shared_ptr<const mia_hand_msgs::action::JointTraj::Goal> goal);

  /**
   * \brief Callback function for a #index_jnt_traj_act_ cancel request.
   */
  rclcpp_action::CancelResponse index_jnt_traj_act_canc_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::JointTraj>> goal_handle);

  /**
   * \brief Callback function for accepting a #index_jnt_traj_act_ goal.
   */
  void index_jnt_traj_act_accept_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::JointTraj>> goal_handle);

  /**
   * \brief Function for executing #index_jnt_traj_act_ action.
   */
  void index_jnt_traj_act_execute(
    std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>> action);

  /**
   * \brief Callback function for a #mrl_jnt_traj_act_ goal request.
   */
  rclcpp_action::GoalResponse mrl_jnt_traj_act_goal_fun(
      const rclcpp_action::GoalUUID& uuid, 
      std::shared_ptr<const mia_hand_msgs::action::JointTraj::Goal> goal);

  /**
   * \brief Callback function for a #mrl_jnt_traj_act_ cancel request.
   */
  rclcpp_action::CancelResponse mrl_jnt_traj_act_canc_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::JointTraj>> goal_handle);

  /**
   * \brief Callback function for accepting a #mrl_jnt_traj_act_ goal.
   */
  void mrl_jnt_traj_act_accept_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::JointTraj>> goal_handle);

  /**
   * \brief Function for executing #mrl_jnt_traj_act_ action.
   */
  void mrl_jnt_traj_act_execute(
    std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>> action);

  /**
   * \brief Callback function of #get_cyl_refs_srv_ service.
   */
  void get_cyl_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #get_pin_refs_srv_ service.
   */
  void get_pin_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #get_lat_refs_srv_ service.
   */
  void get_lat_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #get_pup_refs_srv_ service.
   */
  void get_pup_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #get_pdw_refs_srv_ service.
   */
  void get_pdw_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #get_sph_refs_srv_ service.
   */
  void get_sph_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #get_tri_refs_srv_ service.
   */
  void get_tri_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #get_rlx_refs_srv_ service.
   */
  void get_rlx_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::GetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #set_cyl_refs_srv_ service.
   */
  void set_cyl_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #set_pin_refs_srv_ service.
   */
  void set_pin_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #set_lat_refs_srv_ service.
   */
  void set_lat_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #set_pup_refs_srv_ service.
   */
  void set_pup_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #set_pdw_refs_srv_ service.
   */
  void set_pdw_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #set_sph_refs_srv_ service.
   */
  void set_sph_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #set_tri_refs_srv_ service.
   */
  void set_tri_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #set_rlx_refs_srv_ service.
   */
  void set_rlx_refs_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::SetGraspReferences::Response> rsp);

  /**
   * \brief Callback function of #execute_cyl_srv_ service.
   */
  void execute_cyl_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp);

  /**
   * \brief Callback function of #execute_pin_srv_ service.
   */
  void execute_pin_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp);

  /**
   * \brief Callback function of #execute_lat_srv_ service.
   */
  void execute_lat_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp);

  /**
   * \brief Callback function of #execute_pup_srv_ service.
   */
  void execute_pup_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp);

  /**
   * \brief Callback function of #execute_pdw_srv_ service.
   */
  void execute_pdw_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp);

  /**
   * \brief Callback function of #execute_sph_srv_ service.
   */
  void execute_sph_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp);

  /**
   * \brief Callback function of #execute_tri_srv_ service.
   */
  void execute_tri_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp);

  /**
   * \brief Callback function of #execute_rlx_srv_ service.
   */
  void execute_rlx_srv_fun(
      const std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Request> req,
      std::shared_ptr<mia_hand_msgs::srv::ExecuteGrasp::Response> rsp);

  /**
   * \brief Callback function for a #cyl_act_ goal request.
   */
  rclcpp_action::GoalResponse cyl_act_goal_fun(
      const rclcpp_action::GoalUUID& uuid, 
      std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal);

  /**
   * \brief Callback function for a #cyl_act_ cancel request.
   */
  rclcpp_action::CancelResponse cyl_act_canc_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Callback function for accepting a #cyl_act_ goal.
   */
  void cyl_act_accept_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Function for executing #cyl_act_ action.
   */
  void cyl_act_execute(
    std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action);

  /**
   * \brief Callback function for a #pin_act_ goal request.
   */
  rclcpp_action::GoalResponse pin_act_goal_fun(
      const rclcpp_action::GoalUUID& uuid, 
      std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal);

  /**
   * \brief Callback function for a #pin_act_ cancel request.
   */
  rclcpp_action::CancelResponse pin_act_canc_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Callback function for accepting a #pin_act_ goal.
   */
  void pin_act_accept_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Function for executing #pin_act_ action.
   */
  void pin_act_execute(
    std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action);

  /**
   * \brief Callback function for a #lat_act_ goal request.
   */
  rclcpp_action::GoalResponse lat_act_goal_fun(
      const rclcpp_action::GoalUUID& uuid, 
      std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal);

  /**
   * \brief Callback function for a #lat_act_ cancel request.
   */
  rclcpp_action::CancelResponse lat_act_canc_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Callback function for accepting a #lat_act_ goal.
   */
  void lat_act_accept_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Function for executing #lat_act_ action.
   */
  void lat_act_execute(
    std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action);

  /**
   * \brief Callback function for a #pup_act_ goal request.
   */
  rclcpp_action::GoalResponse pup_act_goal_fun(
      const rclcpp_action::GoalUUID& uuid, 
      std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal);

  /**
   * \brief Callback function for a #pup_act_ cancel request.
   */
  rclcpp_action::CancelResponse pup_act_canc_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Callback function for accepting a #pup_act_ goal.
   */
  void pup_act_accept_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Function for executing #pup_act_ action.
   */
  void pup_act_execute(
    std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action);

  /**
   * \brief Callback function for a #pdw_act_ goal request.
   */
  rclcpp_action::GoalResponse pdw_act_goal_fun(
      const rclcpp_action::GoalUUID& uuid, 
      std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal);

  /**
   * \brief Callback function for a #pdw_act_ cancel request.
   */
  rclcpp_action::CancelResponse pdw_act_canc_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Callback function for accepting a #pdw_act_ goal.
   */
  void pdw_act_accept_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Function for executing #pdw_act_ action.
   */
  void pdw_act_execute(
    std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action);

  /**
   * \brief Callback function for a #sph_act_ goal request.
   */
  rclcpp_action::GoalResponse sph_act_goal_fun(
      const rclcpp_action::GoalUUID& uuid, 
      std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal);

  /**
   * \brief Callback function for a #sph_act_ cancel request.
   */
  rclcpp_action::CancelResponse sph_act_canc_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Callback function for accepting a #sph_act_ goal.
   */
  void sph_act_accept_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Function for executing #sph_act_ action.
   */
  void sph_act_execute(
    std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action);

  /**
   * \brief Callback function for a #tri_act_ goal request.
   */
  rclcpp_action::GoalResponse tri_act_goal_fun(
      const rclcpp_action::GoalUUID& uuid, 
      std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal);

  /**
   * \brief Callback function for a #tri_act_ cancel request.
   */
  rclcpp_action::CancelResponse tri_act_canc_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Callback function for accepting a #tri_act_ goal.
   */
  void tri_act_accept_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Function for executing #tri_act_ action.
   */
  void tri_act_execute(
    std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action);

  /**
   * \brief Callback function for a #rlx_act_ goal request.
   */
  rclcpp_action::GoalResponse rlx_act_goal_fun(
      const rclcpp_action::GoalUUID& uuid, 
      std::shared_ptr<const mia_hand_msgs::action::Grasp::Goal> goal);

  /**
   * \brief Callback function for a #rlx_act_ cancel request.
   */
  rclcpp_action::CancelResponse rlx_act_canc_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Callback function for accepting a #rlx_act_ goal.
   */
  void rlx_act_accept_fun(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<mia_hand_msgs::action::Grasp>> goal_handle);

  /**
   * \brief Function for executing #rlx_act_ action.
   */
  void rlx_act_execute(
    std::shared_ptr<Action<mia_hand_msgs::action::Grasp>> action);

  /**
   * \brief Callback function of #stream_tmr_.
   *
   * Here, depending on the active data streams, the related messages are
   * published over the dedicated topics.
   */
  void stream_tmr_fun();

  /**
   * \brief Callback function of #switch_mot_pos_stream_srv_.
   */
  void switch_mot_pos_stream_srv_fun(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> req, 
      std::shared_ptr<std_srvs::srv::SetBool::Response> rsp);

  /**
   * \brief Callback function of #switch_mot_spe_stream_srv_.
   */
  void switch_mot_spe_stream_srv_fun(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> req, 
      std::shared_ptr<std_srvs::srv::SetBool::Response> rsp);

  /**
   * \brief Callback function of #switch_mot_cur_stream_srv_.
   */
  void switch_mot_cur_stream_srv_fun(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> req, 
      std::shared_ptr<std_srvs::srv::SetBool::Response> rsp);

  /**
   * \brief Callback function of #switch_jnt_pos_stream_srv_.
   */
  void switch_jnt_pos_stream_srv_fun(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> req, 
      std::shared_ptr<std_srvs::srv::SetBool::Response> rsp);

  /**
   * \brief Callback function of #switch_jnt_spe_stream_srv_.
   */
  void switch_jnt_spe_stream_srv_fun(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> req, 
      std::shared_ptr<std_srvs::srv::SetBool::Response> rsp);

  /**
   * \brief Callback function of #switch_fin_for_stream_srv_.
   */
  void switch_fin_for_stream_srv_fun(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> req, 
      std::shared_ptr<std_srvs::srv::SetBool::Response> rsp);

  /**
   * \brief Function for aborting all the active goals of a queue of motor
   *        trajectory actions.
   */
  void abort_active_goals(
    const std::vector<std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>>>& actions);
    
  /**
   * \brief Function for aborting all the active goals of a queue of joint
   *        trajectory actions.
   */
  void abort_active_goals(
    const std::vector<std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>>>& actions);

  /**
   * \brief Function for aborting all the active goals of a queue of grasp 
   *        actions.
   */
  void abort_active_goals(
    const std::vector<std::shared_ptr<Action<mia_hand_msgs::action::Grasp>>>& actions);

  /**
   * \brief Function for removing the motor trajectory action with a specific 
   *        goal ID from a queue.
   */
  void remove_by_goal_id(
    std::vector<std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>>>& actions, 
    const rclcpp_action::GoalUUID& uuid);

  /**
   * \brief Function for removing the joint trajectory action with a specific 
   *        goal ID from a queue.
   */
  void remove_by_goal_id(
    std::vector<std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>>>& actions, 
    const rclcpp_action::GoalUUID& uuid);

  /**
   * \brief Function for removing the grasp action with a specific goal ID from 
   *        a queue.
   */
  void remove_by_goal_id(
    std::vector<std::shared_ptr<Action<mia_hand_msgs::action::Grasp>>>& actions, 
    const rclcpp_action::GoalUUID& uuid);

  std::unique_ptr<CppDriver> mia_hand_;  //!< Unique pointer to Mia Hand cpp driver object.

  std::shared_ptr<rclcpp::Node> node_;  //!< Shared pointer to Mia Hand node.

  bool is_connected_;  //!< Used for monitoring Mia Hand connection status.
  std::shared_ptr<rclcpp::TimerBase> connection_tmr_;  //!< Timer for checking periodically the connection with Mia Hand.

  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::Connect>> 
    connect_srv_;  //!< Srv for connecting to Mia Hand.
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> 
    disconnect_srv_;  //!< Srv for disconnecting from Mia Hand.

  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> 
    stop_srv_;  //!< Srv for triggering Mia Hand emergency stop.
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> 
    play_srv_;  //!< Srv for restoring Mia Hand normal operation after emergency stop.

  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> 
    calibrate_mot_pos_srv_;  //!< Srv for re-calibrating Mia Hand motor positions.

  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> 
    store_current_sets_srv_;  //!< Srv for storing the current Mia Hand settings.

  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> 
    get_fw_version_srv_; //!< Srv for getting the Mia Hand firmware version.

  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::GetMotorData>>
    get_mot_pos_srv_;  //!< Srv for getting current Mia Hand motor positions. 
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::GetMotorData>>
    get_mot_spe_srv_;  //!< Srv for getting current Mia Hand motor speeds. 
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::GetMotorData>>
    get_mot_cur_srv_;  //!< Srv for getting current Mia Hand motor currents. 

  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::GetForceData>>
    get_fin_for_srv_;  //!< Srv for getting current Mia Hand finger forces. 

  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::GetJointData>>
    get_jnt_pos_srv_;  //!< Srv for getting current Mia Hand joint positions. 
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::GetJointData>>
    get_jnt_spe_srv_;  //!< Srv for getting current Mia Hand joint speeds. 

  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetMotorSpeed>>
    set_thumb_mot_spe_srv_;  //!< Srv for setting a target thumb motor speed.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetMotorSpeed>>
    set_index_mot_spe_srv_;  //!< Srv for setting a target index motor speed.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetMotorSpeed>>
    set_mrl_mot_spe_srv_;  //!< Srv for setting a target mrl motor speed.

  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetMotorTraj>>
    set_thumb_mot_traj_srv_;  //!< Srv for setting a target thumb motor trajectory.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetMotorTraj>>
    set_index_mot_traj_srv_;  //!< Srv for setting a target index motor trajectory.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetMotorTraj>>
    set_mrl_mot_traj_srv_;  //!< Srv for setting a target mrl motor trajectory.

  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetJointSpeed>>
    set_thumb_jnt_spe_srv_;  //!< Srv for setting a target thumb joint speed.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetJointSpeed>>
    set_index_jnt_spe_srv_;  //!< Srv for setting a target index joint speed.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetJointSpeed>>
    set_mrl_jnt_spe_srv_;  //!< Srv for setting a target mrl joint speed.

  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetJointTraj>>
    set_thumb_jnt_traj_srv_;  //!< Srv for setting a target thumb joint trajectory.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetJointTraj>>
    set_index_jnt_traj_srv_;  //!< Srv for setting a target index joint trajectory.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetJointTraj>>
    set_mrl_jnt_traj_srv_;  //!< Srv for setting a target mrl joint trajectory.

  std::shared_ptr<rclcpp_action::Server<mia_hand_msgs::action::MotorTraj>>
    thumb_mot_traj_act_;  //!< Action for setting a target thumb motor trajectory.
  std::shared_ptr<rclcpp_action::Server<mia_hand_msgs::action::MotorTraj>>
    index_mot_traj_act_;  //!< Action for setting a target index motor trajectory.
  std::shared_ptr<rclcpp_action::Server<mia_hand_msgs::action::MotorTraj>>
    mrl_mot_traj_act_;  //!< Action for setting a target mrl motor trajectory.

  std::shared_ptr<rclcpp_action::Server<mia_hand_msgs::action::JointTraj>>
    thumb_jnt_traj_act_;  //!< Action for setting a target thumb joint trajectory.
  std::shared_ptr<rclcpp_action::Server<mia_hand_msgs::action::JointTraj>>
    index_jnt_traj_act_;  //!< Action for setting a target index joint trajectory.
  std::shared_ptr<rclcpp_action::Server<mia_hand_msgs::action::JointTraj>>
    mrl_jnt_traj_act_;  //!< Action for setting a target mrl joint trajectory.

  int32_t cyl_open_pos_[3];   //!< Cylindrical grasp OPEN motor positions.
  int32_t cyl_close_pos_[3];  //!< Cylindrical grasp CLOSE motor positions.
  int32_t pin_open_pos_[3];   //!< Pinch grasp OPEN motor positions.
  int32_t pin_close_pos_[3];  //!< Pinch grasp CLOSE motor positions.
  int32_t lat_open_pos_[3];   //!< Lateral grasp OPEN motor positions.
  int32_t lat_close_pos_[3];  //!< Lateral grasp CLOSE motor positions.
  int32_t pup_open_pos_[3];   //!< Point-up gesture OPEN motor positions.
  int32_t pup_close_pos_[3];  //!< Point-up gesture CLOSE motor positions.
  int32_t pdw_open_pos_[3];   //!< Point-down gesture OPEN motor positions.
  int32_t pdw_close_pos_[3];  //!< Point-down gesture CLOSE motor positions.
  int32_t sph_open_pos_[3];   //!< Spherical grasp OPEN motor positions.
  int32_t sph_close_pos_[3];  //!< Spherical grasp CLOSE motor positions.
  int32_t tri_open_pos_[3];   //!< Tri-digital grasp OPEN motor positions.
  int32_t tri_close_pos_[3];  //!< Tri-digital grasp CLOSE motor positions.
  int32_t rlx_open_pos_[3];   //!< Relax gesture OPEN motor positions.
  int32_t rlx_close_pos_[3];  //!< Relax gesture CLOSE motor positions.

  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::GetGraspReferences>> 
    get_cyl_refs_srv_;  //!< Srv for getting the cylidrical grasp references.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::GetGraspReferences>> 
    get_pin_refs_srv_;  //!< Srv for getting the pinch grasp references.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::GetGraspReferences>> 
    get_lat_refs_srv_;  //!< Srv for getting the lateral grasp references.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::GetGraspReferences>> 
    get_pup_refs_srv_;  //!< Srv for getting the point-up grasp references.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::GetGraspReferences>> 
    get_pdw_refs_srv_;  //!< Srv for getting the point-down grasp references.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::GetGraspReferences>> 
    get_sph_refs_srv_;  //!< Srv for getting the spherical grasp references.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::GetGraspReferences>> 
    get_tri_refs_srv_;  //!< Srv for getting the tri-digital grasp references.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::GetGraspReferences>> 
    get_rlx_refs_srv_;  //!< Srv for getting the relax grasp references.

  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetGraspReferences>> 
    set_cyl_refs_srv_;  //!< Srv for setting the cylidrical grasp references.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetGraspReferences>> 
    set_pin_refs_srv_;  //!< Srv for setting the pinch grasp references.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetGraspReferences>> 
    set_lat_refs_srv_;  //!< Srv for setting the lateral grasp references.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetGraspReferences>> 
    set_pup_refs_srv_;  //!< Srv for setting the point-up grasp references.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetGraspReferences>> 
    set_pdw_refs_srv_;  //!< Srv for setting the point-down grasp references.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetGraspReferences>> 
    set_sph_refs_srv_;  //!< Srv for setting the spherical grasp references.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetGraspReferences>> 
    set_tri_refs_srv_;  //!< Srv for setting the tri-digital grasp references.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::SetGraspReferences>> 
    set_rlx_refs_srv_;  //!< Srv for setting the relax grasp references.

  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::ExecuteGrasp>>
    execute_cyl_srv_;  //!< Srv for executing a cylindrical grasp.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::ExecuteGrasp>>
    execute_pin_srv_;  //!< Srv for executing a pinch grasp.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::ExecuteGrasp>>
    execute_lat_srv_;  //!< Srv for executing a lateral grasp.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::ExecuteGrasp>>
    execute_pup_srv_;  //!< Srv for executing a point-up gesture.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::ExecuteGrasp>>
    execute_pdw_srv_;  //!< Srv for executing a point-down gesture.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::ExecuteGrasp>>
    execute_sph_srv_;  //!< Srv for executing a spherical grasp.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::ExecuteGrasp>>
    execute_tri_srv_;  //!< Srv for executing a tri-digital grasp.
  std::shared_ptr<rclcpp::Service<mia_hand_msgs::srv::ExecuteGrasp>>
    execute_rlx_srv_;  //!< Srv for executing a relax grasp.

  std::shared_ptr<rclcpp_action::Server<mia_hand_msgs::action::Grasp>>
    cyl_act_;  //!< Action for executing a cylindrical grasp.
  std::shared_ptr<rclcpp_action::Server<mia_hand_msgs::action::Grasp>>
    pin_act_;  //!< Action for executing a pinch grasp.
  std::shared_ptr<rclcpp_action::Server<mia_hand_msgs::action::Grasp>>
    lat_act_;  //!< Action for executing a lateral grasp.
  std::shared_ptr<rclcpp_action::Server<mia_hand_msgs::action::Grasp>>
    pup_act_;  //!< Action for executing a point-up gesture.
  std::shared_ptr<rclcpp_action::Server<mia_hand_msgs::action::Grasp>>
    pdw_act_;  //!< Action for executing a point-down gesture.
  std::shared_ptr<rclcpp_action::Server<mia_hand_msgs::action::Grasp>>
    sph_act_;  //!< Action for executing a spherical grasp.
  std::shared_ptr<rclcpp_action::Server<mia_hand_msgs::action::Grasp>>
    tri_act_;  //!< Action for executing a tri-digital grasp.
  std::shared_ptr<rclcpp_action::Server<mia_hand_msgs::action::Grasp>>
    rlx_act_;  //!< Action for executing a relax gesture.

  std::vector<std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>>>
    thumb_mot_traj_actions_;  //!< Thumb motor trajectory actions queue. 
  std::vector<std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>>>
    index_mot_traj_actions_;  //!< Index motor trajectory actions queue. 
  std::vector<std::shared_ptr<Action<mia_hand_msgs::action::MotorTraj>>>
    mrl_mot_traj_actions_;    //!< Mrl motor trajectory actions queue. 

  std::vector<std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>>>
    thumb_jnt_traj_actions_;  //!< Thumb joint trajectory actions under execution. 
  std::vector<std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>>>
    index_jnt_traj_actions_;  //!< Index joint trajectory actions under execution. 
  std::vector<std::shared_ptr<Action<mia_hand_msgs::action::JointTraj>>>
    mrl_jnt_traj_actions_;    //!< Mrl joint trajectory actions under execution. 

  std::vector<std::shared_ptr<Action<mia_hand_msgs::action::Grasp>>>
    grasp_actions_;  //!< Grasp actions under execution.

  std::mutex actions_mtx_;  //!< Used for synchronizing access to action queues.

  std::shared_ptr<rclcpp::TimerBase> stream_tmr_;  //!< Timer for publishing the active data stream messages.

  bool mot_pos_stream_on_;  //!< Flag for monitoring motor positions streaming status.
  bool mot_spe_stream_on_;  //!< Flag for monitoring motor speeds streaming status.
  bool mot_cur_stream_on_;  //!< Flag for monitoring motor currents streaming status.
  bool fin_for_stream_on_;  //!< Flag for monitoring finger forces streaming status.
  bool jnt_pos_stream_on_;  //!< Flag for monitoring joint positions streaming status.
  bool jnt_spe_stream_on_;  //!< Flag for monitoring joint speeds streaming status.
  uint32_t n_streams_;      //!< Number of active streams.

  std::shared_ptr<rclcpp::Publisher<mia_hand_msgs::msg::MotorData>>
    mot_pos_pub_;  //!< Motor positions publisher.
  std::shared_ptr<rclcpp::Publisher<mia_hand_msgs::msg::MotorData>>
    mot_spe_pub_;  //!< Motor speeds publisher.
  std::shared_ptr<rclcpp::Publisher<mia_hand_msgs::msg::MotorData>>
    mot_cur_pub_;  //!< Motor currents publisher.
  std::shared_ptr<rclcpp::Publisher<mia_hand_msgs::msg::JointData>>
    jnt_pos_pub_;  //!< Joint positions publisher.
  std::shared_ptr<rclcpp::Publisher<mia_hand_msgs::msg::JointData>>
    jnt_spe_pub_;  //!< Joint speeds publisher.
  std::shared_ptr<rclcpp::Publisher<mia_hand_msgs::msg::ForceData>>
    fin_for_pub_;  //!< Finger forces publisher.

  mia_hand_msgs::msg::MotorData mot_data_msg_;  //!< Msg for publishing motor data.
  mia_hand_msgs::msg::JointData jnt_data_msg_;  //!< Msg for publishing joint data.
  mia_hand_msgs::msg::ForceData for_data_msg_;  //!< Msg for publishing force data.

  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>>
    switch_mot_pos_stream_srv_;  //!< Srv for switching on/off the motor positions streaming.
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>>
    switch_mot_spe_stream_srv_;  //!< Srv for switching on/off the motor speeds streaming.
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>>
    switch_mot_cur_stream_srv_;  //!< Srv for switching on/off the motor currents streaming.
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>>
    switch_jnt_pos_stream_srv_;  //!< Srv for switching on/off the joint positions streaming.
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>>
    switch_jnt_spe_stream_srv_;  //!< Srv for switching on/off the joint speeds streaming.
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>>
    switch_fin_for_stream_srv_;  //!< Srv for switching on/off the finger forces streaming.
};
}  // namespace

#endif  // MIA_HAND_DRIVER_ROS_DRIVER_HPP
