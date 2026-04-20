#ifndef MIA_HAND_MUJOCO_INTERACTIVE_SYSTEM_INTERFACE_HPP
#define MIA_HAND_MUJOCO_INTERACTIVE_SYSTEM_INTERFACE_HPP

#include <atomic>
#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "interactive_simulator.hpp"

namespace mia_hand_mujoco
{
class InteractiveSystemInterface : public hardware_interface::SystemInterface
{
public:
  InteractiveSystemInterface();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  bool read_joints_info(
    const std::vector<hardware_interface::ComponentInfo>& info);

  bool has_cmd_interface(
    const hardware_interface::ComponentInfo& jnt_info,
    const std::string& interface_name);

  bool has_state_interface(
    const hardware_interface::ComponentInfo& jnt_info,
    const std::string& interface_name);

  void read_rviz2_joints_info(
    const std::vector<hardware_interface::ComponentInfo>& jnt_info);

  // Scene pose ROS callbacks (instant teleport)
  void on_hand_pose_msg(const geometry_msgs::msg::Pose::SharedPtr msg);
  void on_object_pose_msg(const geometry_msgs::msg::Pose::SharedPtr msg);
  void on_camera_pose_msg(const geometry_msgs::msg::Pose::SharedPtr msg);

  // Smooth motion ROS callbacks (PoseStamped: stamp encodes duration in seconds)
  void on_hand_motion_msg(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void on_object_motion_msg(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void on_camera_motion_msg(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Helper: convert geometry_msgs Pose (xyzw quaternion) to InteractiveSimulator API
  static void pose_msg_to_sim(
    const geometry_msgs::msg::Pose& msg,
    double pos[3], double quat_wxyz[4]);

  void trigger_preshaping_service();

  std::unique_ptr<rclcpp::Logger> logger_;

  std::string xml_model_path_;

  struct Rviz2JointInfo
  {
    std::string name;
    double pos;
    double vel;
  };

  enum class CommandMode
  {
    kNone = 0,
    kPosition = 1,
    kVelocity = 2
  };

  std::array<std::string, 3> jnt_names_;
  std::array<std::string, 3> rviz2_jnt_names_;

  bool b_jnt_pos_cmd_defined_[3];
  bool b_jnt_pos_state_defined_[3];

  double jnt_pos_cmd_[3];
  double jnt_pos_state_[3];

  bool b_jnt_vel_cmd_defined_[3];
  bool b_jnt_vel_state_defined_[3];

  double jnt_vel_state_[3];
  double jnt_vel_cmd_[3];

  std::array<CommandMode, 3> jnt_cmd_modes_;

  std::array<Rviz2JointInfo, 3> rviz2_joints_;

  std::thread sim_trd_;

  // Scene pose subscriptions and publishers
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr hand_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr object_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr camera_pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr hand_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr hand_pose_alias_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr hand_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr object_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr camera_pose_pub_;

  // Smooth motion subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr motion_hand_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr motion_obj_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr motion_cam_sub_;

  // IMU and simulation time publishers (front camera)
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sim_time_pub_;

  double hand_twist_prev_pos_[3];
  double hand_twist_prev_quat_[4];
  double hand_twist_prev_sim_time_;
  bool hand_twist_initialized_;

  // Wrist camera IMU publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag2_pub_;

  // Preshaping Trigger service client
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr preshaping_trigger_client_;
  std::atomic<bool> preshaping_call_running_;

  int pose_pub_counter_;
  int imu_pub_counter_;
};
}  // namespace mia_hand_mujoco

#endif  // MIA_HAND_MUJOCO_INTERACTIVE_SYSTEM_INTERFACE_HPP
