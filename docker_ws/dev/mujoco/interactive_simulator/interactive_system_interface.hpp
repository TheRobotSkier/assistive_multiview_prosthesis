#ifndef MIA_HAND_MUJOCO_INTERACTIVE_SYSTEM_INTERFACE_HPP
#define MIA_HAND_MUJOCO_INTERACTIVE_SYSTEM_INTERFACE_HPP

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/state.hpp"

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
};
}  // namespace mia_hand_mujoco

#endif  // MIA_HAND_MUJOCO_INTERACTIVE_SYSTEM_INTERFACE_HPP
