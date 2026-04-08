#ifndef MIA_HAND_ROS2_CONTROL_MIA_HAND_SYSTEM_INTERFACE_HPP
#define MIA_HAND_ROS2_CONTROL_MIA_HAND_SYSTEM_INTERFACE_HPP

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mia_hand_driver/cpp_driver.hpp"

namespace mia_hand_ros2_control
{
class MiaHandSystemInterface : public hardware_interface::SystemInterface
{
public:

  /**
   * \brief Class constructor.
   */
  MiaHandSystemInterface();

  /**
   * \brief Function for initializing Mia Hand system interface.
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) override;

  /**
   * \brief Function for configuring Mia hand system interface and connecting to
   *        Mia Hand.
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  /**
   * \brief Function for disconnecting from Mia Hand.
   */
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State& previous_state) override;

  /**
   * \brief Function for activating Mia Hand system interface.
   */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  /**
   * \brief Function for de-activating Mia Hand system interface.
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  /**
   * \brief Function for exporting the interfaces to joint states.
   */
  std::vector<hardware_interface::StateInterface> 
    export_state_interfaces() override;

  /**
   * \brief Function for exporting the interfaces to joint commands.
   */
  std::vector<hardware_interface::CommandInterface> 
    export_command_interfaces() override;

  /**
   * \brief Function for switching to new joint control modes.
   */
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) override;

  /**
   * \brief TODO.
   */
  hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /**
   * \brief TODO.
   */
  hardware_interface::return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:

  /**
   * \brief Function for reading all Mia Hand joints info.
   *
   * @param info joints info, taken from hardware info.
   * @return \code true \endcode if joints info reading is successful, \code 
   *         false \endcode if not.
   */
  bool read_joints_info(
    const std::vector<hardware_interface::ComponentInfo>& info);

  /**
   * \brief Function for checking if a specific joint command interface is 
   *        defined.
   *
   * @param jnt_info Joint info.
   * @param interface_name Interface name.
   * @return \code true \endcode if the interface is found, \code false \endcode
   *         if not.
   */
  bool has_cmd_interface(
    const hardware_interface::ComponentInfo& jnt_info, 
    const std::string& interface_name);

  /**
   * \brief Function for checking if a specific joint state interface is 
   *        defined.
   *
   * @param jnt_info Joint info.
   * @param interface_name Interface name.
   * @return \code true \endcode if the interface is found, \code false \endcode
   *         if not.
   */
  bool has_state_interface(
    const hardware_interface::ComponentInfo& jnt_info, 
    const std::string& interface_name);

  /**
   * \brief Function for reading all info related to the extra joints exploited
   *        for Rviz2 visualization.
   *
   * @param jnt_info Hardware joints info, taken from URDF.
   */
  void read_rviz2_joints_info(
    const std::vector<hardware_interface::ComponentInfo>& jnt_info);

  std::unique_ptr<rclcpp::Logger> logger_;  //!< For info or error messages.

  std::string serial_port_;  //!< Mia Hand serial port.
  std::unique_ptr<mia_hand_driver::CppDriver> mia_hand_;  //!< Mia Hand driver.

  /**
   * \brief Structure for representing extra joints necessary only for Rviz2
   *        visualization.
   */
  struct Rviz2JointInfo
  {
    std::string name;
    double pos;
    double vel;
  };

  /**
   * \brief Enum class to represent the current joint control mode. 
   */
  enum class CommandMode
  {
    kNone = 0,
    kPosition = 1,
    kVelocity = 2
  };

  std::array<std::string, 3> jnt_names_;  //!< Joint names.
  std::array<std::string, 3> rviz2_jnt_names_;  //!< RViz2 joint names.

  bool b_jnt_pos_cmd_defined_[3];    //!< Joint position commands defined or not.
  bool b_jnt_pos_state_defined_[3];  //!< Joint position states defined or not.

  double jnt_pos_cmd_[3];    //!< Joint position commands.
  double jnt_pos_state_[3];  //!< Joint position states.

  bool b_jnt_vel_cmd_defined_[3];    //!< Joint velocity commands defined or not.
  bool b_jnt_vel_state_defined_[3];  //!< Joint velocity states defined or not.

  double jnt_vel_cmd_[3];    //!< Joint velocity commands.
  double jnt_vel_state_[3];  //!< Joint velocity states.

  std::array<CommandMode, 3> jnt_cmd_modes_;  //!< Current joint command modes.

  std::array<Rviz2JointInfo, 3> rviz2_joints_;  //!< Rviz2 joints info.
};
}  // namespace

#endif  // MIA_HAND_ROS2_CONTROL_MIA_HAND_SYSTEM_INTERFACE_HPP




