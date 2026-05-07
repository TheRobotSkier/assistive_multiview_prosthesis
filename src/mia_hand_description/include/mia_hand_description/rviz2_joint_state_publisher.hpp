#ifndef MIA_HAND_DESCRIPTION_RVIZ2_JOINT_STATE_PUBLISHER_HPP
#define MIA_HAND_DESCRIPTION_RVIZ2_JOINT_STATE_PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "mia_hand_description/thumb_joint_opposition.hpp"

namespace mia_hand_description
{
class Rviz2JointStatePublisher
{
public:

  /**
   * \brief Factory function for creating a new Rviz2JointStatePublisher object.
   *
   * A factory function is used for avoiding to throw exceptions in the class
   * constructor, in case of errors during object initialization.
   *
   * @param node Shared pointer to the state publisher ROS node.
   * @return \code std::unique_ptr \endcode to the new Rviz2JointStatePublisher object.
   *         \code nullptr \endcode in case of object initialization failure.
   */
  static std::unique_ptr<Rviz2JointStatePublisher> create(
      std::shared_ptr<rclcpp::Node> node);

  /* Preventing Rviz2JointStatePublisher objects to be copyable, either through the copy
   * constructor and the assignment operator, to ensure the unique access to
   * them through the std::unique_ptr.
   */
  Rviz2JointStatePublisher(Rviz2JointStatePublisher&) = delete;
  Rviz2JointStatePublisher& operator=(Rviz2JointStatePublisher&) = delete;

private:

  /**
   * \brief Class constructor.
   *
   * The class constructor is private to allow instantiations of new
   * Rviz2JointStatePublisher objects only through #create() factory function.
   */
  Rviz2JointStatePublisher();

  /**
   * \brief Function for initializing a Rviz2JointStatePublisher object.
   *
   * This function is necessary for accessing all the non-static members through
   * the static factory function #create().
   *
   * @param node Shared pointer to the state publisher ROS node.
   * @return \code true \endcode if object initialization is successful, 
   *         \code false \endcode if not.
   */
  bool init(std::shared_ptr<rclcpp::Node> node);

  /**
   * \brief Callback function of #jnt_state_raw_sub_ subscription.
   */
  void jnt_state_raw_sub_fun(const sensor_msgs::msg::JointState& msg);

  std::unique_ptr<ThumbJointOpposition> thumb_jnt_opp_;  //!< Thumb joint opposition object.

  std::shared_ptr<rclcpp::Node> node_;  //!< State publisher ROS node.
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>>
    jnt_state_raw_sub_;  //!< Subscriber for the raw joint states, that need to be remapped.
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>>
    jnt_state_remap_pub_;  //!< Publisher for remapped joint states.

  std::string thumb_jnt_name_;   //!< Name of thumb joint in URDF.
  std::string index_jnt_name_;   //!< Name of index joint in URDF.
};
}  // namespace

#endif  // MIA_HAND_DESCRIPTION_RVIZ2_JOINT_STATE_PUBLISHER_HPP

