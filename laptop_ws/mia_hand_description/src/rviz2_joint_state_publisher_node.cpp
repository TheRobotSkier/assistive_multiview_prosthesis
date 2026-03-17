#include "mia_hand_description/rviz2_joint_state_publisher.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> rviz2_joint_state_publisher_node = 
    std::make_shared<rclcpp::Node>("rviz2_joint_state_publisher");

  std::unique_ptr<mia_hand_description::Rviz2JointStatePublisher> 
    rviz2_joint_state_publisher =
      mia_hand_description::Rviz2JointStatePublisher::create(
        rviz2_joint_state_publisher_node);

  if (nullptr != rviz2_joint_state_publisher)
  {
    rclcpp::spin(rviz2_joint_state_publisher_node);
  }

  rclcpp::shutdown();

  return 0;
}
