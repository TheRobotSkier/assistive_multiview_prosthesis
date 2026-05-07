#include "rclcpp/rclcpp.hpp"

#include "mia_hand_driver/ros_driver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> mia_hand_driver_node =
    std::make_shared<rclcpp::Node>("driver");

  std::unique_ptr<mia_hand_driver::RosDriver> mia_hand_ros_driver =
                      mia_hand_driver::RosDriver::create(mia_hand_driver_node);

  if (nullptr != mia_hand_ros_driver)
  {
    rclcpp::spin(mia_hand_driver_node);
  }

  rclcpp::shutdown();

  return 0;
}
