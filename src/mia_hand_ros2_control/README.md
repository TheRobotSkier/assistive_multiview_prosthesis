# mia_hand_ros2_control

This package provides an interface for Mia Hand devices with the [ROS2 Control](https://control.ros.org/jazzy/index.html)
framework, allowing to control Mia Hand using standard or custom [ROS2 Controllers](https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html).

## Launching Mia Hand system interface

To launch Mia Hand system interface and spawn the related controllers, the 
script `mia_hand_system_interface_launch.py` can be used. The bash command for 
doing so is `ros2 launch mia_hand_ros2_control mia_hand_system_interface_launch.py`.
The script takes the following arguments:

* `serial_port`: serial port to which Mia Hand is connected. It can be read by 
      invoking `sudo dmesg | grep tty`. Defaults to `/dev/ttyUSB0`. This parameter 
      is ignored if `use_mock_hardware` is set to `true` (see below).
* `rviz2_gui`: selects if starting also a RViz2 GUI to visualize the hand state.
      Accepts `true` or `false`. Defaults to `true`.
* `laterality`: Mia Hand laterality (for URDF loading). Accepts `right` or `left`. 
      Defaults to `right`.
* `prefix`: optional prefix to be added to Mia Hand URDF element names.
* `robot_ns`: namespace assigned to robot-specific nodes. Defaults to `mia_hand`
* `use_mock_hardware`: selects if using a mock hardware instead of the physical 
      one. Accepts `true` or `false`. Defaults to `false`. If set to `true`, the 
      parameter `serial_port` is ignored.

This script spawns, for each joint, a velocity controller, a position controller 
and a trajectory controller. By default, position controllers are active. 
