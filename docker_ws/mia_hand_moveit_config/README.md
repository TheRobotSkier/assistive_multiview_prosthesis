# mia_hand_moveit_config

This package enables the generation and execution of motion plans with Moveit2 by using a simulated, mock, or real hardware interfaces of the Mia Hand gripper.

## Moveit2

By default, the `mia_hand_moveit.launch.py` launch file is designed to interface the real hardware interface with Moveit2. 

```bash
ros2 launch mia_hand_moveit_config mia_hand_moveit.launch.py 
```
A list of the launch arguments is reported below:

* `serial_port`: serial port to which Mia Hand is connected. Default value is `/dev/ttyUSB0`.
* `rviz2_gui`: to start automatically Rviz2. Default value is `True`.
* `laterality`: parameter for loading a right or left hand in RViz2. Default value is `right`.
* `prefix`: prefix to be added before Mia Hand link and joint names. Default value is ``.
* `robot_ns`: robot namespace. Default value is `mia_hand`.
* `use_mock_hardware`: start robot with mock hardware mirroring command to its states. Default value is `False`.
* `use_mujoco_sim`: interface Moveit2 with `ros2_control` and `MuJoCo` simulation. Default value is `False`.

To generate motion plans and execute them with MuJoCo and the simulated interface:

```bash
ros2 launch mia_hand_moveit_config mia_hand_moveit.launch.py \
  use_mujoco_sim:=true 
```

and to use the mock hardware interface:

```bash
ros2 launch mia_hand_moveit_config mia_hand_moveit.launch.py \
  use_mock_hardware:=true 
```
Instead, to work with the physical robot and generate/execute path with Moveit:
```bash
ros2 launch mia_hand_moveit_config mia_hand_moveit.launch.py 
```
