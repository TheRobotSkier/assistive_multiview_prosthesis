# mia_hand_description

This package contains the [URDF](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html) 
files that can be used to visualize or simulate Mia Hand devices. This package allows 
also to visualize Mia Hand in [RViz2](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-Main.html).

The details about how to exploit Mia Hand URDF and visualize it in RViz2 are 
illustrated below.

## URDF

Mia Hand URDF files include the [xacro](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html) 
functionalities. This allows for enhanced flexibility when using Mia Hand URDF 
inside projects.

To include Mia Hand in a custom URDF, the file [mia_hand.urdf.xacro](https://bitbucket.org/prensilia-ros/mia_hand_ros2_pkgs/src/master/mia_hand_description/urdf/mia_hand.urdf.xacro) 
should be included. This file defines a `<xacro:macro/>` block that can be used 
to create a Mia Hand instance. First, the block `<xacro:include filename="$(find mia_hand_description)/urdf/mia_hand.urdf.xacro"/>` 
should be added, to include `mia_hand.urdf.xacro` file. Then, the block `<xacro:mia_hand/>`
should be used to create an instance of Mia Hand.

The block `<xacro:mia_hand/>` accepts some parameters, that are used to adjust 
the URDF behaviour. Such parameters are:

* `parent`: Mia Hand parent link (e.g., `world`, in case Mia Hand is not 
      attached to any other robot).
* `*origin`: `<origin/>` block specifying Mia Hand position and orientation with 
      respect to the `parent` link frame.
* `laterality` (optional): Mia Hand laterality. Can be set to `'right'` or `'left'`. If not 
      specified, this parameter defaults to `'right'`.
* `prefix` (optional): prefix applied to all the component names. This parameter 
      allows to include multiple hands in the same URDF without name collisions.
* `joint_limits_config_file` (optional): name of the YAML file inside 
      `calibration/` folder where joint limits are specified. Normally, it 
      defaults to the generic `joint_limits_default.yaml` file. However, when a 
      Mia Hand device is purchased, a hand-specific joint limits file is 
      provided. This file can be placed in the `calibration/` folder and passed 
      as parameter, to align the URDF joint limits to the real ones. 

An example of the use of Mia Hand URDF can be found in 
[mia_hand_description.urdf.xacro](https://bitbucket.org/prensilia-ros/mia_hand_ros2_pkgs/src/master/mia_hand_description/urdf/mia_hand_description.urdf.xacro).
Here, Mia Hand is attached to the `world` link, and the other parameters are 
passed as `<xacro:arg/>` (except `*origin`, since it is a XML block). This allows 
to pass them also from a launch script.

## RViz2

To visualize Mia Hand in RViz2, `mia_hand_rviz_launch.py` script can be exploited, 
by calling `ros2 launch mia_hand_description mia_hand_rviz_launch.py`.

This script exploits the previously mentioned `mia_hand_description.urdf.xacro` 
URDF to visualiza the hand. Therefore, the script accepts the optional arguments 
`laterality`, `prefix` and `joint_limits_config_file`. Moreover, three additional
optional arguments can be specified:

* `use_joints_gui`: starts an additional `joint_state_publisher_gui` node, to set 
      joint positions through a GUI. Admits `true` or `false`. Defaults to `true`.
* `load_ur_flange`: attaches to Mia Hand a flange compatible with UR manipulators.
      Valid only for right hands. Admits `true` or `false`. Defaults to `false`.
* `robot_ns`: additional namespace assigned to robot-specific nodes. Defaults to 
      `mia_hand`.

It should be noted that, in the script, an additional node, named 
`rviz2_joint_state_publisher` is started. This node is used to map the mechanical 
coupling between index flexion/extension and thumb abduction/adduction in RViz2.
This mapping is computed depending on the index joint angles at which thumb 
opposition should be started and stopped. These angles are imported from a YAML 
file in `calibration/` folder. By default, the generic file is 
`transmission_config_default.yaml`. However, if a Mia Hand device is purchased, a 
hand-specific file is provided, named `transmission_config.yaml`. By placing this 
file into `calibration/` folder, the remapping will be computed automatically from 
it.
