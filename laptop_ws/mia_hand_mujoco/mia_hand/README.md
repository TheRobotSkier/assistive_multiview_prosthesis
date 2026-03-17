## URDF ⟶  MJCF derivation steps

1. Converted the STL [mesh files](https://bitbucket.org/prensilia-ros/mia_hand_ros2_pkgs/src/master/mia_hand_description/meshes/stl/)
   to OBJ format using [Blender](https://www.blender.org/).
   When exporting the meshes, Y was set as forward axis, and Z was set as up axis.
2. Processed '.obj' mesh files with [obj2mjcf](https://github.com/kevinzakka/obj2mjcf).
3. Removed the '<xacro:macro>' tags from the [URDF](https://bitbucket.org/prensilia-ros/mia_hand_ros2_pkgs/src/master/mia_hand_description/urdf/mia_hand_right.urdf.xacro).
4. Replaced all the joint limits within the '<limit>' tags with the values from 
   [joint_limits_default.yaml](https://bitbucket.org/prensilia-ros/mia_hand_ros2_pkgs/src/master/mia_hand_description/calibration/joint_limits_default.yaml) 
   and removed the '<xacro:include>' and '<xacro:joint_limits>' tags. 
5. Removed all the '${prefix}' occurrences.
6. Replaced '<xacro:insert_block name="origin"/>' with '<origin xyz="0 0 0.2" rpy="1.57 0 0"/>'.
7. Added '<link name="world">' and replaced '${parent}' with 'world'. 
8. At every '<mesh>' tag, removed the path to the mesh file, leaving only the file 
   name, and changed the format to '.obj'.
9. Added '<mujoco> <compiler meshdir="assets/" balanceinertia="true" 
   discardvisual="false" fusestatic="false"/> <mujoco/>'.
10. Converted URDF file from '.urdf.xacro' to '.urdf' format.
11. Loaded the URDF file in MuJoCo and saved the corresponding MJCF.
12. Added joint equalities, contact exclusions and actuators.
13. Extracted together common properties under '<default>' section.
14. Created 'scene_right.xml', which includes the hand, with a textured groundplane, 
    skybox and haze.
