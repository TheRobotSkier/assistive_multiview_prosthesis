cmake_minimum_required(VERSION 3.8)
project(mia_hand_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/MotorData.msg"
  "msg/JointData.msg"
  "msg/ForceData.msg"
)

set(srv_files
  "srv/Connect.srv"
  "srv/GetMotorData.srv"
  "srv/GetJointData.srv"
  "srv/GetForceData.srv"
  "srv/SetMotorSpeed.srv"
  "srv/SetMotorTraj.srv"
  "srv/SetJointSpeed.srv"
  "srv/SetJointTraj.srv"
  "srv/GetGraspReferences.srv"
  "srv/SetGraspReferences.srv"
  "srv/ExecuteGrasp.srv"
)

set(action_files
  "action/MotorTraj.action"
  "action/JointTraj.action"
  "action/Grasp.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}

  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
