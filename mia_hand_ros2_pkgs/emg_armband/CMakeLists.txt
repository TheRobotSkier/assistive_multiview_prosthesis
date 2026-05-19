cmake_minimum_required(VERSION 3.15)
project(emg_armband)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the emg_classifier Python package so it is importable after
# `source install/setup.bash` without modifying sys.path manually.
ament_python_install_package(emg_classifier)

# Install Python node executables
install(PROGRAMS
  nodes/emg_node.py
  nodes/emg_parser_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install standalone scripts (data collection, training, standalone runner, monitor, setup)
install(PROGRAMS
  scripts/collect_data.py
  scripts/train.py
  scripts/run_classifier.py
  scripts/emg_monitor.py
  scripts/setup.sh
  DESTINATION lib/${PROJECT_NAME}/scripts
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
