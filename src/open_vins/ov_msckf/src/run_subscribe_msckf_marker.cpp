/*
 * Phase 2 marker-enabled ROS 2 entrypoint for OpenVINS.
 *
 * This intentionally lives beside the upstream run_subscribe_msckf executable so
 * the original launch path can remain untouched.
 */

#include <memory>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "utils/dataset_reader.h"

#if ROS_AVAILABLE == 1
#include "ros/ROS1Visualizer.h"
#include <ros/ros.h>
#elif ROS_AVAILABLE == 2
#include "ros/ROS2Visualizer.h"
#include <rclcpp/rclcpp.hpp>
#endif

using namespace ov_msckf;

std::shared_ptr<VioManager> sys_marker;
#if ROS_AVAILABLE == 1
std::shared_ptr<ROS1Visualizer> viz_marker;
#elif ROS_AVAILABLE == 2
std::shared_ptr<ROS2Visualizer> viz_marker;
#endif

int main(int argc, char **argv) {

  std::string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }

#if ROS_AVAILABLE == 1
  ros::init(argc, argv, "run_subscribe_msckf_marker");
  auto nh = std::make_shared<ros::NodeHandle>("~");
  nh->param<std::string>("config_path", config_path, config_path);
#elif ROS_AVAILABLE == 2
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("run_subscribe_msckf_marker", options);
  node->get_parameter<std::string>("config_path", config_path);
#endif

  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
#if ROS_AVAILABLE == 1
  parser->set_node_handler(nh);
#elif ROS_AVAILABLE == 2
  parser->set_node(node);
#endif

  std::string verbosity = "DEBUG";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = true;
  sys_marker = std::make_shared<VioManager>(params);
#if ROS_AVAILABLE == 1
  viz_marker = std::make_shared<ROS1Visualizer>(nh, sys_marker);
  viz_marker->setup_subscribers(parser);
#elif ROS_AVAILABLE == 2
  viz_marker = std::make_shared<ROS2Visualizer>(node, sys_marker);
  viz_marker->setup_subscribers(parser);
#endif

  if (!parser->successful()) {
    PRINT_ERROR(RED "unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  PRINT_DEBUG("done...spinning marker-enabled OpenVINS to ros\n");
#if ROS_AVAILABLE == 1
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
#elif ROS_AVAILABLE == 2
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
#endif

  viz_marker->visualize_final();
#if ROS_AVAILABLE == 1
  viz_marker.reset();
  sys_marker.reset();
  ros::shutdown();
#elif ROS_AVAILABLE == 2
  viz_marker.reset();
  sys_marker.reset();
  rclcpp::shutdown();
#endif

  return EXIT_SUCCESS;
}
