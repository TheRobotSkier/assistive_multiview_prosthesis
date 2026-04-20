#include <array>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
struct GraspPoseFFI
{
  double px;
  double py;
  double pz;
  double qx;
  double qy;
  double qz;
  double qw;
};

struct GraspTwistFFI
{
  double lx;
  double ly;
  double lz;
  double ax;
  double ay;
  double az;
  std::array<double, 36> covariance;
};

struct PointCloudViewFFI
{
  size_t width;
  size_t height;
  size_t point_step;
  size_t x_off;
  size_t y_off;
  size_t z_off;
  const uint8_t * data_ptr;
  size_t data_len;
};

struct GraspComputeRequestFFI
{
  GraspPoseFFI pose;
  GraspTwistFFI twist;
  PointCloudViewFFI cloud;
};

struct GraspComputeResponseFFI
{
  uint8_t success;
  double closure_amount;
  double combined_score;
  int32_t grasp_type;
};

using GraspComputeFn = int (*) (
  const GraspComputeRequestFFI *,
  GraspComputeResponseFFI *,
  char *,
  size_t);
using GraspApiVersionFn = uint32_t (*) ();

constexpr int kGraspComputeOk = 0;
}

class PreshapingServiceBridgeNode : public rclcpp::Node
{
public:
  PreshapingServiceBridgeNode()
  : Node("preshaping_service_bridge"),
    has_pose_(false),
    has_twist_(false),
    has_cloud_(false),
    rust_lib_handle_(nullptr),
    rust_compute_fn_(nullptr),
    rust_api_version_fn_(nullptr)
  {
    hand_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
      "/hand_pose", 10,
      [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_pose_ = *msg;
        has_pose_ = true;
      });

    hand_twist_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/hand_twist", 10,
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_twist_ = *msg;
        has_twist_ = true;
      });

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/segmented_object_cloud", 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_cloud_ = *msg;
        has_cloud_ = true;
      });

    thumb_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/thumb_pos_ff_controller/commands", 10);
    index_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/index_pos_ff_controller/commands", 10);
    mrl_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mrl_pos_ff_controller/commands", 10);

    initialize_rust_backend();

    service_ = create_service<std_srvs::srv::Trigger>(
      "/grasp_preshaping/compute_grasp",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        try_handle_direct_request(response);
      });

    RCLCPP_INFO(
      get_logger(),
      "Preshaping service bridge ready on /grasp_preshaping/compute_grasp");
  }

  ~PreshapingServiceBridgeNode() override
  {
    if (rust_lib_handle_ != nullptr) {
      dlclose(rust_lib_handle_);
      rust_lib_handle_ = nullptr;
    }
  }

private:
  void initialize_rust_backend()
  {
    std::vector<std::string> candidates;

    const char * env_path = std::getenv("GRASP_PRESHAPING_LIB_PATH");
    if (env_path != nullptr && std::strlen(env_path) > 0) {
      candidates.emplace_back(env_path);
    }

    candidates.emplace_back("/miahand_ws/install/grasp_preshaping/lib/libgrasp_preshaping.so");
    candidates.emplace_back("/miahand_ws/install/lib/libgrasp_preshaping.so");

    candidates.emplace_back("/miahand_ws/src/dev/grasp_preshaping/target/release/libgrasp_preshaping.so");
    candidates.emplace_back("/miahand_ws/src/dev/grasp_preshaping/target/debug/libgrasp_preshaping.so");

    for (const auto & candidate : candidates) {
      void * handle = dlopen(candidate.c_str(), RTLD_NOW | RTLD_LOCAL);
      if (handle == nullptr) {
        continue;
      }

      auto compute_fn = reinterpret_cast<GraspComputeFn>(dlsym(handle, "grasp_preshaping_compute"));
      if (compute_fn == nullptr) {
        dlclose(handle);
        continue;
      }

      auto version_fn = reinterpret_cast<GraspApiVersionFn>(
        dlsym(handle, "grasp_preshaping_api_version"));

      rust_lib_handle_ = handle;
      rust_compute_fn_ = compute_fn;
      rust_api_version_fn_ = version_fn;

      if (rust_api_version_fn_ != nullptr) {
        RCLCPP_INFO(
          get_logger(),
          "Loaded Rust preshaping backend (%s), API version=%u",
          candidate.c_str(),
          rust_api_version_fn_());
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Loaded Rust preshaping backend (%s)",
          candidate.c_str());
      }
      return;
    }

    RCLCPP_FATAL(
      get_logger(),
      "Rust preshaping backend not found — cannot start bridge node.");
    throw std::runtime_error("Rust preshaping backend not found");
  }

  void publish_joint_commands(double position)
  {
    std_msgs::msg::Float64MultiArray command;
    command.data = {position};
    thumb_cmd_pub_->publish(command);
    index_cmd_pub_->publish(command);
    mrl_cmd_pub_->publish(command);
  }

  bool try_handle_direct_request(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::TwistWithCovarianceStamped twist;
    sensor_msgs::msg::PointCloud2 cloud;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (!has_pose_) {
        response->success = false;
        response->message = "No pose data received yet";
        return true;
      }
      if (!has_twist_) {
        response->success = false;
        response->message = "No twist data received yet";
        return true;
      }
      if (!has_cloud_) {
        response->success = false;
        response->message = "No point cloud data received yet";
        return true;
      }

      pose = latest_pose_;
      twist = latest_twist_;
      cloud = latest_cloud_;
    }

    GraspComputeRequestFFI request{};
    request.pose.px = pose.position.x;
    request.pose.py = pose.position.y;
    request.pose.pz = pose.position.z;
    request.pose.qx = pose.orientation.x;
    request.pose.qy = pose.orientation.y;
    request.pose.qz = pose.orientation.z;
    request.pose.qw = pose.orientation.w;

    request.twist.lx = twist.twist.twist.linear.x;
    request.twist.ly = twist.twist.twist.linear.y;
    request.twist.lz = twist.twist.twist.linear.z;
    request.twist.ax = twist.twist.twist.angular.x;
    request.twist.ay = twist.twist.twist.angular.y;
    request.twist.az = twist.twist.twist.angular.z;
    for (size_t i = 0; i < request.twist.covariance.size(); ++i) {
      request.twist.covariance[i] = twist.twist.covariance[i];
    }

    request.cloud.width = cloud.width;
    request.cloud.height = cloud.height;
    request.cloud.point_step = cloud.point_step;
    request.cloud.x_off = 0;
    request.cloud.y_off = 4;
    request.cloud.z_off = 8;
    for (const auto & field : cloud.fields) {
      if (field.name == "x") {
        request.cloud.x_off = field.offset;
      } else if (field.name == "y") {
        request.cloud.y_off = field.offset;
      } else if (field.name == "z") {
        request.cloud.z_off = field.offset;
      }
    }
    request.cloud.data_ptr = cloud.data.data();
    request.cloud.data_len = cloud.data.size();

    GraspComputeResponseFFI ffi_response{};
    std::array<char, 512> ffi_message{};
    const int status = rust_compute_fn_(
      &request,
      &ffi_response,
      ffi_message.data(),
      ffi_message.size());

    const std::string message(ffi_message.data());
    if (status != kGraspComputeOk) {
      response->success = false;
      response->message = message.empty() ? "Rust compute call failed" : message;
      return true;
    }

    if (ffi_response.success == 0U) {
      response->success = false;
      response->message = message.empty() ? "Preshaping failed" : message;
      return true;
    }

    publish_joint_commands(ffi_response.closure_amount);
    response->success = true;
    response->message = message.empty() ? "Preshaping completed" : message;
    return true;
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr hand_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr hand_twist_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thumb_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr index_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mrl_cmd_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

  std::mutex input_mutex_;
  geometry_msgs::msg::Pose latest_pose_;
  geometry_msgs::msg::TwistWithCovarianceStamped latest_twist_;
  sensor_msgs::msg::PointCloud2 latest_cloud_;
  bool has_pose_;
  bool has_twist_;
  bool has_cloud_;

  void * rust_lib_handle_;
  GraspComputeFn rust_compute_fn_;
  GraspApiVersionFn rust_api_version_fn_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreshapingServiceBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
