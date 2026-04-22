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
#include "grasp_preshaping/ffi_types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Import FFI types into the global namespace so the rest of the node code
// does not need to be changed from the original version.
using GraspComputeFn      = grasp_preshaping::GraspComputeFn;
using GraspApiVersionFn   = grasp_preshaping::GraspApiVersionFn;
using GraspPoseFFI        = grasp_preshaping::GraspPoseFFI;
using GraspTwistFFI       = grasp_preshaping::GraspTwistFFI;
using PointCloudViewFFI   = grasp_preshaping::PointCloudViewFFI;
using GraspComputeRequestFFI  = grasp_preshaping::GraspComputeRequestFFI;
using GraspComputeResponseFFI = grasp_preshaping::GraspComputeResponseFFI;
using CameraPositionFFI       = grasp_preshaping::CameraPositionFFI;

constexpr int kGraspComputeOk = grasp_preshaping::kGraspComputeOk;

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
    // TF2 buffer and listener for camera pose lookups
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Configurable camera frame names
    camera_frames_ = declare_parameter<std::vector<std::string>>(
      "camera_frames",
      std::vector<std::string>{
        "mujoco_front_depth_cam",
        "mujoco_camera_wrist_cam"
      });
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

  void publish_joint_commands(double thumb, double index, double mrl)
  {
    std_msgs::msg::Float64MultiArray command;
    command.data = {thumb};
    thumb_cmd_pub_->publish(command);
    command.data = {index};
    index_cmd_pub_->publish(command);
    command.data = {mrl};
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

    // Resolve camera positions from TF
    uint32_t n_cameras = 0;
    for (const auto & frame : camera_frames_) {
      if (n_cameras >= 4) {break;}
      try {
        auto transform = tf_buffer_->lookupTransform(
          "world", frame, tf2::TimePointZero);
        request.cameras[n_cameras].x = static_cast<float>(
          transform.transform.translation.x);
        request.cameras[n_cameras].y = static_cast<float>(
          transform.transform.translation.y);
        request.cameras[n_cameras].z = static_cast<float>(
          transform.transform.translation.z);
        ++n_cameras;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(
          get_logger(),
          "Could not lookup camera frame '%s': %s",
          frame.c_str(), ex.what());
      }
    }

    // Fallback: estimate camera from hand pose if no TF cameras found
    if (n_cameras == 0) {
      RCLCPP_WARN(
        get_logger(),
        "No camera TF resolved, estimating camera from hand pose");
      // Estimate: camera_position = hand_position + hand_rotation * (-0.08, -0.46, 0.10)
      const auto & p = pose.position;
      const auto & q = pose.orientation;
      // Rotate offset by hand orientation (simplified quaternion rotation)
      double ox = -0.08, oy = -0.46, oz = 0.10;
      // q * v + q_conj * v  (inline quaternion-vector multiply)
      double qxx = q.x * q.x, qyy = q.y * q.y, qzz = q.z * q.z;
      double qxy = q.x * q.y, qxz = q.x * q.z, qyz = q.y * q.z;
      double qwx = q.w * q.x, qwy = q.w * q.y, qwz = q.w * q.z;
      double rx = ox * (1.0 - 2.0 * (qyy + qzz)) + oy * (2.0 * (qxy - qwz)) + oz * (2.0 * (qxz + qwy));
      double ry = ox * (2.0 * (qxy + qwz)) + oy * (1.0 - 2.0 * (qxx + qzz)) + oz * (2.0 * (qyz - qwx));
      double rz = ox * (2.0 * (qxz - qwy)) + oy * (2.0 * (qyz + qwx)) + oz * (1.0 - 2.0 * (qxx + qyy));
      request.cameras[0].x = static_cast<float>(p.x + rx);
      request.cameras[0].y = static_cast<float>(p.y + ry);
      request.cameras[0].z = static_cast<float>(p.z + rz);
      n_cameras = 1;
    }
    request.n_cameras = n_cameras;

    RCLCPP_INFO(
      get_logger(),
      "Planner called with %u camera(s):"
      " cam0=(%.3f,%.3f,%.3f)",
      n_cameras,
      n_cameras > 0 ? request.cameras[0].x : 0.0f,
      n_cameras > 0 ? request.cameras[0].y : 0.0f,
      n_cameras > 0 ? request.cameras[0].z : 0.0f);

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

    publish_joint_commands(
      ffi_response.thumb_closure,
      ffi_response.index_closure,
      ffi_response.mrl_closure);
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

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<std::string> camera_frames_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreshapingServiceBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
