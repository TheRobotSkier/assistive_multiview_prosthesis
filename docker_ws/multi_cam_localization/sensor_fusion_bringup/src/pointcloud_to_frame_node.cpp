#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace {

constexpr uint8_t FLOAT32 = sensor_msgs::msg::PointField::FLOAT32;

struct VoxelKey {
  int64_t x;
  int64_t y;
  int64_t z;

  bool operator==(const VoxelKey &other) const { return x == other.x && y == other.y && z == other.z; }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &key) const {
    std::size_t seed = 0;
    auto combine = [&seed](int64_t value) {
      std::size_t h = std::hash<int64_t>{}(value);
      seed ^= h + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    };
    combine(key.x);
    combine(key.y);
    combine(key.z);
    return seed;
  }
};

bool is_zero_stamp(const builtin_interfaces::msg::Time &stamp) { return stamp.sec == 0 && stamp.nanosec == 0; }

std::size_t point_count(const sensor_msgs::msg::PointCloud2 &cloud) {
  return static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
}

int field_offset(const sensor_msgs::msg::PointCloud2 &cloud, const std::string &name) {
  for (const auto &field : cloud.fields) {
    if (field.name == name && field.datatype == FLOAT32 && field.count >= 1) {
      return static_cast<int>(field.offset);
    }
  }
  return -1;
}

float read_float32(const std::vector<uint8_t> &data, std::size_t offset, bool big_endian) {
  uint8_t bytes[4];
  std::memcpy(bytes, data.data() + offset, sizeof(bytes));
  if (big_endian) {
    std::reverse(bytes, bytes + 4);
  }
  float value = 0.0F;
  std::memcpy(&value, bytes, sizeof(value));
  return value;
}

tf2::Transform to_tf2(const geometry_msgs::msg::Transform &msg) {
  tf2::Quaternion q(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w);
  if (!std::isfinite(q.length2()) || q.length2() <= 0.0) {
    q.setRPY(0.0, 0.0, 0.0);
  } else {
    q.normalize();
  }
  return tf2::Transform(q, tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));
}

geometry_msgs::msg::Transform to_msg(const tf2::Transform &transform) {
  geometry_msgs::msg::Transform msg;
  msg.translation.x = transform.getOrigin().x();
  msg.translation.y = transform.getOrigin().y();
  msg.translation.z = transform.getOrigin().z();
  msg.rotation.x = transform.getRotation().x();
  msg.rotation.y = transform.getRotation().y();
  msg.rotation.z = transform.getRotation().z();
  msg.rotation.w = transform.getRotation().w();
  return msg;
}

std::string json_escape(const std::string &value) {
  std::ostringstream out;
  for (char ch : value) {
    if (ch == '"' || ch == '\\') {
      out << '\\' << ch;
    } else if (ch == '\n') {
      out << "\\n";
    } else {
      out << ch;
    }
  }
  return out.str();
}

} // namespace

class PointCloudToFrameNode : public rclcpp::Node {
public:
  PointCloudToFrameNode()
      : Node("pointcloud_to_frame_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    input_topic_ = declare_parameter<std::string>("input_topic", "/camera/camera/depth/color/points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/camera/camera/points_marker_map");
    target_frame_ = declare_parameter<std::string>("target_frame", "marker_map");
    camera_pose_frame_ = declare_parameter<std::string>("camera_pose_frame", "cam0");
    camera_color_optical_frame_ = declare_parameter<std::string>("camera_color_optical_frame", "camera_color_optical_frame");
    marker_map_locked_topic_ = declare_parameter<std::string>("marker_map_locked_topic", "");
    require_marker_map_locked_ = declare_parameter<bool>("require_marker_map_locked", true);
    max_rate_hz_ = declare_parameter<double>("max_rate_hz", 15.0);
    voxel_leaf_m_ = declare_parameter<double>("voxel_leaf_m", 0.01);
    transform_timeout_s_ = declare_parameter<double>("transform_timeout_s", 0.02);
    max_tf_age_s_ = declare_parameter<double>("max_tf_age_s", 0.50);
    status_period_s_ = declare_parameter<double>("status_period_s", 1.0);
    use_latest_tf_ = declare_parameter<bool>("use_latest_tf", true);

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<std_msgs::msg::String>(output_topic_ + "/status", 10);

    if (!marker_map_locked_topic_.empty()) {
      auto lock_qos = rclcpp::QoS(1).transient_local().reliable();
      lock_sub_ = create_subscription<std_msgs::msg::Bool>(
          marker_map_locked_topic_, lock_qos, [this](const std_msgs::msg::Bool::SharedPtr msg) { marker_map_locked_ = msg->data; });
    } else {
      marker_map_locked_ = !require_marker_map_locked_;
    }

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(), std::bind(&PointCloudToFrameNode::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Transforming %s into %s on %s", input_topic_.c_str(), target_frame_.c_str(), output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Camera pose alias: %s == %s", camera_pose_frame_.c_str(), camera_color_optical_frame_.c_str());
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (require_marker_map_locked_ && !marker_map_locked_) {
      publish_status(false, "marker_map_not_locked", *msg, 0, "waiting for marker-map lock");
      return;
    }

    const rclcpp::Time now = get_clock()->now();
    if (max_rate_hz_ > 0.0 && last_publish_time_.nanoseconds() > 0) {
      const double dt = (now - last_publish_time_).seconds();
      if (dt >= 0.0 && dt < (1.0 / max_rate_hz_)) {
        publish_status(false, "rate_limited", *msg, 0, "");
        return;
      }
    }

    geometry_msgs::msg::TransformStamped transform;
    std::string reason;
    if (!lookup_composed_transform(*msg, transform, reason)) {
      publish_status(false, reason, *msg, 0, "");
      return;
    }

    sensor_msgs::msg::PointCloud2 transformed;
    try {
      tf2::doTransform(*msg, transformed, transform);
    } catch (const std::exception &exc) {
      publish_status(false, "transform_failed", *msg, 0, exc.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 output = voxel_downsample(transformed);
    output.header.stamp = msg->header.stamp;
    output.header.frame_id = target_frame_;
    pub_->publish(output);
    last_publish_time_ = now;
    publish_status(true, "published", *msg, point_count(output), "");
  }

  bool lookup_composed_transform(const sensor_msgs::msg::PointCloud2 &cloud, geometry_msgs::msg::TransformStamped &out,
                                 std::string &reason) {
    if (cloud.header.frame_id.empty()) {
      reason = "empty_source_frame";
      return false;
    }

    const tf2::TimePoint lookup_time = use_latest_tf_ || is_zero_stamp(cloud.header.stamp)
                                           ? tf2::TimePointZero
                                           : tf2::timeFromSec(rclcpp::Time(cloud.header.stamp).seconds());
    const tf2::Duration timeout = tf2::durationFromSec(std::max(0.0, transform_timeout_s_));

    geometry_msgs::msg::TransformStamped target_to_camera;
    geometry_msgs::msg::TransformStamped color_to_source;
    try {
      target_to_camera = tf_buffer_.lookupTransform(target_frame_, camera_pose_frame_, lookup_time, timeout);
      color_to_source = tf_buffer_.lookupTransform(camera_color_optical_frame_, cloud.header.frame_id, lookup_time, timeout);
    } catch (const tf2::TransformException &exc) {
      reason = std::string("tf_unavailable:") + exc.what();
      return false;
    }

    if (max_tf_age_s_ > 0.0 && !is_zero_stamp(target_to_camera.header.stamp)) {
      const double age_s = (get_clock()->now() - rclcpp::Time(target_to_camera.header.stamp)).seconds();
      if (std::isfinite(age_s) && age_s > max_tf_age_s_) {
        std::ostringstream ss;
        ss << "tf_stale:" << age_s;
        reason = ss.str();
        return false;
      }
    }

    const tf2::Transform composed = to_tf2(target_to_camera.transform) * to_tf2(color_to_source.transform);
    out.header.stamp = cloud.header.stamp;
    out.header.frame_id = target_frame_;
    out.child_frame_id = cloud.header.frame_id;
    out.transform = to_msg(composed);
    return true;
  }

  sensor_msgs::msg::PointCloud2 voxel_downsample(const sensor_msgs::msg::PointCloud2 &cloud) const {
    if (voxel_leaf_m_ <= 0.0 || point_count(cloud) == 0 || cloud.point_step == 0) {
      return cloud;
    }

    const int x_offset = field_offset(cloud, "x");
    const int y_offset = field_offset(cloud, "y");
    const int z_offset = field_offset(cloud, "z");
    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
      return cloud;
    }

    sensor_msgs::msg::PointCloud2 out = cloud;
    out.height = 1;
    out.width = 0;
    out.row_step = 0;
    out.is_dense = true;
    out.data.clear();
    out.data.reserve(cloud.data.size());

    std::unordered_set<VoxelKey, VoxelKeyHash> seen;
    const std::size_t count = point_count(cloud);
    for (std::size_t i = 0; i < count; ++i) {
      const std::size_t base = i * static_cast<std::size_t>(cloud.point_step);
      const std::size_t max_offset =
          base + static_cast<std::size_t>(std::max({x_offset, y_offset, z_offset})) + sizeof(float);
      if (max_offset > cloud.data.size()) {
        continue;
      }

      const float x = read_float32(cloud.data, base + static_cast<std::size_t>(x_offset), cloud.is_bigendian);
      const float y = read_float32(cloud.data, base + static_cast<std::size_t>(y_offset), cloud.is_bigendian);
      const float z = read_float32(cloud.data, base + static_cast<std::size_t>(z_offset), cloud.is_bigendian);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const VoxelKey key{static_cast<int64_t>(std::floor(x / voxel_leaf_m_)),
                         static_cast<int64_t>(std::floor(y / voxel_leaf_m_)),
                         static_cast<int64_t>(std::floor(z / voxel_leaf_m_))};
      if (!seen.insert(key).second) {
        continue;
      }

      const auto begin = cloud.data.begin() + static_cast<std::vector<uint8_t>::difference_type>(base);
      const auto end = begin + static_cast<std::vector<uint8_t>::difference_type>(cloud.point_step);
      out.data.insert(out.data.end(), begin, end);
      ++out.width;
    }

    out.row_step = out.width * out.point_step;
    return out;
  }

  void publish_status(bool accepted, const std::string &reason, const sensor_msgs::msg::PointCloud2 &input,
                      std::size_t output_points, const std::string &detail) {
    const rclcpp::Time now = get_clock()->now();
    const bool reason_changed = reason != last_status_reason_;
    if (!reason_changed && status_period_s_ > 0.0 && last_status_time_.nanoseconds() > 0 &&
        (now - last_status_time_).seconds() < status_period_s_) {
      return;
    }

    std::ostringstream ss;
    ss << "{\"accepted\":" << (accepted ? "true" : "false");
    ss << ",\"reason\":\"" << json_escape(reason) << "\"";
    ss << ",\"input_topic\":\"" << json_escape(input_topic_) << "\"";
    ss << ",\"output_topic\":\"" << json_escape(output_topic_) << "\"";
    ss << ",\"source_frame\":\"" << json_escape(input.header.frame_id) << "\"";
    ss << ",\"target_frame\":\"" << json_escape(target_frame_) << "\"";
    ss << ",\"camera_pose_frame\":\"" << json_escape(camera_pose_frame_) << "\"";
    ss << ",\"camera_color_optical_frame\":\"" << json_escape(camera_color_optical_frame_) << "\"";
    ss << ",\"marker_map_locked\":" << (marker_map_locked_ ? "true" : "false");
    ss << ",\"input_points\":" << point_count(input);
    ss << ",\"output_points\":" << output_points;
    ss << ",\"max_rate_hz\":" << max_rate_hz_;
    ss << ",\"voxel_leaf_m\":" << voxel_leaf_m_;
    if (!detail.empty()) {
      ss << ",\"detail\":\"" << json_escape(detail) << "\"";
    }
    ss << "}";

    std_msgs::msg::String msg;
    msg.data = ss.str();
    status_pub_->publish(msg);
    last_status_time_ = now;
    last_status_reason_ = reason;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  std::string camera_pose_frame_;
  std::string camera_color_optical_frame_;
  std::string marker_map_locked_topic_;
  bool require_marker_map_locked_ = true;
  bool marker_map_locked_ = false;
  double max_rate_hz_ = 15.0;
  double voxel_leaf_m_ = 0.01;
  double transform_timeout_s_ = 0.02;
  double max_tf_age_s_ = 0.50;
  double status_period_s_ = 1.0;
  bool use_latest_tf_ = true;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lock_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  std::string last_status_reason_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToFrameNode>());
  rclcpp::shutdown();
  return 0;
}
