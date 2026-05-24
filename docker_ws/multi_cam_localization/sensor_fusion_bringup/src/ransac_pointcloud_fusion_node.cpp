#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/search/kdtree.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

namespace {

void rosToPCL(const sensor_msgs::msg::PointCloud2 &ros_msg, pcl::PCLPointCloud2 &pcl_msg) {
  pcl_msg.header.seq = ros_msg.header.stamp.sec;
  pcl_msg.header.stamp = static_cast<uint64_t>(ros_msg.header.stamp.sec) * 1000000ULL +
                         static_cast<uint64_t>(ros_msg.header.stamp.nanosec) / 1000ULL;
  pcl_msg.header.frame_id = ros_msg.header.frame_id;
  pcl_msg.height = ros_msg.height;
  pcl_msg.width = ros_msg.width;
  pcl_msg.fields.resize(ros_msg.fields.size());
  for (std::size_t i = 0; i < ros_msg.fields.size(); ++i) {
    pcl_msg.fields[i].name = ros_msg.fields[i].name;
    pcl_msg.fields[i].offset = ros_msg.fields[i].offset;
    pcl_msg.fields[i].datatype = ros_msg.fields[i].datatype;
    pcl_msg.fields[i].count = ros_msg.fields[i].count;
  }
  pcl_msg.is_bigendian = ros_msg.is_bigendian;
  pcl_msg.point_step = ros_msg.point_step;
  pcl_msg.row_step = ros_msg.row_step;
  pcl_msg.is_dense = static_cast<bool>(ros_msg.is_dense);
  pcl_msg.data = ros_msg.data;
}

void pclToROS(const pcl::PCLPointCloud2 &pcl_msg, sensor_msgs::msg::PointCloud2 &ros_msg) {
  ros_msg.header.stamp.sec = static_cast<int32_t>(pcl_msg.header.stamp / 1000000ULL);
  ros_msg.header.stamp.nanosec = static_cast<uint32_t>((pcl_msg.header.stamp % 1000000ULL) * 1000ULL);
  ros_msg.header.frame_id = pcl_msg.header.frame_id;
  ros_msg.height = pcl_msg.height;
  ros_msg.width = pcl_msg.width;
  ros_msg.fields.resize(pcl_msg.fields.size());
  for (std::size_t i = 0; i < pcl_msg.fields.size(); ++i) {
    ros_msg.fields[i].name = pcl_msg.fields[i].name;
    ros_msg.fields[i].offset = pcl_msg.fields[i].offset;
    ros_msg.fields[i].datatype = pcl_msg.fields[i].datatype;
    ros_msg.fields[i].count = pcl_msg.fields[i].count;
  }
  ros_msg.is_bigendian = pcl_msg.is_bigendian;
  ros_msg.point_step = pcl_msg.point_step;
  ros_msg.row_step = pcl_msg.row_step;
  ros_msg.is_dense = pcl_msg.is_dense;
  ros_msg.data = pcl_msg.data;
}

template<typename PointT>
void fromROSMsg(const sensor_msgs::msg::PointCloud2 &ros_msg, pcl::PointCloud<PointT> &cloud) {
  pcl::PCLPointCloud2 pcl_msg;
  rosToPCL(ros_msg, pcl_msg);
  pcl::fromPCLPointCloud2(pcl_msg, cloud);
}

template<typename PointT>
void toROSMsg(const pcl::PointCloud<PointT> &cloud, sensor_msgs::msg::PointCloud2 &ros_msg) {
  pcl::PCLPointCloud2 pcl_msg;
  pcl::toPCLPointCloud2(cloud, pcl_msg);
  pclToROS(pcl_msg, ros_msg);
}

} // namespace

class RansacPointCloudFusionNode : public rclcpp::Node {
public:
  RansacPointCloudFusionNode()
      : Node("ransac_pointcloud_fusion_node") {
    head_topic_ = declare_parameter<std::string>("head_cloud_topic", "/head/d435i_head/points_marker_map");
    arm_topic_ = declare_parameter<std::string>("arm_cloud_topic", "/arm/d435i_arm/points_marker_map");
    output_topic_ = declare_parameter<std::string>("output_topic", "/pointcloud_fused_ransac");
    ransac_min_correspondences_ = declare_parameter<int>("ransac_min_correspondences", 8);
    ransac_max_iterations_ = declare_parameter<int>("ransac_max_iterations", 2000);
    ransac_max_correspondence_dist_m_ = declare_parameter<double>("ransac_max_correspondence_dist_m", 0.05);
    downsample_leaf_m_ = declare_parameter<double>("downsample_leaf_m", 0.02);
    fuse_voxel_leaf_m_ = declare_parameter<double>("fuse_voxel_leaf_m", 0.01);
    max_cloud_age_s_ = declare_parameter<double>("max_cloud_age_s", 1.0);
    max_rate_hz_ = declare_parameter<double>("max_rate_hz", 10.0);
    enable_ransac_fusion_ = declare_parameter<bool>("enable_ransac_fusion", true);
    normal_radius_m_ = declare_parameter<double>("normal_radius_m", 0.05);
    feature_radius_m_ = declare_parameter<double>("feature_radius_m", 0.10);

    auto input_qos = rclcpp::SensorDataQoS().keep_last(2);
    auto output_qos = rclcpp::QoS(2).reliable();

    head_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        head_topic_, input_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloud_callback(*msg, latest_head_, head_stamp_); });
    arm_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        arm_topic_, input_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloud_callback(*msg, latest_arm_, arm_stamp_); });

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, output_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(output_topic_ + "/status", 10);

    RCLCPP_INFO(get_logger(), "RANSAC fusion: %s + %s -> %s (min_corr=%d)",
                head_topic_.c_str(), arm_topic_.c_str(), output_topic_.c_str(), ransac_min_correspondences_);
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2 &msg,
                      sensor_msgs::msg::PointCloud2 &store,
                      rclcpp::Time &stamp_store) {
    store = msg;
    stamp_store = get_clock()->now();
    maybe_fuse();
  }

  void maybe_fuse() {
    if (latest_head_.data.empty() || latest_arm_.data.empty()) {
      return;
    }

    const rclcpp::Time now = get_clock()->now();

    // Rate limit
    if (max_rate_hz_ > 0.0 && last_publish_time_.nanoseconds() > 0) {
      const double dt = (now - last_publish_time_).seconds();
      if (dt >= 0.0 && dt < (1.0 / max_rate_hz_)) {
        return;
      }
    }

    // Age check
    const double head_age = (now - head_stamp_).seconds();
    const double arm_age = (now - arm_stamp_).seconds();
    if (head_age > max_cloud_age_s_ || arm_age > max_cloud_age_s_) {
      return;
    }

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr head_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr arm_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    fromROSMsg(latest_head_, *head_cloud);
    fromROSMsg(latest_arm_, *arm_cloud);

    if (head_cloud->empty() || arm_cloud->empty()) {
      return;
    }

    // Downsample for feature extraction
    pcl::PointCloud<pcl::PointXYZ>::Ptr head_ds(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr arm_ds(new pcl::PointCloud<pcl::PointXYZ>());
    {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setLeafSize(downsample_leaf_m_, downsample_leaf_m_, downsample_leaf_m_);
      vg.setInputCloud(head_cloud);
      vg.filter(*head_ds);
      vg.setInputCloud(arm_cloud);
      vg.filter(*arm_ds);
    }

    if (head_ds->size() < 10 || arm_ds->size() < 10) {
      // Not enough points for feature extraction, fall through to simple merge
      publish_fused(head_cloud, arm_cloud, pcl::PointCloud<pcl::PointXYZ>(),
                    Eigen::Matrix4f::Identity(), false, "insufficient_points");
      return;
    }

    bool ransac_ok = false;
    Eigen::Matrix4f ransac_transform = Eigen::Matrix4f::Identity();
    int correspondences = 0;

    if (enable_ransac_fusion_) {
      ransac_ok = compute_ransac_alignment(*head_ds, *arm_ds, ransac_transform, correspondences);
    }

    if (ransac_ok) {
      // Transform the full-resolution arm cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr arm_aligned(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::transformPointCloud(*arm_cloud, *arm_aligned, ransac_transform);

      publish_fused(head_cloud, arm_cloud, *arm_aligned, ransac_transform, true, "ransac_aligned");
    } else {
      publish_fused(head_cloud, arm_cloud, pcl::PointCloud<pcl::PointXYZ>(),
                    Eigen::Matrix4f::Identity(), false, "ransac_insufficient_features");
    }
  }

  bool compute_ransac_alignment(const pcl::PointCloud<pcl::PointXYZ> &head,
                                const pcl::PointCloud<pcl::PointXYZ> &arm,
                                Eigen::Matrix4f &transform_out,
                                int &correspondences_out) {
    // Normals
    pcl::PointCloud<pcl::Normal>::Ptr head_normals(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr arm_normals(new pcl::PointCloud<pcl::Normal>());
    {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setSearchMethod(tree);
      ne.setRadiusSearch(normal_radius_m_);
      ne.setInputCloud(head.makeShared());
      ne.compute(*head_normals);
      ne.setInputCloud(arm.makeShared());
      ne.compute(*arm_normals);
    }

    // FPFH features
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr head_features(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr arm_features(new pcl::PointCloud<pcl::FPFHSignature33>());
    {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
      fpfh.setSearchMethod(tree);
      fpfh.setRadiusSearch(feature_radius_m_);
      fpfh.setInputCloud(head.makeShared());
      fpfh.setInputNormals(head_normals);
      fpfh.compute(*head_features);
      fpfh.setInputCloud(arm.makeShared());
      fpfh.setInputNormals(arm_normals);
      fpfh.compute(*arm_features);
    }

    if (head_features->size() < ransac_min_correspondences_ ||
        arm_features->size() < ransac_min_correspondences_) {
      correspondences_out = static_cast<int>(std::min(head_features->size(), arm_features->size()));
      return false;
    }

    // SAC-IA alignment
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setMinSampleDistance(0.02);
    sac_ia.setMaxCorrespondenceDistance(ransac_max_correspondence_dist_m_);
    sac_ia.setMaximumIterations(ransac_max_iterations_);
    sac_ia.setInputSource(arm.makeShared());
    sac_ia.setSourceFeatures(arm_features);
    sac_ia.setInputTarget(head.makeShared());
    sac_ia.setTargetFeatures(head_features);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    sac_ia.align(aligned);

    if (!sac_ia.hasConverged()) {
      return false;
    }

    const double fitness = sac_ia.getFitnessScore();
    // Estimate correspondence count from fitness: fitness ≈ 1.0 - (inliers / total_points)
    const std::size_t source_pts = arm_ds.size();
    if (fitness > 0.0 && fitness < 1.0 && source_pts > 0) {
      correspondences_out = static_cast<int>((1.0 - fitness) * static_cast<double>(source_pts));
    } else {
      correspondences_out = static_cast<int>(source_pts);
    }

    if (correspondences_out < ransac_min_correspondences_) {
      return false;
    }

    transform_out = sac_ia.getFinalTransformation();
    return true;
  }

  void publish_fused(const pcl::PointCloud<pcl::PointXYZ>::Ptr &head_full,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr &arm_full,
                     const pcl::PointCloud<pcl::PointXYZ> &arm_aligned,
                     const Eigen::Matrix4f &transform,
                     bool ransac_used,
                     const std::string &reason) {
    // Merge: head + (transformed) arm
    pcl::PointCloud<pcl::PointXYZ> merged;
    merged.reserve(head_full->size() + arm_full->size());

    // Add head points
    for (const auto &p : head_full->points) {
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
        merged.push_back(p);
      }
    }

    // Add arm points (either aligned or raw)
    if (ransac_used && !arm_aligned.empty()) {
      for (const auto &p : arm_aligned.points) {
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
          merged.push_back(p);
        }
      }
    } else {
      for (const auto &p : arm_full->points) {
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
          merged.push_back(p);
        }
      }
    }

    // Voxel deduplication on merged cloud
    sensor_msgs::msg::PointCloud2 output;
    if (fuse_voxel_leaf_m_ > 0.0 && !merged.empty()) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr dedup(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setLeafSize(fuse_voxel_leaf_m_, fuse_voxel_leaf_m_, fuse_voxel_leaf_m_);
      vg.setInputCloud(merged.makeShared());
      vg.filter(*dedup);

      // Preserve color from head cloud if available (use nearest-neighbor)
      // For now just use XYZ
      toROSMsg(*dedup, output);
    } else {
      toROSMsg(merged, output);
    }

    output.header.stamp = get_clock()->now();
    output.header.frame_id = "marker_map";
    pub_->publish(output);
    last_publish_time_ = get_clock()->now();

    publish_status(merged.size(), output.data.size() / output.point_step, reason, ransac_used);
  }

  void publish_status(std::size_t merged_points, std::size_t output_points,
                      const std::string &reason, bool ransac_used) {
    const std::size_t head_points = pcl::PointCloud<pcl::PointXYZ>().size(); // unused
    std::ostringstream ss;
    ss << "{";
    ss << "\"ransac_used\":" << (ransac_used ? "true" : "false") << ",";
    ss << "\"reason\":\"" << reason << "\",";
    ss << "\"head_topic\":\"" << head_topic_ << "\",";
    ss << "\"arm_topic\":\"" << arm_topic_ << "\",";
    ss << "\"output_topic\":\"" << output_topic_ << "\",";
    ss << "\"ransac_min_correspondences\":" << ransac_min_correspondences_ << ",";
    ss << "\"merged_points\":" << merged_points << ",";
    ss << "\"output_points\":" << output_points;
    ss << "}";

    auto msg = std_msgs::msg::String();
    msg.data = ss.str();
    status_pub_->publish(msg);
  }

  std::string head_topic_;
  std::string arm_topic_;
  std::string output_topic_;
  int ransac_min_correspondences_ = 8;
  int ransac_max_iterations_ = 2000;
  double ransac_max_correspondence_dist_m_ = 0.05;
  double downsample_leaf_m_ = 0.02;
  double fuse_voxel_leaf_m_ = 0.01;
  double max_cloud_age_s_ = 1.0;
  double max_rate_hz_ = 10.0;
  bool enable_ransac_fusion_ = true;
  double normal_radius_m_ = 0.05;
  double feature_radius_m_ = 0.10;

  sensor_msgs::msg::PointCloud2 latest_head_;
  sensor_msgs::msg::PointCloud2 latest_arm_;
  rclcpp::Time head_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time arm_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr head_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr arm_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RansacPointCloudFusionNode>());
  rclcpp::shutdown();
  return 0;
}
