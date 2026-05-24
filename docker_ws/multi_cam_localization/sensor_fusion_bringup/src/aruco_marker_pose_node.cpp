#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_fusion_msgs/msg/marker_pose_observation.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

// Quaternion xyzw from rotation matrix.
void rot_to_quat(const cv::Matx33d &R, double *qx, double *qy, double *qz, double *qw) {
  double tr = R(0, 0) + R(1, 1) + R(2, 2);
  if (tr > 0.0) {
    double s = std::sqrt(tr + 1.0) * 2.0;
    *qw = 0.25 * s;
    *qx = (R(2, 1) - R(1, 2)) / s;
    *qy = (R(0, 2) - R(2, 0)) / s;
    *qz = (R(1, 0) - R(0, 1)) / s;
  } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
    double s = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;
    *qw = (R(2, 1) - R(1, 2)) / s;
    *qx = 0.25 * s;
    *qy = (R(0, 1) + R(1, 0)) / s;
    *qz = (R(0, 2) + R(2, 0)) / s;
  } else if (R(1, 1) > R(2, 2)) {
    double s = std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;
    *qw = (R(0, 2) - R(2, 0)) / s;
    *qx = (R(0, 1) + R(1, 0)) / s;
    *qy = 0.25 * s;
    *qz = (R(1, 2) + R(2, 1)) / s;
  } else {
    double s = std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;
    *qw = (R(1, 0) - R(0, 1)) / s;
    *qx = (R(0, 2) + R(2, 0)) / s;
    *qy = (R(1, 2) + R(2, 1)) / s;
    *qz = 0.25 * s;
  }
}

// Rotation angle in degrees between two rotation matrices.
double rotation_angle_deg(const cv::Matx33d &R_a, const cv::Matx33d &R_b) {
  cv::Matx33d R = R_a * R_b.t();
  double tr = R(0, 0) + R(1, 1) + R(2, 2);
  double cos_a = std::max(-1.0, std::min(1.0, (tr - 1.0) * 0.5));
  return std::acos(cos_a) * kRadToDeg;
}

// Side length ratio and angle spread → geometry score.
double marker_geometry_score(const std::vector<cv::Point2f> &corners,
                             double min_angle_deg, double max_angle_deg) {
  if (corners.size() != 4) return 0.0;

  double side_lengths[4];
  double angles[4];
  for (int i = 0; i < 4; i++) {
    cv::Point2f p_prev = corners[(i + 3) % 4];
    cv::Point2f p = corners[i];
    cv::Point2f p_next = corners[(i + 1) % 4];

    cv::Point2f v1 = p_prev - p;
    cv::Point2f v2 = p_next - p;
    double n1 = std::sqrt(v1.x * v1.x + v1.y * v1.y);
    double n2 = std::sqrt(v2.x * v2.x + v2.y * v2.y);
    if (n1 <= 1e-6 || n2 <= 1e-6) return 0.0;

    double cos_ang = std::max(-1.0, std::min(1.0, (v1.x * v2.x + v1.y * v2.y) / (n1 * n2)));
    angles[i] = std::acos(cos_ang) * kRadToDeg;

    cv::Point2f d = p_next - p;
    side_lengths[i] = std::sqrt(d.x * d.x + d.y * d.y);
  }

  double min_angle = *std::min_element(angles, angles + 4);
  double max_angle_val = *std::max_element(angles, angles + 4);

  if (min_angle < min_angle_deg || max_angle_val > max_angle_deg) return 0.0;

  double min_side = *std::min_element(side_lengths, side_lengths + 4);
  double max_side = *std::max_element(side_lengths, side_lengths + 4);
  double side_ratio = min_side / std::max(max_side, 1e-6);
  double angle_spread = max_angle_val - min_angle;

  return std::max(0.0, std::min(1.0, 0.5 * side_ratio + 0.5 * (1.0 - angle_spread / 180.0)));
}

// Each 4x4 matrix stored as 16-element vector, row-major.
// T_A_B maps frame B into frame A.
using Matrix4d = std::array<double, 16>;
using Matrix3d = cv::Matx33d;
using Vector3d = cv::Vec3d;

Matrix4d eye4() {
  return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

Matrix3d R_from_T(const Matrix4d &T) {
  return Matrix3d(T[0], T[1], T[2],
                  T[4], T[5], T[6],
                  T[8], T[9], T[10]);
}

Vector3d t_from_T(const Matrix4d &T) {
  return Vector3d(T[3], T[7], T[11]);
}

void set_R(Matrix4d &T, const Matrix3d &R) {
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      T[r * 4 + c] = R(r, c);
}

void set_t(Matrix4d &T, const Vector3d &t) {
  T[3] = t[0]; T[7] = t[1]; T[11] = t[2];
}

Matrix4d T_mul(const Matrix4d &A, const Matrix4d &B) {
  Matrix4d C = {0};
  for (int r = 0; r < 4; r++)
    for (int c = 0; c < 4; c++)
      for (int k = 0; k < 4; k++)
        C[r * 4 + c] += A[r * 4 + k] * B[k * 4 + c];
  return C;
}

Matrix4d T_inv(const Matrix4d &T) {
  Matrix3d R = R_from_T(T).t();
  Vector3d t = T_inv_translation(T);
  Matrix4d Inv = eye4();
  set_R(Inv, R);
  set_t(Inv, t);
  return Inv;
}

Vector3d T_inv_translation(const Matrix4d &T) {
  Matrix3d Rt = R_from_T(T).t();
  Vector3d t = t_from_T(T);
  return Vector3d(-(Rt(0, 0) * t[0] + Rt(0, 1) * t[1] + Rt(0, 2) * t[2]),
                   -(Rt(1, 0) * t[0] + Rt(1, 1) * t[1] + Rt(1, 2) * t[2]),
                   -(Rt(2, 0) * t[0] + Rt(2, 1) * t[1] + Rt(2, 2) * t[2]));
}

double translation_norm(const Matrix4d &T) {
  double dx = T[3], dy = T[7], dz = T[11];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Build T_cam_marker from OpenCV solvePnP output.
Matrix4d build_T_cam_marker(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
  Matrix4d T = eye4();
  cv::Matx33d R;
  cv::Rodrigues(rvec, R);
  set_R(T, R);
  set_t(T, tvec);
  return T;
}

// Marker object points (centered, x-right, y-down in marker plane).
std::vector<cv::Point3f> marker_object_points(double size_m) {
  double h = size_m / 2.0;
  return {
    cv::Point3f(-h, -h, 0.f),
    cv::Point3f( h, -h, 0.f),
    cv::Point3f( h,  h, 0.f),
    cv::Point3f(-h,  h, 0.f),
  };
}

double compute_reprojection_error(const std::vector<cv::Point3f> &obj_points,
                                  const std::vector<cv::Point2f> &img_points,
                                  const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                                  const cv::Matx33d &K, const cv::Mat &D) {
  std::vector<cv::Point2f> projected;
  cv::projectPoints(obj_points, rvec, tvec, K, D, projected);
  double sum_sq = 0.0;
  for (size_t i = 0; i < 4; i++) {
    double dx = projected[i].x - img_points[i].x;
    double dy = projected[i].y - img_points[i].y;
    sum_sq += dx * dx + dy * dy;
  }
  return std::sqrt(sum_sq / 4.0);
}

double marker_view_angle_deg(const Matrix4d &T_cam_marker) {
  Matrix3d R_cm = R_from_T(T_cam_marker);
  Vector3d normal = Vector3d(R_cm(0, 2), R_cm(1, 2), R_cm(2, 2));
  double n = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
  if (n <= 1e-9) return 90.0;
  double cos_a = std::abs(normal[2] / n);
  cos_a = std::max(-1.0, std::min(1.0, cos_a));
  return std::acos(cos_a) * kRadToDeg;
}

// Convert 36-element covariance row-major list to diagonal-only form.
std::array<double, 36> covariance_from_diag(double x, double y, double z,
                                             double rx, double ry, double rz) {
  std::array<double, 36> cov{};
  cov[0] = x; cov[7] = y; cov[13] = z;
  cov[21] = rx; cov[28] = ry; cov[35] = rz;
  return cov;
}

bool is_inside_image(const std::vector<cv::Point2f> &corners, int w, int h, double margin) {
  for (const auto &p : corners) {
    if (p.x < margin || p.x > w - margin || p.y < margin || p.y > h - margin)
      return false;
  }
  return true;
}

} // namespace

class ArucoMarkerPoseCppNode : public rclcpp::Node {
public:
  ArucoMarkerPoseCppNode()
      : Node("aruco_marker_pose_cpp_node"), camera_info_received_(false),
        K_(cv::Matx33d::eye()), D_(cv::Mat::zeros(5, 1, CV_64F)) {
    using namespace std::placeholders;

    // ── Topics ──────────────────────────────────────────────────────────
    const std::string image_topic = declare_parameter("image_topic",
        "/head/d435i_head/color/image_raw");
    const std::string camera_info_topic = declare_parameter("camera_info_topic",
        "/head/d435i_head/color/camera_info");
    const std::string output_prefix = declare_parameter("output_topic_prefix",
        "/head/marker_pose");
    output_topic_ = declare_parameter("output_topic",
        output_prefix + "/observation");

    // ── Frame / marker identity ─────────────────────────────────────────
    target_frame_ = declare_parameter("target_frame", "head_imu");
    marker_map_frame_ = declare_parameter("marker_map_frame", "marker_map");
    camera_frame_ = declare_parameter("camera_frame", "head_d435i_head_color_optical_frame");
    marker_id_ = declare_parameter("marker_id", 0);
    marker_frame_ = declare_parameter("marker_frame", "marker_0");
    marker_size_m_ = declare_parameter("marker_size_m", 0.100);

    // ArUco dictionary
    const std::string dict_name = declare_parameter("marker_dictionary", "DICT_6X6_1000");
    int dict_id = cv::aruco::DICT_6X6_1000;
    if (dict_name == "DICT_4X4_1000") dict_id = cv::aruco::DICT_4X4_1000;
    else if (dict_name == "DICT_5X5_1000") dict_id = cv::aruco::DICT_5X5_1000;
    else if (dict_name == "DICT_7X7_1000") dict_id = cv::aruco::DICT_7X7_1000;
    else if (dict_name == "DICT_ARUCO_ORIGINAL") dict_id = cv::aruco::DICT_ARUCO_ORIGINAL;
    aruco_dict_ = cv::aruco::getPredefinedDictionary(dict_id);

    // ── Extrinsic transforms ────────────────────────────────────────────
    T_cam_imu_ = load_tf_param("T_cam_imu", eye4());
    T_map_marker_ = load_tf_param("T_map_marker", eye4());

    // Derive T_map_cam from marker via T_cam_imu so we can compute T_map_imu later.
    // We'll compute at runtime: T_map_imu = T_map_marker * inv(T_cam_marker) * T_cam_imu

    // ── Quality gates ───────────────────────────────────────────────────
    min_marker_area_px2_ = declare_parameter("min_marker_area_px2", 800.0);
    max_reprojection_error_px_ = declare_parameter("max_reprojection_error_px", 3.0);
    max_marker_distance_m_ = declare_parameter("max_marker_distance_m", 2.0);
    border_margin_px_ = declare_parameter("border_margin_px", 8.0);
    min_corner_angle_deg_ = declare_parameter("min_corner_angle_deg", 25.0);
    max_corner_angle_deg_ = declare_parameter("max_corner_angle_deg", 155.0);
    min_border_geometry_score_ = declare_parameter("min_border_geometry_score", 0.35);
    stable_frames_required_ = declare_parameter("stable_frames_required", 8);
    max_marker_translation_jump_m_ = declare_parameter("max_marker_translation_jump_m", 0.20);
    max_marker_rotation_jump_deg_ = declare_parameter("max_marker_rotation_jump_deg", 15.0);

    // ── Covariance model (simplified fixed model) ───────────────────────
    cov_xy_std_m_ = declare_parameter("covariance_std_translation_xy", 0.01);
    cov_z_std_m_ = declare_parameter("covariance_std_translation_z", 0.02);
    cov_roll_pitch_std_deg_ = declare_parameter("covariance_std_rotation_rp_deg", 2.0);
    cov_yaw_std_deg_ = declare_parameter("covariance_std_rotation_yaw_deg", 3.0);

    // ── Rate limiting ──────────────────────────────────────────────────
    marker_detection_rate_hz_ = declare_parameter("marker_detection_rate_hz", 0.0);

    // ── Subscriptions ──────────────────────────────────────────────────
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    auto reliable_qos = rclcpp::QoS(10).reliable();

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, sensor_qos,
        std::bind(&ArucoMarkerPoseCppNode::image_callback, this, _1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, reliable_qos,
        std::bind(&ArucoMarkerPoseCppNode::camera_info_callback, this, _1));

    // ── Publishers ─────────────────────────────────────────────────────
    observation_pub_ = create_publisher<sensor_fusion_msgs::msg::MarkerPoseObservation>(
        output_topic_, 10);
    marker_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
        output_prefix + "/marker_valid", 10);
    active_marker_pub_ = create_publisher<std_msgs::msg::Int32>(
        output_prefix + "/active_marker_id", 10);
    diag_pub_ = create_publisher<std_msgs::msg::String>(
        output_prefix + "/diagnostics", 10);

    // ── Info logging ──────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "C++ ArUco marker pose node started");
    RCLCPP_INFO(get_logger(), "  image: %s", image_topic.c_str());
    RCLCPP_INFO(get_logger(), "  camera_info: %s", camera_info_topic.c_str());
    RCLCPP_INFO(get_logger(), "  output: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  marker_id: %d, size: %.3f m, dict: %s",
                marker_id_, marker_size_m_, dict_name.c_str());
    RCLCPP_INFO(get_logger(), "  target_frame: %s, map_frame: %s",
                target_frame_.c_str(), marker_map_frame_.c_str());
  }

private:
  // ── Parameters ──────────────────────────────────────────────────────
  std::string output_topic_;
  std::string target_frame_;
  std::string marker_map_frame_;
  std::string camera_frame_;
  int marker_id_ = 0;
  std::string marker_frame_;
  double marker_size_m_ = 0.100;
  Matrix4d T_cam_imu_;
  Matrix4d T_map_marker_;
  double min_marker_area_px2_ = 800.0;
  double max_reprojection_error_px_ = 3.0;
  double max_marker_distance_m_ = 2.0;
  double border_margin_px_ = 8.0;
  double min_corner_angle_deg_ = 25.0;
  double max_corner_angle_deg_ = 155.0;
  double min_border_geometry_score_ = 0.35;
  int stable_frames_required_ = 8;
  double max_marker_translation_jump_m_ = 0.20;
  double max_marker_rotation_jump_deg_ = 15.0;
  double cov_xy_std_m_ = 0.01;
  double cov_z_std_m_ = 0.02;
  double cov_roll_pitch_std_deg_ = 2.0;
  double cov_yaw_std_deg_ = 3.0;
  double marker_detection_rate_hz_ = 0.0;

  // ── State ───────────────────────────────────────────────────────────
  cv::aruco::Dictionary aruco_dict_;
  bool camera_info_received_;
  cv::Matx33d K_;
  cv::Mat D_;
  rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
  std::deque<Matrix4d> marker_history_;
  bool last_marker_valid_ = false;

  // ── Subscriptions / Publishers ──────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_fusion_msgs::msg::MarkerPoseObservation>::SharedPtr observation_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr marker_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr active_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_pub_;

  // ── Helpers ─────────────────────────────────────────────────────────
  Matrix4d load_tf_param(const std::string &name, const Matrix4d &fallback) {
    std::vector<double> flat;
    try {
      flat = declare_parameter(name, std::vector<double>{});
    } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
      return fallback;
    }
    if (flat.size() != 16) {
      RCLCPP_WARN(get_logger(), "%s: expected 16 values, got %zu, using identity", name.c_str(), flat.size());
      return fallback;
    }
    Matrix4d T;
    for (int i = 0; i < 16; i++) T[i] = flat[i];
    return T;
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (msg->k.size() < 9) {
      RCLCPP_WARN(get_logger(), "camera_info K has < 9 elements, skipping");
      return;
    }
    K_ = cv::Matx33d(msg->k[0], msg->k[1], msg->k[2],
                     msg->k[4], msg->k[5], msg->k[6],
                     msg->k[8], msg->k[9], msg->k[10]);

    size_t d_count = msg->d.size();
    if (d_count >= 5) {
      D_ = cv::Mat(5, 1, CV_64F);
      for (int i = 0; i < 5; i++) D_.at<double>(i, 0) = msg->d[i];
    } else if (d_count > 0) {
      D_ = cv::Mat(static_cast<int>(d_count), 1, CV_64F);
      for (size_t i = 0; i < d_count; i++) D_.at<double>(static_cast<int>(i), 0) = msg->d[i];
    } else {
      D_ = cv::Mat::zeros(5, 1, CV_64F);
    }

    if (!camera_info_received_) {
      camera_info_received_ = true;
      RCLCPP_INFO(get_logger(), "Camera intrinsics received: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                  K_(0, 0), K_(1, 1), K_(0, 2), K_(1, 2));
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!camera_info_received_) {
      publish_diagnostics(msg->header.stamp, false, -1,
                          "no_camera_info", "waiting for camera_info",
                          std::vector<int>{}, std::vector<int>{}, std::vector<std::string>{});
      return;
    }

    // Rate limiting
    rclcpp::Time stamp = msg->header.stamp;
    if (marker_detection_rate_hz_ > 0.0 && last_detection_time_.nanoseconds() > 0) {
      double dt = (stamp - last_detection_time_).seconds();
      if (dt > 0.0 && dt < 1.0 / marker_detection_rate_hz_) {
        return;
      }
    }

    // Convert image to grayscale
    cv::Mat gray;
    if (!image_to_gray(*msg, gray)) {
      publish_diagnostics(msg->header.stamp, false, -1,
                          "decode_error", "image decoding failed",
                          std::vector<int>{}, std::vector<int>{}, std::vector<std::string>{});
      return;
    }

    last_detection_time_ = stamp;

    // Detect ArUco markers
    std::vector<int> detected_ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::string> reject_reasons;
    std::vector<int> rejected_ids;

    cv::aruco::detectMarkers(gray, aruco_dict_, corners, detected_ids);

    if (detected_ids.empty()) {
      set_marker_valid(false, -1);
      publish_diagnostics(msg->header.stamp, false, -1,
                          "no_markers", "no markers detected",
                          detected_ids, rejected_ids, reject_reasons);
      return;
    }

    bool any_accepted = false;

    for (size_t i = 0; i < detected_ids.size(); i++) {
      int id = detected_ids[i];

      if (id != marker_id_) {
        reject_reasons.push_back("unknown_id_" + std::to_string(id));
        rejected_ids.push_back(id);
        continue;
      }

      if (std::count(detected_ids.begin(), detected_ids.end(), id) > 1) {
        reject_reasons.push_back("duplicate_id_" + std::to_string(id));
        rejected_ids.push_back(id);
        continue;
      }

      const auto &corner_pts = corners[i];
      if (corner_pts.size() != 4) {
        reject_reasons.push_back("bad_corners_id_" + std::to_string(id));
        rejected_ids.push_back(id);
        continue;
      }

      // Marker area check
      double area = std::abs(cv::contourArea(corner_pts));
      if (area < min_marker_area_px2_) {
        reject_reasons.push_back("area_too_small_id_" + std::to_string(id));
        rejected_ids.push_back(id);
        continue;
      }

      // Border check
      bool near_border = false;
      for (const auto &p : corner_pts) {
        if (p.x < border_margin_px_ || p.x > static_cast<double>(msg->width) - border_margin_px_ ||
            p.y < border_margin_px_ || p.y > static_cast<double>(msg->height) - border_margin_px_) {
          near_border = true;
          break;
        }
      }

      // Corner geometry
      double geo_score = marker_geometry_score(corner_pts, min_corner_angle_deg_, max_corner_angle_deg_);
      if (geo_score <= 0.0) {
        reject_reasons.push_back("bad_geometry_id_" + std::to_string(id));
        rejected_ids.push_back(id);
        continue;
      }

      if (near_border && geo_score < min_border_geometry_score_) {
        reject_reasons.push_back("near_border_poor_geometry_id_" + std::to_string(id));
        rejected_ids.push_back(id);
        continue;
      }

      // solvePnP
      auto obj_pts = marker_object_points(marker_size_m_);
      cv::Vec3d rvec, tvec;
      bool pnp_ok = cv::solvePnP(obj_pts, corner_pts, K_, D_, rvec, tvec,
                                  false, cv::SOLVEPNP_ITERATIVE);

      if (!pnp_ok) {
        reject_reasons.push_back("solvepnp_failed_id_" + std::to_string(id));
        rejected_ids.push_back(id);
        continue;
      }

      // Distance check
      double dist = std::sqrt(tvec[0] * tvec[0] + tvec[1] * tvec[1] + tvec[2] * tvec[2]);
      if (dist > max_marker_distance_m_) {
        reject_reasons.push_back("too_far_id_" + std::to_string(id));
        rejected_ids.push_back(id);
        continue;
      }

      // Reprojection error
      double reproj_err = compute_reprojection_error(obj_pts, corner_pts, rvec, tvec, K_, D_);
      if (reproj_err > max_reprojection_error_px_) {
        reject_reasons.push_back("reproj_high_id_" + std::to_string(id));
        rejected_ids.push_back(id);
        continue;
      }

      // Build T_cam_marker
      Matrix4d T_cam_marker = build_T_cam_marker(rvec, tvec);

      // View angle check (informational, not a hard gate)
      double view_angle = marker_view_angle_deg(T_cam_marker);

      // Compute T_map_imu = T_map_marker * inv(T_cam_marker) * T_cam_imu
      Matrix4d T_marker_cam = T_inv(T_cam_marker);
      Matrix4d T_map_cam = T_mul(T_map_marker_, T_marker_cam);
      Matrix4d T_map_imu = T_mul(T_map_cam, T_cam_imu_);

      // Pose jump check against history
      if (!marker_history_.empty()) {
        const auto &last_T = marker_history_.back();
        double trans_delta = std::sqrt(
            std::pow(T_map_imu[3] - last_T[3], 2) +
            std::pow(T_map_imu[7] - last_T[7], 2) +
            std::pow(T_map_imu[11] - last_T[11], 2));

        double rot_jump = rotation_angle_deg(R_from_T(T_map_imu),
                                             R_from_T(last_T));

        if (trans_delta > max_marker_translation_jump_m_) {
          reject_reasons.push_back("translation_jump_id_" + std::to_string(id));
          rejected_ids.push_back(id);
          continue;
        }
        if (rot_jump > max_marker_rotation_jump_deg_) {
          reject_reasons.push_back("rotation_jump_id_" + std::to_string(id));
          rejected_ids.push_back(id);
          continue;
        }
      }

      // Update history
      marker_history_.push_back(T_map_imu);
      int max_history = std::max(stable_frames_required_, 1);
      while (static_cast<int>(marker_history_.size()) > max_history) {
        marker_history_.pop_front();
      }

      int stable_frames = static_cast<int>(marker_history_.size());
      bool stable = stable_frames >= stable_frames_required_;
      double stability_factor = static_cast<double>(
          std::max(0, stable_frames_required_ - stable_frames)) /
          std::max(1, stable_frames_required_);

      // Build covariance (simplified: fixed model, rotated to map frame)
      double xy_var = cov_xy_std_m_ * cov_xy_std_m_;
      double z_var = cov_z_std_m_ * cov_z_std_m_;
      double rp_var = (cov_roll_pitch_std_deg_ * kDegToRad) * (cov_roll_pitch_std_deg_ * kDegToRad);
      double yaw_var = (cov_yaw_std_deg_ * kDegToRad) * (cov_yaw_std_deg_ * kDegToRad);
      auto cov = covariance_from_diag(xy_var, xy_var, z_var, rp_var, rp_var, yaw_var);

      // Build and publish observation
      auto obs = sensor_fusion_msgs::msg::MarkerPoseObservation();
      obs.header.stamp = msg->header.stamp;
      obs.header.frame_id = marker_map_frame_;
      obs.marker_id = marker_id_;
      obs.marker_frame = marker_frame_;
      obs.target_frame = target_frame_;

      // Fill pose
      Matrix3d R = R_from_T(T_map_imu);
      Vector3d t = t_from_T(T_map_imu);
      double qx, qy, qz, qw;
      rot_to_quat(R, &qx, &qy, &qz, &qw);

      obs.pose.pose.position.x = t[0];
      obs.pose.pose.position.y = t[1];
      obs.pose.pose.position.z = t[2];
      obs.pose.pose.orientation.x = qx;
      obs.pose.pose.orientation.y = qy;
      obs.pose.pose.orientation.z = qz;
      obs.pose.pose.orientation.w = qw;

      std::copy(cov.begin(), cov.end(), obs.pose.covariance.begin());

      obs.hard_gate_passed = true;
      obs.hard_gate_status = "accepted";
      obs.stable = stable;
      obs.stable_frames = stable_frames;
      obs.stability_factor = stability_factor;
      obs.reprojection_error_px = reproj_err;
      obs.distance_m = dist;
      obs.view_angle_deg = view_angle;
      obs.area_px2 = area;
      obs.side_mean_px = std::sqrt(area);
      obs.side_min_px = std::sqrt(area) * 0.9; // approximate
      obs.geometry_score = geo_score;
      obs.covariance_sigma_px = reproj_err;

      observation_pub_->publish(obs);
      set_marker_valid(true, marker_id_);
      any_accepted = true;

      // Publish diagnostics
      publish_diagnostics(msg->header.stamp, true, marker_id_,
                          "accepted", "",
                          detected_ids, rejected_ids, reject_reasons);
    }

    if (!any_accepted) {
      set_marker_valid(false, detected_ids.empty() ? -1 : detected_ids[0]);
      publish_diagnostics(msg->header.stamp, false, -1,
                          "all_rejected", "all markers rejected",
                          detected_ids, rejected_ids, reject_reasons);
    }
  }

  bool image_to_gray(const sensor_msgs::msg::Image &msg, cv::Mat &gray) {
    const std::string &enc = msg.encoding;
    int h = static_cast<int>(msg.height);
    int w = static_cast<int>(msg.width);
    int step = static_cast<int>(msg.step);
    const uint8_t *data = msg.data.data();
    size_t data_size = msg.data.size();

    std::string enc_lower;
    std::transform(enc.begin(), enc.end(), std::back_inserter(enc_lower),
                   [](char c) { return static_cast<char>(std::tolower(c)); });

    if (enc_lower == "bgr8" || enc_lower == "rgb8") {
      int channels = 3;
      if (static_cast<size_t>(h * step) > data_size) return false;
      cv::Mat color(h, w, CV_8UC3);
      for (int y = 0; y < h; y++) {
        const uint8_t *row = data + static_cast<size_t>(y) * static_cast<size_t>(step);
        std::memcpy(color.ptr(y), row, static_cast<size_t>(w) * 3);
      }
      if (enc_lower == "bgr8")
        cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
      else
        cv::cvtColor(color, gray, cv::COLOR_RGB2GRAY);
      return true;
    }

    if (enc_lower == "mono8" || enc_lower == "8uc1") {
      if (static_cast<size_t>(h * step) > data_size) return false;
      cv::Mat raw(h, w, CV_8UC1);
      for (int y = 0; y < h; y++) {
        const uint8_t *row = data + static_cast<size_t>(y) * static_cast<size_t>(step);
        std::memcpy(raw.ptr(y), row, static_cast<size_t>(w));
      }
      gray = raw;
      return true;
    }

    RCLCPP_WARN(get_logger(), "Unsupported image encoding: %s", enc.c_str());
    return false;
  }

  void set_marker_valid(bool valid, int marker_id) {
    {
      auto msg = std_msgs::msg::Bool();
      msg.data = valid;
      marker_valid_pub_->publish(msg);
    }
    {
      auto msg = std_msgs::msg::Int32();
      msg.data = marker_id;
      active_marker_pub_->publish(msg);
    }
    last_marker_valid_ = valid;
  }

  void publish_diagnostics(const rclcpp::Time &stamp,
                           bool accepted,
                           int best_id,
                           const std::string &reason,
                           const std::string &detail,
                           const std::vector<int> &detected_ids,
                           const std::vector<int> &rejected_ids,
                           const std::vector<std::string> &reject_reasons) {
    auto msg = std_msgs::msg::String();

    std::ostringstream ss;
    ss << "{";
    ss << "\"stamp\":" << std::fixed << stamp.seconds();
    ss << ",\"accepted\":" << (accepted ? "true" : "false");
    ss << ",\"reason\":\"" << reason << "\"";
    if (!detail.empty()) ss << ",\"detail\":\"" << detail << "\"";
    ss << ",\"best_marker_id\":" << best_id;

    ss << ",\"detected_ids\":[";
    for (size_t i = 0; i < detected_ids.size(); i++) {
      if (i > 0) ss << ",";
      ss << detected_ids[i];
    }
    ss << "]";

    ss << ",\"rejected_ids\":[";
    for (size_t i = 0; i < rejected_ids.size(); i++) {
      if (i > 0) ss << ",";
      ss << rejected_ids[i];
    }
    ss << "]";

    ss << ",\"reject_reasons\":[";
    for (size_t i = 0; i < reject_reasons.size(); i++) {
      if (i > 0) ss << ",";
      ss << "\"" << reject_reasons[i] << "\"";
    }
    ss << "]";

    ss << ",\"camera_info_ok\":" << (camera_info_received_ ? "true" : "false");
    ss << "}";

    msg.data = ss.str();
    diag_pub_->publish(msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoMarkerPoseCppNode>());
  rclcpp::shutdown();
  return 0;
}
