std_msgs/Header header

int32 marker_id
string source_camera_frame
string marker_frame
string target_frame

# Pose of the target arm frame in marker_map.
# Pose covariance order is x, y, z, roll, pitch, yaw in marker_map.
geometry_msgs/PoseWithCovariance pose

bool hard_gate_passed
string hard_gate_status
bool stable
int32 stable_frames
float64 stability_factor

float64 reprojection_error_px
float64 distance_m
float64 view_angle_deg
float64 area_px2
float64 side_mean_px
float64 side_min_px
float64 geometry_score
float64 covariance_sigma_px

float64 head_pose_match_dt_s
string head_pose_match_mode
string head_pose_source_topic
string head_pose_source_type
float64 head_pose_time_offset_s
bool dynamic_covariance_fallback
bool head_covariance_fallback
string extrinsic_covariance_source
