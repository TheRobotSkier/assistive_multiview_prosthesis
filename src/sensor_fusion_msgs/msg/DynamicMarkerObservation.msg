std_msgs/Header header

int32 marker_id
string marker_frame
string camera_frame

# Pose of the marker in the observing camera optical frame.
# Transform convention: T_camera_marker maps marker-frame points into camera-frame points.
# Covariance order is x, y, z, roll, pitch, yaw in the camera optical frame.
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
