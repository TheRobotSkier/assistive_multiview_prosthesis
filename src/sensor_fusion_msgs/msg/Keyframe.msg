# A single keyframe returned by the keyframe buffer's GetKeyframesInROI service.
#
# Bundles the data TSDF fusion needs to reconstruct an object at grasp time:
# the point cloud, RGB image, camera pose, and intrinsics.
#
# Reference: V6 plan §6.2 (Keyframe dataclass).

# Unique timestamp of the keyframe (seconds).
builtin_interfaces/Time timestamp
# "head" or "arm".
string camera_id
# Point cloud (xyz + rgb) in the marker_map frame.
sensor_msgs/PointCloud2 cloud
# RGB image used for SAM segmentation.
sensor_msgs/Image image
# Camera pose T_marker_map_camera from GTSAM.
geometry_msgs/Pose pose
# Camera intrinsics.
sensor_msgs/CameraInfo camera_info
# True if the cloud is organised (height > 1).
bool organized
