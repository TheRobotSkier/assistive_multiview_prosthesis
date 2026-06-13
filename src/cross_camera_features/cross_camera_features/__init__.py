"""cross_camera_features — SIFT-based 3D-3D alignment between head and arm cameras.

The pure-logic core (:func:`cross_camera_features.sift_feature_node.match_and_align`)
has ZERO ROS imports so it can be unit-tested on the host with synthetic images.

The ROS node (:mod:`cross_camera_features.sift_feature_node`) syncs head+arm
image pairs, extracts SIFT features, looks up 3-D depth, runs Umeyama alignment,
and publishes ``/vis/head_arm_pose`` (PoseWithCovariance) consumed by the GTSAM
tracker.
"""
