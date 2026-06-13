"""tsdf_fusion — on-demand TSDF fusion triggered at grasp time.

Pure-logic core (:mod:`tsdf_fusion.tsdf_fusion_core`) has ZERO ROS imports so
it can be unit-tested on the host without ROS or Open3D-on-host issues.

The ROS node (:mod:`tsdf_fusion.tsdf_fusion_node`) is a thin wrapper that wires
the keyframe buffer service and MobileSAM HTTP server to the core.
"""
