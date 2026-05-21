#!/usr/bin/env python3
"""
TF Pipeline Diagnostics Node

Periodically checks critical TF chains after a startup delay and logs
one-line summaries so that `make camera-test` produces clear, actionable
output when the TF tree is broken.

Monitored chains:
  - marker_map -> head_d435i_head_depth_optical_frame  (OpenVINS head)
  - marker_map -> arm_d435i_arm_depth_optical_frame    (OpenVINS arm)
  - arm_d435i_arm_link -> palm_frame                     (camera mount root)
  - palm_frame -> grasp_contact_frame                    (camera mount internal)

Usage (standalone):
  ros2 run sensor_fusion_bringup tf_pipeline_diagnostics.py

Usage (launched from pipeline.launch.py with tf_diagnostics:=true)
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


class TFPipelineDiagnostics(Node):
    def __init__(self):
        super().__init__("tf_pipeline_diagnostics")

        self.declare_parameter("startup_delay_sec", 10.0)
        self.declare_parameter("check_period_sec", 10.0)
        self.declare_parameter("openvins_head_pair",
                               ["marker_map", "head_d435i_head_depth_optical_frame"])
        self.declare_parameter("openvins_arm_pair",
                               ["marker_map", "arm_d435i_arm_depth_optical_frame"])
        self.declare_parameter("mount_root_pair",
                               ["arm_d435i_arm_link", "palm_frame"])
        self.declare_parameter("mount_internal_pair",
                               ["palm_frame", "grasp_contact_frame"])

        self._startup_delay = self.get_parameter("startup_delay_sec").value
        self._check_period = self.get_parameter("check_period_sec").value
        self._ov_head = self.get_parameter("openvins_head_pair").value
        self._ov_arm = self.get_parameter("openvins_arm_pair").value
        self._mount_root = self.get_parameter("mount_root_pair").value
        self._mount_internal = self.get_parameter("mount_internal_pair").value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._startup_timer = self.create_timer(self._startup_delay, self._on_startup)
        self._check_timer = None
        self._started = False

        self.get_logger().info(
            f"TF diagnostics will start in {self._startup_delay:.1f}s, "
            f"then check every {self._check_period:.1f}s"
        )

    def _on_startup(self):
        if self._started:
            return
        self._started = True
        self.destroy_timer(self._startup_timer)
        self._check_timer = self.create_timer(self._check_period, self._check_all)
        self._check_all()

    def _can_transform(self, parent: str, child: str) -> bool:
        try:
            return self._tf_buffer.can_transform(
                parent, child, Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().debug(
                f"can_transform({parent}, {child}) threw: {e}"
            )
            return False

    def _check_all(self):
        ov_head_ok = self._can_transform(self._ov_head[0], self._ov_head[1])
        ov_arm_ok = self._can_transform(self._ov_arm[0], self._ov_arm[1])
        mount_root_ok = self._can_transform(self._mount_root[0], self._mount_root[1])
        mount_internal_ok = self._can_transform(
            self._mount_internal[0], self._mount_internal[1]
        )

        # Summarise in one line
        parts = []
        if ov_head_ok:
            parts.append("OpenVINS(head):OK")
        else:
            parts.append("OpenVINS(head):MISSING")
        if ov_arm_ok:
            parts.append("OpenVINS(arm):OK")
        else:
            parts.append("OpenVINS(arm):MISSING")
        if mount_root_ok:
            parts.append("mount_root:OK")
        else:
            parts.append("mount_root:MISSING")
        if mount_internal_ok:
            parts.append("mount_internal:OK")
        else:
            parts.append("mount_internal:MISSING")

        summary = " | ".join(parts)

        if mount_root_ok and mount_internal_ok and ov_head_ok and ov_arm_ok:
            self.get_logger().info(f"[TF-DIAG] All chains healthy — {summary}")
            return

        # Camera mount TFs are fine but OpenVINS is missing
        if mount_root_ok and mount_internal_ok and not (ov_head_ok and ov_arm_ok):
            self.get_logger().warn(
                f"[TF-DIAG] Camera mount TFs are OK, but OpenVINS chain is "
                f"disconnected ({summary}). "
                f"Check Jetson bridge connectivity and that OpenVINS is running."
            )
            return

        # OpenVINS is fine but camera mounts are missing
        if (ov_head_ok or ov_arm_ok) and not (mount_root_ok and mount_internal_ok):
            self.get_logger().warn(
                f"[TF-DIAG] OpenVINS chain is present, but camera mount TFs are "
                f"missing ({summary}). "
                f"Check that publish_camera_mounts.py is running and "
                f"mounts_link_frame is correct."
            )
            return

        # Everything is broken
        self.get_logger().warn(
            f"[TF-DIAG] Multiple TF chains missing ({summary}). "
            f"Check OpenVINS bridge, camera mount publisher, and TF static latch."
        )


def main():
    rclpy.init()
    node = TFPipelineDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
