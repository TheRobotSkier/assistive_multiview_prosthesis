#!/usr/bin/env python3
"""
Cloud Snapshot Node — freezes the segmented object cloud when the pipeline
enters PLANNING/APPROACHING state.

This solves camera occlusion issues when the hand gets close to the object
during a grasp. The frozen cloud is published at a fixed rate so that RViz
display stays current and the preshaping service receives the frozen version.

Subscriptions:
  /pipeline/state            (std_msgs/Int32)          — pipeline state
  /segmentation/object_cloud (sensor_msgs/PointCloud2) — live segmented cloud

Publications:
  /segmentation/object_cloud_snapshot  (sensor_msgs/PointCloud2) — frozen or live
  /segmented_object_cloud              (sensor_msgs/PointCloud2) — frozen version for preshaping

Parameters:
  snapshot_on_state (int, default: 2)  — enter freeze mode on this state  (PLANNING)
  clear_on_state    (int, default: 0)  — exit  freeze mode on this state  (IDLE)
  publish_rate_hz   (float, default: 10.0) — timer publish rate

Pipeline states (from pipeline_manager_node.py):
  IDLE=0, SEGMENTING=1, PLANNING=2, APPROACHING=3,
  GRASPING=4, HOLDING=5, RELEASING=6
"""

import copy

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32

# Pipeline states (must match pipeline_manager_node.py State enum)
IDLE = 0
SEGMENTING = 1
PLANNING = 2
APPROACHING = 3
GRASPING = 4
HOLDING = 5
RELEASING = 6

_STATE_NAMES = {
    IDLE: "IDLE",
    SEGMENTING: "SEGMENTING",
    PLANNING: "PLANNING",
    APPROACHING: "APPROACHING",
    GRASPING: "GRASPING",
    HOLDING: "HOLDING",
    RELEASING: "RELEASING",
}


# ---------------------------------------------------------------------------
# PointCloud2 helpers
# ---------------------------------------------------------------------------

def _deep_copy_cloud(src: PointCloud2) -> PointCloud2:
    """Create a deep copy of a PointCloud2 message with an independent data buffer.

    The raw binary payload is copied through a numpy uint8 buffer so the
    original and the copy do not share memory.
    """
    dst = PointCloud2()
    dst.header = copy.deepcopy(src.header)
    dst.height = src.height
    dst.width = src.width
    dst.fields = copy.deepcopy(src.fields)
    dst.is_bigendian = src.is_bigendian
    dst.point_step = src.point_step
    dst.row_step = src.row_step
    dst.is_dense = src.is_dense
    # Deep-copy the raw binary data via numpy to guarantee independent memory.
    dst.data = np.frombuffer(bytes(src.data), dtype=np.uint8).copy().tobytes()
    return dst


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class CloudSnapshotNode(Node):
    """Freeze the object cloud during planning/approaching to avoid occlusion.

    Behaviour
    ---------
    * IDLE / SEGMENTING — forward the live cloud (pass-through).
    * Transition to PLANNING / APPROACHING — freeze :  deep-copy the latest
      object cloud and keep publishing that copy on a timer.
    * GRASPING / HOLDING — remain frozen.
    * Transition to RELEASING / IDLE — clear the snapshot and resume
      forwarding the live cloud.
    """

    def __init__(self):
        super().__init__("cloud_snapshot_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("snapshot_on_state", PLANNING)
        self.declare_parameter("clear_on_state", IDLE)
        self.declare_parameter("publish_rate_hz", 10.0)

        self._snapshot_on = self.get_parameter("snapshot_on_state").value
        self._clear_on = self.get_parameter("clear_on_state").value
        publish_rate = self.get_parameter("publish_rate_hz").value

        # ── Internal state ────────────────────────────────────────────────
        self._pipeline_state: int = IDLE
        self._live_cloud: PointCloud2 | None = None
        self._frozen_cloud: PointCloud2 | None = None
        self._frozen_mode: bool = False

        # ── Publishers ────────────────────────────────────────────────────
        self._pub_snapshot = self.create_publisher(
            PointCloud2, "/segmentation/object_cloud_snapshot", 10,
        )
        self._pub_preshape = self.create_publisher(
            PointCloud2, "/segmented_object_cloud", 10,
        )

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            Int32, "/pipeline/state", self._on_state, 10,
        )
        self.create_subscription(
            PointCloud2, "/segmentation/object_cloud", self._on_cloud, 10,
        )

        # ── Timer — periodic publish at fixed rate ────────────────────────
        self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f"Cloud snapshot node ready.  "
            f"Freeze on state={self._snapshot_on} "
            f"({_STATE_NAMES.get(self._snapshot_on, '?')}), "
            f"clear on state={self._clear_on} "
            f"({_STATE_NAMES.get(self._clear_on, '?')}), "
            f"publish rate={publish_rate} Hz"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_state(self, msg: Int32) -> None:
        """Handle pipeline state transitions.

        Freeze when entering *snapshot_on* or APPROACHING.
        Clear  when entering *clear_on*    or RELEASING.
        """
        new_state = msg.data
        was_frozen = self._frozen_mode
        self._pipeline_state = new_state

        should_freeze = new_state in (self._snapshot_on, APPROACHING)
        should_clear = new_state in (self._clear_on, RELEASING)

        if should_freeze and not was_frozen:
            self._take_snapshot(new_state)
        elif should_clear and was_frozen:
            self._clear_snapshot(new_state)

    def _on_cloud(self, msg: PointCloud2) -> None:
        """Keep the latest live cloud (used when not frozen)."""
        self._live_cloud = msg

    # ------------------------------------------------------------------
    # Snapshot management
    # ------------------------------------------------------------------

    def _take_snapshot(self, new_state: int) -> None:
        """Deep-copy the latest live cloud and enter freeze mode."""
        if self._live_cloud is None:
            self.get_logger().warn(
                f"Cannot freeze — no object cloud received yet.  "
                f"State={_STATE_NAMES.get(new_state, '?')}",
            )
            return

        self._frozen_cloud = _deep_copy_cloud(self._live_cloud)
        self._frozen_mode = True
        n_points = self._frozen_cloud.width * self._frozen_cloud.height
        self.get_logger().info(
            f"State {_STATE_NAMES.get(new_state, '?')}: "
            f"frozen object cloud snapshot ({n_points} points)",
        )

    def _clear_snapshot(self, new_state: int) -> None:
        """Exit freeze mode and discard the frozen cloud."""
        self._frozen_cloud = None
        self._frozen_mode = False
        self.get_logger().info(
            f"State {_STATE_NAMES.get(new_state, '?')}: "
            f"cleared snapshot, resuming live cloud forwarding",
        )

    # ------------------------------------------------------------------
    # Periodic publish (10 Hz)
    # ------------------------------------------------------------------

    def _publish(self) -> None:
        """Publish the appropriate cloud on both topics."""
        if self._frozen_mode and self._frozen_cloud is not None:
            cloud = self._frozen_cloud
        elif self._live_cloud is not None:
            cloud = self._live_cloud
        else:
            return  # nothing to publish yet

        # Update the timestamp so RViz treats the output as live.
        cloud.header.stamp = self.get_clock().now().to_msg()
        self._pub_snapshot.publish(cloud)
        self._pub_preshape.publish(cloud)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = CloudSnapshotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
