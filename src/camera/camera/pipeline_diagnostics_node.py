#!/usr/bin/env python3
"""pipeline_diagnostics_node — live cross-node TF and topic diagnostics.

A passive observer that watches the TF tree and key topics from the outside,
logging periodic structured summaries that correlate signals no single node
can see.  Designed to run alongside the normal pipeline so you get richer
logs while recording a rosbag.

Monitors:
  1. TF jump detection — per-edge consecutive-delta tracking with jump
     counts and magnitudes.  Flags any edge that moves more than a
     configurable threshold between consecutive broadcasts.
  2. TF existence + staleness — which edges exist in the buffer and how
     stale each is.  Catches "TF went silent" instantly.
  3. Topic rate monitor — rolling Hz for critical topics (odom, clouds,
     poses, visual factors).
  4. Pose divergence tracker — translation norm over time for odom and
     GTSAM poses, with threshold warnings.
  5. Clock offset — sensor stamp vs host clock per topic.
  6. TF chain connectivity — checks marker_map -> depth_optical_frame
     chains are connected.
  7. Pointcloud chain health — per-side stage rates (compressed → local
     raw → points) that localise where the drop happens when points are
     absent despite depth/rgb traffic arriving.

The node only *reads* — it subscribes to /tf, /tf_static, and key topics
but never publishes anything that could perturb the system.

Usage:
  ros2 run camera pipeline_diagnostics_node
  ros2 launch prosthesis_launch pipeline.launch.py debug_monitor:=true

Reference: V6 diagnostics, post-run analysis 2026-06-16.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PoseWithCovarianceStamped


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class EdgeStats:
    """Per-edge (parent, child) tracking for jump detection."""
    parent: str
    child: str
    last_translation: Optional[tuple] = None  # (x, y, z)
    last_rotation: Optional[tuple] = None     # (x, y, z, w)
    last_stamp: float = 0.0
    update_count: int = 0
    jump_count: int = 0
    max_jump_m: float = 0.0
    recent_jumps: deque = field(default_factory=lambda: deque(maxlen=10))

    def update(self, t, stamp_f: float, jump_threshold_m: float):
        """Record a new transform value and detect jumps."""
        self.update_count += 1
        new_t = (t.transform.translation.x,
                 t.transform.translation.y,
                 t.transform.translation.z)

        if self.last_translation is not None:
            dx = new_t[0] - self.last_translation[0]
            dy = new_t[1] - self.last_translation[1]
            dz = new_t[2] - self.last_translation[2]
            jump = math.sqrt(dx * dx + dy * dy + dz * dz)
            if jump > jump_threshold_m:
                self.jump_count += 1
                self.recent_jumps.append((stamp_f, jump))
                if jump > self.max_jump_m:
                    self.max_jump_m = jump

        self.last_translation = new_t
        self.last_rotation = (
            t.transform.rotation.x, t.transform.rotation.y,
            t.transform.rotation.z, t.transform.rotation.w)
        self.last_stamp = stamp_f


@dataclass
class TopicStats:
    """Rolling rate tracker for a single topic."""
    msg_count: int = 0
    stamps: deque = field(default_factory=lambda: deque(maxlen=200))
    # Host arrival times — used for stall detection independent of sensor
    # clock drift (cloud delivery stalls are a host-side / DDS symptom).
    arrival_times: deque = field(default_factory=lambda: deque(maxlen=200))
    # Snapshot of msg_count at the last periodic summary, used to detect
    # plateaus (delivery stops while the topic was previously flowing).
    last_summary_count: int = 0
    last_summary_host_time: float = 0.0

    def on_msg(self, stamp_f: float, host_time: float = 0.0):
        self.msg_count += 1
        self.stamps.append(stamp_f)
        if host_time > 0.0:
            self.arrival_times.append(host_time)

    def rate_hz(self, window_s: float = 5.0) -> float:
        if len(self.stamps) < 2:
            return 0.0
        now = self.stamps[-1]
        cutoff = now - window_s
        recent = [s for s in self.stamps if s >= cutoff]
        if len(recent) < 2:
            return 0.0
        dt = recent[-1] - recent[0]
        if dt <= 0:
            return 0.0
        return (len(recent) - 1) / dt

    def is_stalled(self, host_now: float, stall_threshold_s: float) -> bool:
        """Return True if the topic has gone silent.

        A topic is "stalled" when it has received at least one message before
        but nothing in the last *stall_threshold_s* seconds (by host arrival
        time).  This catches the cloud-delivery plateau where the Jetson stops
        delivering clouds mid-run.
        """
        if self.msg_count == 0:
            return False  # never started — not a stall
        if not self.arrival_times:
            return False
        return (host_now - self.arrival_times[-1]) > stall_threshold_s

    def is_plateaued(self, host_now: float, plateau_window_s: float) -> bool:
        """Return True if msg_count has not increased since the last summary.

        Unlike :meth:`is_stalled` (which uses arrival-time of the last msg),
        this compares the *total count* between successive summaries — robust
        even when arrival timestamps are unreliable.
        """
        if self.last_summary_count == 0:
            return False  # nothing to compare against yet
        if self.last_summary_host_time <= 0:
            return False
        elapsed = host_now - self.last_summary_host_time
        if elapsed < plateau_window_s:
            return False
        return self.msg_count == self.last_summary_count


# ---------------------------------------------------------------------------
# Default parameters
# ---------------------------------------------------------------------------

DEFAULT_PARAMS = {
    # Periodic summary interval (seconds)
    "summary_interval_s": 5.0,
    # TF jump threshold (meters) — flag edges that jump more than this
    "tf_jump_threshold_m": 0.20,
    # Pose norm warning thresholds (meters)
    "pose_norm_warn_m": 1.0,
    "pose_norm_critical_m": 2.0,
    # Clock offset warning (seconds)
    "clock_offset_warn_s": 0.15,
    # Rate window for Hz calculation
    "rate_window_s": 5.0,
    # Cloud delivery stall detection — warn when a topic that was flowing
    # goes silent for this many seconds (host arrival time).  Catches the
    # Jetson-side cloud plateau (60 clouds then stalls).
    "cloud_stall_threshold_s": 5.0,
    # Cloud delivery plateau window — flag when msg_count hasn't increased
    # between summaries spaced this far apart.
    "cloud_plateau_window_s": 4.0,
    # Topics to monitor for rate + clock offset
    "head_odom_topic": "/jetson/head/odom",
    "arm_odom_topic": "/jetson/arm/odom",
    "head_cloud_topic": "/jetson/head/points",
    "arm_cloud_topic": "/jetson/arm/points",
    "head_pose_topic": "/gtsam/head_pose",
    "arm_pose_topic": "/gtsam/arm_pose",
    "visual_factor_topic": "/vis/head_arm_pose",
    "head_image_topic": "/jetson/head/image",
    "arm_image_topic": "/jetson/arm/image",
    # ── Pointcloud chain stage topics ─────────────────────────────
    # Full chain: compressed (wire) → local raw (decompressed) → points.
    # Monitoring every stage localises where the drop happens.
    "head_depth_compressed_topic": "/jetson/head/depth/compressed",
    "arm_depth_compressed_topic": "/jetson/arm/depth/compressed",
    "head_image_compressed_topic": "/jetson/head/image/compressed",
    "arm_image_compressed_topic": "/jetson/arm/image/compressed",
    "head_depth_raw_topic": "/local/head/depth_raw",
    "arm_depth_raw_topic": "/local/arm/depth_raw",
    "head_image_raw_topic": "/local/head/image_raw",
    "arm_image_raw_topic": "/local/arm/image_raw",
    # Camera_info is a critical input to the depth_image_proc
    # ApproximateTimeSynchronizer but was previously unmonitored.
    "head_camera_info_raw_topic": "/local/head/camera_info",
    "arm_camera_info_raw_topic": "/local/arm/camera_info",
    # Seconds with input present but zero points output before flagging
    "pc_stall_flag_s": 5.0,
}


def create_node():
    """Factory for testability (mirrors other nodes in the codebase)."""

    class PipelineDiagnosticsNode(Node):
        """Live cross-node diagnostics observer."""

        def __init__(self):
            super().__init__("pipeline_diagnostics")

            # ── Declare parameters ─────────────────────────────────────
            for key, default in DEFAULT_PARAMS.items():
                self.declare_parameter(key, default)

            p = lambda k: self.get_parameter(k).value  # noqa: E731

            self._summary_interval = float(p("summary_interval_s"))
            self._jump_threshold = float(p("tf_jump_threshold_m"))
            self._norm_warn = float(p("pose_norm_warn_m"))
            self._norm_critical = float(p("pose_norm_critical_m"))
            self._clock_warn = float(p("clock_offset_warn_s"))
            self._rate_window = float(p("rate_window_s"))
            self._stall_threshold = float(p("cloud_stall_threshold_s"))
            self._plateau_window = float(p("cloud_plateau_window_s"))
            self._pc_stall_flag_s = float(p("pc_stall_flag_s"))

            # ── TF monitoring ──────────────────────────────────────────
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._edge_stats: dict[str, EdgeStats] = {}
            self._static_edges_seen: set[str] = set()

            best_effort = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=100,
            )

            # Subscribe to /tf and /tf_static directly to catch every
            # broadcast (the Buffer may coalesce/drop under load).
            self.create_subscription(
                TFMessage, "/tf", self._on_tf, best_effort)
            self.create_subscription(
                TFMessage, "/tf_static", self._on_tf_static, best_effort)

            # ── Topic rate monitors ────────────────────────────────────
            self._topic_stats: dict[str, TopicStats] = {}

            odom_topics = [
                str(p("head_odom_topic")),
                str(p("arm_odom_topic")),
            ]
            cloud_topics = [
                str(p("head_cloud_topic")),
                str(p("arm_cloud_topic")),
            ]
            pose_topics = [
                str(p("head_pose_topic")),
                str(p("arm_pose_topic")),
            ]
            image_topics = [
                str(p("head_image_topic")),
                str(p("arm_image_topic")),
            ]
            visual_topic = str(p("visual_factor_topic"))

            # Odom — also tracked for pose norm divergence
            self._odom_norms: dict[str, deque] = {
                t: deque(maxlen=500) for t in odom_topics
            }
            for topic in odom_topics:
                self._topic_stats[topic] = TopicStats()
                self.create_subscription(
                    Odometry, topic,
                    lambda msg, t=topic: self._on_odom(msg, t),
                    best_effort)

            # Cloud — rate only
            for topic in cloud_topics:
                self._topic_stats[topic] = TopicStats()
                self.create_subscription(
                    PointCloud2, topic,
                    lambda msg, t=topic: self._on_generic(msg, t),
                    best_effort)

            # GTSAM poses — rate + norm
            self._pose_norms: dict[str, deque] = {
                t: deque(maxlen=500) for t in pose_topics
            }
            for topic in pose_topics:
                self._topic_stats[topic] = TopicStats()
                self.create_subscription(
                    PoseWithCovarianceStamped, topic,
                    lambda msg, t=topic: self._on_pose(msg, t),
                    best_effort)

            # Visual factors — rate only
            self._topic_stats[visual_topic] = TopicStats()
            self.create_subscription(
                PoseWithCovarianceStamped, visual_topic,
                lambda msg, t=visual_topic: self._on_generic(msg, t),
                best_effort)

            # Images — rate only
            for topic in image_topics:
                self._topic_stats[topic] = TopicStats()
                self.create_subscription(
                    Image, topic,
                    lambda msg, t=topic: self._on_generic(msg, t),
                    best_effort)

            # ── Pointcloud chain stage monitors ────────────────────────
            # Track every stage so the [DIAG-PC] block can localise where
            # the drop happens: compressed → local raw → points.
            # Camera_info is included because the C++ 4-way synchronizer
            # requires depth + rgb + camera_info; a missing camera_info
            # stream produces zero points with no other symptom.
            from sensor_msgs.msg import CompressedImage, CameraInfo
            pc_chain_topics = [
                str(p("head_depth_compressed_topic")),
                str(p("arm_depth_compressed_topic")),
                str(p("head_image_compressed_topic")),
                str(p("arm_image_compressed_topic")),
                str(p("head_depth_raw_topic")),
                str(p("arm_depth_raw_topic")),
                str(p("head_image_raw_topic")),
                str(p("arm_image_raw_topic")),
                str(p("head_camera_info_raw_topic")),
                str(p("arm_camera_info_raw_topic")),
            ]
            self._pc_chain_topics = pc_chain_topics
            for topic in pc_chain_topics:
                self._topic_stats[topic] = TopicStats()
                # Compressed topics use CompressedImage; camera_info uses
                # CameraInfo; local raw images use Image.
                if topic.endswith("/compressed"):
                    msg_type = CompressedImage
                elif topic.endswith("/camera_info"):
                    msg_type = CameraInfo
                else:
                    msg_type = Image
                self.create_subscription(
                    msg_type, topic,
                    lambda msg, t=topic: self._on_generic(msg, t),
                    best_effort)
            # Convenience refs for the [DIAG-PC] block
            self._pc_topics = {
                "head": {
                    "depth_compressed": str(p("head_depth_compressed_topic")),
                    "image_compressed": str(p("head_image_compressed_topic")),
                    "depth_raw": str(p("head_depth_raw_topic")),
                    "image_raw": str(p("head_image_raw_topic")),
                    "camera_info": str(p("head_camera_info_raw_topic")),
                    "points": str(p("head_cloud_topic")),
                },
                "arm": {
                    "depth_compressed": str(p("arm_depth_compressed_topic")),
                    "image_compressed": str(p("arm_image_compressed_topic")),
                    "depth_raw": str(p("arm_depth_raw_topic")),
                    "image_raw": str(p("arm_image_raw_topic")),
                    "camera_info": str(p("arm_camera_info_raw_topic")),
                    "points": str(p("arm_cloud_topic")),
                },
            }

            # ── Periodic timer ─────────────────────────────────────────
            self.create_timer(self._summary_interval, self._periodic_summary)

            self.get_logger().info(
                f"PipelineDiagnostics ready (summary_interval="
                f"{self._summary_interval}s, jump_threshold="
                f"{self._jump_threshold}m)")

        # ── TF callbacks ──────────────────────────────────────────────

        def _on_tf(self, msg: TFMessage):
            host_now = time.time()
            for t in msg.transforms:
                key = f"{t.header.frame_id} -> {t.child_frame_id}"
                stamp_f = (t.header.stamp.sec +
                           t.header.stamp.nanosec * 1e-9)
                if key not in self._edge_stats:
                    self._edge_stats[key] = EdgeStats(
                        parent=t.header.frame_id,
                        child=t.child_frame_id)
                edge = self._edge_stats[key]
                edge.update(t, stamp_f, self._jump_threshold)

                # Immediate warn on large jumps (>0.3m)
                if edge.recent_jumps and edge.recent_jumps[-1][1] > 0.3:
                    jump_m = edge.recent_jumps[-1][1]
                    self.get_logger().warn(
                        f"TF JUMP: {key} jumped {jump_m:.3f}m "
                        f"(stamp={stamp_f:.3f}, age={host_now - stamp_f:.3f}s)"
                    )

        def _on_tf_static(self, msg: TFMessage):
            for t in msg.transforms:
                key = f"{t.header.frame_id} -> {t.child_frame_id}"
                self._static_edges_seen.add(key)

        # ── Topic callbacks ────────────────────────────────────────────

        def _stamp_to_float(self, header_stamp) -> float:
            return header_stamp.sec + header_stamp.nanosec * 1e-9

        def _on_odom(self, msg: Odometry, topic: str):
            stamp_f = self._stamp_to_float(msg.header.stamp)
            self._topic_stats[topic].on_msg(stamp_f, time.time())
            # Track pose norm
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            pz = msg.pose.pose.position.z
            norm = math.sqrt(px * px + py * py + pz * pz)
            self._odom_norms[topic].append((stamp_f, norm))

            # Immediate warn on critical divergence
            if norm > self._norm_critical:
                self.get_logger().warn(
                    f"DIVERGENCE: {topic} norm={norm:.2f}m "
                    f"(>{self._norm_critical}m)",
                    throttle_duration_sec=5.0)

        def _on_pose(self, msg: PoseWithCovarianceStamped, topic: str):
            stamp_f = self._stamp_to_float(msg.header.stamp)
            self._topic_stats[topic].on_msg(stamp_f, time.time())
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            pz = msg.pose.pose.position.z
            norm = math.sqrt(px * px + py * py + pz * pz)
            self._pose_norms[topic].append((stamp_f, norm))

        def _on_generic(self, msg, topic: str):
            """Rate-only tracking for topics where we just need Hz."""
            stamp_f = self._stamp_to_float(msg.header.stamp)
            self._topic_stats[topic].on_msg(stamp_f, time.time())

        # ── Periodic summary ───────────────────────────────────────────

        def _periodic_summary(self):
            """Log a structured multi-line summary of all diagnostics."""
            host_now = time.time()
            lines = []

            # ── TF edges: existence, staleness, jumps ──────────────────
            lines.append("-" * 60)
            lines.append("[DIAG-TF] Dynamic TF edges:")
            if not self._edge_stats:
                lines.append("  (none seen yet)")
            for key in sorted(self._edge_stats.keys()):
                e = self._edge_stats[key]
                age = host_now - e.last_stamp if e.last_stamp > 0 else -1
                status = "OK"
                if age > 3.0:
                    status = "STALE"
                elif e.jump_count > 0:
                    status = "JUMPING"
                lines.append(
                    f"  {key}: updates={e.update_count} "
                    f"jumps={e.jump_count} max_jump={e.max_jump_m:.3f}m "
                    f"age={age:.1f}s [{status}]")

            lines.append("[DIAG-TF] Static TF edges:")
            if not self._static_edges_seen:
                lines.append("  (none seen yet)")
            else:
                for key in sorted(self._static_edges_seen):
                    lines.append(f"  {key}")

            # ── Topic rates + stall/plateau detection ────────────────
            lines.append("[DIAG-RATE] Topic rates:")
            stalled_topics = []
            plateaued_topics = []
            for topic in sorted(self._topic_stats.keys()):
                ts = self._topic_stats[topic]
                rate = ts.rate_hz(self._rate_window)
                total = ts.msg_count
                # Clock offset (latest stamp vs host)
                clock_off = -1.0
                if ts.stamps:
                    clock_off = host_now - ts.stamps[-1]
                clock_warn = "!" if clock_off > self._clock_warn else ""
                # Stall / plateau detection
                status_tag = ""
                if ts.is_stalled(host_now, self._stall_threshold):
                    status_tag = " [STALLED]"
                    stalled_topics.append(topic)
                elif ts.is_plateaued(host_now, self._plateau_window):
                    status_tag = " [PLATEAUED]"
                    plateaued_topics.append(topic)
                lines.append(
                    f"  {topic}: {rate:.1f} Hz "
                    f"(total={total}, clock_off={clock_off:.3f}s{clock_warn})"
                    f"{status_tag}")

            # ── Cloud delivery stall / plateau warnings ────────────────
            if stalled_topics or plateaued_topics:
                lines.append("[DIAG-STALL] Cloud delivery issues:")
                for topic in stalled_topics:
                    ts = self._topic_stats[topic]
                    last_age = (host_now - ts.arrival_times[-1]
                                if ts.arrival_times else -1.0)
                    lines.append(
                        f"  {topic}: STALLED — no msgs for "
                        f"{last_age:.1f}s (total={ts.msg_count})")
                    self.get_logger().warn(
                        f"CLOUD STALL: {topic} stopped delivering "
                        f"({last_age:.1f}s since last msg, "
                        f"total={ts.msg_count})",
                        throttle_duration_sec=self._summary_interval)
                for topic in plateaued_topics:
                    ts = self._topic_stats[topic]
                    lines.append(
                        f"  {topic}: PLATEAUED — msg_count stuck at "
                        f"{ts.msg_count}")
                    self.get_logger().warn(
                        f"CLOUD PLATEAU: {topic} msg_count stuck at "
                        f"{ts.msg_count}",
                        throttle_duration_sec=self._summary_interval)

            # Snapshot counts + time for the next plateau comparison.
            for ts in self._topic_stats.values():
                ts.last_summary_count = ts.msg_count
                ts.last_summary_host_time = host_now
            # ── Pose divergence ─────────────────────────────────────────
            lines.append("[DIAG-NORM] Pose norms:")
            for label, norms_dict in [("odom", self._odom_norms),
                                      ("gtsam", self._pose_norms)]:
                for topic in sorted(norms_dict.keys()):
                    norms = norms_dict[topic]
                    if not norms:
                        lines.append(f"  {label} {topic}: (no data)")
                        continue
                    latest_norm = norms[-1][1]
                    max_norm = max(n for _, n in norms)
                    warn = ""
                    if latest_norm > self._norm_critical:
                        warn = " !!!CRITICAL"
                    elif latest_norm > self._norm_warn:
                        warn = " !warn"
                    lines.append(
                        f"  {label} {topic}: "
                        f"norm={latest_norm:.3f}m "
                        f"max={max_norm:.3f}m{warn}")

            # ── TF chain connectivity check ────────────────────────────
            lines.append("[DIAG-CHAIN] TF chain connectivity:")
            chains = [
                ("marker_map", "head_d435i_head_depth_optical_frame"),
                ("marker_map", "arm_d435i_arm_depth_optical_frame"),
                ("marker_map", "head_imu"),
                ("marker_map", "arm_imu"),
            ]
            for parent, child in chains:
                can = self._tf_buffer.can_transform(
                    parent, child, rclpy.time.Time())
                lines.append(
                    f"  {parent} -> {child}: "
                    f"{'CONNECTED' if can else 'DISCONNECTED'}")

            # ── Pointcloud chain health ────────────────────────────────
            # Per-side stage rates localise where the drop happens:
            #   compressed (wire) → local raw (decompressed) → points (assembler)
            lines.append("[DIAG-PC] Pointcloud chain health:")
            for side in ("head", "arm"):
                tops = self._pc_topics[side]
                depth_c = self._topic_stats.get(tops["depth_compressed"])
                image_c = self._topic_stats.get(tops["image_compressed"])
                depth_r = self._topic_stats.get(tops["depth_raw"])
                image_r = self._topic_stats.get(tops["image_raw"])
                ci_r = self._topic_stats.get(tops["camera_info"])
                pts = self._topic_stats.get(tops["points"])
                dc_hz = depth_c.rate_hz(self._rate_window) if depth_c else 0.0
                ic_hz = image_c.rate_hz(self._rate_window) if image_c else 0.0
                dr_hz = depth_r.rate_hz(self._rate_window) if depth_r else 0.0
                ir_hz = image_r.rate_hz(self._rate_window) if image_r else 0.0
                ci_hz = ci_r.rate_hz(self._rate_window) if ci_r else 0.0
                pt_hz = pts.rate_hz(self._rate_window) if pts else 0.0
                pt_total = pts.msg_count if pts else 0
                # Determine the failing stage
                stage = "OK"
                if dc_hz < 0.1 and ic_hz < 0.1:
                    stage = "NO_INPUT"
                elif dr_hz < 0.1 and ir_hz < 0.1:
                    stage = "DECOMPRESS_FAIL"
                elif pt_hz < 0.1 and ci_hz < 0.1 and (
                    dr_hz > 0.1 or ir_hz > 0.1
                ):
                    # Depth/RGB present but camera_info absent → the naive
                    # assembler cannot build a colored cloud.
                    stage = "CI_MISSING"
                elif pt_hz < 0.1 and (dr_hz > 0.1 or ir_hz > 0.1):
                    # All inputs present but no points → assembler not
                    # running, TF lookups failing, or timer stalled.
                    stage = "ASM_STALL"
                lines.append(
                    f"  {side}: depth_c={dc_hz:.1f}Hz "
                    f"image_c={ic_hz:.1f}Hz -> "
                    f"depth_r={dr_hz:.1f}Hz image_r={ir_hz:.1f}Hz "
                    f"ci_raw={ci_hz:.1f}Hz -> "
                    f"points={pt_hz:.1f}Hz (n={pt_total}) [{stage}]")
                # One-shot warn when input flows but points stay zero
                if stage == "CI_MISSING":
                    self.get_logger().warn(
                        f"PC CI MISSING: {side} has decompressed "
                        f"depth={dr_hz:.1f}Hz image={ir_hz:.1f}Hz but "
                        f"camera_info=0Hz — check Jetson relay token "
                        f"gate or camera_info_bridge",
                        throttle_duration_sec=self._summary_interval * 3)
                elif stage == "ASM_STALL":
                    self.get_logger().warn(
                        f"PC ASM STALL: {side} has decompressed "
                        f"depth={dr_hz:.1f}Hz image={ir_hz:.1f}Hz "
                        f"ci={ci_hz:.1f}Hz but points=0Hz — the "
                        f"naive_pointcloud_assembler is not producing "
                        f"output (not running, TF lookup failing, or "
                        f"timer stalled?)",
                        throttle_duration_sec=self._summary_interval * 3)
                elif stage == "DECOMPRESS_FAIL":
                    self.get_logger().warn(
                        f"PC DECOMPRESS FAIL: {side} compressed "
                        f"depth={dc_hz:.1f}Hz image={ic_hz:.1f}Hz but "
                        f"no /local/* output — decompress_bridge not running?",
                        throttle_duration_sec=self._summary_interval * 3)

            # Log as a single multi-line info block
            self.get_logger().info("\n".join(lines))

    return PipelineDiagnosticsNode


def main(args=None):
    """Entry point for the ``pipeline_diagnostics_node`` console script."""
    rclpy.init(args=args)
    NodeClass = create_node()
    node = NodeClass()
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
