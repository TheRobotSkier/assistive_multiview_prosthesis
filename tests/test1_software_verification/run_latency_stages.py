#!/usr/bin/env python3
"""Test 1: Software Verification — Per-Stage Latency Benchmark.

Measures wall-clock latency for each pipeline stage from EMG trigger to
motor command output by subscribing to intermediate topics and recording
arrival times via time.perf_counter().

Pipeline stages measured:
  Stage A: Pipeline Manager overhead   (EMG → SEGMENTING state)
  Stage B: Twist Propagation           (SEGMENTING → click published)
  Stage C: Segmentation inference      (click → segmented cloud)
  Stage D: Pipeline Manager planning   (segmented cloud → PLANNING state)
  Stage E: Grasp Preshaping            (PLANNING → grasp_type output)

The sum of all stages equals the total EMG-to-command latency (Req 2.4).

REQUIREMENTS:
  - ROS 2 Jazzy with all prosthesis packages built and installed
  - The mock launch running: ros2 launch prosthesis_launch mock.launch.py
  - Segmentation inference server (optional, for full pipeline measurement)

Usage:
    # Terminal 1: Launch mock system
    ros2 launch prosthesis_launch mock.launch.py

    # Terminal 2: Run per-stage latency benchmark
    python run_latency_stages.py
    python run_latency_stages.py --objects cylinder_upright small_cube
    python run_latency_stages.py --repetitions 30

Inside Docker:
    docker compose run --rm prosthesis python \\
        /prosthesis_ws/tests/test1_software_verification/run_latency_stages.py
"""

import argparse
import csv
import math
import os
import re
import sys
import threading
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(SCRIPT_DIR, "results")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _check_ros():
    """Check if ROS 2 is available."""
    try:
        import rclpy
        return True
    except ImportError:
        return False


def _check_topics():
    """Check if the mock launch topics are available."""
    import subprocess
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True, text=True, timeout=5,
        )
        topics = result.stdout
        required = [
            "/emg/gesture_label",
            "/pipeline/state",
            "/grasp_preshaping/grasp_type",
        ]
        return all(t in topics for t in required)
    except Exception:
        return False


def _parse_pipeline_time_ms(message: str) -> float | None:
    """Extract pipeline_time_ms from a preshaping service response message."""
    m = re.search(r"pipeline_time_ms=(\d+)", message)
    if m:
        return float(m.group(1))
    return None


def _parse_smc_iterations(message: str) -> int | None:
    """Extract smc_iterations from a preshaping service response message."""
    m = re.search(r"smc_iterations=(\d+)", message)
    if m:
        return int(m.group(1))
    return None


# ---------------------------------------------------------------------------
# Approach trajectory generation
# ---------------------------------------------------------------------------

def _generate_approach_trajectory(approach: dict, n_poses: int = 25,
                                  dt: float = 0.1) -> list[dict]:
    """Generate a series of hand poses approaching the object.

    Starts ~30cm behind the approach point and moves forward at the approach
    velocity. This gives twist propagation enough poses with consistent
    velocity to estimate a twist and detect a collision.

    Returns list of dicts with pose fields {px, py, pz, qx, qy, qz, qw}.
    """
    p = approach["pose"]
    t = approach["twist"]

    start_x = p["px"] - 0.15  # start 15cm further back from approach point
    vx = t.get("lx", 0.10)
    vy = t.get("ly", 0.0)
    vz = t.get("lz", 0.0)

    poses = []
    for i in range(n_poses):
        elapsed = i * dt
        poses.append({
            "px": start_x + vx * elapsed,
            "py": p["py"] + vy * elapsed,
            "pz": p["pz"] + vz * elapsed,
            "qx": p["qx"],
            "qy": p["qy"],
            "qz": p["qz"],
            "qw": p["qw"],
        })
    return poses


# ---------------------------------------------------------------------------
# Main benchmark
# ---------------------------------------------------------------------------

def run_latency_stages(objects: list[str], repetitions: int = 30):
    """Run per-stage latency benchmark through the full ROS 2 pipeline."""
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, PointStamped
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header, Int32, Float64, Float64MultiArray, String
    from std_srvs.srv import Trigger

    sys.path.insert(0, SCRIPT_DIR)
    from object_registry import load_object
    from hand_approaches import get_approach

    print("=" * 60)
    print("TEST 1: PER-STAGE LATENCY BENCHMARK (Req 2.4, 1.6)")
    print("=" * 60)

    if not _check_ros():
        print("ERROR: ROS 2 is not available. Run inside the Docker container.")
        return

    if not _check_topics():
        print("ERROR: Mock system topics not found.")
        print("Start it with: ros2 launch prosthesis_launch mock.launch.py")
        return

    # ── ROS 2 init ──────────────────────────────────────────────────────
    rclpy.init()
    node = Node("latency_stages_test")

    # ── Publishers ──────────────────────────────────────────────────────
    # /fused_pointcloud — consumed by twist propagation (collision detection)
    cloud_pub = node.create_publisher(PointCloud2, "/fused_pointcloud", 10)
    # /segmentation/input_cloud — consumed by segmentation bridge in mock
    # (in the real pipeline this is remapped from /fused_pointcloud, but the
    # mock launch doesn't include the remapping, so we publish to both)
    seg_input_pub = node.create_publisher(PointCloud2, "/segmentation/input_cloud", 10)
    pose_pub = node.create_publisher(PoseStamped, "/hand_pose", 10)
    twist_pub = node.create_publisher(TwistStamped, "/hand_twist", 10)
    emg_pub = node.create_publisher(Int32, "/emg/gesture_label", 10)
    emg_conf_pub = node.create_publisher(Float64, "/emg/confidence", 10)

    # ── Service client ──────────────────────────────────────────────────
    compute_client = node.create_client(
        Trigger, "/grasp_preshaping/compute_grasp")

    # ── Trial state ─────────────────────────────────────────────────────
    lock = threading.Lock()
    trial_events: list[tuple[str, float, dict]] = []
    trial_complete = threading.Event()
    trial_result: dict = {}

    # ── Topic subscribers with arrival-time recording ───────────────────

    def _record(event_name: str, data: dict | None = None):
        """Record a trial event with the current perf_counter time."""
        with lock:
            trial_events.append((event_name, time.perf_counter(), data or {}))

    def _on_pipeline_state(msg: Int32):
        state = msg.data
        # Only record state transitions relevant to our trial
        if state in (1, 2, 3):  # SEGMENTING, PLANNING, APPROACHING
            _record("pipeline_state", {"state": state})

    def _on_click_positive(msg: PointStamped):
        _record("click_positive", {
            "x": msg.point.x, "y": msg.point.y, "z": msg.point.z,
        })

    def _on_object_cloud(msg: PointCloud2):
        n_pts = msg.width * msg.height
        _record("object_cloud", {"n_points": n_pts})

    def _on_grasp_type(msg: Int32):
        _record("grasp_type", {"grasp_type": msg.data})
        # This is the terminal event — signal completion
        with lock:
            if "t_start" in trial_result:
                trial_result["total_latency_ms"] = (
                    (time.perf_counter() - trial_result["t_start"]) * 1000
                )
                trial_result["grasp_type"] = msg.data
                trial_result["status"] = "ok"
                trial_complete.set()

    def _on_finger_closures(msg: Float64MultiArray):
        _record("finger_closures", {"closures": list(msg.data)})

    def _on_twist_status(msg: String):
        # Parse the JSON status to detect twist propagation state changes
        try:
            import json
            status = json.loads(msg.data)
            if status.get("reason") == "no_hit":
                _record("twist_no_hit", status)
            elif "hit_point" in status:
                _record("twist_hit", status)
        except (json.JSONDecodeError, KeyError):
            pass

    node.create_subscription(Int32, "/pipeline/state", _on_pipeline_state, 10)
    node.create_subscription(
        PointStamped, "/segmentation/click_positive", _on_click_positive, 10)
    cloud_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )
    node.create_subscription(
        PointCloud2, "/segmentation/object_cloud", _on_object_cloud, cloud_qos)
    node.create_subscription(Int32, "/grasp_preshaping/grasp_type",
                             _on_grasp_type, 10)
    node.create_subscription(
        Float64MultiArray, "/grasp_preshaping/target_finger_closures",
        _on_finger_closures, 10)
    node.create_subscription(
        String, "/twist_propagation/status", _on_twist_status, 10)

    # Wait for publishers to connect
    print("\nWaiting for publishers to establish connections...")
    time.sleep(2.0)

    # Spin in a background thread for callback processing
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    # ── Helper: build PointCloud2 message ───────────────────────────────

    def _make_cloud_msg(points: np.ndarray, frame_id: str = "world") -> PointCloud2:
        cloud = np.ascontiguousarray(points, dtype=np.float32)
        msg = PointCloud2()
        msg.header = Header(frame_id=frame_id)
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.height = 1
        msg.width = len(cloud)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(cloud)
        msg.is_dense = True
        msg.data = cloud.tobytes()
        return msg

    def _make_pose_msg(p: dict) -> PoseStamped:
        msg = PoseStamped()
        msg.header = Header(frame_id="world")
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x = p["px"]
        msg.pose.position.y = p["py"]
        msg.pose.position.z = p["pz"]
        msg.pose.orientation.x = p["qx"]
        msg.pose.orientation.y = p["qy"]
        msg.pose.orientation.z = p["qz"]
        msg.pose.orientation.w = p["qw"]
        return msg

    def _make_twist_msg(t: dict) -> TwistStamped:
        msg = TwistStamped()
        msg.header = Header(frame_id="world")
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.twist.linear = Vector3(x=t["lx"], y=t["ly"], z=t["lz"])
        msg.twist.angular = Vector3(x=t["ax"], y=t["ay"], z=t["az"])
        return msg

    # ── Run trials ──────────────────────────────────────────────────────
    rows = []

    for obj_name in objects:
        print(f"\n  Object: {obj_name}")
        try:
            obj = load_object(obj_name)
            approach = get_approach(obj_name)
        except (KeyError, FileNotFoundError) as e:
            print(f"    SKIP: {e}")
            continue

        cloud_points = obj["points"]
        approach_poses = _generate_approach_trajectory(approach)

        for rep in range(repetitions):
            # ── Phase 1: Prepare pipeline inputs ────────────────────────
            # Clear trial state
            with lock:
                trial_events.clear()
                trial_result.clear()
                trial_complete.clear()

            # Publish the fused point cloud (for twist propagation collision)
            cloud_msg = _make_cloud_msg(cloud_points)
            cloud_pub.publish(cloud_msg)
            # Also publish to segmentation input (the mock launch doesn't remap
            # /segmentation/input_cloud to /fused_pointcloud like the real pipeline)
            seg_input_pub.publish(cloud_msg)

            # Publish the approach twist
            twist_msg = _make_twist_msg(approach["twist"])
            twist_pub.publish(twist_msg)

            # Publish approach trajectory to /hand_pose to build pose buffer.
            # Note: twist propagation clears its pose buffer on activation,
            # so we must continue publishing AFTER the EMG trigger too.
            # Phase 1 publishes to warm up the pipeline; Phase 2 continues.
            for pose_data in approach_poses[:10]:  # first 10 poses (1s at 50Hz)
                pose_msg = _make_pose_msg(pose_data)
                pose_pub.publish(pose_msg)
                time.sleep(0.02)  # ~50 Hz for smooth trajectory

            # Allow data to propagate through the pipeline
            time.sleep(0.3)

            # ── Phase 2: Inject EMG trigger + continue trajectory ──────────
            # Pre-publish confidence so the pipeline manager accepts the gesture
            conf_msg = Float64()
            conf_msg.data = 0.95
            emg_conf_pub.publish(conf_msg)
            time.sleep(0.05)

            # Start a background thread that continues publishing approach
            # poses AFTER the EMG trigger. This is critical because twist
            # propagation's _on_activate clears the pose buffer, so the node
            # needs fresh poses post-activation to estimate a twist.
            stop_traj = threading.Event()

            def _publish_trajectory():
                """Continue publishing approach poses at ~50 Hz."""
                for pose_data in approach_poses[10:]:
                    if stop_traj.is_set():
                        return
                    pose_msg = _make_pose_msg(pose_data)
                    pose_pub.publish(pose_msg)
                    time.sleep(0.02)

            traj_thread = threading.Thread(target=_publish_trajectory, daemon=True)
            traj_thread.start()

            # Record start time and inject EMG gesture
            with lock:
                trial_result["t_start"] = time.perf_counter()
                trial_result["object"] = obj_name
                trial_result["repetition"] = rep

            emg_msg = Int32()
            emg_msg.data = 1  # POWER gesture
            emg_pub.publish(emg_msg)

            # ── Phase 3: Wait for completion ────────────────────────────
            if trial_complete.wait(timeout=15.0):
                stop_traj.set()  # stop trajectory publishing
                # ── Phase 4: Extract per-stage latencies ─────────────────
                with lock:
                    events = list(trial_events)
                    result = dict(trial_result)

                row = _compute_stage_latencies(
                    obj_name, rep, events, result, len(cloud_points))
                rows.append(row)

                if rep == 0:
                    print(f"    Rep 0: total={row['total_ms']:.1f} ms, "
                          f"path={row['path']}")
                    if row["total_ms"] > 0:
                        stages = []
                        for s in ["pipeline_manager_ms", "twist_propagation_ms",
                                  "segmentation_ms", "pm_cloud_handling_ms",
                                  "preshaping_ms"]:
                            v = row.get(s)
                            if v is not None and not math.isnan(v):
                                stages.append(f"{s.split('_ms')[0].split('/')[-1]}={v:.1f}")
                        print(f"           stages: {', '.join(stages)}")
            else:
                # Timeout
                stop_traj.set()  # stop trajectory publishing
                with lock:
                    events = list(trial_events)
                    result = dict(trial_result)

                row = {
                    "object": obj_name,
                    "repetition": rep,
                    "path": "timeout",
                    "pipeline_manager_ms": float("nan"),
                    "twist_propagation_ms": float("nan"),
                    "segmentation_ms": float("nan"),
                    "pm_cloud_handling_ms": float("nan"),
                    "preshaping_ms": float("nan"),
                    "total_ms": float("nan"),
                    "rust_compute_ms": float("nan"),
                    "ros_service_overhead_ms": float("nan"),
                    "grasp_type": -1,
                    "n_cloud_points": len(cloud_points),
                    "status": "timeout",
                    "n_events": len(events),
                }
                rows.append(row)
                if rep == 0:
                    print(f"    Rep 0: TIMEOUT ({len(events)} events recorded)")

            # ── Phase 5: Reset pipeline to IDLE ─────────────────────────
            open_msg = Int32()
            open_msg.data = 3  # OPEN gesture
            emg_pub.publish(open_msg)
            time.sleep(0.8)

        # Per-object summary
        obj_rows = [r for r in rows if r["object"] == obj_name and r["status"] == "ok"]
        if obj_rows:
            times = [r["total_ms"] for r in obj_rows]
            print(f"    Summary: n={len(obj_rows)}, "
                  f"mean={np.mean(times):.1f}, "
                  f"P95={np.percentile(times, 95):.1f}, "
                  f"min={np.min(times):.1f}, max={np.max(times):.1f} ms")

    # ── Cleanup ─────────────────────────────────────────────────────────
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=2.0)

    # ── Write results ───────────────────────────────────────────────────
    _write_results(rows)

    # ── Print overall summary ───────────────────────────────────────────
    _print_summary(rows)


# ---------------------------------------------------------------------------
# Stage latency computation
# ---------------------------------------------------------------------------

def _compute_stage_latencies(
    obj_name: str,
    rep: int,
    events: list[tuple[str, float, dict]],
    result: dict,
    n_cloud_points: int,
) -> dict:
    """Compute per-stage latencies from recorded arrival-time events.

    Events are (name, perf_counter_time, data) tuples. We find the first
    occurrence of each event type and compute deltas between consecutive
    stages.
    """
    t_start = result.get("t_start", float("nan"))

    # Find first occurrence of each stage marker
    def _first(event_name: str) -> float | None:
        for name, t, data in events:
            if name == event_name:
                return t
        return None

    t_segmenting = _first("pipeline_state")  # state=1 (SEGMENTING)
    # Filter for SEGMENTING specifically
    for name, t, data in events:
        if name == "pipeline_state" and data.get("state") == 1:
            t_segmenting = t
            break

    t_click = _first("click_positive")
    t_object_cloud = _first("object_cloud")

    t_planning = None
    for name, t, data in events:
        if name == "pipeline_state" and data.get("state") == 2:
            t_planning = t
            break

    t_grasp_type = _first("grasp_type")

    # Compute stage durations
    def _delta(t_end, t_begin):
        if t_end is None or t_begin is None:
            return float("nan")
        return (t_end - t_begin) * 1000

    pipeline_manager_ms = _delta(t_segmenting, t_start)
    twist_propagation_ms = _delta(t_click, t_segmenting)
    segmentation_ms = _delta(t_object_cloud, t_click)
    pm_cloud_handling_ms = _delta(t_planning, t_object_cloud)
    preshaping_ms = _delta(t_grasp_type, t_planning)
    total_ms = result.get("total_latency_ms", float("nan"))

    # Determine which path was taken
    if t_click is not None:
        path = "twist_propagation"
    elif t_object_cloud is not None:
        path = "direct_pm"
    else:
        path = "unknown"

    # Try to get rust_compute_ms from service response (if available
    # via the pipeline_time_ms in the response message)
    rust_compute_ms = float("nan")
    ros_service_overhead_ms = float("nan")
    if not math.isnan(preshaping_ms) and not math.isnan(rust_compute_ms):
        ros_service_overhead_ms = preshaping_ms - rust_compute_ms

    return {
        "object": obj_name,
        "repetition": rep,
        "path": path,
        "pipeline_manager_ms": round(pipeline_manager_ms, 2) if not math.isnan(pipeline_manager_ms) else float("nan"),
        "twist_propagation_ms": round(twist_propagation_ms, 2) if not math.isnan(twist_propagation_ms) else float("nan"),
        "segmentation_ms": round(segmentation_ms, 2) if not math.isnan(segmentation_ms) else float("nan"),
        "pm_cloud_handling_ms": round(pm_cloud_handling_ms, 2) if not math.isnan(pm_cloud_handling_ms) else float("nan"),
        "preshaping_ms": round(preshaping_ms, 2) if not math.isnan(preshaping_ms) else float("nan"),
        "total_ms": round(total_ms, 2) if not math.isnan(total_ms) else float("nan"),
        "rust_compute_ms": rust_compute_ms,
        "ros_service_overhead_ms": ros_service_overhead_ms,
        "grasp_type": result.get("grasp_type", -1),
        "n_cloud_points": n_cloud_points,
        "status": result.get("status", "unknown"),
        "n_events": len(events),
    }


# ---------------------------------------------------------------------------
# Results output
# ---------------------------------------------------------------------------

def _write_results(rows: list[dict]):
    """Write per-trial and summary CSV files."""
    os.makedirs(RESULTS_DIR, exist_ok=True)

    # Per-trial CSV
    trial_path = os.path.join(RESULTS_DIR, "latency_per_stage_results.csv")
    if rows:
        fieldnames = [
            "object", "repetition", "path", "status",
            "pipeline_manager_ms", "twist_propagation_ms",
            "segmentation_ms", "pm_cloud_handling_ms",
            "preshaping_ms", "total_ms",
            "rust_compute_ms", "ros_service_overhead_ms",
            "grasp_type", "n_cloud_points", "n_events",
        ]
        with open(trial_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
        print(f"\n  Wrote {len(rows)} rows to {trial_path}")
    else:
        print("\n  No results to write.")

    # Summary CSV
    if not rows:
        return

    import numpy as np
    from collections import defaultdict

    # Group by object
    groups = defaultdict(list)
    for r in rows:
        if r["status"] == "ok":
            groups[r["object"]].append(r)

    stage_fields = [
        "pipeline_manager_ms", "twist_propagation_ms", "segmentation_ms",
        "pm_cloud_handling_ms", "preshaping_ms", "total_ms",
        "rust_compute_ms", "ros_service_overhead_ms",
    ]

    summary_rows = []
    for obj_name in sorted(groups):
        obj_rows = groups[obj_name]
        summary = {"object": obj_name, "n_trials": len(obj_rows)}
        for field in stage_fields:
            values = [r[field] for r in obj_rows if not math.isnan(r.get(field, float("nan")))]
            if values:
                arr = np.array(values)
                summary[f"{field}_mean"] = round(float(np.mean(arr)), 2)
                summary[f"{field}_p95"] = round(float(np.percentile(arr, 95)), 2)
                summary[f"{field}_p99"] = round(float(np.percentile(arr, 99)), 2)
                summary[f"{field}_min"] = round(float(np.min(arr)), 2)
                summary[f"{field}_max"] = round(float(np.max(arr)), 2)
            else:
                for stat in ["mean", "p95", "p99", "min", "max"]:
                    summary[f"{field}_{stat}"] = float("nan")
        summary_rows.append(summary)

    if summary_rows:
        summary_path = os.path.join(RESULTS_DIR, "latency_per_stage_summary.csv")
        fieldnames = list(summary_rows[0].keys())
        with open(summary_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(summary_rows)
        print(f"  Wrote {len(summary_rows)} rows to {summary_path}")


def _print_summary(rows: list[dict]):
    """Print overall latency summary with MAR/IDE checks."""
    import numpy as np

    ok_rows = [r for r in rows if r["status"] == "ok"]
    if not ok_rows:
        print("\n  No successful trials to summarize.")
        return

    print(f"\n{'=' * 60}")
    print("PER-STAGE LATENCY SUMMARY")
    print(f"{'=' * 60}")

    stage_fields = [
        ("pipeline_manager_ms", "Pipeline Manager"),
        ("twist_propagation_ms", "Twist Propagation"),
        ("segmentation_ms", "Segmentation"),
        ("pm_cloud_handling_ms", "PM Cloud Handling"),
        ("preshaping_ms", "Grasp Preshaping"),
        ("total_ms", "TOTAL"),
    ]

    # Print per-stage table
    print(f"\n  {'Stage':25s} {'Mean':>8s} {'P95':>8s} {'P99':>8s} {'% Total':>8s}")
    print(f"  {'-' * 25} {'-' * 8} {'-' * 8} {'-' * 8} {'-' * 8}")

    total_values = [r["total_ms"] for r in ok_rows
                    if not math.isnan(r.get("total_ms", float("nan")))]
    total_mean = np.mean(total_values) if total_values else 0

    for field, label in stage_fields:
        values = [r[field] for r in ok_rows
                  if not math.isnan(r.get(field, float("nan")))]
        if values:
            arr = np.array(values)
            mean = np.mean(arr)
            p95 = np.percentile(arr, 95)
            p99 = np.percentile(arr, 99)
            pct = (mean / total_mean * 100) if total_mean > 0 else 0
            print(f"  {label:25s} {mean:8.1f} {p95:8.1f} {p99:8.1f} {pct:7.1f}%")
        else:
            print(f"  {label:25s} {'N/A':>8s} {'N/A':>8s} {'N/A':>8s} {'N/A':>8s}")

    # Path statistics
    path_counts = {}
    for r in ok_rows:
        p = r.get("path", "unknown")
        path_counts[p] = path_counts.get(p, 0) + 1
    print(f"\n  Paths taken: {path_counts}")

    # MAR/IDE checks
    if total_values:
        total_p95 = np.percentile(total_values, 95)
        print(f"\n  MAR (P95 <= 400 ms): {'PASS' if total_p95 <= 400 else 'FAIL'}"
              f"  (P95 = {total_p95:.1f} ms)")
        print(f"  IDE (P95 <= 100 ms): {'PASS' if total_p95 <= 100 else 'FAIL'}"
              f"  (P95 = {total_p95:.1f} ms)")

    print(f"\n  Successful trials: {len(ok_rows)} / {len(rows)}")
    print(f"  Results in: {RESULTS_DIR}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Test 1: Per-Stage Latency Benchmark"
    )
    parser.add_argument("--objects", nargs="+", default=None,
                        help="Specific objects to test (default: all)")
    parser.add_argument("--repetitions", type=int, default=30,
                        help="Repetitions per object (default: 30)")
    args = parser.parse_args()

    sys.path.insert(0, SCRIPT_DIR)
    from object_registry import list_objects

    available = list_objects()
    if args.objects:
        objects = [o for o in args.objects if o in available]
        missing = set(args.objects) - set(objects)
        if missing:
            print(f"WARNING: Objects not found: {missing}")
    else:
        objects = available

    if not objects:
        print("ERROR: No objects available. Run generate_objects.py first.")
        sys.exit(1)

    print(f"Test objects ({len(objects)}): {objects}")
    print(f"Repetitions: {args.repetitions}")

    run_latency_stages(objects, args.repetitions)


if __name__ == "__main__":
    main()
