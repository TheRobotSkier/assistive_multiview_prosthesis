#!/usr/bin/env python3
"""Test 1: Software Verification — Tier B (Full ROS Pipeline Latency).

This test measures end-to-end latency from EMG trigger to motor command
through the full ROS 2 pipeline. Two measurement methods are supported:

Method A — Full EMG path (primary):
  1. Publish synthetic data (cloud, pose, twist) to pipeline input topics
  2. Publish EMG gesture trigger to /emg/gesture_label
  3. Wait for grasp_type output on /grasp_preshaping/grasp_type
  4. Measure wall-clock time from EMG injection to output receipt

Method B — Direct service call (fallback / comparison):
  1. Publish synthetic data (cloud, pose, twist) to pipeline input topics
  2. Call /grasp_preshaping/compute_grasp service directly
  3. Measure wall-clock time from service call to response

Method A captures the full pipeline including pipeline manager state machine
overhead and all ROS middleware latency. Method B isolates the grasp planning
computation plus ROS service overhead.

REQUIREMENTS:
  - ROS 2 Jazzy with all prosthesis packages built and installed
  - The mock launch file running: ros2 launch prosthesis_launch mock.launch.py

Usage:
    # Terminal 1: Launch mock system
    ros2 launch prosthesis_launch mock.launch.py

    # Terminal 2: Run Tier B test
    python run_tier_b.py                       # default: method A (full EMG)
    python run_tier_b.py --method service      # method B (direct service)
    python run_tier_b.py --method both         # both methods for comparison

Inside Docker:
    docker compose run --rm prosthesis python /prosthesis_ws/tests/test1_software_verification/run_tier_b.py
"""

import argparse
import csv
import os
import re
import sys
import threading
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(SCRIPT_DIR, "results")


def check_ros_available() -> bool:
    """Check if ROS 2 is available."""
    try:
        import rclpy
        return True
    except Exception:
        return False


def check_mock_running() -> bool:
    """Check if the mock launch is active by looking for key topics."""
    try:
        import subprocess
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True, text=True, timeout=5,
        )
        topics = result.stdout
        has_grasp_type = "/grasp_preshaping/grasp_type" in topics
        has_emg = "/emg/gesture_label" in topics
        return has_grasp_type and has_emg
    except Exception:
        return False


def run_tier_b(objects: list[str], repetitions: int = 5, method: str = "emg"):
    """Run full pipeline latency test via ROS 2.

    Args:
        objects: List of object names to test.
        repetitions: Number of repetitions per object.
        method: "emg" (full EMG path), "service" (direct service), or "both".
    """
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header, Int32, Float32, Float64MultiArray, Bool
    from std_srvs.srv import Trigger

    print("=" * 60)
    print("TEST 1: TIER B — FULL ROS PIPELINE LATENCY")
    print("=" * 60)

    if not check_ros_available():
        print("ERROR: ROS 2 is not available. This test requires a running")
        print("ROS 2 environment. Run inside the Docker container with:")
        print("  docker compose run --rm prosthesis python ...")
        return

    if not check_mock_running():
        print("ERROR: Mock system is not running or topics not found.")
        print("Start it with:")
        print("  ros2 launch prosthesis_launch mock.launch.py")
        print("\nRequired topics:")
        print("  /emg/gesture_label           (pipeline_manager)")
        print("  /grasp_preshaping/grasp_type  (preshaping_service_bridge)")
        return

    # Import test helpers for object loading
    sys.path.insert(0, SCRIPT_DIR)
    from object_registry import load_object
    from hand_approaches import get_approach

    # ── ROS 2 init ──────────────────────────────────────────────────────
    rclpy.init()
    node = Node("tier_b_latency_test")

    # ── Publishers ──────────────────────────────────────────────────────
    cloud_pub = node.create_publisher(
        PointCloud2, "/segmentation/object_cloud", 10)
    pose_pub = node.create_publisher(
        PoseStamped, "/hand_pose", 10)
    twist_pub = node.create_publisher(
        TwistStamped, "/hand_twist", 10)
    emg_pub = node.create_publisher(
        Int32, "/emg/gesture_label", 10)
    emg_conf_pub = node.create_publisher(
        Float32, "/emg/confidence", 10)
    hit_pub = node.create_publisher(
        Bool, "/twist_propagation/hit_detected",
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    # ── State monitoring ─────────────────────────────────────────────────
    pipeline_state = {"value": None}
    state_received = threading.Event()
    stage_timestamps: dict[str, float] = {}
    stage_lock = threading.Lock()

    def on_pipeline_state(msg: Int32):
        pipeline_state["value"] = msg.data
        now = time.perf_counter()
        state_name = {0: "idle", 1: "twisting", 2: "segmenting",
                      3: "planning", 4: "approaching", 5: "grasping",
                      6: "holding", 7: "volitional", 8: "releasing"}.get(msg.data, f"unknown_{msg.data}")
        with stage_lock:
            stage_timestamps[state_name] = now
        state_received.set()

    node.create_subscription(Int32, "/pipeline/state",
                             on_pipeline_state, 10)

    # ── Service client ──────────────────────────────────────────────────
    compute_client = node.create_client(
        Trigger, "/grasp_preshaping/compute_grasp")

    # ── Shared state ────────────────────────────────────────────────────
    lock = threading.Lock()
    emg_result = {}  # filled by grasp_type callback
    emg_received = threading.Event()

    def on_grasp_type(msg: Int32):
        """Called when grasp_type is published — end-of-pipeline signal."""
        t_end = time.perf_counter()
        with lock:
            if "t_start" in emg_result:
                emg_result["total_latency_ms"] = (t_end - emg_result["t_start"]) * 1000
                emg_result["grasp_type"] = msg.data
                emg_result["status"] = "ok"
                with stage_lock:
                    emg_result["stage_timestamps"] = dict(stage_timestamps)
                emg_received.set()

    node.create_subscription(Int32, "/grasp_preshaping/grasp_type",
                             on_grasp_type, 10)

    # Wait for publishers to connect and pipeline to be ready
    print("\nWaiting for publishers to establish connections...")
    time.sleep(1.5)

    # Publish initial confidence so EMG gestures are accepted
    conf_msg = Float32()
    conf_msg.data = 1.0
    for _ in range(5):
        emg_conf_pub.publish(conf_msg)
        time.sleep(0.1)

    # Spin in a background thread for callback processing
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    # Reset pipeline to IDLE (in case it's in a stale state from a prior run)
    open_gesture = Int32()
    open_gesture.data = 2  # GESTURE_OPEN
    for _ in range(20):
        emg_pub.publish(open_gesture)
        emg_conf_pub.publish(conf_msg)
        time.sleep(0.1)
    time.sleep(0.5)

    rows = []

    for obj_name in objects:
        print(f"\n  Object: {obj_name}")
        try:
            obj = load_object(obj_name)
            approach = get_approach(obj_name)
        except (KeyError, FileNotFoundError) as e:
            print(f"    SKIP: {e}")
            continue

        # ── Prepare synthetic messages ──────────────────────────────────
        cloud_points = obj["points"]

        cloud_msg = PointCloud2()
        cloud_msg.header = Header(frame_id="world")
        cloud_msg.height = 1
        cloud_msg.width = len(cloud_points)
        cloud_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12
        cloud_msg.row_step = 12 * len(cloud_points)
        cloud_msg.is_dense = True
        cloud_msg.data = np.ascontiguousarray(
            cloud_points, dtype=np.float32).tobytes()

        pose_msg = PoseStamped()
        pose_msg.header = Header(frame_id="world")
        p = approach["pose"]
        pose_msg.pose.position.x = p["px"]
        pose_msg.pose.position.y = p["py"]
        pose_msg.pose.position.z = p["pz"]
        pose_msg.pose.orientation.x = p["qx"]
        pose_msg.pose.orientation.y = p["qy"]
        pose_msg.pose.orientation.z = p["qz"]
        pose_msg.pose.orientation.w = p["qw"]

        twist_msg = TwistStamped()
        twist_msg.header = Header(frame_id="world")
        t = approach["twist"]
        twist_msg.twist.linear = Vector3(x=t["lx"], y=t["ly"], z=t["lz"])
        twist_msg.twist.angular = Vector3(x=t["ax"], y=t["ay"], z=t["az"])

        # ── Run repetitions ─────────────────────────────────────────────
        for rep in range(repetitions):
            # ── Method A: Full EMG path ──────────────────────────────────
            if method in ("emg", "both"):
                # Clear stage timestamps for this repetition
                with stage_lock:
                    stage_timestamps.clear()

                # 1. Publish pose/twist (available for twist propagation)
                now = node.get_clock().now().to_msg()
                pose_msg.header.stamp = now
                pose_pub.publish(pose_msg)
                twist_msg.header.stamp = now
                twist_pub.publish(twist_msg)
                time.sleep(0.1)

                # 2. Ensure high confidence so EMG gestures are accepted
                for _ in range(3):
                    emg_conf_pub.publish(conf_msg)
                    time.sleep(0.05)

                # 3. Send POWER gesture and hold for > gesture_hold_timeout_s
                t_emg_sent = time.perf_counter()
                with lock:
                    emg_result.clear()
                    emg_result["t_emg_sent"] = t_emg_sent
                    emg_result["object"] = obj_name
                    emg_result["repetition"] = rep
                    emg_result["method"] = "emg"
                    emg_result["n_cloud_points"] = len(cloud_points)
                    emg_received.clear()

                emg_power = Int32()
                emg_power.data = 1  # GESTURE_POWER
                # Hold POWER for 1.5s (> gesture_hold_timeout_s=1.0)
                for _ in range(15):
                    emg_pub.publish(emg_power)
                    emg_conf_pub.publish(conf_msg)
                    time.sleep(0.1)

                # 4. Trigger hit detection if in TWISTING
                #    Pipeline may already be in TWISTING from the POWER hold
                time.sleep(0.3)  # brief settle for state callback
                if pipeline_state["value"] == 1:
                    # State 1 = TWISTING — publish hit detection
                    hit_msg = Bool()
                    hit_msg.data = True
                    for _ in range(3):
                        hit_pub.publish(hit_msg)
                        time.sleep(0.1)

                    # 5. Wait for SEGMENTING state, then publish cloud
                    state_received.clear()
                    if state_received.wait(timeout=3.0) and pipeline_state["value"] == 2:
                        # State 2 = SEGMENTING — publish cloud with fresh timestamp
                        t_cloud_sent = time.perf_counter()
                        now = node.get_clock().now().to_msg()
                        cloud_msg.header.stamp = now
                        cloud_pub.publish(cloud_msg)

                        # Set t_start to cloud publish for accurate pipeline latency
                        with lock:
                            emg_result["t_start"] = t_cloud_sent
                            emg_result["t_cloud_sent"] = t_cloud_sent
                    else:
                        # Pipeline did not reach SEGMENTING — fall back to t_emg_sent
                        with lock:
                            emg_result["t_start"] = t_emg_sent
                else:
                    # Pipeline not in TWISTING — fall back to t_emg_sent
                    with lock:
                        emg_result["t_start"] = t_emg_sent

                # 6. Ensure t_start is set (fallback if state machine didn't reach SEGMENTING)
                with lock:
                    if "t_start" not in emg_result:
                        emg_result["t_start"] = t_emg_sent

                # 7. Wait for grasp_type output (end of pipeline)
                if emg_received.wait(timeout=15.0):
                    ts = emg_result.get("stage_timestamps", {})
                    t_emg = emg_result.get("t_emg_sent", 0)
                    t_cloud = emg_result.get("t_cloud_sent", t_emg)

                    # Compute per-stage latencies
                    stage_lat = {}
                    if "twisting" in ts and t_emg:
                        stage_lat["emg_to_twisting_ms"] = round((ts["twisting"] - t_emg) * 1000, 2)
                    if "segmenting" in ts and "twisting" in ts:
                        stage_lat["twisting_to_segmenting_ms"] = round((ts["segmenting"] - ts["twisting"]) * 1000, 2)
                    if "planning" in ts and t_cloud:
                        stage_lat["cloud_to_planning_ms"] = round((ts["planning"] - t_cloud) * 1000, 2)
                    if "approaching" in ts and "planning" in ts:
                        stage_lat["planning_to_approaching_ms"] = round((ts["approaching"] - ts["planning"]) * 1000, 2)

                    row = {
                        "object": obj_name,
                        "repetition": rep,
                        "method": "emg",
                        "total_latency_ms": round(emg_result.get("total_latency_ms", float("nan")), 2),
                        "grasp_type": emg_result.get("grasp_type", -1),
                        "n_cloud_points": len(cloud_points),
                        "status": emg_result.get("status", "unknown"),
                    }
                    row.update(stage_lat)
                    rows.append(row)
                    if rep == 0:
                        total = row['total_latency_ms']
                        stages = " | ".join(f"{k}={v:.1f}" for k, v in stage_lat.items())
                        print(f"    EMG rep 0: total={total:.1f} ms  ({stages})")
                else:
                    rows.append({
                        "object": obj_name,
                        "repetition": rep,
                        "method": "emg",
                        "total_latency_ms": float("nan"),
                        "grasp_type": -1,
                        "n_cloud_points": len(cloud_points),
                        "status": "timeout",
                    })
                    if rep == 0:
                        print(f"    EMG rep 0: TIMEOUT")

                # Reset pipeline manager to IDLE by sending OPEN gesture
                open_msg = Int32()
                open_msg.data = 2  # GESTURE_OPEN
                for _ in range(15):
                    emg_pub.publish(open_msg)
                    emg_conf_pub.publish(conf_msg)
                    time.sleep(0.1)
                time.sleep(0.5)

            # ── Method B: Direct service call ────────────────────────────
            if method in ("service", "both"):
                if compute_client.wait_for_service(timeout_sec=2.0):
                    t_start = time.perf_counter()
                    future = compute_client.call_async(Trigger.Request())

                    # Spin locally to process the service response
                    deadline = time.perf_counter() + 15.0
                    while not future.done() and time.perf_counter() < deadline:
                        time.sleep(0.001)

                    t_end = time.perf_counter()

                    if future.done():
                        try:
                            response = future.result()
                            latency_ms = (t_end - t_start) * 1000
                            row = {
                                "object": obj_name,
                                "repetition": rep,
                                "method": "service",
                                "total_latency_ms": round(latency_ms, 2),
                                "grasp_type": -1,
                                "n_cloud_points": len(cloud_points),
                                "status": "ok" if response.success else "service_failed",
                            }

                            # Parse structured fields from response message
                            msg = response.message
                            if response.success and "grasp_type=" in msg:
                                try:
                                    gt_part = msg.split("grasp_type=")[1]
                                    gt_str = gt_part.split(",")[0].split(")")[0]
                                    row["grasp_type"] = int(gt_str)
                                except (ValueError, IndexError):
                                    pass

                            # Parse pipeline_time_ms (Rust algorithmic latency)
                            ptm = re.search(r"pipeline_time_ms=(\d+)", msg)
                            if ptm:
                                row["pipeline_time_ms"] = int(ptm.group(1))
                                row["ros_overhead_ms"] = round(
                                    latency_ms - int(ptm.group(1)), 2)

                            # Parse smc_iterations
                            si = re.search(r"smc_iterations=(\d+)", msg)
                            if si:
                                row["smc_iterations"] = int(si.group(1))

                            rows.append(row)
                            ptm_str = f", rust={ptm.group(1)}ms" if ptm else ""
                            if rep == 0:
                                print(f"    Service rep 0: {latency_ms:.1f} ms"
                                      f"{ptm_str}, success={response.success}")
                        except Exception as e:
                            rows.append({
                                "object": obj_name,
                                "repetition": rep,
                                "method": "service",
                                "total_latency_ms": float("nan"),
                                "grasp_type": -1,
                                "n_cloud_points": len(cloud_points),
                                "status": f"exception: {e}",
                            })
                    else:
                        rows.append({
                            "object": obj_name,
                            "repetition": rep,
                            "method": "service",
                            "total_latency_ms": float("nan"),
                            "grasp_type": -1,
                            "n_cloud_points": len(cloud_points),
                            "status": "timeout",
                        })
                else:
                    rows.append({
                        "object": obj_name,
                        "repetition": rep,
                        "method": "service",
                        "total_latency_ms": float("nan"),
                        "grasp_type": -1,
                        "n_cloud_points": len(cloud_points),
                        "status": "service_unavailable",
                    })

            # Inter-rep delay
            time.sleep(0.1)

        # Per-object summary
        for m in ([method] if method != "both" else ["emg", "service"]):
            obj_rows = [r for r in rows
                        if r["object"] == obj_name
                        and r["method"] == m
                        and r["status"] == "ok"]
            if obj_rows:
                times = [r["total_latency_ms"] for r in obj_rows]
                print(f"    {m} summary: mean={np.mean(times):.1f}, "
                      f"std={np.std(times):.1f}, "
                      f"min={np.min(times):.1f}, max={np.max(times):.1f} ms")

    # ── Cleanup ─────────────────────────────────────────────────────────
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=2.0)

    # ── Write results ───────────────────────────────────────────────────
    out_path = os.path.join(RESULTS_DIR, "tier_b_latency_results.csv")
    os.makedirs(RESULTS_DIR, exist_ok=True)
    if rows:
        fieldnames = [
            "object", "repetition", "method", "total_latency_ms",
            "emg_to_twisting_ms", "twisting_to_segmenting_ms",
            "cloud_to_planning_ms", "planning_to_approaching_ms",
            "pipeline_time_ms", "ros_overhead_ms", "smc_iterations",
            "grasp_type", "n_cloud_points", "status",
        ]
        with open(out_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
        print(f"\n  Wrote {len(rows)} rows to {out_path}")
    else:
        print("\n  No results to write.")

    # ── Overall summary ─────────────────────────────────────────────────
    for m in ([method] if method != "both" else ["emg", "service"]):
        ok_rows = [r for r in rows if r["method"] == m and r["status"] == "ok"]
        if ok_rows:
            all_times = [r["total_latency_ms"] for r in ok_rows]
            p95 = np.percentile(all_times, 95)
            print(f"\n{'=' * 60}")
            print(f"TIER B LATENCY SUMMARY — {m.upper()} method")
            print(f"{'=' * 60}")
            print(f"  Measurements: {len(ok_rows)}")
            print(f"  Mean:   {np.mean(all_times):.1f} ms")
            print(f"  Std:    {np.std(all_times):.1f} ms")
            print(f"  P95:    {p95:.1f} ms")
            print(f"  P99:    {np.percentile(all_times, 99):.1f} ms")
            print(f"  Min:    {np.min(all_times):.1f} ms")
            print(f"  Max:    {np.max(all_times):.1f} ms")
            print(f"\n  MAR (≤400 ms): {'PASS' if p95 <= 400 else 'FAIL'}")
            print(f"  IDE (≤100 ms): {'PASS' if p95 <= 100 else 'FAIL'}")

    print(f"\nDone. Results in: {RESULTS_DIR}")


def main():
    parser = argparse.ArgumentParser(
        description="Test 1 Tier B: Full ROS Pipeline Latency"
    )
    parser.add_argument("--objects", nargs="+", default=None,
                        help="Specific objects to test")
    parser.add_argument("--repetitions", type=int, default=5,
                        help="Repetitions per object (default: 5)")
    parser.add_argument("--method", default="emg",
                        choices=["emg", "service", "both"],
                        help="Measurement method (default: emg)")
    args = parser.parse_args()

    # Default objects for Tier B (subset — this test is slower)
    default_objects = [
        "cylinder_upright",
        "tapered_bottle",
        "cross_shape",
        "small_cube",
    ]

    objects = args.objects or default_objects
    run_tier_b(objects, args.repetitions, args.method)


if __name__ == "__main__":
    main()
