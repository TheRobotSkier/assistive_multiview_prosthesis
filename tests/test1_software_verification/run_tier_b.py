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
    from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header, Int32, Float64MultiArray
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
                emg_received.set()

    node.create_subscription(Int32, "/grasp_preshaping/grasp_type",
                             on_grasp_type, 10)

    # Wait for publishers to connect
    print("\nWaiting for publishers to establish connections...")
    time.sleep(1.5)

    # Spin in a background thread for callback processing
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

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
            # Publish input data (ensure preshaping bridge has latest)
            now = node.get_clock().now().to_msg()
            cloud_msg.header.stamp = now
            cloud_pub.publish(cloud_msg)
            pose_msg.header.stamp = now
            pose_pub.publish(pose_msg)
            twist_msg.header.stamp = now
            twist_pub.publish(twist_msg)

            # Allow messages to propagate
            time.sleep(0.1)

            # ── Method A: Full EMG path ──────────────────────────────────
            if method in ("emg", "both"):
                with lock:
                    emg_result.clear()
                    emg_result["t_start"] = time.perf_counter()
                    emg_result["object"] = obj_name
                    emg_result["repetition"] = rep
                    emg_result["method"] = "emg"
                    emg_result["n_cloud_points"] = len(cloud_points)
                    emg_received.clear()

                # Inject EMG gesture (POWER = 1)
                emg_msg = Int32()
                emg_msg.data = 1  # GESTURE_POWER
                emg_pub.publish(emg_msg)

                # Wait for grasp_type output
                if emg_received.wait(timeout=15.0):
                    row = {
                        "object": obj_name,
                        "repetition": rep,
                        "method": "emg",
                        "total_latency_ms": round(emg_result.get("total_latency_ms", float("nan")), 2),
                        "grasp_type": emg_result.get("grasp_type", -1),
                        "n_cloud_points": len(cloud_points),
                        "status": emg_result.get("status", "unknown"),
                    }
                    rows.append(row)
                    if rep == 0:
                        print(f"    EMG rep 0: {row['total_latency_ms']:.1f} ms")
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
                open_msg.data = 3  # GESTURE_OPEN
                emg_pub.publish(open_msg)
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

                            # Parse grasp type from response message
                            if response.success and "grasp_type=" in response.message:
                                try:
                                    gt_part = response.message.split("grasp_type=")[1]
                                    gt_str = gt_part.split(",")[0].split(")")[0]
                                    row["grasp_type"] = int(gt_str)
                                except (ValueError, IndexError):
                                    pass

                            rows.append(row)
                            if rep == 0:
                                print(f"    Service rep 0: {latency_ms:.1f} ms, "
                                      f"success={response.success}")
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
        "l_block",
        "small_cube",
        "thin_plate",
    ]

    objects = args.objects or default_objects
    run_tier_b(objects, args.repetitions, args.method)


if __name__ == "__main__":
    main()
