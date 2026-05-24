#!/usr/bin/env python3
"""Per-stage latency benchmark for the grasp planning pipeline.

This test measures the latency of each pipeline stage from EMG trigger to
grasp command output by recording arrival timestamps on intermediate topics.

Usage:
    python3 run_latency_stages.py --objects cylinder_upright ellipsoid --repetitions 10

Results are saved to results/latency_per_stage_results.csv.
"""

import argparse
import csv
import json
import math
import os
import sys
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Int32, String, Float32
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, PointStamped

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(__file__))
from object_registry import load_object
from hand_approaches import get_approach


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# Propagation origin offset from twist_propagation_node.py
# This shifts the collision check point from the hand position
propagation_offset = (0.1543, -0.1485, 0.1352)

# Twist propagation parameters (must match config/prosthesis_config.yaml)
TWIST_HORIZON = 2.0  # seconds
TWIST_DT = 0.02  # seconds (50Hz)
MIN_TIME_TO_HIT = 0.4  # seconds
EFFECTIVE_HIT_THRESH = 0.10  # meters


# ---------------------------------------------------------------------------
# Trajectory generation
# ---------------------------------------------------------------------------

def _generate_approach_trajectory(approach: dict, n_poses: int = 400, dt: float = 0.02
                                  ) -> list[dict]:
    """Generate approach trajectory from the hand_approaches definition.

    Uses the default approach pose with IDENTITY orientation so the propagation
    offset is applied in the world frame (body = world). The extended scene cloud
    covers both the object area and the offset corridor, ensuring twist propagation
    always detects a collision.
    """
    p = approach["pose"]
    t = approach["twist"]
    vx = t.get("lx", 0.10)

    # Start further back to account for activation delay (~2.0s from first pose to twist activation)
    activation_delay = 2.0  # seconds from first pose to twist activation
    start_px = p["px"] - vx * activation_delay

    # Use identity orientation so offset is applied in world frame
    # This makes the check point position predictable for scene cloud generation
    poses = []
    for i in range(n_poses):
        elapsed = i * dt
        poses.append({
            "px": start_px + vx * elapsed,
            "py": p["py"],
            "pz": p["pz"],
            "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,  # identity orientation
        })
    return poses


# ---------------------------------------------------------------------------
# Main benchmark
# ---------------------------------------------------------------------------

def run_latency_stages(objects: list[str], repetitions: int = 30,
                       publish_twist: bool = True, label: str = "",
                       simulate_segmentation: bool = False):
    """Run per-stage latency benchmark.

    Args:
        objects: List of object names to test
        repetitions: Number of trials per object
        publish_twist: Whether to publish TwistStamped alongside PoseStamped
        label: Label for results (e.g., "optimized", "baseline")
        simulate_segmentation: If True, bypass real segmentation inference
    """
    rclpy.init()
    node = rclpy.create_node("latency_stages_test")

    # ---------------------------------------------------------------------------
    # Publishers
    # ---------------------------------------------------------------------------

    # EMG gesture publisher (use TRANSIENT_LOCAL for confidence to ensure it's always available)
    emg_pub = node.create_publisher(Int32, "/emg/gesture_label", 10)
    latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    emg_confidence_pub = node.create_publisher(Float32, "/emg/confidence", latched_qos)

    # Hand pose and twist publishers
    pose_pub = node.create_publisher(PoseStamped, "/hand_pose", 10)
    twist_pub = node.create_publisher(TwistStamped, "/hand_twist", 10)

    # Point cloud publishers
    cloud_pub = node.create_publisher(PointCloud2, "/fused_pointcloud", 10)
    seg_input_pub = node.create_publisher(PointCloud2, "/segmentation/input_cloud", 10)
    seg_object_pub = node.create_publisher(PointCloud2, "/segmentation/object_cloud", 10)

    # ---------------------------------------------------------------------------
    # Results tracking
    # ---------------------------------------------------------------------------

    results = []
    lock = threading.Lock()
    current_trial: Optional[dict] = None
    traj_thread: Optional[threading.Thread] = None
    traj_stop_event = threading.Event()

    # ---------------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------------

    def _make_cloud(points: np.ndarray, frame_id: str = "world") -> PointCloud2:
        """Create a PointCloud2 message from numpy array."""
        msg = PointCloud2()
        msg.header = Header(frame_id=frame_id)
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        msg.data = points.astype(np.float32).tobytes()
        return msg

    def _make_pose(px: float, py: float, pz: float,
                   qx: float, qy: float, qz: float, qw: float,
                   frame_id: str = "world") -> PoseStamped:
        msg = PoseStamped()
        msg.header = Header(frame_id=frame_id)
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x = px
        msg.pose.position.y = py
        msg.pose.position.z = pz
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _make_twist(lx: float, ly: float, lz: float,
                    ax: float, ay: float, az: float,
                    frame_id: str = "world") -> TwistStamped:
        msg = TwistStamped()
        msg.header = Header(frame_id=frame_id)
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.twist.linear.x = lx
        msg.twist.linear.y = ly
        msg.twist.linear.z = lz
        msg.twist.angular.x = ax
        msg.twist.angular.y = ay
        msg.twist.angular.z = az
        return msg

    # ---------------------------------------------------------------------------
    # Pipeline state tracking
    # ---------------------------------------------------------------------------

    def _on_pipeline_state(msg: Int32):
        with lock:
            t = time.perf_counter()
            state = msg.data

            # Debug: log all state changes
            if current_trial is not None:
                state_names = {0: "IDLE", 1: "SEGMENTING", 2: "PLANNING", 3: "APPROACH", 4: "RELEASING"}
                state_name = state_names.get(state, f"UNKNOWN({state})")
                print(f"    State: {state_name}", flush=True)

            if current_trial is None:
                return

            if state == 1:  # SEGMENTING
                if current_trial["t0"] is None:
                    current_trial["t0"] = t
                    print(f"    [t0] SEGMENTING state received", flush=True)
            elif state == 2:  # PLANNING
                if current_trial["t4"] is None:
                    current_trial["t4"] = t
                    print(f"    [t4] PLANNING state received", flush=True)
                if current_trial["t3"] is not None:
                    current_trial["pm_cloud_handling_ms"] = (t - current_trial["t3"]) * 1000

    def _on_click_positive(msg: PointStamped):
        if current_trial is None:
            return

        with lock:
            t = time.perf_counter()
            # Use t0 (time when SEGMENTING state was entered) as the start of twist propagation
            if current_trial["t0"] is not None and current_trial["t2"] is None:
                current_trial["t2"] = t
                current_trial["twist_propagation_ms"] = (t - current_trial["t0"]) * 1000
                print(f"    [t2] Click received, twist_propagation={current_trial['twist_propagation_ms']:.1f}ms", flush=True)

                # Stop trajectory thread so the corrected hand pose below isn't overwritten
                traj_stop_event.set()

                # Publish corrected hand pose at object center for preshaping ROI prediction
                # The preshaping bridge uses the latest hand pose to predict the ROI.
                corrected_pose = PoseStamped()
                corrected_pose.header = Header(frame_id="world")
                corrected_pose.header.stamp = node.get_clock().now().to_msg()
                corrected_pose.pose.position.x = 0.0
                corrected_pose.pose.position.y = 0.0
                corrected_pose.pose.position.z = 0.0
                corrected_pose.pose.orientation.w = 1.0
                pose_pub.publish(corrected_pose)

                # Publish zero twist so preshaping predicts ROI at current position
                zero_twist = TwistStamped()
                zero_twist.header = Header(frame_id="world")
                zero_twist.header.stamp = node.get_clock().now().to_msg()
                zero_twist.twist.linear.x = 0.0
                zero_twist.twist.linear.y = 0.0
                zero_twist.twist.linear.z = 0.0
                zero_twist.twist.angular.x = 0.0
                zero_twist.twist.angular.y = 0.0
                zero_twist.twist.angular.z = 0.0
                twist_pub.publish(zero_twist)

                # In simulated mode, publish the object cloud directly so the pipeline
                # manager transitions SEGMENTING → PLANNING and calls the preshaping.
                if simulate_segmentation and current_trial.get("object_cloud") is not None:
                    seg_object_pub.publish(current_trial["object_cloud"])

    def _on_object_cloud(msg: PointCloud2):
        if current_trial is None:
            return

        with lock:
            t = time.perf_counter()
            if current_trial["t2"] is not None and current_trial["t3"] is None:
                current_trial["t3"] = t
                current_trial["segmentation_ms"] = (t - current_trial["t2"]) * 1000

    def _on_grasp_type(msg: Int32):
        if current_trial is None:
            return

        with lock:
            t = time.perf_counter()
            print(f"    [t5] Grasp type received: {msg.data}", flush=True)
            if current_trial["t4"] is not None and current_trial["t5"] is None:
                current_trial["t5"] = t
                current_trial["preshaping_ms"] = (t - current_trial["t4"]) * 1000

                # Mark trial complete
                current_trial["status"] = "ok"
                traj_stop_event.set()
                print(f"    Trial complete! preshaping={current_trial['preshaping_ms']:.1f}ms", flush=True)

    # Create subscribers
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)

    state_sub = node.create_subscription(Int32, "/pipeline/state", _on_pipeline_state, qos)
    click_sub = node.create_subscription(PointStamped, "/segmentation/click_positive", _on_click_positive, qos)
    cloud_sub = node.create_subscription(PointCloud2, "/segmentation/object_cloud", _on_object_cloud, qos)
    grasp_sub = node.create_subscription(Int32, "/grasp_preshaping/grasp_type", _on_grasp_type, qos)

    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Wait for publishers to connect
    time.sleep(2.0)

    # ---------------------------------------------------------------------------
    # Trial execution
    # ---------------------------------------------------------------------------

    for obj_name in objects:
        print(f"\n=== Testing {obj_name} ===", flush=True)

        obj = load_object(obj_name)
        approach = get_approach(obj_name)
        poses = _generate_approach_trajectory(approach)

        for rep in range(repetitions):
            print(f"  Repetition {rep + 1}/{repetitions}...", flush=True)

            # Initialize trial state
            with lock:
                current_trial = {
                    "object": obj_name,
                    "repetition": rep + 1,
                    "t0": None,  # EMG trigger / state SEGMENTING
                    "t1": None,  # state SEGMENTING (after PM)
                    "t2": None,  # click_positive
                    "t3": None,  # object_cloud
                    "t4": None,  # state PLANNING
                    "t5": None,  # grasp_type
                    "pipeline_manager_ms": float("nan"),
                    "twist_propagation_ms": float("nan"),
                    "segmentation_ms": float("nan"),
                    "pm_cloud_handling_ms": float("nan"),
                    "preshaping_ms": float("nan"),
                    "rust_compute_ms": float("nan"),
                    "ros_service_overhead_ms": float("nan"),
                    "smc_iterations": None,
                    "status": "timeout",
                    "n_events": 0,
                }

                # Store object cloud for simulated segmentation
                current_trial["object_cloud"] = _make_cloud(obj["points"])

            # Reset stop event
            traj_stop_event.clear()

            # -------------------------------------------------------------------
            # Phase 1: Publish approach poses and clouds
            # -------------------------------------------------------------------

            # Create extended scene cloud covering object AND offset area
            # This ensures twist propagation collision detection works
            ox, oy, oz = propagation_offset
            object_pts = obj["points"]
            offset_pts = np.random.uniform(
                low=[ox - 0.05, oy - 0.05, oz - 0.05],
                high=[ox + 0.05, oy + 0.05, oz + 0.05],
                size=(100, 3)
            ).astype(np.float32)
            scene_cloud_pts = np.vstack([object_pts, offset_pts])

            # Publish scene cloud to /fused_pointcloud (for twist propagation)
            # and object cloud to /segmentation/input_cloud (for segmentation)
            scene_cloud = _make_cloud(scene_cloud_pts)
            scene_cloud.header.stamp = node.get_clock().now().to_msg()
            cloud_pub.publish(scene_cloud)

            seg_input_cloud = _make_cloud(object_pts)
            seg_input_cloud.header.stamp = node.get_clock().now().to_msg()
            seg_input_pub.publish(seg_input_cloud)

            # Publish first 25 poses (0.5s) to warm up background cycle
            phase1_poses = 25
            for i in range(phase1_poses):
                pose = poses[i]
                pose_pub.publish(_make_pose(**pose))
                if publish_twist:
                    twist = approach["twist"]
                    twist_pub.publish(_make_twist(**twist))
                time.sleep(0.02)

            # Wait for background cycle to process — keep twist fresh
            for _ in range(5):
                if traj_stop_event.is_set():
                    break
                if publish_twist:
                    twist_pub.publish(_make_twist(**approach["twist"]))
                time.sleep(0.1)

            # -------------------------------------------------------------------
            # Phase 2: Start trajectory thread and trigger EMG
            # -------------------------------------------------------------------

            def _trajectory_publisher():
                """Publish approach trajectory in background thread."""
                for i in range(phase1_poses, len(poses)):
                    if traj_stop_event.is_set():
                        break
                    pose = poses[i]
                    pose_pub.publish(_make_pose(**pose))
                    if publish_twist:
                        twist = approach["twist"]
                        twist_pub.publish(_make_twist(**twist))
                    time.sleep(0.02)

            traj_thread = threading.Thread(target=_trajectory_publisher, daemon=True)
            traj_thread.start()

            # Wait briefly for trajectory to start
            time.sleep(0.1)

            # Publish confidence BEFORE gesture (ensures it arrives first)
            emg_confidence_pub.publish(Float32(data=0.95))
            time.sleep(0.05)  # Small delay to ensure confidence arrives

            # Trigger EMG gesture (POWER = 1, PINCH = 2, POINT = 4)
            emg_pub.publish(Int32(data=1))

            # -------------------------------------------------------------------
            # Phase 3: Wait for completion or timeout
            # -------------------------------------------------------------------

            timeout = 10.0  # seconds
            start_wait = time.time()
            while time.time() - start_wait < timeout:
                with lock:
                    if current_trial["status"] == "ok":
                        break
                time.sleep(0.01)

            # Stop trajectory thread
            traj_stop_event.set()

            # Count events received
            with lock:
                n_events = sum(
                    1 for t in [current_trial["t0"], current_trial["t1"], current_trial["t2"],
                               current_trial["t3"], current_trial["t4"], current_trial["t5"]]
                    if t is not None
                )
                current_trial["n_events"] = n_events

                # Debug: print what events were received
                if current_trial["status"] != "ok":
                    print(f"    Timeout: received {n_events} events", flush=True)
                    for i, t in enumerate([current_trial["t0"], current_trial["t1"], current_trial["t2"],
                                            current_trial["t3"], current_trial["t4"], current_trial["t5"]]):
                        status = "OK" if t is not None else "MISSING"
                        print(f"      t{i} ({status})", flush=True)

                # Compute total latency
                if all(current_trial[f"t{i}"] is not None for i in range(6)):
                    current_trial["total_ms"] = (current_trial["t5"] - current_trial["t0"]) * 1000
                else:
                    current_trial["total_ms"] = float("nan")

                # Compute ROS overhead
                if not math.isnan(current_trial["rust_compute_ms"]):
                    current_trial["ros_service_overhead_ms"] = (
                        current_trial["preshaping_ms"] - current_trial["rust_compute_ms"]
                    )

                # Remove temporary timestamp fields before saving
                trial_copy = current_trial.copy()
                for key in ["t0", "t1", "t2", "t3", "t4", "t5", "object_cloud"]:
                    trial_copy.pop(key, None)
                results.append(trial_copy)

            # Inter-trial reset delay
            time.sleep(1.0)

    # ---------------------------------------------------------------------------
    # Save results
    # ---------------------------------------------------------------------------

    os.makedirs("results", exist_ok=True)
    results_file = "results/latency_per_stage_results.csv"

    fieldnames = [
        "object", "repetition", "status", "n_events",
        "pipeline_manager_ms", "twist_propagation_ms", "segmentation_ms",
        "pm_cloud_handling_ms", "preshaping_ms", "rust_compute_ms",
        "ros_service_overhead_ms", "total_ms", "smc_iterations"
    ]

    with open(results_file, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)

    print(f"\n=== Results saved to {results_file} ===", flush=True)

    # Compute summary statistics
    ok_results = [r for r in results if r["status"] == "ok"]
    print(f"\n=== Summary ===", flush=True)
    print(f"Total trials: {len(results)}", flush=True)
    print(f"Successful trials: {len(ok_results)}", flush=True)

    if ok_results:
        stages = [
            "pipeline_manager_ms", "twist_propagation_ms", "segmentation_ms",
            "pm_cloud_handling_ms", "preshaping_ms", "total_ms"
        ]

        print("\nPer-stage latency (ms):", flush=True)
        for stage in stages:
            values = [float(r[stage]) for r in ok_results if not math.isnan(float(r[stage]))]
            if values:
                mean_val = np.mean(values)
                p95_val = np.percentile(values, 95)
                print(f"  {stage:25s}: mean={mean_val:7.2f}, p95={p95_val:7.2f}", flush=True)

        # Check MAR and IDE thresholds
        total_values = [float(r["total_ms"]) for r in ok_results if not math.isnan(float(r["total_ms"]))]
        if total_values:
            p95_total = np.percentile(total_values, 95)
            mar_pass = p95_total <= 400
            ide_pass = p95_total <= 100
            print(f"\nRequirement 2.4 (Pipeline Latency):", flush=True)
            print(f"  MAR (P95 ≤ 400 ms): {p95_total:.1f} ms - {'PASS' if mar_pass else 'FAIL'}", flush=True)
            print(f"  IDE (P95 ≤ 100 ms): {p95_total:.1f} ms - {'PASS' if ide_pass else 'FAIL'}", flush=True)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Per-stage latency benchmark")
    parser.add_argument("--objects", nargs="+", required=True, help="Objects to test")
    parser.add_argument("--repetitions", type=int, default=30, help="Trials per object")
    parser.add_argument("--no-external-twist", action="store_false", dest="publish_twist",
                        help="Don't publish external twist (use finite differences)")
    parser.add_argument("--label", default="", help="Label for results")
    parser.add_argument("--simulate-segmentation", action="store_true",
                        help="Bypass real segmentation inference")
    args = parser.parse_args()

    run_latency_stages(
        objects=args.objects,
        repetitions=args.repetitions,
        publish_twist=args.publish_twist,
        label=args.label,
        simulate_segmentation=args.simulate_segmentation
    )