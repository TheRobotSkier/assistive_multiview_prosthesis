#!/usr/bin/env python3
"""Test 1: Software Verification — Tier B (Full ROS Pipeline Latency).

This test measures end-to-end latency from EMG trigger to motor command
through the full ROS 2 pipeline, including:
  - EMG signal processing / twist estimation
  - Segmentation (HTTP + CNN inference)
  - Grasp planning
  - Motor command output

REQUIREMENTS:
  - ROS 2 Jazzy with all prosthesis packages built and installed
  - The mock launch file running: ros2 launch prosthesis_launch mock.launch.py
  - A synthetic point cloud publisher

This is a skeleton — it will be completed once the full pipeline is
integrated and the mock launch environment is stable.

Usage (future):
    # Terminal 1: Launch mock system
    ros2 launch prosthesis_launch mock.launch.py

    # Terminal 2: Run Tier B test
    python run_tier_b.py

Inside Docker:
    docker compose run --rm prosthesis python /prosthesis_ws/tests/test1_software_verification/run_tier_b.py
"""

import argparse
import csv
import json
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(SCRIPT_DIR, "results")


def check_ros_available() -> bool:
    """Check if ROS 2 is available and the mock system is running."""
    try:
        import rclpy
        rclpy.init()
        rclpy.shutdown()
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
        return "/prosthesis/grasp_command" in topics
    except Exception:
        return False


def run_tier_b(objects: list[str], repetitions: int = 5):
    """Run full pipeline latency test via ROS 2.

    Architecture:
      1. Publish a synthetic point cloud to the segmentation input topic
      2. Inject an EMG trigger signal
      3. Listen for the grasp command output
      4. Measure wall-clock time from EMG injection to command receipt

    This requires:
      - rclpy
      - sensor_msgs (PointCloud2)
      - std_msgs (Trigger)
      - prosthesis_interfaces (GraspCommand)
    """
    print("=" * 60)
    print("TEST 1: TIER B — FULL ROS PIPELINE LATENCY")
    print("=" * 60)

    if not check_ros_available():
        print("ERROR: ROS 2 is not available. This test requires a running")
        print("ROS 2 environment. Run inside the Docker container with:")
        print("  docker compose run --rm prosthesis python ...")
        return

    if not check_mock_running():
        print("ERROR: Mock system is not running. Start it with:")
        print("  ros2 launch prosthesis_launch mock.launch.py")
        return

    # TODO: Implement full ROS 2 pipeline test
    # Steps:
    #   1. Create ROS 2 node
    #   2. Subscribe to /prosthesis/grasp_command
    #   3. For each object:
    #      a. Load point cloud
    #      b. Convert to PointCloud2 message
    #      c. Publish to segmentation input topic
    #      d. Inject EMG trigger
    #      e. Record timestamp
    #      f. Wait for grasp command callback
    #      g. Record timestamp
    #      h. Compute delta
    #   4. Write results to CSV

    print("\nTier B is not yet fully implemented.")
    print("This test requires the complete ROS 2 pipeline to be running.")
    print("See the comments in this file for the implementation plan.")

    # Placeholder: just record that we attempted
    rows = []
    for obj_name in objects:
        for rep in range(repetitions):
            rows.append({
                "object": obj_name,
                "repetition": rep,
                "total_latency_ms": float("nan"),
                "segmentation_ms": float("nan"),
                "grasp_planning_ms": float("nan"),
                "ros_overhead_ms": float("nan"),
                "status": "not_implemented",
            })

    out_path = os.path.join(RESULTS_DIR, "tier_b_latency_results.csv")
    os.makedirs(RESULTS_DIR, exist_ok=True)
    if rows:
        with open(out_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nWrote {len(rows)} placeholder rows to {out_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Test 1 Tier B: Full ROS Pipeline Latency"
    )
    parser.add_argument("--objects", nargs="+", default=None,
                        help="Specific objects to test")
    parser.add_argument("--repetitions", type=int, default=5,
                        help="Repetitions per object (default: 5)")
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
    run_tier_b(objects, args.repetitions)


if __name__ == "__main__":
    main()
