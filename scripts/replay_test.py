#!/usr/bin/env python3
"""replay_test.py — hardware-free regression test via golden-bag replay.

Orchestrates the full replay-test workflow:

  1. Verify the golden bag exists and the prosthesis container is running.
  2. Start the pipeline in the background (tsdf_preview, debug_monitor, no hw).
  3. Start recording host outputs (ros2 bag record) + sysmon (--no-jetson).
  4. Replay ONLY the /jetson/* topics from the golden bag (ros2 bag play).
  5. When the bag finishes, stop recording + the pipeline.
  6. Run analyze_bag + analyze_log → text reports + JSON metrics.
  7. Compare JSON metrics vs tests/baselines/replay.json (if it exists).
  8. Print a regression report (red/green per metric).

The golden bag (data/bags/golden_replay/) contains ONLY the /jetson/* inputs.
The host pipeline regenerates ALL its own outputs (GTSAM, TSDF, segmentation)
from those inputs — exactly as it would with a real Jetson connected.

Usage:
  python3 scripts/replay_test.py                    # full run + compare
  python3 scripts/replay_test.py --capture          # capture baseline
  python3 scripts/replay_test.py --no-analyze       # just replay + record
  python3 scripts/replay_test.py --keep-pipeline    # don't stop the pipeline after

Requires: the prosthesis container running (make dev) and the workspace built.
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
GOLDEN_BAG = REPO_ROOT / "data" / "bags" / "golden_replay"
BASELINE_PATH = REPO_ROOT / "tests" / "baselines" / "replay.json"
RESULTS_DIR = REPO_ROOT / "logs" / "replay-results"

# Hardware-free DDS config. The container's default CYCLONEDDS_URI points at
# cyclonedds_peer.xml which hardcodes 10.42.0.1 (the Jetson link) — that
# interface doesn't exist without hardware, so CycloneDDS fails to bind and
# every ros2 command dies. We override with cyclonedds_local.xml (multicast
# loopback) for the replay test, exactly as test1-mock does (Makefile:970).
CYCLONEDDS_LOCAL = "/prosthesis_ws/config/cyclonedds_local.xml"

# The ROS environment prefix every exec needs. Sets the hardware-free DDS
# config + sources the ROS + workspace setups.
ROS_ENV = (
    f"export CYCLONEDDS_URI={CYCLONEDDS_LOCAL} "
    "&& export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "
    "&& export ROS_DOMAIN_ID=0 "
    "&& source /opt/ros/jazzy/setup.bash "
    "&& source /prosthesis_ws/install/setup.bash 2>/dev/null"
)

# Topics to replay from the golden bag (ONLY /jetson/* — the host regenerates
# everything else: TF, GTSAM poses, TSDF fusion, segmentation).
REPLAY_TOPICS = [
    "/jetson/head/points",
    "/jetson/arm/points",
    "/jetson/head/odom",
    "/jetson/arm/odom",
    "/jetson/head/image",
    "/jetson/arm/image",
    "/jetson/head/camera_info",
    "/jetson/arm/camera_info",
]

# Topics to record (the host-generated outputs we want to compare).
RECORD_TOPICS = [
    "/gtsam/head_pose",
    "/gtsam/arm_pose",
    "/vis/head_arm_pose",
    "/segmentation/object_cloud",
    "/keyframe_buffer/diagnostics",
    "/tf",
    "/tf_static",
]

# Pipeline launch arguments for the replay test. tsdf_preview runs the full
# V6 stack (GTSAM + keyframe_buffer + cross_camera_features + tsdf_fusion)
# without the grasp-trigger path, which is what we want to benchmark.
LAUNCH_ARGS = [
    "mia_hand:=false", "wrist:=false", "emg:=false", "haptic:=false",
    "camera:=true",
    "mounts_config:=/prosthesis_ws/src/sensor_fusion_bringup/config/camera_mounts.yaml",
    "tf_diagnostics:=true",
    "fusion_mode:=tsdf_preview",
    "debug_monitor:=true",
    "rviz:=false",
]

# ANSI colors
if sys.stdout.isatty():
    RED = "\033[31m"; GREEN = "\033[32m"; YELLOW = "\033[33m"
    CYAN = "\033[36m"; BOLD = "\033[1m"; DIM = "\033[2m"; RESET = "\033[0m"
else:
    RED = GREEN = YELLOW = CYAN = BOLD = DIM = RESET = ""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def log(msg: str, color: str = "", end: str = "\n"):
    print(f"{color}{msg}{RESET}" if color else msg, flush=True, end=end)


def run(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    """Run a command, streaming output. Returns CompletedProcess."""
    return subprocess.run(cmd, **kw)


def container_is_running() -> bool:
    """Check if the prosthesis container is up via docker/podman inspect."""
    backend = os.environ.get("DOCKER_CMD", "podman")
    try:
        r = subprocess.run(
            [backend, "inspect", "-f", "{{.State.Running}}", "prosthesis"],
            capture_output=True, text=True, timeout=10)
        return r.returncode == 0 and "true" in r.stdout.lower()
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def container_exec(cmd: str, **kw) -> subprocess.CompletedProcess:
    """Exec a command inside the prosthesis container as the prosthesis user.

    Automatically prepends the hardware-free ROS environment (local DDS config,
    RMW, domain, setup.bash) so ros2 commands work without the Jetson connected.
    """
    backend = os.environ.get("DOCKER_CMD", "podman")
    full = [backend, "exec", "--user", "prosthesis", "prosthesis",
            "/bin/bash", "-lc", f"{ROS_ENV} && {cmd}"]
    return subprocess.run(full, **kw)


def container_exec_bg(cmd: str) -> subprocess.Popen:
    """Start a command in the container in the background. Returns the Popen.

    Automatically prepends the hardware-free ROS environment.
    """
    backend = os.environ.get("DOCKER_CMD", "podman")
    full = [backend, "exec", "--user", "prosthesis", "prosthesis",
            "/bin/bash", "-lc", f"{ROS_ENV} && {cmd}"]
    return subprocess.Popen(full, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def kill_ros_node(node_name: str):
    """Kill a ROS node by name inside the container (best-effort).

    Uses pkill directly WITHOUT sourcing ROS — 'ros2 daemon stop' hangs when
    DDS is broken, and pkill doesn't need the ROS env anyway.
    """
    backend = os.environ.get("DOCKER_CMD", "podman")
    # Don't use container_exec here — we don't want to source ROS (which can
    # hang if DDS is in a bad state). pkill works without it.
    full = [backend, "exec", "--user", "prosthesis", "prosthesis",
            "/bin/bash", "-c",
            f"pkill -f '{node_name}' 2>/dev/null; "
            f"pkill -f 'ros2 launch' 2>/dev/null; "
            f"pkill -f 'ros2 bag' 2>/dev/null; "
            f"true"]
    try:
        subprocess.run(full, capture_output=True, timeout=10)
    except subprocess.TimeoutExpired:
        pass  # best-effort


# ---------------------------------------------------------------------------
# Orchestration
# ---------------------------------------------------------------------------

def step_preflight():
    """Verify prerequisites."""
    log("=" * 72, CYAN)
    log("REPLAY TEST — hardware-free regression via golden-bag replay", BOLD)
    log("=" * 72, CYAN)
    print()

    # Golden bag
    if not GOLDEN_BAG.is_dir():
        log(f"ERROR: golden bag not found at {GOLDEN_BAG}", RED)
        log(f"Expected data/bags/golden_replay/ with /jetson/* topics.", YELLOW)
        sys.exit(1)
    meta = GOLDEN_BAG / "metadata.yaml"
    if not meta.exists():
        log(f"ERROR: golden bag missing metadata.yaml at {meta}", RED)
        sys.exit(1)
    log(f"  Golden bag:   {GOLDEN_BAG.relative_to(REPO_ROOT)}", DIM)

    # Container
    if not container_is_running():
        log("ERROR: prosthesis container is not running.", RED)
        log("Start it first with 'make dev' and build the workspace.", YELLOW)
        sys.exit(1)
    log("  Container:    prosthesis (running)", DIM)

    # Workspace built?
    r = container_exec(
        "test -f /prosthesis_ws/install/setup.bash && echo BUILT || echo MISSING",
        capture_output=True, text=True, timeout=10)
    if "BUILT" not in (r.stdout or ""):
        log("ERROR: workspace not built. Run 'make build' inside the container first.", RED)
        sys.exit(1)
    log("  Workspace:    built", DIM)

    # Kill any leftover processes from a previous (possibly failed) run.
    # Orphaned nodes hold CycloneDDS participant slots and will cause
    # 'Failed to find a free participant index' on the next launch.
    stale = _count_proc_in_container("/prosthesis_ws/install/")
    if stale > 0:
        log(f"  Found {stale} stale pipeline process(es) — cleaning up...", YELLOW)
        cleanup_stale_processes()
        remaining = _count_proc_in_container("/prosthesis_ws/install/")
        if remaining > 0:
            log(f"  WARNING: {remaining} process(es) still alive after cleanup", YELLOW)
        else:
            log(f"  Cleanup complete", DIM)
    print()


def step_run_pipeline() -> subprocess.Popen | None:
    """Start the pipeline in the background inside the container."""
    log("[1/6] Starting pipeline (tsdf_preview, debug_monitor)...", BOLD)
    launch_cmd = (
        "cd /prosthesis_ws && "
        "ros2 launch prosthesis_launch pipeline.launch.py " + " ".join(LAUNCH_ARGS)
    )
    proc = container_exec_bg(launch_cmd)
    log(f"  Pipeline PID (host-side): {proc.pid}", DIM)

    # Wait for nodes to come up
    log("  Waiting for pipeline nodes to register...", DIM, end="")
    for _ in range(30):  # up to 30s
        time.sleep(1)
        r = container_exec(
            "ros2 node list 2>/dev/null",
            capture_output=True, text=True, timeout=10)
        nodes = (r.stdout or "").strip().split("\n")
        # Look for a key node that indicates the pipeline is up
        if any("gtsam_tracker" in n or "tsdf_fusion" in n or "pointcloud_fusion" in n
               for n in nodes):
            log(f" {GREEN}up ({len([n for n in nodes if n.strip()])} nodes){RESET}")
            return proc
        print(".", end="", flush=True)
    log(f" {YELLOW}timeout — proceeding anyway{RESET}")
    return proc


def step_start_recording(run_dir: Path) -> tuple[subprocess.Popen | None, Path]:
    """Start recording host outputs + sysmon. Returns (record_proc, bag_dir)."""
    log("[2/6] Starting recording (host outputs + sysmon)...", BOLD)
    bag_dir = run_dir / "replay_bag"
    topics_str = " ".join(RECORD_TOPICS)
    # NOTE: do NOT pre-create bag_dir — 'ros2 bag record -o <dir>' fails if the
    # directory already exists ("Output folder already exists"). It creates the
    # dir itself. We only ensure the parent exists.
    bag_parent = bag_dir.parent
    record_cmd = (
        f"mkdir -p /prosthesis_ws/{bag_parent.relative_to(REPO_ROOT)} && "
        f"ros2 bag record -o /prosthesis_ws/{bag_dir.relative_to(REPO_ROOT)} "
        f"--topics {topics_str}"
    )
    rec_proc = container_exec_bg(record_cmd)
    log(f"  Recording to: {bag_dir.relative_to(REPO_ROOT)}", DIM)

    # Sysmon (host-only, no Jetson)
    sysmon_path = run_dir / "sysmon.jsonl"
    sysmon_cmd = (
        f"cd /prosthesis_ws && python3 scripts/sysmon.py "
        f"--output /prosthesis_ws/{sysmon_path.relative_to(REPO_ROOT)} --no-jetson"
    )
    sysmon_proc = container_exec_bg(sysmon_cmd)
    log(f"  Sysmon:       {sysmon_path.relative_to(REPO_ROOT)} (--no-jetson)", DIM)

    # Give the recorder a moment to subscribe before we start replaying
    time.sleep(3)
    return rec_proc, bag_dir


def step_replay_bag() -> int:
    """Replay the golden bag's /jetson/* topics. Returns duration in seconds."""
    log("[3/6] Replaying golden bag (/jetson/* topics only, 1x rate)...", BOLD)

    # Get the bag duration from metadata so we can show progress
    duration = _bag_duration()
    log(f"  Bag duration: {duration:.0f}s", DIM)

    topics_str = " ".join(REPLAY_TOPICS)
    # Play inside the container. The bag is bind-mounted at the same host path.
    bag_container_path = f"/prosthesis_ws/{GOLDEN_BAG.relative_to(REPO_ROOT)}"
    play_cmd = (
        f"ros2 bag play {bag_container_path} --topics {topics_str} -r 1.0"
    )
    log(f"  Playing {len(REPLAY_TOPICS)} topics from {bag_container_path}...", DIM)
    print()

    # Run synchronously — this blocks until the bag finishes playing
    start = time.time()
    r = container_exec(play_cmd, timeout=int(duration) + 120)
    elapsed = time.time() - start
    print()
    if r.returncode != 0:
        log(f"  WARNING: ros2 bag play exited with code {r.returncode}", YELLOW)
    log(f"  Replay finished in {elapsed:.0f}s", DIM)
    return int(elapsed)


def _pkill_in_container(pattern: str, sig: int = signal.SIGINT, timeout: int = 15):
    """Send a signal to processes matching a pattern INSIDE the container.

    This is necessary because sending signals to the host-side `podman exec`
    Popen does NOT reliably propagate to the child process inside the container.
    ros2 bag record needs SIGINT (not SIGTERM/SIGKILL) to flush metadata.yaml.
    """
    backend = os.environ.get("DOCKER_CMD", "podman")
    sig_name = f"-{sig}"  # e.g. "-2" for SIGINT, "-15" for SIGTERM
    full = [backend, "exec", "--user", "prosthesis", "prosthesis",
            "/bin/bash", "-c",
            f"pkill -{sig} -f '{pattern}' 2>/dev/null; true"]
    try:
        subprocess.run(full, capture_output=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        pass  # best-effort


def _count_proc_in_container(pattern: str) -> int:
    """Count processes matching a pattern inside the container."""
    backend = os.environ.get("DOCKER_CMD", "podman")
    full = [backend, "exec", "--user", "prosthesis", "prosthesis",
            "/bin/bash", "-c",
            f"pgrep -f '{pattern}' 2>/dev/null | wc -l"]
    try:
        r = subprocess.run(full, capture_output=True, text=True, timeout=10)
        return int((r.stdout or "0").strip() or "0")
    except (subprocess.TimeoutExpired, ValueError):
        return 0


def cleanup_stale_processes():
    """Kill any leftover pipeline/recording processes from a previous run.

    This is CRITICAL: orphaned nodes (reparented to PID 1 when 'ros2 launch'
    dies) each hold a CycloneDDS participant. After a failed run, the leftover
    nodes exhaust the participant pool, causing 'Failed to find a free
    participant index for domain 0' on the next run.

    We match on /prosthesis_ws/install/ (covers all launched nodes) plus
    ros2 bag / sysmon / ros2 launch.
    """
    patterns = [
        "/prosthesis_ws/install/",  # all pipeline nodes (most robust)
        "ros2 bag",
        "sysmon.py",
        "ros2 launch",
    ]
    for p in patterns:
        _pkill_in_container(p, sig=signal.SIGTERM, timeout=10)
    # SIGTERM may be ignored by stuck DDS processes; escalate to SIGKILL
    time.sleep(2)
    for p in patterns:
        _pkill_in_container(p, sig=signal.SIGKILL, timeout=10)
    time.sleep(1)


def stop_pipeline():
    """Stop the pipeline and ALL its node processes.

    'ros2 launch' does not reliably propagate signals to its children when
    killed via podman exec — the child nodes get reparented to PID 1 and
    keep running, holding CycloneDDS participant slots. We must kill the
    nodes explicitly by their install-path prefix.
    """
    # SIGINT to ros2 launch first (lets it shut down gracefully if possible)
    _pkill_in_container("ros2 launch", sig=signal.SIGINT, timeout=10)
    _pkill_in_container("pipeline.launch", sig=signal.SIGINT, timeout=10)
    time.sleep(2)
    # Kill all node processes by install path (catches orphans)
    _pkill_in_container("/prosthesis_ws/install/", sig=signal.SIGTERM, timeout=10)
    time.sleep(1)
    _pkill_in_container("/prosthesis_ws/install/", sig=signal.SIGKILL, timeout=10)


def step_stop(rec_proc: subprocess.Popen | None, pipeline_proc: subprocess.Popen | None,
              keep_pipeline: bool, bag_dir: Path | None = None):
    """Stop recording and the pipeline."""
    log("[4/6] Stopping recording + pipeline...", BOLD)

    # Stop the bag recorder. We must send SIGINT *inside* the container —
    # sending it to the host-side podman exec Popen does not propagate, and
    # ros2 bag record needs SIGINT to flush metadata.yaml + .db3 properly.
    _pkill_in_container("ros2 bag record", sig=signal.SIGINT, timeout=15)
    # Also terminate the host-side Popen wrapper
    if rec_proc and rec_proc.poll() is None:
        rec_proc.terminate()
        try:
            rec_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            rec_proc.kill()
    # Give the recorder a moment to finalize the bag files
    time.sleep(2)
    if bag_dir and (bag_dir / "metadata.yaml").exists():
        log("  Recording stopped (bag flushed)", DIM)
    else:
        log("  WARNING: bag metadata.yaml not found — recording may be incomplete", YELLOW)

    # Kill sysmon
    _pkill_in_container("sysmon.py", sig=signal.SIGTERM, timeout=10)

    # Stop the pipeline and ALL its node processes (not just ros2 launch —
    # orphaned nodes hold CycloneDDS participant slots and break the next run)
    if not keep_pipeline:
        stop_pipeline()
        if pipeline_proc and pipeline_proc.poll() is None:
            pipeline_proc.terminate()
            try:
                pipeline_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pipeline_proc.kill()
        log("  Pipeline stopped", DIM)
    else:
        log("  Pipeline kept running (--keep-pipeline)", DIM)
    print()


def step_analyze(run_dir: Path, bag_dir: Path) -> tuple[Path | None, Path | None]:
    """Run analyze_bag + analyze_log, producing text reports + JSON metrics."""
    log("[5/6] Analyzing results...", BOLD)

    # Copy sysmon.jsonl next to the replay bag (analyze_bag looks for it there)
    sysmon_src = run_dir / "sysmon.jsonl"
    if sysmon_src.exists():
        shutil.copy2(sysmon_src, bag_dir / "sysmon.jsonl")

    bag_metrics = run_dir / "bag_metrics.json"
    log_metrics = run_dir / "log_metrics.json"

    # analyze_bag (deep replay + JSON metrics)
    bag_container = f"/prosthesis_ws/{bag_dir.relative_to(REPO_ROOT)}"
    report_path = run_dir / "bag_analysis.txt"
    r = container_exec(
        f"cd /prosthesis_ws && python3 scripts/analyze_bag.py "
        f"/prosthesis_ws/{bag_dir.relative_to(REPO_ROOT)} "
        f"-o /prosthesis_ws/{report_path.relative_to(REPO_ROOT)} "
        f"--json /prosthesis_ws/{bag_metrics.relative_to(REPO_ROOT)}",
        capture_output=True, text=True, timeout=300)
    if r.returncode == 0 and bag_metrics.exists():
        log(f"  Bag analysis:   {report_path.relative_to(REPO_ROOT)}", DIM)
        log(f"  Bag metrics:    {bag_metrics.relative_to(REPO_ROOT)}", DIM)
    else:
        log(f"  WARNING: analyze_bag failed (rc={r.returncode})", YELLOW)
        if r.stderr:
            log(f"  {r.stderr[:300]}", DIM)
        bag_metrics = None

    # analyze_log — find the latest host-log
    r = container_exec(
        "ls -t /prosthesis_ws/logs/host-log-*.txt 2>/dev/null | head -1",
        capture_output=True, text=True, timeout=10)
    log_path = (r.stdout or "").strip()
    if log_path:
        log_report = run_dir / "log_analysis.txt"
        r = container_exec(
            f"cd /prosthesis_ws && python3 scripts/analyze_log.py {log_path} "
            f"-o /prosthesis_ws/{log_report.relative_to(REPO_ROOT)} "
            f"--json /prosthesis_ws/{log_metrics.relative_to(REPO_ROOT)}",
            capture_output=True, text=True, timeout=60)
        if r.returncode == 0 and log_metrics.exists():
            log(f"  Log analysis:   {log_report.relative_to(REPO_ROOT)}", DIM)
            log(f"  Log metrics:    {log_metrics.relative_to(REPO_ROOT)}", DIM)
        else:
            log(f"  WARNING: analyze_log failed (rc={r.returncode})", YELLOW)
            log_metrics = None
    else:
        log("  No host-log found — skipping log analysis", YELLOW)
        log_metrics = None
    print()
    return bag_metrics, log_metrics


def step_compare(run_dir: Path, bag_metrics: Path | None,
                 log_metrics: Path | None, capture: bool):
    """Compare metrics against baseline (or capture a new baseline)."""
    if capture:
        log("[6/6] Capturing baseline metrics...", BOLD)
        BASELINE_PATH.parent.mkdir(parents=True, exist_ok=True)
        combined = _combine_metrics(bag_metrics, log_metrics)
        with open(BASELINE_PATH, "w") as f:
            json.dump(combined, f, indent=2, sort_keys=True)
        log(f"  Baseline written to {BASELINE_PATH.relative_to(REPO_ROOT)}", GREEN)
        log(f"\n  {GREEN}{BOLD}Baseline captured. Future 'make test-replay' runs will "
            f"compare against it.{RESET}")
        return

    if not BASELINE_PATH.exists():
        log("[6/6] No baseline found — skipping comparison", BOLD)
        log(f"  Run 'make test-replay-baseline' first to capture golden metrics.", YELLOW)
        log(f"  (expected at {BASELINE_PATH.relative_to(REPO_ROOT)})", DIM)
        return

    log("[6/6] Comparing against baseline...", BOLD)
    # Write a combined current metrics file
    combined_path = run_dir / "metrics.json"
    combined = _combine_metrics(bag_metrics, log_metrics)
    with open(combined_path, "w") as f:
        json.dump(combined, f, indent=2, sort_keys=True)

    # Run the comparison
    r = run([sys.executable, str(REPO_ROOT / "scripts" / "compare_replay.py"),
             "--baseline", str(BASELINE_PATH),
             "--current", str(combined_path)])
    if r.returncode == 0:
        log(f"\n  {GREEN}{BOLD}PASS: no regressions detected.{RESET}")
    else:
        log(f"\n  {RED}{BOLD}FAIL: regressions detected — see above.{RESET}")


def _combine_metrics(bag_metrics: Path | None, log_metrics: Path | None) -> dict:
    """Combine bag + log metrics into a single dict."""
    combined: dict = {}
    if bag_metrics and bag_metrics.exists():
        with open(bag_metrics) as f:
            combined["bag"] = json.load(f)
    if log_metrics and log_metrics.exists():
        with open(log_metrics) as f:
            combined["log"] = json.load(f)
    return combined


def _bag_duration() -> float:
    """Read the golden bag duration from metadata.yaml."""
    try:
        import yaml
        with open(GOLDEN_BAG / "metadata.yaml") as f:
            d = yaml.safe_load(f)
        ns = d["rosbag2_bagfile_information"]["duration"]["nanoseconds"]
        return ns / 1e9
    except Exception:
        return 180.0  # fallback


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture", action="store_true",
                    help="Capture current metrics as the golden baseline")
    ap.add_argument("--no-analyze", action="store_true",
                    help="Skip analysis + comparison (just replay + record)")
    ap.add_argument("--keep-pipeline", action="store_true",
                    help="Don't stop the pipeline after replay (for inspection)")
    args = ap.parse_args()

    step_preflight()

    # Create a timestamped run directory
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = RESULTS_DIR / f"replay_{ts}"
    run_dir.mkdir(parents=True, exist_ok=True)
    log(f"  Run dir:      {run_dir.relative_to(REPO_ROOT)}", DIM)
    print()

    pipeline_proc = step_run_pipeline()
    rec_proc, bag_dir = step_start_recording(run_dir)
    step_replay_bag()
    step_stop(rec_proc, pipeline_proc, args.keep_pipeline, bag_dir)

    if args.no_analyze:
        log("\nDone (--no-analyze). Outputs in:", CYAN)
        log(f"  {run_dir.relative_to(REPO_ROOT)}/", DIM)
        return

    bag_metrics, log_metrics = step_analyze(run_dir, bag_dir)
    step_compare(run_dir, bag_metrics, log_metrics, args.capture)

    log(f"\n{CYAN}All outputs saved to: {run_dir.relative_to(REPO_ROOT)}/{RESET}", DIM)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        log("\nInterrupted — cleaning up...", YELLOW)
        _pkill_in_container("ros2 bag record", sig=signal.SIGINT, timeout=5)
        _pkill_in_container("sysmon.py", sig=signal.SIGTERM, timeout=5)
        stop_pipeline()
        sys.exit(130)
