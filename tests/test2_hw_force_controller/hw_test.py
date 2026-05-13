#!/usr/bin/env python3
"""Interactive Hardware Test — Force Controller & Pipeline Manager.

Assumes the Mia Hand driver is already running on /driver.
Auto-launches command_bridge, force_controller, and pipeline_manager,
then presents a menu-driven interface for testing.

Usage:
    # Shell 1: start the driver
    ros2 run mia_hand_driver mia_hand_driver_node --ros-args -p serial_port:=/dev/ttyUSB0

    # Shell 2: run this test
    python3 /prosthesis_ws/tests/test2_hw_force_controller/hw_test.py
"""

from __future__ import annotations

import atexit
import csv
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from typing import Optional

# ── Constants ────────────────────────────────────────────────────────────────

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(SCRIPT_DIR, "results")
CONFIG_PATH = "/prosthesis_ws/config/prosthesis_config.yaml"
SETUP_CMD = "source /opt/ros/jazzy/setup.bash && source /prosthesis_ws/install/setup.bash"

# Pipeline states (must match force_controller_node.py / pipeline_manager_node.py)
STATE_IDLE = 0
STATE_SEGMENTING = 1
STATE_PLANNING = 2
STATE_APPROACHING = 3
STATE_GRASPING = 4
STATE_HOLDING = 5
STATE_RELEASING = 6

STATE_NAMES = {
    STATE_IDLE: "IDLE",
    STATE_SEGMENTING: "SEGMENTING",
    STATE_PLANNING: "PLANNING",
    STATE_APPROACHING: "APPROACHING",
    STATE_GRASPING: "GRASPING",
    STATE_HOLDING: "HOLDING",
    STATE_RELEASING: "RELEASING",
}

FINGER_NAMES = ["thumb", "index", "mrl"]
JOINT_NAMES = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]

# Emergency force threshold (raw sensor units)
EMERGENCY_FORCE = 500.0

# ── Colors ───────────────────────────────────────────────────────────────────

class C:
    """ANSI color codes."""
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    RESET = "\033[0m"


def _c(color: str, text: str) -> str:
    return f"{color}{text}{C.RESET}"


# ── Subprocess Manager ───────────────────────────────────────────────────────

class NodeManager:
    """Manages ROS2 node subprocesses launched by this script."""

    def __init__(self):
        self._procs: list[subprocess.Popen] = []
        self._names: list[str] = []

    def launch(self, node_name: str, cmd: str) -> bool:
        """Launch a ROS2 node as a subprocess. Returns True on success."""
        full_cmd = f"{SETUP_CMD} && {cmd}"
        proc = subprocess.Popen(
            ["bash", "-c", full_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )
        self._procs.append(proc)
        self._names.append(node_name)
        return True

    def wait_for_nodes(self, expected: list[str], timeout: float = 15.0) -> bool:
        """Wait until all expected node names appear in ros2 node list."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                result = subprocess.run(
                    ["bash", "-c", f"{SETUP_CMD} && ros2 node list"],
                    capture_output=True, text=True, timeout=5,
                )
                active = set(result.stdout.strip().split("\n"))
                active.discard("")
                missing = [n for n in expected if n not in active]
                if not missing:
                    return True
                remaining = int(deadline - time.time())
                if remaining > 0:
                    print(f"  Waiting for: {missing} ({remaining}s remaining)    ", end="\r")
            except subprocess.TimeoutExpired:
                pass
            time.sleep(1)
        return False

    def kill_all(self):
        """Kill all launched subprocesses."""
        for proc in reversed(self._procs):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except (ProcessLookupError, OSError):
                pass
        time.sleep(0.5)
        for proc in reversed(self._procs):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass
        self._procs.clear()
        self._names.clear()

    @property
    def running(self) -> list[str]:
        return [n for n, p in zip(self._names, self._procs) if p.poll() is None]


# ── ROS2 Helpers (via subprocess) ────────────────────────────────────────────

def _ros_cmd(cmd: str, timeout: float = 5.0) -> tuple[int, str, str]:
    """Run a bash command with ROS setup. Returns (exit_code, stdout, stderr)."""
    full = f"{SETUP_CMD} && {cmd}"
    try:
        result = subprocess.run(
            ["bash", "-c", full],
            capture_output=True, text=True, timeout=timeout,
        )
        return result.returncode, result.stdout.strip(), result.stderr.strip()
    except subprocess.TimeoutExpired:
        return -1, "", "timeout"


def get_node_list() -> set[str]:
    _, out, _ = _ros_cmd("ros2 node list")
    nodes = set(out.strip().split("\n"))
    nodes.discard("")
    return nodes


def get_topic_list() -> set[str]:
    _, out, _ = _ros_cmd("ros2 topic list")
    topics = set(out.strip().split("\n"))
    topics.discard("")
    return topics


def publish_int(topic: str, value: int):
    _ros_cmd(
        f"ros2 topic pub {topic} std_msgs/Int32 '{{data: {value}}}' --once",
        timeout=3.0,
    )


def publish_float64(topic: str, value: float):
    _ros_cmd(
        f"ros2 topic pub {topic} std_msgs/Float64MultiArray '{{data: [{value}]}}' --once",
        timeout=3.0,
    )


def call_set_bool(service: str, value: bool) -> bool:
    val = "true" if value else "false"
    _, out, _ = _ros_cmd(
        f"ros2 service call {service} std_srvs/srv/SetBool '{{data: {val}}}'",
        timeout=5.0,
    )
    return "success=True" in out or "True" in out


# ── Data Collector (runs as subprocess, writes to file) ──────────────────────

def _collect_data(duration_s: float, topics: list[str], output_csv: str) -> str:
    """Spawn a Python subprocess that subscribes to topics and writes CSV.

    Returns the path to the CSV file.
    """
    collector_script = f'''
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import csv, time, sys

class Collector(Node):
    def __init__(self):
        super().__init__('hw_data_collector')
        self.rows = []
        self.start = time.monotonic()
        self.latest_js = {{}}
        self.latest_force = {{}}
        self.latest_status = {{}}

        self.create_subscription(
            __import__('sensor_msgs.msg', fromlist=['JointState']).JointState,
            '/joint_states', self._on_js, 10)
        self.create_subscription(
            __import__('mia_hand_msgs.msg', fromlist=['ForceData']).ForceData,
            'data_streams/fingers/forces/data', self._on_force, 10)
        self.create_subscription(
            __import__('mia_hand_msgs.msg', fromlist=['ForceControllerStatus']).ForceControllerStatus,
            '/force_controller/status', self._on_status, 10)

    def _on_js(self, msg):
        for i, n in enumerate(msg.name):
            self.latest_js[n] = msg.position[i]

    def _on_force(self, msg):
        self.latest_force = {{
            'thumb_nfor': msg.thumb_nfor, 'index_nfor': msg.index_nfor, 'mrl_nfor': msg.mrl_nfor,
            'thumb_tfor': msg.thumb_tfor, 'index_tfor': msg.index_tfor, 'mrl_tfor': msg.mrl_tfor,
        }}

    def _on_status(self, msg):
        self.latest_status = {{
            'active': msg.active,
            'force_stable': msg.force_stable,
            'slip_detected': msg.slip_detected,
            'state': msg.state,
            'err_thumb': msg.force_errors[0], 'err_index': msg.force_errors[1], 'err_mrl': msg.force_errors[2],
            'nfor_thumb': msg.current_normal_forces[0], 'nfor_index': msg.current_normal_forces[1], 'nfor_mrl': msg.current_normal_forces[2],
        }}

    def timer_cb(self):
        elapsed = time.monotonic() - self.start
        row = {{'elapsed_s': f'{{elapsed:.3f}}'}}
        row.update(self.latest_js)
        row.update(self.latest_force)
        row.update(self.latest_status)
        self.rows.append(row)
        if elapsed >= {duration_s}:
            self._write()
            raise SystemExit

    def _write(self):
        if not self.rows:
            return
        keys = list(self.rows[0].keys())
        with open('{output_csv}', 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(self.rows)

rclpy.init()
node = Collector()
node.create_timer(0.1, node.timer_cb)
try:
    rclpy.spin(node)
except SystemExit:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
'''
    proc = subprocess.Popen(
        ["bash", "-c", f"{SETUP_CMD} && python3 -c {repr(collector_script)}"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    # Wait with timeout (+ grace period)
    try:
        proc.wait(timeout=duration_s + 5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()

    return output_csv


# ── Live Monitor ─────────────────────────────────────────────────────────────

def _monitor_live(duration_s: float, pipeline_sequence: Optional[list[int]] = None) -> list[dict]:
    """Monitor forces and status live, printing a table. Returns collected rows.

    Args:
        duration_s: How long to monitor.
        pipeline_sequence: Optional list of pipeline states to inject sequentially
            with 0.5s delays (e.g. [3, 4] for APPROACHING -> GRASPING).
            Published from within the ROS context to avoid race conditions with
            the pipeline manager's latched topic.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int32
    from std_srvs.srv import SetBool
    from mia_hand_msgs.msg import ForceData, ForceControllerStatus

    rows: list[dict] = []

    rclpy.init()
    node = Node("hw_live_monitor")

    # Publisher for pipeline state injection (same ROS context as subscriber)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state_pub = node.create_publisher(Int32, "/pipeline/state", latched)

    # Activate force streaming via the driver service
    force_switch = node.create_client(SetBool, "data_streams/fingers/forces/switch")
    if force_switch.wait_for_service(timeout_sec=3.0):
        req = SetBool.Request()
        req.data = True
        future = force_switch.call_async(req)
        # Spin briefly to process the response
        start_wait = time.monotonic()
        while not future.done() and (time.monotonic() - start_wait) < 2.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if future.done() and future.result().success:
            print(f"  {_c(C.GREEN, 'Force streaming activated.')}")
        else:
            print(f"  {_c(C.YELLOW, 'WARN: Could not activate force streaming.')}")
    else:
        print(f"  {_c(C.YELLOW, 'WARN: Force stream switch service not available.')}")

    state = {
        "js": {}, "force": {}, "status": {},
        "js_recv": False, "force_recv": False, "status_recv": False,
    }

    def on_js(msg):
        for i, n in enumerate(msg.name):
            state["js"][n] = msg.position[i]
        state["js_recv"] = True

    def on_force(msg):
        state["force"] = {
            "thumb_n": msg.thumb_nfor, "index_n": msg.index_nfor, "mrl_n": msg.mrl_nfor,
            "thumb_t": msg.thumb_tfor, "index_t": msg.index_tfor, "mrl_t": msg.mrl_tfor,
        }
        state["force_recv"] = True

    def on_status(msg):
        state["status"] = {
            "active": msg.active,
            "stable": msg.force_stable,
            "slip": msg.slip_detected,
            "state": msg.state,
            "errors": list(msg.force_errors),
            "normals": list(msg.current_normal_forces),
        }
        state["status_recv"] = True

    node.create_subscription(JointState, "/joint_states", on_js, 10)
    node.create_subscription(ForceData, "data_streams/fingers/forces/data", on_force, 10)
    node.create_subscription(ForceControllerStatus, "/force_controller/status", on_status, 10)

    start = time.monotonic()
    print()
    print(f"  {_c(C.CYAN, 'Monitoring for %.1f seconds...' % duration_s)}")
    print(f"  {_c(C.DIM, 'Press Ctrl+C to stop early')}")
    print()

    # Header
    header = (
        f"  {'Time':>6s} | {'Thumb_N':>8s} {'Index_N':>8s} {'MRL_N':>8s} | "
        f"{'Thumb_t':>8s} {'Index_t':>8s} {'MRL_t':>8s} | "
        f"{'Active':>6s} {'Stable':>6s} {'Slip':>5s} | {'State':>12s}"
    )
    print(header)
    print("  " + "-" * 90)

    # Track injection timing — each state gets injected, then continuously
    # re-published to counter the pipeline manager's latched IDLE messages.
    inject_times: list[float] = []
    inject_done: list[bool] = []
    current_inject_state: list[Optional[int]] = [None]  # mutable container for closure
    if pipeline_sequence:
        for i, st_val in enumerate(pipeline_sequence):
            inject_times.append(start + 1.0 + i * 1.0)  # 1s apart, starting at t=1s
            inject_done.append(False)

    def timer_cb():
        nonlocal inject_done
        elapsed = time.monotonic() - start
        if elapsed >= duration_s:
            raise SystemExit

        # Inject pipeline states at scheduled times, then keep re-publishing
        if pipeline_sequence:
            now = time.monotonic()
            for i, (t, st_val) in enumerate(zip(inject_times, pipeline_sequence)):
                if not inject_done[i] and now >= t:
                    current_inject_state[0] = st_val
                    inject_done[i] = True
                    print(f"\n  {_c(C.BLUE, '[inject] %s (%d)' % (STATE_NAMES.get(st_val, '?'), st_val))}")

            # Re-publish current state every tick to counter pipeline manager's
            # latched TRANSIENT_LOCAL IDLE messages
            if current_inject_state[0] is not None:
                msg = Int32()
                msg.data = current_inject_state[0]
                state_pub.publish(msg)

        f = state["force"]
        s = state["status"]
        js = state["js"]

        # Emergency check
        if f:
            max_force = max(f.get("thumb_n", 0), f.get("index_n", 0), f.get("mrl_n", 0))
            if max_force > EMERGENCY_FORCE:
                print()
                print(_c(C.RED, f"  EMERGENCY: Force {max_force} exceeds {EMERGENCY_FORCE}!"))
                emerg = Int32()
                emerg.data = STATE_RELEASING
                state_pub.publish(emerg)
                time.sleep(0.5)
                emerg.data = STATE_IDLE
                state_pub.publish(emerg)
                raise SystemExit

        row = {
            "elapsed_s": round(elapsed, 3),
            "j_thumb": js.get("j_thumb_fle", ""),
            "j_index": js.get("j_index_fle", ""),
            "j_mrl": js.get("j_mrl_fle", ""),
            "thumb_nfor": f.get("thumb_n", ""),
            "index_nfor": f.get("index_n", ""),
            "mrl_nfor": f.get("mrl_n", ""),
            "thumb_tfor": f.get("thumb_t", ""),
            "index_tfor": f.get("index_t", ""),
            "mrl_tfor": f.get("mrl_t", ""),
            "active": s.get("active", ""),
            "stable": s.get("stable", ""),
            "slip": s.get("slip", ""),
            "state": s.get("state", ""),
            "err_thumb": s.get("errors", ["", "", ""])[0],
            "err_index": s.get("errors", ["", "", ""])[1],
            "err_mrl": s.get("errors", ["", "", ""])[2],
        }
        rows.append(row)

        # Print live line
        tn = f.get("thumb_n", "-")
        in_ = f.get("index_n", "-")
        mn = f.get("mrl_n", "-")
        tt = f.get("thumb_t", "-")
        it = f.get("index_t", "-")
        mt = f.get("mrl_t", "-")
        act_s = "YES" if s.get("active") else "no"
        stab_s = "YES" if s.get("stable") else "no"
        slip_s = "YES" if s.get("slip") else "no"
        st = s.get("state", "-")

        print(
            f"  {elapsed:6.1f} | {tn:>8} {in_:>8} {mn:>8} | "
            f"{tt:>8} {it:>8} {mt:>8} | "
            f"{act_s:>6} {stab_s:>6} {slip_s:>5} | {st:>12}"
            + "   ", end="\r"
        )

    node.create_timer(0.1, timer_cb)

    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print()  # newline after \r
    return rows


def _save_rows(rows: list[dict], prefix: str) -> Optional[str]:
    """Save collected rows to a timestamped CSV. Returns path or None."""
    if not rows:
        return None
    os.makedirs(RESULTS_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(RESULTS_DIR, f"{prefix}_{ts}.csv")
    keys = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    return path


def _summarize(rows: list[dict]):
    """Print a summary of collected force data."""
    if not rows:
        print(_c(C.YELLOW, "  No data collected."))
        return

    print()
    print(f"  {_c(C.BOLD, 'Summary (%d samples)' % len(rows))}")

    # Per-finger normal forces
    print(f"  {'Finger':>8s} | {'Min':>8s} {'Max':>8s} {'Mean':>8s} {'Last':>8s}")
    print(f"  {'':->8s}-+-{'':->8s}-{'':->8s}-{'':->8s}-{'':->8s}")
    for finger, key in [("Thumb", "thumb_nfor"), ("Index", "index_nfor"), ("MRL", "mrl_nfor")]:
        vals = [r[key] for r in rows if isinstance(r.get(key), (int, float))]
        if vals:
            print(
                f"  {finger:>8s} | {min(vals):8.1f} {max(vals):8.1f} "
                f"{sum(vals)/len(vals):8.1f} {vals[-1]:8.1f}"
            )

    # Controller status
    active_count = sum(1 for r in rows if r.get("active"))
    stable_count = sum(1 for r in rows if r.get("stable"))
    slip_count = sum(1 for r in rows if r.get("slip"))
    print()
    print(
        f"  Controller active: {active_count}/{len(rows)} samples, "
        f"stable: {stable_count}, slip: {slip_count}"
    )


# ── Pre-flight Checks ────────────────────────────────────────────────────────

def preflight() -> bool:
    """Check prerequisites. Returns True if OK to proceed."""
    print(_c(C.BOLD, "\n  Pre-flight checks"))
    print("  " + "-" * 40)

    ok = True

    # 1. Check /dev/ttyUSB0
    if os.path.exists("/dev/ttyUSB0"):
        print(f"  {_c(C.GREEN, 'OK')} /dev/ttyUSB0 exists")
    else:
        print(f"  {_c(C.RED, 'FAIL')} /dev/ttyUSB0 not found")
        ok = False

    # 2. Check driver node
    nodes = get_node_list()
    if "/driver" in nodes:
        print(f"  {_c(C.GREEN, 'OK')} /driver node running")
    else:
        print(f"  {_c(C.RED, 'FAIL')} /driver node not found")
        print(f"         Start it: ros2 run mia_hand_driver mia_hand_driver_node --ros-args -p serial_port:=/dev/ttyUSB0")
        ok = False

    # 3. Check force data topic exists
    topics = get_topic_list()
    if "data_streams/fingers/forces/data" in topics:
        print(f"  {_c(C.GREEN, 'OK')} Force data topic available")
    else:
        print(f"  {_c(C.YELLOW, 'WARN')} Force data topic not yet available (will activate later)")

    # 4. Check joint position topic
    if "data_streams/joints/positions/data" in topics:
        print(f"  {_c(C.GREEN, 'OK')} Joint position data topic available")
    else:
        print(f"  {_c(C.YELLOW, 'WARN')} Joint position data topic not yet available")

    print()
    return ok


# ── Auto-launch ──────────────────────────────────────────────────────────────

def auto_launch(mgr: NodeManager, include_pipeline_manager: bool = True) -> bool:
    """Launch command_bridge, force_controller, and optionally pipeline_manager.

    Skips any node that is already running (e.g. user started it manually).
    The pipeline manager is NOT launched by default for force controller testing,
    because it publishes IDLE on /pipeline/state at 5 Hz with TRANSIENT_LOCAL
    durability, which overrides injected test states.
    """
    print(_c(C.BOLD, "  Launching nodes..."))
    print()

    active = get_node_list()
    nodes_to_launch = {
        "command_bridge": (
            "/command_bridge",
            "ros2 run command_bridge command_bridge_node --ros-params-file " + CONFIG_PATH,
        ),
        "force_controller": (
            "/force_controller",
            "ros2 run force_controller force_controller_node --ros-params-file " + CONFIG_PATH,
        ),
    }
    if include_pipeline_manager:
        nodes_to_launch["pipeline_manager"] = (
            "/pipeline_manager",
            "ros2 run pipeline_manager pipeline_manager_node --ros-params-file " + CONFIG_PATH,
        )

    for name, (ros_name, cmd) in nodes_to_launch.items():
        if ros_name in active:
            print(f"  {_c(C.GREEN, 'SKIP')} {name} already running")
        else:
            mgr.launch(name, cmd)
            print(f"  {_c(C.DIM, 'Started ' + name)}")
            time.sleep(1)

    print()
    print("  Waiting for nodes to come up...")
    expected = [n for n in ["/command_bridge", "/force_controller", "/pipeline_manager"]
                if n in {ros_name for ros_name, _ in nodes_to_launch.values()}]
    if mgr.wait_for_nodes(expected, timeout=20.0):
        print(f"  {_c(C.GREEN, 'All nodes running.')}")
        if not include_pipeline_manager:
            print(_c(C.DIM, "  Note: pipeline_manager not launched (would conflict with state injection)"))
        return True
    else:
        active = get_node_list()
        missing = [n for n in expected if n not in active]
        print(f"  {_c(C.RED, 'Some nodes failed to start: %s' % missing)}")
        return False


# ── Menu Actions ─────────────────────────────────────────────────────────────

def action_finger_jog():
    """Move a single finger to a target angle."""
    print()
    print(_c(C.CYAN, "  Finger Jog"))
    print(f"  Fingers: {', '.join(FINGER_NAMES)}")
    finger = input("  Finger name: ").strip().lower()
    if finger not in FINGER_NAMES:
        print(_c(C.RED, f"  Unknown finger '{finger}'. Use: {FINGER_NAMES}"))
        return

    angle_str = input("  Target angle (rad, e.g. 0.5): ").strip()
    try:
        angle = float(angle_str)
    except ValueError:
        print(_c(C.RED, "  Invalid number."))
        return

    topic = f"/{finger}_pos_ff_controller/commands"
    print(f"  Sending {angle:.3f} rad to {topic}...")
    publish_float64(topic, angle)
    print(_c(C.GREEN, "  Sent."))


def action_open_hand():
    """Open all fingers to 0.0."""
    print()
    print(_c(C.CYAN, "  Opening hand..."))
    for finger in FINGER_NAMES:
        publish_float64(f"/{finger}_pos_ff_controller/commands", 0.0)
    print(_c(C.GREEN, "  Done."))


def action_close_hand():
    """Close all fingers to a specified amount."""
    print()
    angle_str = input("  Closure amount in rad [1.5]: ").strip()
    try:
        angle = float(angle_str) if angle_str else 1.5
    except ValueError:
        angle = 1.5

    print(_c(C.CYAN, f"  Closing hand to {angle:.2f} rad..."))
    for finger in FINGER_NAMES:
        publish_float64(f"/{finger}_pos_ff_controller/commands", angle)
    print(_c(C.GREEN, "  Done."))


def action_force_monitor():
    """Monitor forces and status for N seconds."""
    print()
    dur_str = input("  Duration in seconds [5]: ").strip()
    try:
        duration = float(dur_str) if dur_str else 5.0
    except ValueError:
        duration = 5.0

    rows = _monitor_live(duration)
    path = _save_rows(rows, "force_monitor")
    _summarize(rows)
    if path:
        print(f"\n  {_c(C.DIM, 'Data saved: ' + path)}")


def action_activate_force_controller():
    """Inject APPROACHING -> GRASPING to activate force controller, then monitor."""
    print()
    dur_str = input("  Monitor duration in seconds [10]: ").strip()
    try:
        duration = float(dur_str) if dur_str else 10.0
    except ValueError:
        duration = 10.0

    print(_c(C.CYAN, "  Will inject APPROACHING -> GRASPING at t=1s and t=2s..."))
    print(_c(C.DIM, "  (states are continuously re-published to override pipeline manager)"))
    rows = _monitor_live(duration, pipeline_sequence=[STATE_APPROACHING, STATE_GRASPING])
    path = _save_rows(rows, "force_active")
    _summarize(rows)
    if path:
        print(f"\n  {_c(C.DIM, 'Data saved: ' + path)}")


def action_release():
    """Inject RELEASING -> IDLE."""
    print()
    print(_c(C.CYAN, "  Injecting RELEASING (6)..."))
    publish_int("/pipeline/state", STATE_RELEASING)
    time.sleep(0.5)
    print(_c(C.CYAN, "  Injecting IDLE (0)..."))
    publish_int("/pipeline/state", STATE_IDLE)
    print(_c(C.GREEN, "  Released."))


def action_inject_state():
    """Manually inject a pipeline state."""
    print()
    print("  States:")
    for val, name in STATE_NAMES.items():
        print(f"    {val} = {name}")
    state_str = input("  State value: ").strip()
    try:
        state_val = int(state_str)
    except ValueError:
        print(_c(C.RED, "  Invalid number."))
        return

    if state_val not in STATE_NAMES:
        print(_c(C.RED, "  Unknown state."))
        return

    print(_c(C.CYAN, f"  Injecting {STATE_NAMES[state_val]} ({state_val})..."))
    publish_int("/pipeline/state", state_val)
    print(_c(C.GREEN, "  Done."))


def action_full_grasp_routine():
    """Orchestrated grasp sequence."""
    print()
    print(_c(C.BOLD, "  Full Grasp Routine"))
    print("  This will:")
    print("    1. Close hand partially (approach position)")
    print("    2. Inject APPROACHING -> GRASPING + monitor force regulation")
    print("    3. Inject RELEASING -> IDLE")
    print("    4. Open hand")
    print()

    close_str = input("  Approach closure (rad) [0.8]: ").strip()
    try:
        close_angle = float(close_str) if close_str else 0.8
    except ValueError:
        close_angle = 0.8

    dur_str = input("  Grasp duration (seconds) [8]: ").strip()
    try:
        grasp_dur = float(dur_str) if dur_str else 8.0
    except ValueError:
        grasp_dur = 8.0

    input(f"  {_c(C.YELLOW, 'Press Enter to start...')}")

    # Step 1: Partial close
    print()
    print(_c(C.CYAN, f"  [1/5] Closing to {close_angle:.2f} rad (approach)..."))
    for finger in FINGER_NAMES:
        publish_float64(f"/{finger}_pos_ff_controller/commands", close_angle)
    time.sleep(2.0)

    # Step 2-3: Monitor with state injection (continuously re-publishes to override PM)
    print(_c(C.CYAN, f"  [2/4] Monitoring with APPROACHING->GRASPING injection..."))
    rows = _monitor_live(grasp_dur, pipeline_sequence=[STATE_APPROACHING, STATE_GRASPING])

    # Step 3: Release
    print(_c(C.CYAN, "  [3/4] Releasing..."))
    publish_int("/pipeline/state", STATE_RELEASING)
    time.sleep(0.5)
    publish_int("/pipeline/state", STATE_IDLE)
    time.sleep(0.5)

    # Step 4: Open hand
    for finger in FINGER_NAMES:
        publish_float64(f"/{finger}_pos_ff_controller/commands", 0.0)
    print(_c(C.GREEN, "  Hand opened."))

    # Summary
    path = _save_rows(rows, "full_grasp")
    _summarize(rows)
    if path:
        print(f"\n  {_c(C.DIM, 'Data saved: ' + path)}")


def action_calibration():
    """Stream raw joint positions and force readings for calibration."""
    print()
    print(_c(C.BOLD, "  Calibration Helper"))
    print("  Streaming raw readings for 5 seconds.")
    print("  Use this to note baselines (open hand, touching object, etc.).")
    print()
    input(f"  {_c(C.YELLOW, 'Press Enter to start...')}")

    rows = _monitor_live(5.0)
    path = _save_rows(rows, "calibration")

    print()
    print(_c(C.BOLD, "  Calibration Summary"))
    if rows:
        last = rows[-1]
        print(f"  Last joint positions:")
        for jn in JOINT_NAMES:
            val = last.get(jn, "?")
            print(f"    {jn}: {val}")
        print(f"  Last normal forces:")
        for fn in ["thumb_nfor", "index_nfor", "mrl_nfor"]:
            val = last.get(fn, "?")
            print(f"    {fn}: {val}")

    if path:
        print(f"\n  {_c(C.DIM, 'Data saved: ' + path)}")


# ── Main Menu ────────────────────────────────────────────────────────────────

MENU = """
  ══════════════════════════════════════════════════════════
  Hardware Test — Force Controller & Pipeline Manager
  ══════════════════════════════════════════════════════════

   1  Finger Jog         Move one finger to target angle
   2  Open Hand          Send 0.0 to all fingers
   3  Close Hand         Send closure to all fingers
   4  Force Monitor      Watch forces + status for N seconds
   5  Activate FC        Inject APPROACHING->GRASPING, monitor
   6  Release            Inject RELEASING->IDLE
   7  Inject State       Manually inject any pipeline state
   8  Full Grasp         Orchestrated grasp routine
   9  Calibration        Stream raw readings for 5 seconds
   0  Quit               Kill nodes and exit

  ══════════════════════════════════════════════════════════
"""

ACTIONS = {
    "1": action_finger_jog,
    "2": action_open_hand,
    "3": action_close_hand,
    "4": action_force_monitor,
    "5": action_activate_force_controller,
    "6": action_release,
    "7": action_inject_state,
    "8": action_full_grasp_routine,
    "9": action_calibration,
}


def main():
    print(_c(C.BOLD, "\n  Mia Hand — Hardware Test Suite"))
    print(_c(C.DIM, "  tests/test2_hw_force_controller/hw_test.py"))
    print()

    # Pre-flight
    if not preflight():
        print(_c(C.RED, "\n  Pre-flight failed. Fix the issues above and try again."))
        sys.exit(1)

    # Auto-launch
    mgr = NodeManager()
    atexit.register(mgr.kill_all)

    if not auto_launch(mgr, include_pipeline_manager=False):
        print(_c(C.RED, "\n  Failed to launch nodes. Check logs above."))
        mgr.kill_all()
        sys.exit(1)

    # Main loop
    while True:
        print(MENU)
        # Show current pipeline state
        nodes = get_node_list()
        running = [n.strip("/") for n in nodes if n.strip("/") in
                   ["driver", "command_bridge", "force_controller", "pipeline_manager"]]
        print(f"  Active nodes: {_c(C.GREEN, ', '.join(sorted(running)))}")

        try:
            choice = input("  Choice: ").strip()
        except (KeyboardInterrupt, EOFError):
            print()
            break

        if choice == "0":
            print(_c(C.CYAN, "\n  Shutting down..."))
            break

        action = ACTIONS.get(choice)
        if action:
            try:
                action()
            except (KeyboardInterrupt, EOFError):
                print()
                print(_c(C.YELLOW, "  Interrupted."))
                # Emergency release on interrupt during force control
                publish_int("/pipeline/state", STATE_RELEASING)
                time.sleep(0.3)
                publish_int("/pipeline/state", STATE_IDLE)
        else:
            print(_c(C.RED, "  Unknown option."))

        input(f"\n  {_c(C.DIM, 'Press Enter to continue...')}")

    # Cleanup
    mgr.kill_all()
    print(_c(C.GREEN, "  Done. All nodes stopped."))


if __name__ == "__main__":
    main()
