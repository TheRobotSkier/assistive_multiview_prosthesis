#!/usr/bin/env python3
"""analyze_log.py — summarize a prosthesis host-log for fast debugging.

Parses a ROS 2 Jazzy launch log (logs/host-log-<ts>.txt) and produces a
structured text report:

  1. Run header  — nodes started, launch args, wall-clock span
  2. Severity tally per node
  3. Known-failure digest — pattern-matches the recurring failure modes,
     one line each: count + first/last timestamp + root-cause hint
  4. Diagnostics timeline — turns the 5s ``[DIAG-*]`` blocks emitted by
     pipeline_diagnostics_node into a compact time-series (topic rates,
     clock offset, pose-norm divergence, TF-chain flips, TF-jump bursts)
  5. First/last ERROR context
  6. Crash / process-death detection

Pure stdlib — no external dependencies. Optional ``--plot`` writes PNG
timelines if matplotlib is available.

Usage:
  python3 scripts/analyze_log.py                          # latest log
  python3 scripts/analyze_log.py logs/host-log-<ts>.txt   # specific log
  python3 scripts/analyze_log.py --plot                   # also write PNGs
  python3 scripts/analyze_log.py --all                    # summarize all logs
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import sys
from collections import defaultdict
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

LOGS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")

# Thresholds reused from pipeline_diagnostics_node (jump=0.05m) and the
# keyframe_buffer (norm warn=1.0m, crit=2.0m) so offline matches live.
TF_JUMP_THRESHOLD_M = 0.05
NORM_WARN_M = 1.0
NORM_CRIT_M = 2.0

# ---------------------------------------------------------------------------
# Regexes
# ---------------------------------------------------------------------------

# [node_name-N] [LEVEL] [epoch.ts] [logger]: message
# Capture: node-prefix (optional), level, timestamp (optional), rest
# NOTE: LEVEL also matches pseudo-levels like DIAG-TF, DIAG-RATE, DIAG-NORM,
# DIAG-CHAIN, DIAG-STALL emitted by pipeline_diagnostics_node.
RE_LOG_LINE = re.compile(
    r"^(?:\[([a-zA-Z0-9_.\-]+)-(\d+)\]\s+)?"     # [node-1]  (optional)
    r"\[(DEBUG|INFO|WARN|ERROR|FATAL|DIAG-[A-Z]+|MARKER_REJECT|STARTUP|DRIFT-INCIDENT)\]\s*"  # [LEVEL]
    r"(?:\[(\d+\.\d+)\]\s*)?"                     # [epoch.ts] (optional)
    r"(?:\[([^\]]*)\]:\s*)?"                      # [logger]: (optional)
    r"(.*)$"                                       # message
)
# Lines without the standard level tag (e.g. "[pipeline] fusion_mode=...").
RE_BARE_LINE = re.compile(r"^\[([a-zA-Z_]+)\]\s+(.*)$")
# Diag continuation line: only a [node-N] prefix then indented data
# e.g. "[pipeline_diagnostics_node-7]   /jetson/head/points: 3.9 Hz ..."
RE_DIAG_CONT = re.compile(r"^\[([a-zA-Z0-9_.\-]+)-\d+\]\s+(.*)$")
RE_PROC_START = re.compile(r"process started with pid \[(\d+)\]")
RE_PROC_DEATH = re.compile(r"process has died|process has finished|core dumped|Traceback")

RE_TF_JUMP = re.compile(
    r"TF JUMP: ([a-zA-Z0-9_>\- ]+?) jumped ([\d.]+)m \(stamp=([\d.]+), age=([\d.]+)s\)"
)
RE_ODOM_SUPPRESS = re.compile(
    r"(head|arm) odom pose jumped ([\d.]+)m \(> ([\d.]+)m\) — suppressing TF"
)
RE_VELOCITY_SUPPRESS = re.compile(
    r"(head|arm) odom velocity ([\d.]+)m/s exceeds max ([\d.]+)m/s — suppressing TF "
    r"\(total=(\d+)\)"
)
RE_GTSAM_REJECT = re.compile(
    r"Rejecting odometry delta — head jump ([\d.]+)m, arm jump ([\d.]+)m exceed max ([\d.]+)m"
)
RE_TSDF_TIMEOUT = re.compile(r"GetAllKeyframes call timed out")
RE_TFDIAG_CHAIN = re.compile(r"\[TF-DIAG\].*?(OpenVINS\(head\):(\w+).*?OpenVINS\(arm\):(\w+))")
RE_RELAY_STATUS = re.compile(r"\[RELAY-STATUS\].*?head: (\w+).*?arm: (\w+)")

# Diagnostics block sections
RE_DIAG_TF_EDGE = re.compile(
    r"([a-zA-Z0-9_>\- ]+?):\s+updates=(\d+)\s+jumps=(\d+)\s+max_jump=([\d.]+)m\s+age=([\d.]+)s"
)
RE_DIAG_RATE = re.compile(
    r"([a-zA-Z0-9_/]+):\s+([\d.]+)\s+Hz\s+\(total=(\d+),\s*clock_off=([\-\d.]+)s?\)?"
)
RE_DIAG_NORM = re.compile(r"(odom|gtsam)\s+([a-zA-Z0-9_/]+):\s+(.*)")
RE_DIAG_CHAIN_LINE = re.compile(r"([a-zA-Z0-9_> ]+?):\s+(CONNECTED|OK|DISCONNECTED|STALE)")
RE_DIAG_PC_LINE = re.compile(r"(head|arm):\s+(.*)")
RE_STARTUP_EVENT = re.compile(r"\[STARTUP\]\s+phase=([a-zA-Z0-9_\-]+)\s*(.*)")
RE_DRIFT_INCIDENT = re.compile(
    r"\[DRIFT-INCIDENT\]\s+side=(\w+)\s+reason=([a-zA-Z0-9_\-]+)\s+"
    r"pos_norm=([\d.]+)m\s+speed=([\d.]+)mps(?:\s+pos_slope=([\d.]+)mps)?"
)
RE_ODOM_NOT_READY = re.compile(r"No OpenVINS odom received yet")

# Feature 1: rclcpp QoS contract warnings
RE_QOS_WARN = re.compile(
    r"New (publisher|subscription) discovered on topic '([^']+)',\s*"
    r"(offering|requesting) incompatible QoS\.\s*"
    r"No messages will be (received from|sent to) it\.\s*"
    r"Last incompatible policy:\s*(\w+)"
)

# Feature 2: VIO anchor / marker correction diagnostics from aruco_marker_pose_node
RE_VIO_DIAG = re.compile(
    r"\[DIAG\]\s+markers_detected=(\d+)\s+corrections=(\d+)/(\d+)"
    r"\s+vio=(\w+)\s+pos_norm=([\d.]+)m\s+odom=(\w+)\s+locked=(\w+)"
)

# Feature 3: Structured marker-rejection lines from C++ updaters + Python loops
# Format: [MARKER_REJECT] side=arm type=chi2 val=54.21 lim=50.00
# The [MARKER_REJECT] prefix is optional because RE_LOG_LINE consumes it as a
# level token, leaving msg="side=arm type=chi2 ...".  Making the prefix
# optional lets the same regex match both the full line (pseudo-level path)
# and the stripped message (main path).
RE_MARKER_REJECT = re.compile(
    r"(?:\[MARKER_REJECT\]\s+)?side=(\w+)\s+type=(\w+)\s+val=([\d.]+)\s+lim=([\d.]+)"
)

# Fusion node Stats line (pointcloud_fusion_node).
# Format: Stats: published=N (dual=X, cam1_only=Y) received(cam1=A, cam2=B) ...
RE_FUSION_STATS = re.compile(
    r"published=(\d+)\s+\(dual=(\d+),\s+cam1_only=(\d+)\)"
)

# Topics exempt from transport-latency / clock-offset analysis because their
# large clock deltas are an artifact of latched (TRANSIENT_LOCAL) durability,
# not real network latency.  These topics are published once and held, so the
# delta between the original stamp and a late subscriber match is meaningless.
LATCHED_TOPICS = {"/tf_static"}


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class FailureMode:
    name: str
    pattern_re: re.Pattern
    extract: callable
    hint: str
    count: int = 0
    first_ts: float | None = None
    last_ts: float | None = None
    samples: list = field(default_factory=list)


@dataclass
class QoSHit:
    """A single rclcpp QoS incompatibility warning."""
    ts: float | None
    topic: str
    node: str
    direction: str       # "publisher" or "subscription"
    action: str          # "offering" or "requesting"
    blocked_dir: str     # "received from" or "sent to"
    policy: str          # e.g. "RELIABILITY_QOS_POLICY"


@dataclass
class VioAnchorStats:
    """Aggregated VIO marker anchor yield from [DIAG] log lines."""
    head_markers: int = 0
    head_corrections: int = 0
    arm_markers: int = 0
    arm_corrections: int = 0
    head_locked_ever: bool = False
    arm_locked_ever: bool = False
    # Last-seen instantaneous snapshots from the [DIAG] line.  markers/
    # corrections are cumulative sums; these are the most recent values, which
    # carry the key end-of-run signal (e.g. vio=INVALID pos_norm=7.77m).
    head_vio_status: str = "unknown"
    head_pos_norm: float = 0.0
    arm_vio_status: str = "unknown"
    arm_pos_norm: float = 0.0


@dataclass
class MarkerRejection:
    """A single [MARKER_REJECT] structured log line."""
    ts: float | None
    side: str          # "head" or "arm"
    rtype: str         # "chi2", "translation", "rotation", "velocity_fit"
    val: float         # measured value
    lim: float         # gate limit


@dataclass
class FusionStats:
    """A single pointcloud_fusion Stats interval."""
    ts: float | None
    published: int
    dual: int
    cam1_only: int

    @property
    def dual_ratio(self) -> float:
        return self.dual / max(self.dual + self.cam1_only, 1)


@dataclass
class DriftIncident:
    ts: float | None
    side: str
    reason: str
    pos_norm_m: float
    speed_mps: float
    pos_slope_mps: float | None


@dataclass
class StartupEvent:
    ts: float | None
    phase: str
    details: str


@dataclass
class DiagBlock:
    t: float
    rates: dict = field(default_factory=dict)        # topic -> (hz, total, clock_off)
    norms: dict = field(default_factory=dict)        # (kind, topic) -> norm_str
    chains: dict = field(default_factory=dict)       # edge -> status
    tf_edges: dict = field(default_factory=dict)     # edge -> (updates, jumps, max_jump, age)
    pointcloud: dict = field(default_factory=dict)   # side -> parsed DIAG-PC keyvals


@dataclass
class LogReport:
    path: str
    lines: list = field(default_factory=list)
    n_lines: int = 0
    nodes_started: dict = field(default_factory=dict)   # name -> pid
    severity_per_node: dict = field(default_factory=lambda: defaultdict(lambda: defaultdict(int)))
    failures: list = field(default_factory=list)        # FailureMode list
    diag_blocks: list = field(default_factory=list)     # DiagBlock list
    tf_jump_events: list = field(default_factory=list)  # (ts, edge, jump_m)
    errors: list = field(default_factory=list)          # (ts, node, logger, msg)
    crashes: list = field(default_factory=list)         # (line_no, text)
    qos_hits: list = field(default_factory=list)        # QoSHit list
    marker_rejections: list = field(default_factory=list)  # MarkerRejection list
    drift_incidents: list = field(default_factory=list)     # DriftIncident list
    startup_events: list = field(default_factory=list)      # StartupEvent list
    odom_not_ready_hits: list = field(default_factory=list) # timestamps for startup-only warnings
    fusion_stats: list = field(default_factory=list)        # FusionStats list
    vio_stats: VioAnchorStats = field(default_factory=VioAnchorStats)
    t_start: float | None = None
    t_end: float | None = None
    launch_args: list = field(default_factory=list)


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------

def _parse_diag_pc_metrics(payload: str) -> dict:
    """Parse compact pointcloud DIAG payload key=value tokens.

    Actual DIAG-PC format (from pipeline_diagnostics_node):
      ``depth_c=3.3Hz image_c=4.4Hz -> depth_r=3.6Hz image_r=4.5Hz
      ci_raw=15.5Hz -> points=0.0Hz (n=0) [SYNC_DROP]``

    The ``->`` arrows are stage separators (skipped).  ``(n=N)`` is the
    pointcloud count.  ``[REASON]`` in trailing brackets is the drop reason.
    """
    out: dict[str, float | int | str] = {}
    # Extract trailing [REASON] bracket (e.g. [SYNC_DROP], [OK]).
    reason = ""
    bracket_match = re.search(r"\[([A-Z_]+)\]\s*$", payload)
    if bracket_match:
        reason = bracket_match.group(1)
    for tok in payload.split():
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        k = k.strip().lstrip("(")  # handle "(n=0)" -> key "n"
        v = v.strip().rstrip(",)").rstrip("]")
        if not v:
            continue
        if v.endswith("Hz"):
            v = v[:-2]
        if v.endswith("%"):
            v = v[:-1]
        try:
            if "." in v:
                out[k] = float(v)
            else:
                out[k] = int(v)
        except ValueError:
            out[k] = v
    if reason:
        out["reason"] = reason
    return out


def _parse_diag_data(msg: str, section: str | None, diag: DiagBlock):
    """Parse a DIAG block data line into the current DiagBlock."""
    if section is None or diag is None:
        return
    me = RE_DIAG_TF_EDGE.search(msg)
    if me and section == "tf_dyn":
        edge = me.group(1).strip()
        diag.tf_edges[edge] = (
            int(me.group(2)), int(me.group(3)),
            float(me.group(4)), float(me.group(5)))
    mr = RE_DIAG_RATE.search(msg)
    if mr and section == "rate":
        diag.rates[mr.group(1)] = (
            float(mr.group(2)), int(mr.group(3)), float(mr.group(4)))
    mn = RE_DIAG_NORM.search(msg)
    if mn and section == "norm":
        diag.norms[(mn.group(1), mn.group(2))] = mn.group(3).strip()
    mc = RE_DIAG_CHAIN_LINE.search(msg)
    if mc and section == "chain":
        diag.chains[mc.group(1).strip()] = mc.group(2)
    mp = RE_DIAG_PC_LINE.search(msg)
    if mp and section == "pc":
        diag.pointcloud[mp.group(1)] = _parse_diag_pc_metrics(mp.group(2))


def _make_failure_modes():
    def tf_jump_x(m, ts):
        return {"edge": m.group(1).strip(), "jump_m": float(m.group(2)), "stamp": float(m.group(3))}
    def odom_suppress_x(m, ts):
        return {"side": m.group(1), "jump_m": float(m.group(2)), "threshold": float(m.group(3))}
    def velocity_suppress_x(m, ts):
        return {"side": m.group(1), "velocity_mps": float(m.group(2)), "threshold": float(m.group(3))}
    def gtsam_reject_x(m, ts):
        return {"head_m": float(m.group(1)), "arm_m": float(m.group(2)), "max_m": float(m.group(3))}
    def tsdf_timeout_x(m, ts):
        return {}
    def tfdiag_chain_x(m, ts):
        return {"head": m.group(2), "arm": m.group(3)}
    def relay_status_x(m, ts):
        return {"head": m.group(1), "arm": m.group(2)}
    return [
        FailureMode("TF jump (pipeline_diagnostics)", RE_TF_JUMP, tf_jump_x,
                    "VIO/odometry discontinuity — check OpenVINS stability & clock drift"),
        FailureMode("odom pose jumped — TF suppressed", RE_ODOM_SUPPRESS, odom_suppress_x,
                    "openvins_odom_tf_relay rejecting a large VIO jump"),
        FailureMode("odom velocity exceeded — TF suppressed", RE_VELOCITY_SUPPRESS, velocity_suppress_x,
                    "openvins_odom_tf_relay rejecting high-velocity VIO jump (raw path only)"),
        FailureMode("gtsam odometry rejected", RE_GTSAM_REJECT, gtsam_reject_x,
                    "gtsam_tracker detected diverging VIO — re-syncing baseline"),
        FailureMode("tsdf GetAllKeyframes timeout", RE_TSDF_TIMEOUT, tsdf_timeout_x,
                    "keyframe_buffer service hung — check buffer load / deadlocks"),
        FailureMode("TF chain disconnected", RE_TFDIAG_CHAIN, tfdiag_chain_x,
                    "OpenVINS→RealSense TF bridge incomplete — Jetson link down?"),
        FailureMode("relay status change", RE_RELAY_STATUS, relay_status_x,
                    "openvins_odom_tf_relay head/arm message-flow state"),
        FailureMode("startup odom not ready", RE_ODOM_NOT_READY, tsdf_timeout_x,
                    "Expected during startup before OpenVINS init; only actionable if persistent after init"),
    ]


def parse_log(path: str) -> LogReport:
    rep = LogReport(path=path)
    rep.failures = _make_failure_modes()
    with open(path, "r", errors="replace") as f:
        rep.lines = f.readlines()
    rep.n_lines = len(rep.lines)

    current_diag: DiagBlock | None = None
    current_diag_node: str | None = None
    current_section: str | None = None

    for lineno, raw in enumerate(rep.lines, 1):
        line = raw.rstrip("\n")
        m = RE_LOG_LINE.match(line)
        if not m:
            # Allow pseudo-level structured lines that do not include [node-N].
            # Example: [MARKER_REJECT] side=arm type=chi2 ...
            pseudo = re.match(r"^\[(MARKER_REJECT|STARTUP|DRIFT-INCIDENT)\]\s+(.*)$", line)
            if pseudo:
                node_prefix = "structured"
                level = pseudo.group(1)
                ts = None
                logger = ""
                msg = line
                effective_node = node_prefix
                node_key = effective_node
                rep.severity_per_node[node_key][level] += 1

                rj = RE_MARKER_REJECT.search(msg)
                if rj:
                    rep.marker_rejections.append(MarkerRejection(
                        ts=ts,
                        side=rj.group(1),
                        rtype=rj.group(2),
                        val=float(rj.group(3)),
                        lim=float(rj.group(4)),
                    ))
                se = RE_STARTUP_EVENT.search(msg)
                if se:
                    rep.startup_events.append(StartupEvent(ts=ts, phase=se.group(1), details=se.group(2).strip()))
                di = RE_DRIFT_INCIDENT.search(msg)
                if di:
                    rep.drift_incidents.append(DriftIncident(
                        ts=ts,
                        side=di.group(1),
                        reason=di.group(2),
                        pos_norm_m=float(di.group(3)),
                        speed_mps=float(di.group(4)),
                        pos_slope_mps=float(di.group(5)) if di.group(5) else None,
                    ))
                continue

            # Bare line: launch arg or separator
            mb = RE_BARE_LINE.match(line)
            if mb and "=" in mb.group(2):
                rep.launch_args.append(mb.group(2).strip())
            if RE_PROC_DEATH.search(line):
                rep.crashes.append((lineno, line))
            # Diag continuation line (only [node-N] prefix, no level tag)
            md = RE_DIAG_CONT.match(line)
            if md and current_diag is not None and current_diag_node is not None:
                cont_node = md.group(1)
                if current_diag_node in cont_node:
                    _parse_diag_data(md.group(2), current_section, current_diag)
            continue

        node_prefix = m.group(1)
        level = m.group(3)
        ts_str = m.group(4)
        logger = m.group(5) or ""
        msg = m.group(6)
        ts = float(ts_str) if ts_str else None
        if ts is not None:
            if rep.t_start is None or ts < rep.t_start:
                rep.t_start = ts
            if rep.t_end is None or ts > rep.t_end:
                rep.t_end = ts

        # For process-start lines, the node name is in the logger field
        # (e.g. "[INFO] [pipeline_manager_node-1]: process started with pid").
        # Extract it so nodes_started is populated correctly.
        effective_node = node_prefix or logger or "launch"
        if not node_prefix and logger:
            mn = re.match(r"([a-zA-Z0-9_.\-]+)-\d+", logger)
            if mn:
                effective_node = mn.group(1)
        node_key = effective_node
        rep.severity_per_node[node_key][level] += 1

        # process start
        ps = RE_PROC_START.search(msg)
        if ps:
            rep.nodes_started[effective_node] = ps.group(1)

        # crash / death
        if RE_PROC_DEATH.search(msg):
            rep.crashes.append((lineno, line))

        # errors
        if level == "ERROR":
            rep.errors.append((ts, node_key, logger, msg))

        # known failures (only scan real log messages, not DIAG sections)
        if level not in ("DIAG-TF", "DIAG-RATE", "DIAG-NORM",
                          "DIAG-CHAIN", "DIAG-STALL"):
            for fm in rep.failures:
                fm_m = fm.pattern_re.search(msg)
                if fm_m:
                    fm.count += 1
                    if ts is not None:
                        if fm.first_ts is None:
                            fm.first_ts = ts
                        fm.last_ts = ts
                    if len(fm.samples) < 3:
                        fm.samples.append(fm.extract(fm_m, ts))
                    break  # one failure per line

        # TF jump events (separate time-series for the timeline)
        tj = RE_TF_JUMP.search(msg)
        if tj and ts is not None:
            rep.tf_jump_events.append((ts, tj.group(1).strip(), float(tj.group(2))))

        # Feature 1: QoS contract warnings
        qm = RE_QOS_WARN.search(msg)
        if qm and level == "WARN":
            rep.qos_hits.append(QoSHit(
                ts=ts,
                topic=qm.group(2),
                node=effective_node,
                direction=qm.group(1),
                action=qm.group(3),
                blocked_dir=qm.group(4),
                policy=qm.group(5),
            ))

        # Feature 3: Structured marker-rejection logging
        rj = RE_MARKER_REJECT.search(msg)
        if rj:
            rep.marker_rejections.append(MarkerRejection(
                ts=ts,
                side=rj.group(1),
                rtype=rj.group(2),
                val=float(rj.group(3)),
                lim=float(rj.group(4)),
            ))

        # Fusion node dual-view stats (pointcloud_fusion_node Stats lines)
        fs = RE_FUSION_STATS.search(msg)
        if fs and effective_node.startswith("pointcloud_fusion"):
            rep.fusion_stats.append(FusionStats(
                ts=ts,
                published=int(fs.group(1)),
                dual=int(fs.group(2)),
                cam1_only=int(fs.group(3)),
            ))

        # Feature 2: VIO anchor diagnostics from aruco_marker_pose_node
        vm = RE_VIO_DIAG.search(msg)
        if vm and level == "INFO":
            markers = int(vm.group(1))
            corrections = int(vm.group(2))
            locked = vm.group(7) == "True"
            vio_status = vm.group(4)   # "valid" or "INVALID"
            pos_norm = float(vm.group(5))  # position norm in meters
            # Determine head vs arm from the logger
            if "arm_phase2" in logger.lower() or "arm" in effective_node.lower():
                rep.vio_stats.arm_markers += markers
                rep.vio_stats.arm_corrections += corrections
                rep.vio_stats.arm_vio_status = vio_status
                rep.vio_stats.arm_pos_norm = pos_norm
                if locked:
                    rep.vio_stats.arm_locked_ever = True
            else:
                rep.vio_stats.head_markers += markers
                rep.vio_stats.head_corrections += corrections
                rep.vio_stats.head_vio_status = vio_status
                rep.vio_stats.head_pos_norm = pos_norm
                if locked:
                    rep.vio_stats.head_locked_ever = True

        # Feature 4: startup markers and drift incidents emitted by Jetson marker node
        se = RE_STARTUP_EVENT.search(msg)
        if se:
            rep.startup_events.append(StartupEvent(ts=ts, phase=se.group(1), details=se.group(2).strip()))
        di = RE_DRIFT_INCIDENT.search(msg)
        if di:
            rep.drift_incidents.append(DriftIncident(
                ts=ts,
                side=di.group(1),
                reason=di.group(2),
                pos_norm_m=float(di.group(3)),
                speed_mps=float(di.group(4)),
                pos_slope_mps=float(di.group(5)) if di.group(5) else None,
            ))
        if RE_ODOM_NOT_READY.search(msg):
            rep.odom_not_ready_hits.append(ts)

        # Diagnostics block parsing
        if "------------------------------------------------------------" in msg and "pipeline_diagnostics" in effective_node:
            # finalize current block
            if current_diag is not None:
                rep.diag_blocks.append(current_diag)
            current_diag = DiagBlock(t=ts if ts is not None else 0.0)
            current_diag_node = effective_node
            current_section = None
            continue
        if current_diag is not None and current_diag_node is not None:
            if ts is not None and current_diag.t == 0.0:
                current_diag.t = ts
            # Section detection: only parse DIAG sections from the diagnostics node
            if current_diag_node not in effective_node:
                continue
            if level == "DIAG-TF" and "Dynamic" in msg:
                current_section = "tf_dyn"
            elif level == "DIAG-TF" and "Static" in msg:
                current_section = "tf_static"
            elif level == "DIAG-RATE":
                current_section = "rate"
            elif level == "DIAG-NORM":
                current_section = "norm"
            elif level == "DIAG-CHAIN":
                current_section = "chain"
            elif level == "DIAG-STALL":
                current_section = "stall"
            elif level == "DIAG-PC":
                current_section = "pc"
            else:
                _parse_diag_data(msg, current_section, current_diag)

    # finalize last diag block
    if current_diag is not None and current_diag not in rep.diag_blocks:
        rep.diag_blocks.append(current_diag)

    # Reclassify startup-only "no odom yet" warnings as expected when startup
    # markers indicate VIO initialization occurred later in the run.
    startup_phases = {e.phase for e in rep.startup_events}
    if rep.odom_not_ready_hits and ("vio_initialized" in startup_phases or "first_odom_seen" in startup_phases):
        for fm in rep.failures:
            if fm.name == "startup odom not ready":
                fm.count = 0
                fm.first_ts = None
                fm.last_ts = None
                fm.samples.clear()
                break

    return rep


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def _fmt_ts(ts: float | None) -> str:
    if ts is None:
        return "n/a"
    import datetime
    return datetime.datetime.fromtimestamp(ts).strftime("%H:%M:%S")


def _fmt_span(t0, t1):
    if t0 is None or t1 is None:
        return "n/a"
    return f"{_fmt_ts(t0)} – {_fmt_ts(t1)} ({t1 - t0:.1f}s)"


def print_header(rep: LogReport, out):
    p = lambda *a: print(*a, file=out)
    p("=" * 78)
    p(f"LOG ANALYSIS — {os.path.basename(rep.path)}")
    p("=" * 78)
    p(f"File:       {rep.path}")
    p(f"Lines:      {rep.n_lines}")
    p(f"Time span:  {_fmt_span(rep.t_start, rep.t_end)}")
    if rep.launch_args:
        p(f"Launch args: {', '.join(rep.launch_args[:6])}")
    p("")
    p(f"Nodes started ({len(rep.nodes_started)}):")
    for name, pid in sorted(rep.nodes_started.items()):
        p(f"  {name:<42} pid={pid}")
    p("")


def print_severity(rep: LogReport, out):
    p = lambda *a: print(*a, file=out)
    p("-" * 78)
    p("SEVERITY TALLY (per node)")
    p("-" * 78)
    p(f"  {'NODE':<42} {'ERR':>5} {'WARN':>6} {'INFO':>7}")
    rows = []
    for node, sev in rep.severity_per_node.items():
        e = sev.get("ERROR", 0)
        w = sev.get("WARN", 0)
        i = sev.get("INFO", 0)
        if e or w or node in rep.nodes_started:
            rows.append((node, e, w, i))
    rows.sort(key=lambda r: (-r[1], -r[2], r[0]))
    for node, e, w, i in rows:
        flag = "  <-- noisy" if (w > 50 or e > 0) else ""
        p(f"  {node:<42} {e:>5} {w:>6} {i:>7}{flag}")
    p("")


def print_failures(rep: LogReport, out):
    p = lambda *a: print(*a, file=out)
    p("-" * 78)
    p("KNOWN-FAILURE DIGEST")
    p("-" * 78)
    any_hit = False
    for fm in rep.failures:
        if fm.count == 0:
            continue
        any_hit = True
        span = ""
        if fm.first_ts is not None and fm.last_ts is not None:
            span = f"  [{_fmt_ts(fm.first_ts)} .. {_fmt_ts(fm.last_ts)}]"
        p(f"  [{fm.count:>5}x] {fm.name}{span}")
        p(f"          hint: {fm.hint}")
        if fm.samples:
            for s in fm.samples[:2]:
                p(f"          sample: {s}")
    if not any_hit:
        p("  (none of the known failure patterns matched)")
    p("")


def print_timeline(rep: LogReport, out, jetson_sysmon: dict | None = None):
    p = lambda *a: print(*a, file=out)
    p("-" * 78)
    p("DIAGNOSTICS TIMELINE (from [DIAG-*] blocks)")
    p("-" * 78)
    if not rep.diag_blocks or len(rep.diag_blocks) < 2:
        p("  (no diagnostics blocks found — was debug_monitor:=true used?)")
        p("")
        return
    t0 = rep.diag_blocks[0].t
    # Topic rate table — show first, mid, last for each topic that ever appeared
    all_topics = set()
    for b in rep.diag_blocks:
        all_topics.update(b.rates.keys())
    if all_topics:
        p("  Topic rates (Hz) — first / mid / last / min / max:")
        for topic in sorted(all_topics):
            series = [b.rates[topic][0] for b in rep.diag_blocks if topic in b.rates]
            if not series:
                continue
            mid = series[len(series) // 2]
            avg_rate = sum(series) / len(series)
            max_rate = max(series)
            noisy_flag = ""
            if max_rate > 200:
                noisy_flag = (f"  [WARNING: avg={avg_rate:.1f} Hz, peak={max_rate:.1f} Hz "
                              f"— high-frequency topic may cause local CPU exhaustion "
                              f"and network stack choking]")
            p(f"    {topic:<34} {series[0]:>5.1f} / {mid:>5.1f} / {series[-1]:>5.1f}"
              f"   (min {min(series):.1f}, max {max(series):.1f}){noisy_flag}")
        p("")
    # ── Clock offset / Latency analysis ──
    # clock_off = host_now - message_stamp.  This delta conflates true clock
    # drift with message transport + serialization latency.  Use chrony drift
    # from sysmon (if available) to separate the two.
    p("  Clock offset / Transport Latency (max |clock_off| per block):")
    offs = []
    for b in rep.diag_blocks:
        # Exclude latched topics (e.g. /tf_static) whose large clock deltas
        # are an artifact of TRANSIENT_LOCAL durability, not real latency.
        vals = [abs(v[2]) for topic, v in b.rates.items()
                if v[2] != -1.0 and topic not in LATCHED_TOPICS]
        if vals:
            offs.append((b.t, max(vals)))
    if offs:
        mx = max(o for _, o in offs)
        p(f"    peak |clock_off| = {mx:.3f}s at {_fmt_ts(next(t for t, o in offs if o == mx))}")

        # True chrony drift from sysmon (if available)
        true_drift_ms = None
        if jetson_sysmon:
            true_drift_ms = jetson_sysmon.get("drift_offset_max_ms")
        if true_drift_ms is not None:
            p(f"    true chrony drift (sysmon): peak={true_drift_ms:.1f}ms")

        big = [(t, o) for t, o in offs if o > 0.05]
        if big:
            if true_drift_ms is not None and true_drift_ms < 50:
                # True drift is minimal — high clock_off must be transport latency
                p(f"    {len(big)}/{len(offs)} blocks exceeded 50ms delta")
                p(f"    [DIAGNOSIS] True chrony drift is only {true_drift_ms:.1f}ms — "
                  f"the {mx:.0f}ms delta is Message Transport / Serialization Latency, "
                  f"NOT clock skew.")
            else:
                p(f"    {len(big)}/{len(offs)} blocks exceeded 50ms offset "
                  f"(possible clock skew)")
        else:
            p("    all blocks within 50ms — clock sync OK")
    p("")
    # TF chain flips
    p("  TF chain connectivity (marker_map -> depth_optical_frame):")
    chain_flips = defaultdict(list)
    for b in rep.diag_blocks:
        for edge, status in b.chains.items():
            chain_flips[edge].append((b.t, status))
    for edge, hist in sorted(chain_flips.items()):
        statuses = [s for _, s in hist]
        n_disc = statuses.count("DISCONNECTED")
        n_ok = statuses.count("OK") + statuses.count("CONNECTED")
        if n_disc and n_ok:
            p(f"    {edge}: FLIPPED ({n_ok} OK, {n_disc} DISCONNECTED of {len(hist)})")
        elif n_disc and not n_ok:
            p(f"    {edge}: DISCONNECTED entire run ({n_disc} blocks)")
        elif n_ok:
            p(f"    {edge}: OK entire run ({n_ok} blocks)")
    p("")
    # TF jump bursts
    if rep.tf_jump_events:
        p(f"  TF jump events: {len(rep.tf_jump_events)} total")
        # bucket by 5s windows
        buckets = defaultdict(int)
        for ts, _, _ in rep.tf_jump_events:
            buckets[int(ts // 5) * 5] += 1
        peak_b = max(buckets, key=buckets.get)
        p(f"    peak burst: {buckets[peak_b]} jumps in 5s window at {_fmt_ts(peak_b)}")
        edges = defaultdict(int)
        for _, edge, _ in rep.tf_jump_events:
            edges[edge] += 1
        for edge, c in sorted(edges.items(), key=lambda x: -x[1])[:4]:
            p(f"    {c:>5}x  {edge}")
    p("")


def print_startup_phase(rep: LogReport, out):
    p = lambda *a: print(*a, file=out)
    if not rep.startup_events and not rep.odom_not_ready_hits:
        return
    p("-" * 78)
    p("STARTUP PHASE CLASSIFICATION")
    p("-" * 78)
    if rep.startup_events:
        p("  Structured startup events:")
        for e in rep.startup_events[:12]:
            details = f" {e.details}" if e.details else ""
            p(f"    {_fmt_ts(e.ts)}  phase={e.phase}{details}")
    if rep.odom_not_ready_hits:
        p(f"  startup odom-not-ready warnings observed: {len(rep.odom_not_ready_hits)}")
        startup_phases = {e.phase for e in rep.startup_events}
        if "vio_initialized" in startup_phases or "first_odom_seen" in startup_phases:
            p("  classification: EXPECTED DURING STARTUP (deprioritized)")
        else:
            p("  classification: POTENTIALLY ACTIONABLE (no init marker seen)")
    p("")


def print_pointcloud_chain_health_log(rep: LogReport, out):
    p = lambda *a: print(*a, file=out)
    blocks = [b for b in rep.diag_blocks if b.pointcloud]
    if not blocks:
        return
    p("-" * 78)
    p("POINTCLOUD CHAIN HEALTH (from [DIAG-PC] blocks)")
    p("-" * 78)
    latest = blocks[-1]
    for side in ("head", "arm"):
        stats = latest.pointcloud.get(side)
        if not stats:
            continue
        # Actual DIAG-PC key names: depth_c, image_c, depth_r, image_r,
        # ci_raw, points.  reason is parsed from the trailing [BRACKET].
        depth_c = float(stats.get("depth_c", 0.0))
        image_c = float(stats.get("image_c", 0.0))
        depth_r = float(stats.get("depth_r", 0.0))
        image_r = float(stats.get("image_r", 0.0))
        ci_raw = float(stats.get("ci_raw", 0.0))
        points = float(stats.get("points", 0.0))
        reason = stats.get("reason", "unknown")
        p(f"  {side}: depth_c={depth_c:.1f}Hz image_c={image_c:.1f}Hz "
          f"-> depth_r={depth_r:.1f}Hz image_r={image_r:.1f}Hz "
          f"ci_raw={ci_raw:.1f}Hz -> points={points:.1f}Hz reason={reason}")
    p("")


def print_fusion_dual_view(rep: LogReport, out):
    p = lambda *a: print(*a, file=out)
    if not rep.fusion_stats:
        return
    p("-" * 78)
    p("FUSION DUAL-VIEW RATIO (from pointcloud_fusion Stats lines)")
    p("-" * 78)
    total_dual = sum(s.dual for s in rep.fusion_stats)
    total_cam1_only = sum(s.cam1_only for s in rep.fusion_stats)
    total_published = sum(s.published for s in rep.fusion_stats)
    overall_ratio = total_dual / max(total_dual + total_cam1_only, 1)
    p(f"  intervals: {len(rep.fusion_stats)}")
    p(f"  total published: {total_published}")
    p(f"  total dual: {total_dual}, total cam1_only: {total_cam1_only}")
    p(f"  overall dual-view ratio: {overall_ratio:.1%}")
    # Per-interval trend (last 10 intervals)
    recent = rep.fusion_stats[-10:]
    if len(recent) > 1:
        ratios = [s.dual_ratio for s in recent]
        p(f"  recent dual-view ratios (last {len(recent)} intervals): "
          + ", ".join(f"{r:.0%}" for r in ratios))
        degraded = [s for s in rep.fusion_stats if s.dual_ratio < 0.3]
        if degraded:
            p(f"  degraded intervals (<30% dual): {len(degraded)}")
    p("")


def print_drift_incidents(rep: LogReport, out):
    p = lambda *a: print(*a, file=out)
    if not rep.drift_incidents:
        return
    p("-" * 78)
    p("DRIFT INCIDENT TIMELINE")
    p("-" * 78)
    p(f"  total incidents: {len(rep.drift_incidents)}")
    by_side = defaultdict(int)
    by_reason = defaultdict(int)
    for ev in rep.drift_incidents:
        by_side[ev.side] += 1
        by_reason[ev.reason] += 1
    p("  per-side: " + ", ".join(f"{k}={v}" for k, v in sorted(by_side.items())))
    p("  by-reason: " + ", ".join(f"{k}={v}" for k, v in sorted(by_reason.items(), key=lambda kv: -kv[1])[:6]))
    p("  sample incidents:")
    for ev in rep.drift_incidents[:8]:
        slope_txt = "" if ev.pos_slope_mps is None else f" pos_slope={ev.pos_slope_mps:.2f}mps"
        p(f"    {_fmt_ts(ev.ts)} side={ev.side} reason={ev.reason} pos_norm={ev.pos_norm_m:.2f}m "
          f"speed={ev.speed_mps:.2f}mps{slope_txt}")
    p("")


def print_errors(rep: LogReport, out):
    p = lambda *a: print(*a, file=out)
    p("-" * 78)
    p("ERRORS")
    p("-" * 78)
    if not rep.errors:
        p("  (no [ERROR] lines)")
        p("")
        return
    # Deduplicate by message pattern
    seen = {}
    for ts, node, logger, msg in rep.errors:
        key = re.sub(r"[\d.]+", "N", msg)[:80]
        seen.setdefault(key, []).append((ts, node, msg))
    for key, hits in sorted(seen.items(), key=lambda kv: -len(kv[1])):
        first = hits[0]
        last = hits[-1]
        p(f"  [{len(hits)}x] {first[2].strip()[:100]}")
        p(f"        first: {_fmt_ts(first[0])} ({first[1]})  last: {_fmt_ts(last[0])}")
    p("")


def print_crashes(rep: LogReport, out):
    p = lambda *a: print(*a, file=out)
    if rep.crashes:
        p("-" * 78)
        p("CRASH / PROCESS-DEATH DETECTED")
        p("-" * 78)
        for lineno, text in rep.crashes[:10]:
            p(f"  line {lineno}: {text.strip()[:100]}")
        p("")


# ---------------------------------------------------------------------------
# Feature 1: QoS Contract Audit reporting
# ---------------------------------------------------------------------------

def print_qos_audit(rep: LogReport, out):
    """Print MIDDLEWARE LAYER — ROS 2 QoS CONTRACT HEALTH section."""
    p = lambda *a: print(*a, file=out)
    if not rep.qos_hits:
        return
    p("-" * 78)
    p("MIDDLEWARE LAYER — ROS 2 QoS CONTRACT HEALTH")
    p("-" * 78)
    # Group by (topic, policy) and deduplicate
    groups: dict = {}
    for hit in rep.qos_hits:
        key = (hit.topic, hit.policy)
        if key not in groups:
            groups[key] = {"nodes": set(), "direction": hit.direction,
                           "action": hit.action, "blocked_dir": hit.blocked_dir,
                           "count": 0, "first_ts": hit.ts, "last_ts": hit.ts}
        g = groups[key]
        g["nodes"].add(hit.node)
        g["count"] += 1
        if hit.ts and (g["first_ts"] is None or hit.ts < g["first_ts"]):
            g["first_ts"] = hit.ts
        if hit.ts and (g["last_ts"] is None or hit.ts > g["last_ts"]):
            g["last_ts"] = hit.ts

    p(f"  {'TOPIC':<40} {'POLICY':<28} {'NODES':<30} {'COUNT':>6}")
    p(f"  {'-'*40} {'-'*28} {'-'*30} {'-'*6}")
    for (topic, policy), g in sorted(groups.items(), key=lambda kv: -kv[1]["count"]):
        nodes_str = ", ".join(sorted(g["nodes"])[:3])
        if len(g["nodes"]) > 3:
            nodes_str += f" +{len(g['nodes']) - 3} more"
        p(f"  {topic:<40} {policy:<28} {nodes_str:<30} {g['count']:>6}")
    p("")
    p(f"  [CRITICAL ALERT] {len(groups)} QoS-incompatible channel(s) detected.")
    p(f"    These topics are completely blocked at the middleware layer.")
    p(f"    Data flow on these channels is zero — check QoS profiles.")
    p("")


# ---------------------------------------------------------------------------
# Feature 2: VIO Anchor Yield reporting
# ---------------------------------------------------------------------------

def print_vio_yield(rep: LogReport, out):
    """Print ALGORITHMIC LAYER — VIO STATE ESTIMATION HEALTH section."""
    p = lambda *a: print(*a, file=out)
    vs = rep.vio_stats
    # Only suppress entirely if we never saw any [DIAG] line on either side
    # (both sides still at their "unknown" default).  A side that ran but saw
    # 0 markers (vio_status="INVALID") is a critical failure that must render.
    if (vs.head_markers == 0 and vs.arm_markers == 0
            and vs.head_vio_status == "unknown" and vs.arm_vio_status == "unknown"):
        return
    p("-" * 78)
    p("ALGORITHMIC LAYER — VIO STATE ESTIMATION HEALTH")
    p("-" * 78)
    p(f"                Markers    Corrections   Yield    Locked   VIO        PosNorm")
    p(f"                -------    -----------   -----    ------   ---        -------")
    for side, markers, corrections, locked, vio_status, pos_norm in [
            ("Head", vs.head_markers, vs.head_corrections, vs.head_locked_ever,
             vs.head_vio_status, vs.head_pos_norm),
            ("Arm ", vs.arm_markers, vs.arm_corrections, vs.arm_locked_ever,
             vs.arm_vio_status, vs.arm_pos_norm)]:
        yield_pct = corrections / markers * 100 if markers > 0 else 0.0
        lock_str = "YES" if locked else "no"
        yield_str = f"{yield_pct:.1f}%"
        crit_flag = ""
        if markers == 0 and vio_status == "INVALID":
            # Side ran but never locked onto a marker — critical untethered state.
            crit_flag = (f"  [CRITICAL WARNING] {side} state estimator detected 0 markers "
                         f"and reports VIO=INVALID (pos_norm={pos_norm:.2f}m). Check "
                         f"camera occlusion, marker visibility, or whether the node "
                         f"crashed before detection (see traceback above).")
        elif yield_pct == 0.0 and markers > 0:
            crit_flag = (f"  [CRITICAL WARNING] {side} state estimator is running "
                         f"completely untethered. Check camera occlusion or "
                         f"marker visibility.")
        p(f"  {side} VIO:  {markers:>7}    {corrections:>11}     {yield_str:>7}  {lock_str:>5}"
          f"   {vio_status:<8}   {pos_norm:.2f}m")
        if crit_flag:
            p(crit_flag)
    p("")


# ---------------------------------------------------------------------------
# Feature 3: Quantitative marker-rejection reporting
# ---------------------------------------------------------------------------

# Human-readable explanation for each rejection type, keyed by the `type=`
# field emitted by the C++ updaters / Python loops.
_REJECT_TYPE_EXPLANATION: dict[str, str] = {
    "chi2": "Exceeded (S covariance tight)",
    "translation": "Exceeded (Hard jump wall)",
    "rotation": "Exceeded (Hard rotation wall)",
    "velocity_fit": "Exceeded (High jitter)",
}


def print_marker_rejections(rep: LogReport, out):
    """Print QUANTITATIVE EVALUATION — MARKER REJECTIONS BY TYPE."""
    p = lambda *a: print(*a, file=out)
    if not rep.marker_rejections:
        return
    p("-" * 78)
    p("QUANTITATIVE EVALUATION — MARKER REJECTIONS BY TYPE")
    p("-" * 78)
    # Aggregate by rejection type
    by_type: dict[str, list] = {}
    for r in rep.marker_rejections:
        by_type.setdefault(r.rtype, []).append(r)
    p(f"  {'TYPE':<14} {'COUNT':>6} {'MEAN_VAL':>10} {'MAX_VAL':>10} "
      f"{'GATE_LIMIT':>11}  STATUS")
    p(f"  {'-'*14} {'-'*6} {'-'*10} {'-'*10} {'-'*11}  {'-'*36}")
    for rtype, hits in sorted(by_type.items(), key=lambda kv: -len(kv[1])):
        vals = [h.val for h in hits]
        # Gate limit is constant per type; use the most common value
        lims = [h.lim for h in hits]
        lim = max(set(lims), key=lims.count) if lims else 0.0
        mean_v = sum(vals) / len(vals)
        max_v = max(vals)
        status = _REJECT_TYPE_EXPLANATION.get(rtype, "Exceeded")
        p(f"  {rtype:<14} {len(hits):>6} {mean_v:>10.2f} {max_v:>10.2f} "
          f"{lim:>11.2f}  {status}")
    # Per-side breakdown
    by_side: dict[str, list] = {}
    for r in rep.marker_rejections:
        by_side.setdefault(r.side, []).append(r)
    p("")
    p(f"  Per-side totals: " + "  ".join(
        f"{side}={len(hits)}" for side, hits in sorted(by_side.items())))
    p("")


def print_plot(rep: LogReport, out_png: str):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[analyze_log] matplotlib not available — skipping plot", file=sys.stderr)
        return
    if not rep.diag_blocks:
        return
    t0 = rep.diag_blocks[0].t
    xs = [b.t - t0 for b in rep.diag_blocks]

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    # 1. topic rates
    ax = axes[0]
    all_topics = set()
    for b in rep.diag_blocks:
        all_topics.update(b.rates.keys())
    for topic in sorted(all_topics):
        ys = [b.rates[topic][0] if topic in b.rates else None for b in rep.diag_blocks]
        ax.plot(xs, ys, marker=".", label=topic, linewidth=1)
    ax.set_ylabel("Rate (Hz)")
    ax.set_title(f"Topic rates — {os.path.basename(rep.path)}")
    ax.legend(fontsize=6, ncol=2, loc="upper right")
    ax.grid(True, alpha=0.3)
    # 2. clock offset
    ax = axes[1]
    offs = []
    for b in rep.diag_blocks:
        vals = [abs(v[2]) for topic, v in b.rates.items()
                if v[2] != -1.0 and topic not in LATCHED_TOPICS]
        offs.append(max(vals) if vals else None)
    ax.plot(xs, offs, color="red", marker=".")
    ax.axhline(0.05, color="orange", linestyle="--", linewidth=0.8, label="50ms skew threshold")
    ax.set_ylabel("Max |clock_off| (s)")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
    # 3. TF jumps per 5s
    ax = axes[2]
    buckets = defaultdict(int)
    for ts, _, _ in rep.tf_jump_events:
        buckets[int(ts // 5) * 5 - t0] += 1
    if buckets:
        bx = sorted(buckets)
        by = [buckets[x] for x in bx]
        ax.bar(bx, by, width=4, color="purple")
    ax.set_ylabel("TF jumps / 5s")
    ax.set_xlabel("Time since first DIAG block (s)")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    print(f"[analyze_log] plot written to {out_png}", file=sys.stderr)


def report(rep: LogReport, out, plot_path: str | None = None, jetson_sysmon: dict | None = None):
    print_header(rep, out)
    print_severity(rep, out)
    print_failures(rep, out)
    print_startup_phase(rep, out)
    print_qos_audit(rep, out)
    print_timeline(rep, out, jetson_sysmon)
    print_pointcloud_chain_health_log(rep, out)
    print_fusion_dual_view(rep, out)
    print_errors(rep, out)
    print_marker_rejections(rep, out)
    print_vio_yield(rep, out)
    print_drift_incidents(rep, out)
    print_crashes(rep, out)
    if jetson_sysmon:
        print_jetson_sysmon(rep, jetson_sysmon, out)
    if plot_path:
        print_plot(rep, plot_path)


# ---------------------------------------------------------------------------
# JSON metrics extraction (for replay-test regression comparison)
# ---------------------------------------------------------------------------

def to_metrics(rep: LogReport) -> dict:
    """Extract a flat dict of scalar metrics suitable for regression comparison."""
    import datetime
    m: dict = {
        "log": os.path.basename(rep.path),
        "n_lines": rep.n_lines,
        "n_nodes_started": len(rep.nodes_started),
        "captured_at": datetime.datetime.now().isoformat(timespec="seconds"),
    }
    if rep.t_start is not None and rep.t_end is not None:
        m["duration_s"] = round(rep.t_end - rep.t_start, 3)

    # Severity counts (aggregate across nodes)
    sev: dict = {}
    for node, levels in rep.severity_per_node.items():
        for level, count in levels.items():
            sev[level] = sev.get(level, 0) + count
    m["severity"] = sev

    # Known-failure counts
    failures: dict = {}
    for fm in rep.failures:
        if fm.count > 0:
            failures[fm.name] = fm.count
    m["known_failures"] = failures

    # Topic rate min/max from diagnostics timeline
    all_topics = set()
    for b in rep.diag_blocks:
        all_topics.update(b.rates.keys())
    topic_rates: dict = {}
    for topic in sorted(all_topics):
        series = [b.rates[topic][0] for b in rep.diag_blocks if topic in b.rates]
        if not series:
            continue
        topic_rates[topic] = {
            "min_hz": round(min(series), 3),
            "max_hz": round(max(series), 3),
            "first_hz": round(series[0], 3),
            "last_hz": round(series[-1], 3),
        }
    if topic_rates:
        m["topic_rates"] = topic_rates

    # Clock offset peak
    offs = []
    for b in rep.diag_blocks:
        vals = [abs(v[2]) for topic, v in b.rates.items()
                if v[2] != -1.0 and topic not in LATCHED_TOPICS]
        if vals:
            offs.append(max(vals))
    if offs:
        m["clock_offset_peak_s"] = round(max(offs), 4)

    # TF jump events
    m["tf_jump_events_total"] = len(rep.tf_jump_events)

    # Errors + crashes
    m["n_errors"] = len(rep.errors)
    m["n_crashes"] = len(rep.crashes)

    # Fusion dual-view ratio
    if rep.fusion_stats:
        total_dual = sum(s.dual for s in rep.fusion_stats)
        total_cam1_only = sum(s.cam1_only for s in rep.fusion_stats)
        m["fusion_dual_view_ratio"] = round(
            total_dual / max(total_dual + total_cam1_only, 1), 4)
        m["fusion_dual_total"] = total_dual
        m["fusion_cam1_only_total"] = total_cam1_only

    return m


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _find_sibling_jsonl(log_path: str) -> str | None:
    """Find the jetson sysmon jsonl whose timestamp best matches the log.

    Tries to match by extracting the timestamp from the log filename and finding
    the jsonl with the closest timestamp. Falls back to the latest jsonl.
    """
    base = os.path.basename(log_path)
    # Extract timestamp from log name: jetson-<container>-<ts>.txt
    m = re.search(r"-(\d{8}_\d{6})\.txt$", base)
    ts_str = m.group(1) if m else None

    jsonls = glob.glob(os.path.join(LOGS_DIR, "jetson-run-jetson-debug-*.jsonl"))
    if not jsonls:
        return None

    if ts_str:
        # Try exact match first
        exact = [f for f in jsonls if ts_str in f]
        if exact:
            return exact[0]
        # Try prefix match (jsonl may start slightly earlier)
        prefix = ts_str[:8]
        candidates = [f for f in jsonls if prefix in f]
        if candidates:
            return max(candidates, key=os.path.getmtime)

    # Fallback: latest jsonl
    return max(jsonls, key=os.path.getmtime)


def _load_jetson_sysmon(path: str) -> dict | None:
    """Load a jetson-native sysmon jsonl and return summary stats.

    The jetson-native sysmon schema differs from the host sysmon:
      cpu_pct, mem_pct, mem_total_mb, mem_used_mb, net.*.rx_mbps, tx_mbps, etc.
    """
    samples = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            if obj.get("source") == "jetson" and "cpu_pct" in obj:
                samples.append(obj)
    if not samples:
        return None

    cpus = [s["cpu_pct"] for s in samples if isinstance(s.get("cpu_pct"), (int, float))]
    mems = [s["mem_pct"] for s in samples if isinstance(s.get("mem_pct"), (int, float))]
    total_mb = samples[0].get("mem_total_mb", 0)

    # Network stats (pick the DDS interface enP8p1s0 or first non-lo)
    net_stats = defaultdict(list)
    for s in samples:
        net = s.get("net", {})
        for iface, stats in net.items():
            if iface == "lo":
                continue
            for key in ("rx_mbps", "tx_mbps"):
                val = stats.get(key)
                if isinstance(val, (int, float)):
                    net_stats[f"{iface}_{key}"].append(val)

    # Drift stats (chrony system_offset_s — true clock drift from NTP sync)
    drifts = []
    for s in samples:
        drift = s.get("drift", {})
        off = drift.get("system_offset_s")
        if isinstance(off, (int, float)):
            drifts.append(abs(off))

    return {
        "samples": len(samples),
        "cpu_pct": {"min": round(min(cpus), 1), "avg": round(sum(cpus) / len(cpus), 1),
                     "max": round(max(cpus), 1)} if cpus else None,
        "mem_pct": {"min": round(min(mems), 1), "avg": round(sum(mems) / len(mems), 1),
                     "max": round(max(mems), 1)} if mems else None,
        "mem_total_mb": total_mb,
        "net": {k: {"min": round(min(v), 1), "avg": round(sum(v) / len(v), 1),
                      "max": round(max(v), 1)} for k, v in net_stats.items()},
        "drift_offset_max_ms": round(max(drifts) * 1000, 1) if drifts else None,
        "drift_offset_avg_ms": round(sum(drifts) / len(drifts) * 1000, 1) if drifts else None,
    }


def print_jetson_sysmon(rep: LogReport, sysmon: dict, out):
    """Print a condensed sysmon summary for the Jetson native sysmon data."""
    p = lambda *a: print(*a, file=out)
    p("-" * 78)
    p("JETSON SYSMON (from native jsonl)")
    p("-" * 78)
    p(f"  Samples:    {sysmon['samples']}")
    cpu = sysmon.get("cpu_pct")
    if cpu:
        p(f"  CPU:        min={cpu['min']}%  avg={cpu['avg']}%  max={cpu['max']}%")
    mem = sysmon.get("mem_pct")
    if mem:
        mb = sysmon.get("mem_total_mb", 0)
        p(f"  RAM:        min={mem['min']}%  avg={mem['avg']}%  max={mem['max']}%"
          f"  (total {mb} MB)")
    drift_ms = sysmon.get("drift_offset_max_ms")
    if drift_ms is not None:
        avg_ms = sysmon.get("drift_offset_avg_ms", drift_ms)
        p(f"  Drift:      peak={drift_ms:.1f}ms  avg={avg_ms:.1f}ms (chrony)")
    net = sysmon.get("net", {})
    # Pick the first non-empty interface and show rx/tx
    iface_keys = sorted(net.keys())
    if iface_keys:
        # Infer interface name (everything before the first _ in the key)
        sample_key = iface_keys[0]
        iface_name = sample_key.rsplit("_", 2)[0] if sample_key.endswith("_rx_mbps") or sample_key.endswith("_tx_mbps") else sample_key
        rx_key = f"{iface_name}_rx_mbps"
        tx_key = f"{iface_name}_tx_mbps"
        rx_stats = net.get(rx_key)
        tx_stats = net.get(tx_key)
        rx_str = f"rx avg={rx_stats['avg']:.1f} Mbps  max={rx_stats['max']:.1f} Mbps" if rx_stats else ""
        tx_str = f"tx avg={tx_stats['avg']:.1f} Mbps  max={tx_stats['max']:.1f} Mbps" if tx_stats else ""
        p(f"  NIC [{iface_name}]: {rx_str}{'  ' + tx_str if tx_str else ''}")

        # ── Bandwidth cross-reference: NIC util vs useful ROS payload ──
        # Estimate expected ROS payload from active topic rates and typical
        # message sizes (point cloud ~500 KB, odom/pose ~1 KB, TF ~150 B,
        # image ~300 KB, default ~1 KB).
        _SIZE_ESTIMATE: dict = {
            "points": 500e3, "cloud": 500e3, "depth": 300e3,
            "image": 300e3, "odom": 1e3, "pose": 1e3,
            "tf": 150, "imu": 200, "joint_states": 200,
        }
        def _estimate_topic_bytes(topic: str) -> float:
            t = topic.lower()
            for kw, sz in _SIZE_ESTIMATE.items():
                if kw in t:
                    return sz
            return 1e3  # default 1 KB
        total_payload_mbps = 0.0
        all_topics = set()
        for b in rep.diag_blocks:
            all_topics.update(b.rates.keys())
        for topic in all_topics:
            series = [b.rates[topic][0] for b in rep.diag_blocks if topic in b.rates]
            if not series:
                continue
            avg_hz = sum(series) / len(series)
            est_bytes = _estimate_topic_bytes(topic)
            total_payload_mbps += avg_hz * est_bytes / 125e3  # bytes/s -> Mbps (÷125000)
        # Compare to NIC rx avg (downlink traffic on Jetson)
        nic_rx_avg = rx_stats["avg"] if rx_stats else 0
        if total_payload_mbps > 0 and nic_rx_avg > total_payload_mbps * 1.25:
            overhead_pct = (nic_rx_avg / total_payload_mbps - 1) * 100
            p(f"")
            p(f"  [WARNING] Massive DDS/RTPS Network Overhead Detected.")
            p(f"    Expected ROS payload: ~{total_payload_mbps:.1f} Mbps")
            p(f"    Physical NIC rx avg:  {nic_rx_avg:.1f} Mbps")
            p(f"    Overhead: {overhead_pct:.0f}% — check for UDP packet fragmentation")
            p(f"    dropouts or Reliable QoS retransmission storms.")
    p("")

    # ── OS KERNEL NETWORK HEALTH (from /proc/net/snmp deltas) ──
    reasm_max = sysmon.get("snmp_reasm_fails_max_rps")
    reasm_total = sysmon.get("snmp_reasm_fails_total", 0)
    rcvbuf_max = sysmon.get("snmp_rcvbuf_errors_max_rps")
    rcvbuf_total = sysmon.get("snmp_rcvbuf_errors_total", 0)
    has_snmp = reasm_max is not None or rcvbuf_max is not None
    if has_snmp:
        p("-" * 78)
        p("OS KERNEL NETWORK HEALTH (from /proc/net/snmp deltas)")
        p("-" * 78)
        if rcvbuf_max is not None:
            p(f"  UDP RcvbufErrors:  peak={rcvbuf_max:.1f}/s  total={rcvbuf_total}")
        if reasm_max is not None:
            p(f"  IP  ReasmFails:    peak={reasm_max:.1f}/s  total={reasm_total}")
        # Rule A: RcvbufErrors > 5/s
        if rcvbuf_max is not None and rcvbuf_max > 5:
            p(f"")
            p(f"  [WARNING] OS UDP Socket Buffer Overflows Detected.")
            p(f"    The Linux kernel is dropping valid data because the ROS 2")
            p(f"    application threads are stalling or the CPU is starved.")
            p(f"    Check for blocking callbacks or high-frequency exception loops.")
        # Rule B: ReasmFails > 1/s
        if reasm_max is not None and reasm_max > 1:
            p(f"")
            p(f"  [WARNING] IP Packet Reassembly Timeouts Detected.")
            p(f"    The Linux kernel is dropping fragmented data frames (likely")
            p(f"    PointCloud2) because individual UDP fragments are being lost")
            p(f"    on the wire or virtual network bridge. Check physical link")
            p(f"    connections or enable message compression.")
        if (rcvbuf_max is None or rcvbuf_max <= 5) and (reasm_max is None or reasm_max <= 1):
            p(f"  (kernel network counters within healthy range)")
    p("")

def _latest_log():
    """Return the most-recently-modified raw host-log, or None.

    Excludes ``*_analysis.txt`` so that standalone ``analyze_log.py``
    (without an explicit path) does not re-parse a previous analysis report
    as if it were a raw log.
    """
    logs = [f for f in glob.glob(os.path.join(LOGS_DIR, "host-log-*.txt"))
            if not f.endswith("_analysis.txt")]
    if not logs:
        return None
    return max(logs, key=os.path.getmtime)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?", help="Path to host-log (default: latest in logs/)")
    ap.add_argument("--all", action="store_true", help="Summarize all logs in logs/")
    ap.add_argument("--plot", action="store_true", help="Write PNG timeline (needs matplotlib)")
    ap.add_argument("-o", "--output", help="Write report to file (default: <log>_analysis.txt)")
    ap.add_argument("--json", metavar="PATH", help="Write scalar metrics to JSON (for replay-test comparison)")
    ap.add_argument("--jetson", action="store_true", help="Jetson mode: also load sibling sysmon jsonl")
    args = ap.parse_args()

    if args.all:
        logs = sorted(
            f for f in glob.glob(os.path.join(LOGS_DIR, "host-log-*.txt"))
            if not f.endswith("_analysis.txt")
        )
    else:
        log = args.log or _latest_log()
        if not log:
            print("ERROR: no host-log found in logs/. Pass a path or run a pipeline first.",
                  file=sys.stderr)
            sys.exit(1)
        logs = [log]

    for i, logpath in enumerate(logs):
        if not os.path.exists(logpath):
            print(f"ERROR: {logpath} not found", file=sys.stderr)
            sys.exit(1)

        # Default: save report alongside the log
        output_path = args.output
        if output_path is None:
            base = os.path.splitext(logpath)[0]
            output_path = base + "_analysis.txt"
        out = open(output_path, "w")

        rep = parse_log(logpath)

        # Jetson mode: load sibling sysmon jsonl and overlay
        jetson_sysmon = None
        if args.jetson:
            jsonl_path = _find_sibling_jsonl(logpath)
            if jsonl_path:
                print(f"[analyze_log] jetson sysmon: {jsonl_path}", file=sys.stderr)
                jetson_sysmon = _load_jetson_sysmon(jsonl_path)
            else:
                print("[analyze_log] no sibling jetson sysmon jsonl found", file=sys.stderr)

        plot_path = None
        if args.plot:
            base = os.path.splitext(logpath)[0]
            plot_path = base + "_analysis.png"
        report(rep, out, plot_path, jetson_sysmon)

        out.close()
        print(f"[analyze_log] report written to {output_path}", file=sys.stderr)

        if args.json:
            metrics = to_metrics(rep)
            with open(args.json, "w") as jf:
                json.dump(metrics, jf, indent=2, sort_keys=True)
            print(f"[analyze_log] metrics JSON written to {args.json}", file=sys.stderr)

        # Also echo to stdout for convenience
        with open(output_path) as f:
            print(f.read())


if __name__ == "__main__":
    main()
