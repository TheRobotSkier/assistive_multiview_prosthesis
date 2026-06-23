#!/usr/bin/env python3
"""aggregate_analysis.py — pooled pipeline-health metrics across the whole log corpus.

Scans all host logs (logs/host-log-*.txt) and Jetson logs
(logs/jetson-run-jetson-debug-*.txt, logs/jetson-log-*.txt), pairs them by
filename timestamp, and produces a single summary report with easy-to-understand
percentage metrics:

  1. TF chain availability      — % of 5s intervals each camera chain was CONNECTED
  2. Marker correction yield    — corrections / markers detected (per side)
  3. Marker rejection breakdown  — rejections by type (chi2 / translation / rotation)
  4. Hard reanchor frequency     — DRIFT-INCIDENTs per minute + % runs affected
  5. Fusion dual-view ratio      — % of published clouds using both cameras

Pure stdlib — no external dependencies. Reuses regexes and dataclasses from the
sibling ``analyze_log.py`` via direct import.

Usage:
  python3 scripts/aggregate_analysis.py                 # scan all logs, print to stdout
  python3 scripts/aggregate_analysis.py --json out.json # also write JSON metrics
  python3 scripts/aggregate_analysis.py --output rep.txt # write text report to file
  python3 scripts/aggregate_analysis.py --verbose        # per-log progress to stderr
"""
from __future__ import annotations

import argparse
import datetime
import glob
import json
import os
import re
import sys
from collections import defaultdict
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# Import parsing infrastructure from the sibling analyze_log.py
# ---------------------------------------------------------------------------

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from analyze_log import (  # noqa: E402
    DiagBlock,
    DriftIncident,
    LATCHED_TOPICS,
    LogReport,
    MarkerRejection,
    parse_log,
    RE_DIAG_CHAIN_LINE,
    RE_DRIFT_INCIDENT,
    RE_LOG_LINE,
    RE_MARKER_REJECT,
    RE_VIO_DIAG,
)

LOGS_DIR = os.path.join(os.path.dirname(_THIS_DIR), "logs")
DATA_BAGS_DIR = os.path.join(os.path.dirname(_THIS_DIR), "data", "bags")

# clock_off threshold (seconds) used by the live pipeline for the "!" warning.
CLOCK_OFF_WARN_S = 0.150
# Maximum plausible transport latency. clock_off values above this are not real
# latency — they are stale-stream artifacts (a dead stream reports
# ``now - last_message_stamp``, which grows without bound) or latched-topic
# hold values. Such observations are excluded from latency stats and counted
# separately as "stale".
LATENCY_STALE_S = 10.0

# Topic-name substrings that indicate latched/static publishers. These publish
# once and hold the value, so clock_off grows without bound and does not
# represent transport latency. Excluded from latency pooling.
STATIC_TOPIC_SUBSTRINGS = ("/vis/", "/gtsam/", "camera_info", "tf_static",
                           "trackhist", "description", "parameter_events",
                           "rosout")


# ---------------------------------------------------------------------------
# Data classes for Jetson-side parsing
# ---------------------------------------------------------------------------

@dataclass
class JetsonReport:
    """Aggregated VIO/marker diagnostics from a single Jetson log.

    The [DIAG] counters (markers_detected, corrections) are cumulative within a
    single node lifetime, but the ArUco node is known to crash and restart
    mid-run (the documented UnboundLocalError), which resets the counters to 0.
    We therefore track the running max and accumulate across resets.
    """
    path: str
    # Cumulative counters, summed across any node restarts.
    head_markers: int = 0
    head_corrections: int = 0
    arm_markers: int = 0
    arm_corrections: int = 0
    # Last-seen instantaneous state.
    head_vio_status: str = "unknown"
    arm_vio_status: str = "unknown"
    head_pos_norm: float = 0.0
    arm_pos_norm: float = 0.0
    # Per-rejection/incident entries.
    marker_rejections: list = field(default_factory=list)
    drift_incidents: list = field(default_factory=list)
    t_start: float | None = None
    t_end: float | None = None
    has_diag: bool = False
    has_reject: bool = False
    has_drift: bool = False


# ---------------------------------------------------------------------------
# Jetson log parser
# ---------------------------------------------------------------------------

def parse_jetson_log(path: str) -> JetsonReport:
    """Parse a Jetson-side log into a JetsonReport.

    Reuses the regexes from analyze_log (RE_LOG_LINE, RE_VIO_DIAG,
    RE_MARKER_REJECT, RE_DRIFT_INCIDENT). Handles counter resets from node
    crashes by tracking the max cumulative value per side and accumulating
    across resets.
    """
    jr = JetsonReport(path=path)

    # Per-side running maxima for cumulative counters, plus a flag indicating a
    # reset has been seen (so we accumulate).  When the node restarts, counters
    # drop to 0; we detect this and add the pre-reset max to the total.
    prev_head_markers = 0
    prev_head_corrections = 0
    prev_arm_markers = 0
    prev_arm_corrections = 0
    head_reset_markers = 0   # accumulated pre-reset totals
    head_reset_corrections = 0
    arm_reset_markers = 0
    arm_reset_corrections = 0

    with open(path, "r", errors="replace") as f:
        for raw in f:
            line = raw.rstrip("\n")

            # Try the standard [node-N] [LEVEL] [ts] [logger]: msg format.
            m = RE_LOG_LINE.match(line)
            ts = None
            logger = ""
            msg = line
            effective_node = ""

            if m:
                ts_str = m.group(4)
                ts = float(ts_str) if ts_str else None
                logger = m.group(5) or ""
                msg = m.group(6) or ""
                effective_node = m.group(1) or ""
            else:
                # Pseudo-level structured lines without [node-N] prefix.
                pseudo = re.match(
                    r"^\[(MARKER_REJECT|DRIFT-INCIDENT)\]\s+(.*)$", line)
                if pseudo:
                    msg = line

            if ts is not None:
                if jr.t_start is None or ts < jr.t_start:
                    jr.t_start = ts
                if jr.t_end is None or ts > jr.t_end:
                    jr.t_end = ts

            # [DIAG] markers_detected=N corrections=M/N vio=... pos_norm=...m ...
            vm = RE_VIO_DIAG.search(msg)
            if vm:
                jr.has_diag = True
                markers = int(vm.group(1))
                corrections = int(vm.group(2))
                vio_status = vm.group(4)
                pos_norm = float(vm.group(5))
                locked = vm.group(7) == "True"

                is_arm = ("arm_phase2" in logger.lower()
                          or "arm" in effective_node.lower())

                if is_arm:
                    # Detect reset: cumulative counter dropped.
                    if markers < prev_arm_markers:
                        arm_reset_markers += prev_arm_markers
                        arm_reset_corrections += prev_arm_corrections
                    prev_arm_markers = max(prev_arm_markers, markers)
                    prev_arm_corrections = max(prev_arm_corrections, corrections)
                    jr.arm_vio_status = vio_status
                    jr.arm_pos_norm = pos_norm
                else:
                    if markers < prev_head_markers:
                        head_reset_markers += prev_head_markers
                        head_reset_corrections += prev_head_corrections
                    prev_head_markers = max(prev_head_markers, markers)
                    prev_head_corrections = max(prev_head_corrections, corrections)
                    jr.head_vio_status = vio_status
                    jr.head_pos_norm = pos_norm

            # [MARKER_REJECT] side=X type=Y val=Z lim=W
            rj = RE_MARKER_REJECT.search(msg)
            if rj:
                jr.has_reject = True
                jr.marker_rejections.append(MarkerRejection(
                    ts=ts,
                    side=rj.group(1),
                    rtype=rj.group(2),
                    val=float(rj.group(3)),
                    lim=float(rj.group(4)),
                ))

            # [DRIFT-INCIDENT] side=X reason=Y pos_norm=Zm speed=Wmps ...
            di = RE_DRIFT_INCIDENT.search(msg)
            if di:
                jr.has_drift = True
                jr.drift_incidents.append(DriftIncident(
                    ts=ts,
                    side=di.group(1),
                    reason=di.group(2),
                    pos_norm_m=float(di.group(3)),
                    speed_mps=float(di.group(4)),
                    pos_slope_mps=float(di.group(5)) if di.group(5) else None,
                ))

    # Final cumulative totals = accumulated pre-reset maxima + final running max.
    jr.head_markers = head_reset_markers + prev_head_markers
    jr.head_corrections = head_reset_corrections + prev_head_corrections
    jr.arm_markers = arm_reset_markers + prev_arm_markers
    jr.arm_corrections = arm_reset_corrections + prev_arm_corrections

    return jr


# ---------------------------------------------------------------------------
# Host-Jetson log matching by filename timestamp
# ---------------------------------------------------------------------------

_TS_RE = re.compile(r"(\d{8}_\d{6})")


def extract_timestamp(filename: str) -> str | None:
    """Extract YYYYMMDD_HHMMSS from a log filename, or None if absent."""
    m = _TS_RE.search(os.path.basename(filename))
    return m.group(1) if m else None


def _ts_to_epoch(ts_str: str) -> float:
    """Convert YYYYMMDD_HHMMSS to a unix-style epoch for delta math."""
    dt = datetime.datetime.strptime(ts_str, "%Y%m%d_%H%M%S")
    return dt.timestamp()


def find_jetson_pair(host_path: str, jetson_paths: list) -> str | None:
    """Find the Jetson log whose timestamp is closest to the host log.

    The Jetson container typically starts 0-90s before the host pipeline, so we
    accept a Jetson log that is up to 120s earlier or 30s later than the host.
    Returns None if no match within the window or if the host has no timestamp.
    """
    host_ts = extract_timestamp(host_path)
    if host_ts is None:
        return None
    host_epoch = _ts_to_epoch(host_ts)

    best_path = None
    best_delta = None
    for jp in jetson_paths:
        jts = extract_timestamp(jp)
        if jts is None:
            continue
        delta = _ts_to_epoch(jts) - host_epoch
        # Jetson should start before host (negative delta), but allow some slack.
        if -120 <= delta <= 30:
            adelta = abs(delta)
            if best_delta is None or adelta < best_delta:
                best_delta = adelta
                best_path = jp
    return best_path


# ---------------------------------------------------------------------------
# Bag sysmon loader (bandwidth / SNMP / chrony from data/bags/*/sysmon.jsonl)
# ---------------------------------------------------------------------------

@dataclass
class BagSysmonReport:
    """Pooled bandwidth, SNMP, and chrony data from one bag's sysmon.jsonl."""
    path: str
    # Network throughput samples (Mbps), per source per direction.
    host_rx_mbs: list = field(default_factory=list)
    host_tx_mbs: list = field(default_factory=list)
    jetson_rx_mbs: list = field(default_factory=list)
    jetson_tx_mbs: list = field(default_factory=list)
    # Jetson NIC errout counter samples (cumulative, from /proc/net/dev).
    jetson_errout: list = field(default_factory=list)
    # SNMP delta samples (per-interval counts).
    host_reasm_fails: list = field(default_factory=list)
    host_rcvbuf_errors: list = field(default_factory=list)
    jetson_reasm_fails: list = field(default_factory=list)
    jetson_rcvbuf_errors: list = field(default_factory=list)
    # Chrony drift samples (seconds).
    host_drift_s: list = field(default_factory=list)
    jetson_drift_s: list = field(default_factory=list)


def parse_bag_sysmon(path: str) -> BagSysmonReport:
    """Parse a bag-based sysmon.jsonl into a BagSysmonReport.

    Each line is a JSON object tagged ``source: host|jetson`` with nested
    ``net``, ``snmp``, and ``drift`` sub-objects.  See scripts/sysmon.py for
    the producer schema.
    """
    rep = BagSysmonReport(path=path)
    with open(path, "r", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            source = obj.get("source", "")
            net = obj.get("net", {})
            snmp = obj.get("snmp", {})
            drift = obj.get("drift", {})

            # Pick the first non-loopback interface with data.
            rx = tx = errout = None
            for iface, stats in net.items():
                if iface in ("lo", "loopback0"):
                    continue
                if "rx_mbs" in stats:
                    rx = stats["rx_mbs"]
                if "tx_mbs" in stats:
                    tx = stats["tx_mbs"]
                if "errout" in stats:
                    errout = stats["errout"]
                break

            reasm = snmp.get("ip_reasm_fails_delta")
            rcvbuf = snmp.get("udp_rcvbuf_errors_delta")
            drift_off = drift.get("system_offset_s")

            if source == "host":
                if rx is not None:
                    rep.host_rx_mbs.append(float(rx))
                if tx is not None:
                    rep.host_tx_mbs.append(float(tx))
                if reasm is not None:
                    rep.host_reasm_fails.append(int(reasm))
                if rcvbuf is not None:
                    rep.host_rcvbuf_errors.append(int(rcvbuf))
                if drift_off is not None:
                    rep.host_drift_s.append(abs(float(drift_off)))
            elif source == "jetson":
                if rx is not None:
                    rep.jetson_rx_mbs.append(float(rx))
                if tx is not None:
                    rep.jetson_tx_mbs.append(float(tx))
                if errout is not None:
                    rep.jetson_errout.append(int(errout))
                if reasm is not None:
                    rep.jetson_reasm_fails.append(int(reasm))
                if rcvbuf is not None:
                    rep.jetson_rcvbuf_errors.append(int(rcvbuf))
                if drift_off is not None:
                    rep.jetson_drift_s.append(abs(float(drift_off)))
    return rep


# ---------------------------------------------------------------------------
# Metric computation
# ---------------------------------------------------------------------------

# The DIAG-CHAIN line regex in analyze_log captures the edge starting from
# the "->" arrow (the hyphen is not in its character class), so stored keys
# look like "> head_d435i_head_depth_optical_frame". Match by distinctive
# frame-name suffix instead of the full edge string.
HEAD_CHAIN_SUFFIX = "head_d435i_head_depth_optical_frame"
ARM_CHAIN_SUFFIX = "arm_d435i_arm_depth_optical_frame"


def _chain_connected(status: str) -> bool:
    return status in ("CONNECTED", "OK")


def _find_chain_status(chains: dict, suffix: str) -> str | None:
    """Look up a chain status by matching its distinctive frame suffix."""
    for edge, status in chains.items():
        if suffix in edge:
            return status
    return None


def _classify_topic(topic: str) -> str | None:
    """Classify a ROS topic into a latency-relevant category.

    Groups topics by payload type so transport latency can be reported
    per-category (point clouds are large/fragmented, odom is small/frequent).
    Returns None for latched/static topics whose clock_off does not represent
    transport latency (it grows without bound as ``now - publish_time``).
    """
    t = topic.lower()
    if any(s in t for s in STATIC_TOPIC_SUBSTRINGS):
        return None
    if "points" in t or "cloud" in t:
        return "points"
    if "depth" in t:
        return "depth"
    if "image" in t:
        return "image"
    if "odom" in t:
        return "odom"
    return "other"


def compute_metrics(host_reports: list, jetson_reports: list,
                     bag_sysmon_reports: list | None = None) -> dict:
    """Pool all metrics across the matched host/jetson report lists.

    Each entry in host_reports is (path, LogReport|None) and each entry in
    jetson_reports is (path, JetsonReport|None). None means parse failed.
    bag_sysmon_reports is an optional list of BagSysmonReport for bandwidth/
    SNMP/chrony metrics.
    """
    metrics: dict = {
        "tf": {"head_ok": 0, "head_total": 0,
               "arm_ok": 0, "arm_total": 0,
               "both_ok": 0, "both_total": 0},
        "marker_yield": {"head_markers": 0, "head_corrections": 0,
                         "arm_markers": 0, "arm_corrections": 0},
        "rejections": {"aruco": [], "msckf": []},
        "reanchors": {"incidents": [], "monitoring_s": 0.0,
                      "runs_with_data": 0, "runs_with_incidents": 0},
        "fusion": {"total_dual": 0, "total_cam1_only": 0},
        "qos": {"hits": [], "logs_affected": 0},
        "latency": defaultdict(list),   # topic_type -> list of clock_off values
        "latency_stale": 0,             # observations excluded as stale (>10s)
        "bag": {
            "host_rx_mbs": [], "host_tx_mbs": [],
            "jetson_rx_mbs": [], "jetson_tx_mbs": [],
            "jetson_errout": [],
            "host_reasm": [], "host_rcvbuf": [],
            "jetson_reasm": [], "jetson_rcvbuf": [],
            "host_drift_ms": [], "jetson_drift_ms": [],
        },
    }

    # Coverage counters.
    n_host = len(host_reports)
    n_host_chains = 0
    n_host_fusion = 0
    n_host_qos = 0
    n_host_latency = 0
    n_jetson = len([p for p, jr in jetson_reports if jr is not None])
    n_jetson_diag = 0
    n_jetson_reject = 0
    n_jetson_drift = 0

    # --- TF chain availability (host-side) ---
    for _path, rep in host_reports:
        if rep is None:
            continue
        if rep.diag_blocks:
            n_host_chains += 1
        for b in rep.diag_blocks:
            head_status = _find_chain_status(b.chains, HEAD_CHAIN_SUFFIX)
            arm_status = _find_chain_status(b.chains, ARM_CHAIN_SUFFIX)
            if head_status is not None:
                metrics["tf"]["head_total"] += 1
                if _chain_connected(head_status):
                    metrics["tf"]["head_ok"] += 1
            if arm_status is not None:
                metrics["tf"]["arm_total"] += 1
                if _chain_connected(arm_status):
                    metrics["tf"]["arm_ok"] += 1
            # Both-chains-present intervals for dual-camera uptime.
            if head_status is not None and arm_status is not None:
                metrics["tf"]["both_total"] += 1
                if (_chain_connected(head_status)
                        and _chain_connected(arm_status)):
                    metrics["tf"]["both_ok"] += 1
        # Fusion dual-view.
        if rep.fusion_stats:
            n_host_fusion += 1
            for s in rep.fusion_stats:
                metrics["fusion"]["total_dual"] += s.dual
                metrics["fusion"]["total_cam1_only"] += s.cam1_only
        # QoS incompatibility hits.
        if rep.qos_hits:
            n_host_qos += 1
            metrics["qos"]["hits"].extend(rep.qos_hits)
        # Transport latency from DIAG-RATE clock_off values.
        for b in rep.diag_blocks:
            for topic, (hz, total, clock_off) in b.rates.items():
                if topic in LATCHED_TOPICS:
                    continue
                if clock_off == -1.0:
                    continue
                ttype = _classify_topic(topic)
                if ttype is None:
                    continue  # static/latched topic
                if clock_off > LATENCY_STALE_S:
                    metrics["latency_stale"] += 1
                    continue  # stale-stream artifact, not real latency
                metrics["latency"][ttype].append(clock_off)
        if any(b.rates for b in rep.diag_blocks):
            n_host_latency += 1

    # --- Jetson-side metrics ---
    for _path, jr in jetson_reports:
        if jr is None:
            continue
        if jr.has_diag:
            n_jetson_diag += 1
            metrics["marker_yield"]["head_markers"] += jr.head_markers
            metrics["marker_yield"]["head_corrections"] += jr.head_corrections
            metrics["marker_yield"]["arm_markers"] += jr.arm_markers
            metrics["marker_yield"]["arm_corrections"] += jr.arm_corrections
        if jr.has_reject:
            n_jetson_reject += 1
        for r in jr.marker_rejections:
            if r.side == "marker":
                metrics["rejections"]["msckf"].append(r)
            else:
                metrics["rejections"]["aruco"].append(r)
        if jr.has_drift:
            n_jetson_drift += 1
            metrics["reanchors"]["incidents"].extend(jr.drift_incidents)
            if jr.t_start is not None and jr.t_end is not None:
                dur = jr.t_end - jr.t_start
                if dur > 0:
                    metrics["reanchors"]["runs_with_data"] += 1
                    metrics["reanchors"]["monitoring_s"] += dur
                    # A run is "affected" if it had at least one incident.
                    if jr.drift_incidents:
                        metrics["reanchors"]["runs_with_incidents"] += 1

    # --- Bag sysmon metrics (bandwidth / SNMP / chrony) ---
    if bag_sysmon_reports:
        for bsr in bag_sysmon_reports:
            metrics["bag"]["host_rx_mbs"].extend(bsr.host_rx_mbs)
            metrics["bag"]["host_tx_mbs"].extend(bsr.host_tx_mbs)
            metrics["bag"]["jetson_rx_mbs"].extend(bsr.jetson_rx_mbs)
            metrics["bag"]["jetson_tx_mbs"].extend(bsr.jetson_tx_mbs)
            metrics["bag"]["jetson_errout"].extend(bsr.jetson_errout)
            metrics["bag"]["host_reasm"].extend(bsr.host_reasm_fails)
            metrics["bag"]["host_rcvbuf"].extend(bsr.host_rcvbuf_errors)
            metrics["bag"]["jetson_reasm"].extend(bsr.jetson_reasm_fails)
            metrics["bag"]["jetson_rcvbuf"].extend(bsr.jetson_rcvbuf_errors)
            metrics["bag"]["host_drift_ms"].extend(
                v * 1000 for v in bsr.host_drift_s)
            metrics["bag"]["jetson_drift_ms"].extend(
                v * 1000 for v in bsr.jetson_drift_s)

    metrics["coverage"] = {
        "n_host": n_host,
        "n_host_chains": n_host_chains,
        "n_host_fusion": n_host_fusion,
        "n_host_qos": n_host_qos,
        "n_host_latency": n_host_latency,
        "n_jetson": n_jetson,
        "n_jetson_diag": n_jetson_diag,
        "n_jetson_reject": n_jetson_reject,
        "n_jetson_drift": n_jetson_drift,
        "n_bags": len(bag_sysmon_reports) if bag_sysmon_reports else 0,
    }
    return metrics


# ---------------------------------------------------------------------------
# Report output
# ---------------------------------------------------------------------------

def _pct(num: int, den: int) -> float:
    return (num / den * 100) if den > 0 else 0.0


def _print_reject_types(p, hits: list):
    """Print rejection-type rows, sorted by count descending.

    Iterates dynamically over all types present in ``hits`` rather than a
    hardcoded list, so newly-instrumented rejection types (e.g. rotation_hard)
    appear automatically.
    """
    by_type: dict[str, list] = defaultdict(list)
    for r in hits:
        by_type[r.rtype].append(r)
    for rtype in sorted(by_type, key=lambda k: -len(by_type[k])):
        h = by_type[rtype]
        vals = [x.val for x in h]
        lims = [x.lim for x in h]
        lim = max(set(lims), key=lims.count) if lims else 0.0
        mean_v = sum(vals) / len(vals)
        p(f"      {rtype:<14} {len(h):>5} rejections  "
          f"(mean={mean_v:.2f} vs lim={lim:.2f})")


def _percentile(sorted_vals: list, pct: float) -> float:
    """Compute a percentile from a pre-sorted list."""
    if not sorted_vals:
        return 0.0
    k = (len(sorted_vals) - 1) * pct
    f = int(k)
    c = min(f + 1, len(sorted_vals) - 1)
    if f == c:
        return sorted_vals[f]
    return sorted_vals[f] + (sorted_vals[c] - sorted_vals[f]) * (k - f)


def _print_qos_section(p, metrics: dict, cov: dict):
    """Print QoS middleware-blocked topics section."""
    qos = metrics["qos"]
    hits = qos["hits"]
    p("─" * 76)
    p("QoS / MIDDLEWARE-BLOCKED TOPICS (host-side)")
    p(f"  Pooled from {cov['n_host_qos']} host logs with QoS warnings")
    p("")
    if not hits:
        p("  (no QoS incompatibility warnings detected)")
        p("")
        return
    pct_logs = _pct(cov["n_host_qos"], cov["n_host"])
    p(f"  % logs affected:      {pct_logs:5.1f}%  "
      f"({cov['n_host_qos']} / {cov['n_host']} logs)")
    # Unique topics blocked.
    topics = set(h.topic for h in hits)
    p(f"  Unique topics blocked: {len(topics)}")
    # Breakdown by policy.
    by_policy: dict[str, set] = defaultdict(set)
    for h in hits:
        # Normalize policy name (RELIABILITY_QOS_POLICY -> RELIABILITY).
        pol = h.policy.replace("_QOS_POLICY", "")
        by_policy[pol].add(h.topic)
    p(f"  By policy:")
    for pol in sorted(by_policy, key=lambda k: -len(by_policy[k])):
        p(f"    {pol:<14} {len(by_policy[pol])} unique topics blocked")
    # Worst offending topics (most logs affected).
    topic_log_count: dict[str, int] = defaultdict(int)
    for h in hits:
        topic_log_count[h.topic] += 1
    p(f"  Most-blocked topics (by warning count):")
    for topic, cnt in sorted(topic_log_count.items(),
                             key=lambda kv: -kv[1])[:6]:
        p(f"    {topic:<44} {cnt:>4} warnings")
    p("")


def _print_latency_section(p, metrics: dict, cov: dict):
    """Print transport latency section from DIAG-RATE clock_off values."""
    latency: dict = metrics["latency"]
    p("─" * 76)
    p("TRANSPORT LATENCY (clock_off from DIAG-RATE, host-side)")
    p(f"  Pooled from {cov['n_host_latency']} host logs with rate data")
    p("")
    if not latency or not any(latency.values()):
        p("  (no clock_off data available)")
        p("")
        return
    # Chrony drift proof from bag data (if available).
    bag = metrics.get("bag", {})
    host_drift = bag.get("host_drift_ms", [])
    jetson_drift = bag.get("jetson_drift_ms", [])
    if host_drift or jetson_drift:
        all_drift = host_drift + jetson_drift
        max_drift = max(all_drift) if all_drift else 0.0
        p(f"  Chrony drift (from {cov['n_bags']} bag sysmon files): "
          f"peak={max_drift:.2f}ms")
        p(f"  => clock_off values represent transport/queueing latency, "
          f"not clock skew")
        p("")
    # Per-topic-type breakdown.
    total_vals = 0
    total_over = 0
    p(f"  {'TYPE':<10} {'MEDIAN':>8} {'P75':>8} {'P95':>8} {'PEAK':>8}"
      f"  {'>150ms':>8}")
    p(f"  {'-'*10} {'-'*8} {'-'*8} {'-'*8} {'-'*8}  {'-'*8}")
    for ttype in ("points", "depth", "image", "odom", "other"):
        vals = latency.get(ttype, [])
        if not vals:
            continue
        sv = sorted(vals)
        med = _percentile(sv, 0.50)
        p75 = _percentile(sv, 0.75)
        p95 = _percentile(sv, 0.95)
        peak = max(vals)
        over = sum(1 for v in vals if v > CLOCK_OFF_WARN_S)
        total_vals += len(vals)
        total_over += over
        pct_over = _pct(over, len(vals))
        p(f"  {ttype:<10} {med*1000:>7.0f}ms {p75*1000:>7.0f}ms "
          f"{p95*1000:>7.0f}ms {peak*1000:>7.0f}ms  "
          f"{pct_over:>5.1f}% ({over}/{len(vals)})")
    if total_vals:
        p("")
        p(f"  Total observations: {total_vals} across all topic types")
        p(f"  Intervals exceeding {CLOCK_OFF_WARN_S*1000:.0f}ms delay "
          f"threshold: {_pct(total_over, total_vals):.1f}% "
          f"({total_over}/{total_vals})")
    stale = metrics.get("latency_stale", 0)
    if stale:
        p(f"  Stale observations excluded (> {LATENCY_STALE_S:.0f}s, dead "
          f"streams): {stale}")
    p("")


def _print_bandwidth_section(p, metrics: dict, cov: dict):
    """Print network bandwidth utilization section."""
    bag = metrics.get("bag", {})
    p("─" * 76)
    p("NETWORK BANDWIDTH (from bag sysmon, host + jetson)")
    p(f"  Pooled from {cov['n_bags']} bag sysmon files")
    p("")
    if cov["n_bags"] == 0:
        p("  (no bag sysmon files found in data/bags/)")
        p("")
        return
    host_rx = bag.get("host_rx_mbs", [])
    jetson_tx = bag.get("jetson_tx_mbs", [])
    jetson_errout = bag.get("jetson_errout", [])
    if host_rx:
        p(f"  Host NIC rx (downlink from Jetson):")
        p(f"    avg={sum(host_rx)/len(host_rx):.1f} Mbps  "
          f"peak={max(host_rx):.1f} Mbps  "
          f"(from {len(host_rx)} samples)")
    if jetson_tx:
        p(f"  Jetson NIC tx (uplink to host):")
        p(f"    avg={sum(jetson_tx)/len(jetson_tx):.1f} Mbps  "
          f"peak={max(jetson_tx):.1f} Mbps  "
          f"(from {len(jetson_tx)} samples)")
    if jetson_errout:
        # errout is a cumulative counter; report the range observed.
        p(f"  Jetson NIC errout (cumulative tx errors):")
        p(f"    range={min(jetson_errout)}-{max(jetson_errout)}  "
          f"(from {len(jetson_errout)} samples)")
        if max(jetson_errout) > 1000:
            p(f"    [WARNING] Persistent NIC transmit errors indicate "
              f"network-stack congestion on the Jetson.")
    p("")


def _print_reassembly_section(p, metrics: dict, cov: dict):
    """Print kernel network health (SNMP reassembly / buffer errors)."""
    bag = metrics.get("bag", {})
    p("─" * 76)
    p("KERNEL NETWORK HEALTH / REASSEMBLY (from bag sysmon SNMP deltas)")
    p(f"  Pooled from {cov['n_bags']} bag sysmon files")
    p("")
    if cov["n_bags"] == 0:
        p("  (no bag sysmon files found in data/bags/)")
        p("")
        return
    host_reasm = bag.get("host_reasm", [])
    host_rcvbuf = bag.get("host_rcvbuf", [])
    jetson_reasm = bag.get("jetson_reasm", [])
    jetson_rcvbuf = bag.get("jetson_rcvbuf", [])
    all_reasm = host_reasm + jetson_reasm
    all_rcvbuf = host_rcvbuf + jetson_rcvbuf
    if all_reasm:
        total_reasm = sum(all_reasm)
        peak_reasm = max(all_reasm)
        p(f"  IP ReasmFails (fragmented packet reassembly timeouts):")
        p(f"    total={total_reasm}  peak={peak_reasm}/sample  "
          f"(host={sum(host_reasm)}, jetson={sum(jetson_reasm)})")
        if peak_reasm > 1:
            p(f"    [WARNING] Reassembly failures > 1/sample — fragmented "
              f"UDP packets (point clouds) being dropped on the wire.")
    if all_rcvbuf:
        total_rcvbuf = sum(all_rcvbuf)
        peak_rcvbuf = max(all_rcvbuf)
        p(f"  UDP RcvbufErrors (socket buffer overflows):")
        p(f"    total={total_rcvbuf}  peak={peak_rcvbuf}/sample  "
          f"(host={sum(host_rcvbuf)}, jetson={sum(jetson_rcvbuf)})")
        if peak_rcvbuf > 5:
            p(f"    [WARNING] Socket buffer overflows > 5/sample — kernel "
              f"dropping valid data due to application-thread stalls.")
    if all_reasm and max(all_reasm) <= 1 and (not all_rcvbuf
                                              or max(all_rcvbuf) <= 5):
        p(f"  (kernel network counters within healthy range)")
    p("")


def print_aggregate_report(metrics: dict, coverage_meta: dict, out):
    """Print the human-readable pooled report to ``out``."""
    p = lambda *a: print(*a, file=out)
    cov = metrics["coverage"]
    tf = metrics["tf"]
    my = metrics["marker_yield"]
    fus = metrics["fusion"]
    rea = metrics["reanchors"]

    p("=" * 76)
    p("AGGREGATED PIPELINE HEALTH REPORT")
    p("=" * 76)
    p(f"Host logs scanned:     {cov['n_host']}")
    p(f"Jetson logs matched:   {cov['n_jetson']} / {cov['n_host']} host logs")
    if coverage_meta.get("period_start"):
        p(f"Monitoring period:     {coverage_meta['period_start']} "
          f"to {coverage_meta['period_end']}")
    p("")

    # ── TF chain availability ──
    p("─" * 76)
    p("TF CHAIN AVAILABILITY (host-side)")
    p(f"  Pooled from {cov['n_host_chains']} host logs, "
      f"{tf['head_total']} total 5-second intervals")
    p("")
    p(f"  Head chain (marker_map -> head_depth_optical):  "
      f"{_pct(tf['head_ok'], tf['head_total']):5.1f}%  "
      f"({tf['head_ok']} / {tf['head_total']})")
    p(f"  Arm chain  (marker_map -> arm_depth_optical):   "
      f"{_pct(tf['arm_ok'], tf['arm_total']):5.1f}%  "
      f"({tf['arm_ok']} / {tf['arm_total']})")
    p("  " + "─" * 72)
    p(f"  Dual-camera TF uptime (both connected):         "
      f"{_pct(tf['both_ok'], tf['both_total']):5.1f}%  "
      f"({tf['both_ok']} / {tf['both_total']})")
    p("")

    # ── Marker correction yield ──
    p("─" * 76)
    p("MARKER CORRECTION YIELD (jetson-side)")
    p(f"  Pooled from {cov['n_jetson_diag']} jetson logs with DIAG data")
    p("")
    head_y = _pct(my["head_corrections"], my["head_markers"])
    arm_y = _pct(my["arm_corrections"], my["arm_markers"])
    tot_markers = my["head_markers"] + my["arm_markers"]
    tot_corr = my["head_corrections"] + my["arm_corrections"]
    overall_y = _pct(tot_corr, tot_markers)
    p(f"  Head:  {head_y:5.1f}%  ({my['head_corrections']} corrections / "
      f"{my['head_markers']} markers detected)")
    p(f"  Arm:   {arm_y:5.1f}%  ({my['arm_corrections']} corrections / "
      f"{my['arm_markers']} markers detected)")
    p("  " + "─" * 72)
    p(f"  Overall marker yield:  {overall_y:5.1f}%  "
      f"({tot_corr} / {tot_markers})")
    p("")

    # ── Marker rejection breakdown ──
    aruco = metrics["rejections"]["aruco"]
    msckf = metrics["rejections"]["msckf"]
    p("─" * 76)
    p("MARKER REJECTION BREAKDOWN (jetson-side)")
    p(f"  Pooled from {cov['n_jetson_reject']} jetson logs with rejection data")
    p("")
    # ArUco pose plausibility gate (side=head/arm)
    if aruco:
        # Rejection rate = rejected / (rejected + accepted)
        # Accepted ≈ total corrections from the yield section.
        accepted = tot_corr
        total_attempts = len(aruco) + accepted
        rej_rate = _pct(len(aruco), total_attempts)
        p(f"  ArUco pose plausibility gate (side=head/arm):")
        p(f"    Overall rejection rate: {rej_rate:5.1f}%  "
          f"({len(aruco)} rejected / {total_attempts} total attempts)")
        p(f"    By type:")
        _print_reject_types(p, aruco)
    else:
        p(f"  ArUco pose plausibility gate: (no rejection data)")
    p("")
    # MSCKF feature gate (side=marker, C++ node)
    if msckf:
        p(f"  MSCKF feature gate (side=marker, C++ node):")
        _print_reject_types(p, msckf)
    else:
        p(f"  MSCKF feature gate: (no rejection data)")
    p("")

    # ── Hard reanchoring ──
    incidents = rea["incidents"]
    p("─" * 76)
    p("HARD REANCHORING (jetson-side)")
    mon_min = rea["monitoring_s"] / 60.0
    p(f"  Pooled from {rea['runs_with_data']} jetson logs, "
      f"{mon_min:.1f} minutes of monitoring")
    p("")
    p(f"  Total incidents: {len(incidents)}")
    rate = len(incidents) / mon_min if mon_min > 0 else 0.0
    p(f"  Rate:            {rate:.2f} incidents/minute")
    pct_runs = _pct(rea["runs_with_incidents"], rea["runs_with_data"])
    p(f"  % runs affected:  {pct_runs:.1f}%  "
      f"({rea['runs_with_incidents']} / {rea['runs_with_data']} runs with drift data)")
    by_reason: dict[str, int] = defaultdict(int)
    by_side: dict[str, int] = defaultdict(int)
    for ev in incidents:
        by_reason[ev.reason] += 1
        by_side[ev.side] += 1
    if by_reason:
        p(f"  By reason:")
        for reason, cnt in sorted(by_reason.items(), key=lambda kv: -kv[1]):
            p(f"    {reason:<42} {cnt:>5} incidents")
    if by_side:
        p(f"  By side: " + ", ".join(f"{k}={v}" for k, v in
          sorted(by_side.items())))
    p("")

    # ── Fusion dual-view ratio ──
    p("─" * 76)
    p("FUSION DUAL-VIEW RATIO (host-side)")
    p(f"  Pooled from {cov['n_host_fusion']} host logs with fusion stats")
    p("")
    total_pub = fus["total_dual"] + fus["total_cam1_only"]
    ratio = _pct(fus["total_dual"], total_pub)
    p(f"  Overall dual-view ratio: {ratio:5.1f}%  "
      f"({fus['total_dual']} dual / {total_pub} total published)")
    p("")

    # ── QoS / middleware-blocked topics ──
    _print_qos_section(p, metrics, cov)

    # ── Transport latency ──
    _print_latency_section(p, metrics, cov)

    # ── Bandwidth ──
    _print_bandwidth_section(p, metrics, cov)

    # ── Kernel network health (reassembly) ──
    _print_reassembly_section(p, metrics, cov)

    # ── Coverage ──
    p("─" * 76)
    p("COVERAGE")
    p("─" * 76)
    p(f"  Host logs scanned:           {cov['n_host']}")
    p(f"  Host logs with DIAG-CHAIN:   {cov['n_host_chains']}")
    p(f"  Host logs with fusion stats: {cov['n_host_fusion']}")
    p(f"  Host logs with QoS hits:     {cov['n_host_qos']}")
    p(f"  Host logs with latency data: {cov['n_host_latency']}")
    p(f"  Jetson logs matched:         {cov['n_jetson']} / {cov['n_host']}")
    p(f"  Jetson logs with DIAG:       {cov['n_jetson_diag']}")
    p(f"  Jetson logs with REJECT:     {cov['n_jetson_reject']}")
    p(f"  Jetson logs with DRIFT:      {cov['n_jetson_drift']}")
    p(f"  Bag sysmon files:            {cov['n_bags']}")
    p("=" * 76)


def write_json_metrics(metrics: dict, coverage_meta: dict, path: str):
    """Write structured JSON metrics for CI/regression tracking."""
    tf = metrics["tf"]
    my = metrics["marker_yield"]
    fus = metrics["fusion"]
    rea = metrics["reanchors"]
    aruco = metrics["rejections"]["aruco"]
    msckf = metrics["rejections"]["msckf"]
    cov = metrics["coverage"]

    tot_markers = my["head_markers"] + my["arm_markers"]
    tot_corr = my["head_corrections"] + my["arm_corrections"]

    def _reject_block(hits):
        by_type: dict[str, list] = defaultdict(list)
        for r in hits:
            by_type[r.rtype].append(r)
        out = {}
        for rtype, h in by_type.items():
            vals = [x.val for x in h]
            lims = [x.lim for x in h]
            out[rtype] = {
                "count": len(h),
                "mean_val": round(sum(vals) / len(vals), 3),
                "max_val": round(max(vals), 3),
                "gate_limit": max(set(lims), key=lims.count) if lims else 0.0,
            }
        return out

    by_reason: dict[str, int] = defaultdict(int)
    for ev in rea["incidents"]:
        by_reason[ev.reason] += 1
    mon_min = rea["monitoring_s"] / 60.0

    out = {
        "generated_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "coverage": cov,
        "tf_chain_availability": {
            "head_pct": round(_pct(tf["head_ok"], tf["head_total"]), 2),
            "arm_pct": round(_pct(tf["arm_ok"], tf["arm_total"]), 2),
            "dual_pct": round(_pct(tf["both_ok"], tf["both_total"]), 2),
            "head_ok_intervals": tf["head_ok"],
            "head_total_intervals": tf["head_total"],
            "arm_ok_intervals": tf["arm_ok"],
            "arm_total_intervals": tf["arm_total"],
            "both_ok_intervals": tf["both_ok"],
            "both_total_intervals": tf["both_total"],
        },
        "marker_yield": {
            "head_pct": round(_pct(my["head_corrections"], my["head_markers"]), 2),
            "arm_pct": round(_pct(my["arm_corrections"], my["arm_markers"]), 2),
            "overall_pct": round(_pct(tot_corr, tot_markers), 2),
            "head_corrections": my["head_corrections"],
            "head_markers": my["head_markers"],
            "arm_corrections": my["arm_corrections"],
            "arm_markers": my["arm_markers"],
        },
        "marker_rejections": {
            "aruco_rejection_rate_pct": round(
                _pct(len(aruco), len(aruco) + tot_corr), 2),
            "aruco_total": len(aruco),
            "aruco_by_type": _reject_block(aruco),
            "msckf_total": len(msckf),
            "msckf_by_type": _reject_block(msckf),
        },
        "hard_reanchoring": {
            "total_incidents": len(rea["incidents"]),
            "rate_per_minute": round(
                len(rea["incidents"]) / mon_min, 3) if mon_min > 0 else 0.0,
            "monitoring_minutes": round(mon_min, 2),
            "runs_with_data": rea["runs_with_data"],
        "runs_with_incidents": rea.get("runs_with_incidents", 0),
            "by_reason": dict(by_reason),
        },
        "fusion_dual_view": {
            "overall_pct": round(
                _pct(fus["total_dual"],
                     fus["total_dual"] + fus["total_cam1_only"]), 2),
            "total_dual": fus["total_dual"],
            "total_cam1_only": fus["total_cam1_only"],
        },
    }

    # --- QoS / middleware-blocked topics ---
    qos_hits = metrics["qos"]["hits"]
    qos_topics = set(h.topic for h in qos_hits)
    qos_by_policy: dict[str, set] = defaultdict(set)
    for h in qos_hits:
        qos_by_policy[h.policy.replace("_QOS_POLICY", "")].add(h.topic)
    topic_log_count: dict[str, int] = defaultdict(int)
    for h in qos_hits:
        topic_log_count[h.topic] += 1
    out["qos_middleware_blocked"] = {
        "logs_affected": cov["n_host_qos"],
        "pct_logs_affected": round(_pct(cov["n_host_qos"], cov["n_host"]), 2),
        "unique_topics_blocked": len(qos_topics),
        "topics_blocked": sorted(qos_topics),
        "by_policy": {pol: sorted(topics)
                      for pol, topics in qos_by_policy.items()},
        "most_blocked_topics": dict(sorted(topic_log_count.items(),
                                           key=lambda kv: -kv[1])[:10]),
    }

    # --- Transport latency (clock_off by topic type) ---
    latency: dict = metrics["latency"]
    lat_out: dict = {}
    total_lat_vals = 0
    total_lat_over = 0
    for ttype in ("points", "depth", "image", "odom", "other"):
        vals = latency.get(ttype, [])
        if not vals:
            continue
        sv = sorted(vals)
        over = sum(1 for v in vals if v > CLOCK_OFF_WARN_S)
        total_lat_vals += len(vals)
        total_lat_over += over
        lat_out[ttype] = {
            "count": len(vals),
            "median_ms": round(_percentile(sv, 0.50) * 1000, 1),
            "p75_ms": round(_percentile(sv, 0.75) * 1000, 1),
            "p95_ms": round(_percentile(sv, 0.95) * 1000, 1),
            "peak_ms": round(max(vals) * 1000, 1),
            "pct_over_150ms": round(_pct(over, len(vals)), 1),
        }
    # Chrony drift proof (if bag data available).
    bag = metrics.get("bag", {})
    all_drift = bag.get("host_drift_ms", []) + bag.get("jetson_drift_ms", [])
    out["transport_latency"] = {
        "by_topic_type": lat_out,
        "total_observations": total_lat_vals,
        "pct_over_150ms": round(_pct(total_lat_over, total_lat_vals), 1),
        "stale_excluded": metrics.get("latency_stale", 0),
        "chrony_peak_drift_ms": round(max(all_drift), 3) if all_drift else None,
        "note": ("clock_off represents transport/queueing latency; "
                 "chrony drift < 0.2ms confirms it is not clock skew; "
                 "stale/dead-stream observations (>10s) excluded"),
    }

    # --- Bandwidth (bag sysmon) ---
    host_rx = bag.get("host_rx_mbs", [])
    jetson_tx = bag.get("jetson_tx_mbs", [])
    jetson_errout = bag.get("jetson_errout", [])
    out["network_bandwidth"] = {
        "bag_sysmon_files": cov["n_bags"],
        "host_rx_avg_mbps": round(sum(host_rx) / len(host_rx), 1) if host_rx else None,
        "host_rx_peak_mbps": round(max(host_rx), 1) if host_rx else None,
        "jetson_tx_avg_mbps": round(sum(jetson_tx) / len(jetson_tx), 1) if jetson_tx else None,
        "jetson_tx_peak_mbps": round(max(jetson_tx), 1) if jetson_tx else None,
        "jetson_errout_min": min(jetson_errout) if jetson_errout else None,
        "jetson_errout_max": max(jetson_errout) if jetson_errout else None,
    }

    # --- Kernel network health / reassembly (bag sysmon SNMP) ---
    host_reasm = bag.get("host_reasm", [])
    jetson_reasm = bag.get("jetson_reasm", [])
    host_rcvbuf = bag.get("host_rcvbuf", [])
    jetson_rcvbuf = bag.get("jetson_rcvbuf", [])
    all_reasm = host_reasm + jetson_reasm
    all_rcvbuf = host_rcvbuf + jetson_rcvbuf
    out["kernel_network_health"] = {
        "bag_sysmon_files": cov["n_bags"],
        "ip_reasm_fails_total": sum(all_reasm) if all_reasm else 0,
        "ip_reasm_fails_peak": max(all_reasm) if all_reasm else 0,
        "ip_reasm_fails_host": sum(host_reasm),
        "ip_reasm_fails_jetson": sum(jetson_reasm),
        "udp_rcvbuf_errors_total": sum(all_rcvbuf) if all_rcvbuf else 0,
        "udp_rcvbuf_errors_peak": max(all_rcvbuf) if all_rcvbuf else 0,
        "udp_rcvbuf_errors_host": sum(host_rcvbuf),
        "udp_rcvbuf_errors_jetson": sum(jetson_rcvbuf),
    }

    with open(path, "w") as f:
        json.dump(out, f, indent=2, sort_keys=True)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", metavar="PATH",
                    help="Write structured JSON metrics to PATH")
    ap.add_argument("--output", metavar="PATH",
                    help="Write text report to file (default: stdout)")
    ap.add_argument("--logs-dir", metavar="PATH",
                    help=f"Override logs directory (default: {LOGS_DIR})")
    ap.add_argument("--verbose", action="store_true",
                    help="Print per-log parsing progress to stderr")
    args = ap.parse_args()

    logs_dir = args.logs_dir or LOGS_DIR

    # Glob host logs (exclude *_analysis.txt).
    host_paths = sorted(
        f for f in glob.glob(os.path.join(logs_dir, "host-log-*.txt"))
        if not f.endswith("_analysis.txt"))
    # Glob Jetson logs (both naming conventions).
    jetson_paths = sorted(
        glob.glob(os.path.join(logs_dir, "jetson-run-jetson-debug-*.txt"))
        + glob.glob(os.path.join(logs_dir, "jetson-log-*.txt")))

    if not host_paths:
        print(f"ERROR: no host logs found in {logs_dir}", file=sys.stderr)
        sys.exit(1)

    # Glob bag sysmon files (data/bags/*/sysmon.jsonl) for bandwidth/SNMP/chrony.
    bag_paths = sorted(glob.glob(os.path.join(DATA_BAGS_DIR, "*", "sysmon.jsonl")))

    vprint = (lambda *a: print(*a, file=sys.stderr)) if args.verbose else (lambda *a: None)

    # Parse all host logs.
    host_reports: list = []
    n_host = len(host_paths)
    for i, hp in enumerate(host_paths, 1):
        vprint(f"[aggregate] Parsing host log   {i}/{n_host}: {os.path.basename(hp)} ...")
        try:
            rep = parse_log(hp)
        except Exception as e:
            vprint(f"[aggregate]   FAILED: {e}")
            rep = None
        host_reports.append((hp, rep))

    # Match and parse Jetson logs.
    jetson_reports: list = []
    matched_jetson = set()
    for i, (hp, _rep) in enumerate(host_reports, 1):
        jp = find_jetson_pair(hp, jetson_paths)
        if jp is None:
            jetson_reports.append((None, None))
            continue
        matched_jetson.add(jp)
        vprint(f"[aggregate] Parsing jetson log {i}/{n_host}: {os.path.basename(jp)} ...")
        try:
            jr = parse_jetson_log(jp)
        except Exception as e:
            vprint(f"[aggregate]   FAILED: {e}")
            jr = None
        jetson_reports.append((jp, jr))

    # Also parse unmatched Jetson logs (they have no host pair but still
    # contribute to marker/rejection/reanchor pools).
    for jp in jetson_paths:
        if jp in matched_jetson:
            continue
        vprint(f"[aggregate] Parsing unmatched jetson log: {os.path.basename(jp)} ...")
        try:
            jr = parse_jetson_log(jp)
            jetson_reports.append((jp, jr))
        except Exception as e:
            vprint(f"[aggregate]   FAILED: {e}")

    # Parse bag sysmon files for bandwidth/SNMP/chrony data.
    bag_sysmon_reports: list = []
    for i, bp in enumerate(bag_paths, 1):
        vprint(f"[aggregate] Parsing bag sysmon {i}/{len(bag_paths)}: "
              f"{os.path.basename(os.path.dirname(bp))} ...")
        try:
            bsr = parse_bag_sysmon(bp)
            bag_sysmon_reports.append(bsr)
        except Exception as e:
            vprint(f"[aggregate]   FAILED: {e}")

    vprint("[aggregate] Computing metrics...")
    metrics = compute_metrics(host_reports, jetson_reports, bag_sysmon_reports)

    # Compute monitoring period from host timestamps.
    all_ts = []
    for _p, rep in host_reports:
        if rep and rep.t_start is not None:
            all_ts.append(rep.t_start)
    coverage_meta = {}
    if all_ts:
        t_min = min(all_ts)
        t_max = max(all_ts)
        coverage_meta["period_start"] = datetime.datetime.fromtimestamp(
            t_min).strftime("%Y-%m-%d")
        coverage_meta["period_end"] = datetime.datetime.fromtimestamp(
            t_max).strftime("%Y-%m-%d")

    vprint("[aggregate] Done.")

    out = open(args.output, "w") if args.output else sys.stdout
    try:
        print_aggregate_report(metrics, coverage_meta, out)
    finally:
        if args.output:
            out.close()

    if args.json:
        write_json_metrics(metrics, coverage_meta, args.json)
        vprint(f"[aggregate] JSON metrics written to {args.json}")


if __name__ == "__main__":
    main()
