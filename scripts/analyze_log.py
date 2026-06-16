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

# Nominal topic rates (Hz) — used to flag starvation in the rate timeline.
# Kept in sync with the diagnostics node and Makefile.workspace topic list.
EXPECTED_HZ = {
    "/jetson/head/points": 15.0,
    "/jetson/arm/points": 15.0,
    "/jetson/head/image/compressed": 30.0,
    "/jetson/arm/image/compressed": 30.0,
    "/jetson/head/odom": 30.0,
    "/jetson/arm/odom": 30.0,
    "/gtsam/head_pose": 15.0,
    "/gtsam/arm_pose": 15.0,
    "/vis/head_arm_pose": 30.0,
}

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
    r"\[(DEBUG|INFO|WARN|ERROR|FATAL|DIAG-[A-Z]+)\]\s*"  # [LEVEL]
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
RE_DIAG_CHAIN_LINE = re.compile(r"([a-zA-Z0-9_> ]+?):\s+(OK|DISCONNECTED|STALE)")


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
class DiagBlock:
    t: float
    rates: dict = field(default_factory=dict)        # topic -> (hz, total, clock_off)
    norms: dict = field(default_factory=dict)        # (kind, topic) -> norm_str
    chains: dict = field(default_factory=dict)       # edge -> status
    tf_edges: dict = field(default_factory=dict)     # edge -> (updates, jumps, max_jump, age)


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
    t_start: float | None = None
    t_end: float | None = None
    launch_args: list = field(default_factory=list)


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------

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


def _make_failure_modes():
    def tf_jump_x(m, ts):
        return {"edge": m.group(1).strip(), "jump_m": float(m.group(2)), "stamp": float(m.group(3))}
    def odom_suppress_x(m, ts):
        return {"side": m.group(1), "jump_m": float(m.group(2)), "threshold": float(m.group(3))}
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
        FailureMode("gtsam odometry rejected", RE_GTSAM_REJECT, gtsam_reject_x,
                    "gtsam_tracker detected diverging VIO — re-syncing baseline"),
        FailureMode("tsdf GetAllKeyframes timeout", RE_TSDF_TIMEOUT, tsdf_timeout_x,
                    "keyframe_buffer service hung — check buffer load / deadlocks"),
        FailureMode("TF chain disconnected", RE_TFDIAG_CHAIN, tfdiag_chain_x,
                    "OpenVINS→RealSense TF bridge incomplete — Jetson link down?"),
        FailureMode("relay status change", RE_RELAY_STATUS, relay_status_x,
                    "openvins_odom_tf_relay head/arm message-flow state"),
    ]


def parse_log(path: str) -> LogReport:
    rep = LogReport(path=path)
    rep.failures = _make_failure_modes()
    with open(path, "r", errors="replace") as f:
        rep.lines = f.readlines()
    rep.n_lines = len(rep.lines)

    current_diag: DiagBlock | None = None
    current_section: str | None = None

    for lineno, raw in enumerate(rep.lines, 1):
        line = raw.rstrip("\n")
        m = RE_LOG_LINE.match(line)
        if not m:
            # Bare line: launch arg or separator
            mb = RE_BARE_LINE.match(line)
            if mb and "=" in mb.group(2):
                rep.launch_args.append(mb.group(2).strip())
            if RE_PROC_DEATH.search(line):
                rep.crashes.append((lineno, line))
            # Diag continuation line (only [node-N] prefix, no level tag)
            md = RE_DIAG_CONT.match(line)
            if md and current_diag is not None:
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

        # Diagnostics block parsing
        if "------------------------------------------------------------" in msg:
            # finalize current block
            if current_diag is not None:
                rep.diag_blocks.append(current_diag)
            current_diag = DiagBlock(t=ts if ts is not None else 0.0)
            current_section = None
            continue
        if current_diag is not None:
            if ts is not None and current_diag.t == 0.0:
                current_diag.t = ts
            # Section detection: the level tag (DIAG-TF etc.) was consumed by
            # the regex, so check the level + msg content, not "[DIAG-*]" in msg.
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
            else:
                _parse_diag_data(msg, current_section, current_diag)

    # finalize last diag block
    if current_diag is not None and current_diag not in rep.diag_blocks:
        rep.diag_blocks.append(current_diag)

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


def print_timeline(rep: LogReport, out):
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
            exp = EXPECTED_HZ.get(topic)
            mid = series[len(series) // 2]
            flag = ""
            if exp and max(series) > 0 and max(series) < exp * 0.5:
                flag = f"  <-- STARVED (nominal {exp} Hz)"
            elif exp and max(series) > exp * 1.5:
                flag = f"  <-- FLOODED (nominal {exp} Hz)"
            p(f"    {topic:<34} {series[0]:>5.1f} / {mid:>5.1f} / {series[-1]:>5.1f}"
              f"   (min {min(series):.1f}, max {max(series):.1f}){flag}")
        p("")
    # Clock offset — max absolute across topics per block
    p("  Clock offset (max |clock_off| across topics per block):")
    offs = []
    for b in rep.diag_blocks:
        vals = [abs(v[2]) for v in b.rates.values() if v[2] != -1.0]
        if vals:
            offs.append((b.t, max(vals)))
    if offs:
        mx = max(o for _, o in offs)
        p(f"    peak |offset| = {mx:.3f}s at {_fmt_ts(next(t for t, o in offs if o == mx))}")
        big = [(t, o) for t, o in offs if o > 0.05]
        if big:
            p(f"    {len(big)}/{len(offs)} blocks exceeded 50ms offset (clock skew)")
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
        n_ok = statuses.count("OK")
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
        vals = [abs(v[2]) for v in b.rates.values() if v[2] != -1.0]
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


def report(rep: LogReport, out, plot_path: str | None = None):
    print_header(rep, out)
    print_severity(rep, out)
    print_failures(rep, out)
    print_timeline(rep, out)
    print_errors(rep, out)
    print_crashes(rep, out)
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
        vals = [abs(v[2]) for v in b.rates.values() if v[2] != -1.0]
        if vals:
            offs.append(max(vals))
    if offs:
        m["clock_offset_peak_s"] = round(max(offs), 4)

    # TF jump events
    m["tf_jump_events_total"] = len(rep.tf_jump_events)

    # Errors + crashes
    m["n_errors"] = len(rep.errors)
    m["n_crashes"] = len(rep.crashes)

    return m


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _latest_log():
    """Return the most-recently-modified host-log, or None."""
    logs = glob.glob(os.path.join(LOGS_DIR, "host-log-*.txt"))
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
    args = ap.parse_args()

    if args.all:
        logs = sorted(glob.glob(os.path.join(LOGS_DIR, "host-log-*.txt")))
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
        plot_path = None
        if args.plot:
            base = os.path.splitext(logpath)[0]
            plot_path = base + "_analysis.png"
        report(rep, out, plot_path)

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
