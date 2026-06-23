#!/usr/bin/env python3
"""analyze_bag.py — analyze a rosbag2 (MCAP) + sibling sysmon.jsonl for debugging.

Two tiers of analysis:

  Tier A (metadata.yaml, no MCAP read):
    - Per-topic table: count, effective rate vs nominal, health flags
    - Bag duration, total messages, start time
    - Instantly answers "did the Jetson deliver clouds?" etc.

  Tier B (MCAP deep replay via mcap + mcap-ros2-support):
    - Inter-message jitter / gap analysis per topic
    - Per-topic bandwidth (MB/s) derived from message sizes
    - Pose-trajectory divergence (norm over time) for odom/pose topics
    - TF jump reconstruction from /tf messages
    - Clock offset series from message stamp vs receive time

  Sysmon overlay (if sibling sysmon.jsonl exists in the bag dir):
    - CPU/GPU/RAM timeline for host + Jetson
    - Chrony drift series
    - NIC bandwidth + drops on the DDS interface
    - Correlation callouts: aligns system events with topic-rate anomalies

Usage:
  python3 scripts/analyze_bag.py                           # latest bag
  python3 scripts/analyze_bag.py data/bags/v6_<ts>/        # specific bag
  python3 scripts/analyze_bag.py --plot                    # write PNG timelines
  python3 scripts/analyze_bag.py --no-deep                 # Tier A only (fast)
"""
from __future__ import annotations

import argparse
import datetime
import glob
import json
import math
import os
import re
import struct
import sys
from collections import defaultdict
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BAGS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", "bags")

# Health thresholds (used for absolute-rate comparisons, not nominal-relative)
RATE_MIN_HZ = 1.0         # topics below this are flagged as low-rate
GAP_WARN_FRAC = 2.0       # inter-msg gap > 2x median = gap event

# Pose analysis thresholds
TRANS_JUMP_THRESH = 0.05  # m — frame-to-frame translation jump flag
ROT_JUMP_THRESH = 5.0     # deg — frame-to-frame rotation jump flag
VELOCITY_WARN = 2.0       # m/s — implausible velocity for head/arm-mounted cameras
BURST_GAP_S = 2.0         # s — jumps within this window are one "burst"

# Topics that carry pose data for trajectory analysis
POSE_TOPICS = (
    "/jetson/head/odom",
    "/jetson/arm/odom",
    "/gtsam/head_pose",
    "/gtsam/arm_pose",
    "/vis/head_arm_pose",
)

# odom -> gtsam matching pairs for agreement analysis
ODOM_GTSAM_PAIRS = (
    ("/jetson/head/odom", "/gtsam/head_pose"),
    ("/jetson/arm/odom", "/gtsam/arm_pose"),
)

# head/arm pairs for relative-distance analysis (within odom and within gtsam)
HEAD_ARM_PAIRS = (
    ("/jetson/head/odom", "/jetson/arm/odom"),
    ("/gtsam/head_pose", "/gtsam/arm_pose"),
)

# Topics exempt from clock-offset / transport-latency analysis because their
# large clock deltas are an artifact of latched (TRANSIENT_LOCAL) durability,
# not real network latency.  These topics are published once at boot and held;
# the delta between the original (frozen) stamp and a late subscriber match is
# meaningless and produces false-positive "14776ms clock skew" callouts.
LATCHED_TOPICS = {"/tf_static"}


# ---------------------------------------------------------------------------
# Quaternion / rotation helpers (no external deps)
# ---------------------------------------------------------------------------

def quat_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Quaternion (x,y,z,w) -> (roll, pitch, yaw) in degrees."""
    # Normalize to avoid numerical issues
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return 0.0, 0.0, 0.0
    x, y, z, w = x / n, y / n, z / n, w / n
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (z * z + y * y)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


def quat_angular_delta(a: tuple, b: tuple) -> float:
    """Angular distance between two quaternions (x,y,z,w) in degrees."""
    # dot product of two unit quaternions = cos(theta/2)
    dot = abs(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3])
    dot = min(1.0, max(0.0, dot))
    return math.degrees(2 * math.acos(dot))


def _angle_delta_deg(a: float, b: float) -> float:
    """Shortest angular difference between two angles in degrees (wraps at +/-180)."""
    d = (a - b + 180) % 360 - 180
    return abs(d)


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class TopicInfo:
    name: str
    msg_type: str
    count: int
    effective_hz: float | None = None
    # Tier B fields (filled during deep replay)
    sizes: list = field(default_factory=list)       # message sizes (bytes)
    stamps: list = field(default_factory=list)      # publish timestamps (ns)
    recv_times: list = field(default_factory=list)  # receive/log times (ns)
    median_dt_ms: float | None = None
    max_gap_ms: float | None = None
    min_gap_ms: float | None = None
    n_gaps: int = 0
    cv_dt: float | None = None
    avg_size_kb: float | None = None
    bandwidth_mbs: float | None = None

@dataclass
class PoseSample:
    """One timestamped pose sample (6-DOF)."""
    t: float           # timestamp (s)
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    roll: float        # degrees
    pitch: float       # degrees
    yaw: float         # degrees
    cov_diag: tuple | None = None  # (var_x, var_y, var_z, var_roll, var_pitch, var_yaw) or None


@dataclass
class PoseTopicStats:
    """Aggregated statistics for a single pose topic."""
    topic: str
    n: int = 0
    duration_s: float = 0.0
    # Translation
    x_range: tuple = (0, 0)
    y_range: tuple = (0, 0)
    z_range: tuple = (0, 0)
    path_length: float = 0.0
    max_trans_jump: float = 0.0
    max_trans_jump_t: float = 0.0
    max_trans_jump_axis: str = ""
    # Rotation
    roll_range: tuple = (0, 0)
    pitch_range: tuple = (0, 0)
    yaw_range: tuple = (0, 0)
    max_rot_jump: float = 0.0       # degrees
    max_rot_jump_t: float = 0.0
    max_rot_jump_axis: str = ""
    # Velocity
    max_velocity: float = 0.0       # m/s
    max_velocity_t: float = 0.0
    velocity_warns: int = 0
    # Discontinuities (frame-to-frame jumps above threshold)
    trans_jumps: list = field(default_factory=list)   # (t_rel, magnitude, axis)
    rot_jumps: list = field(default_factory=list)     # (t_rel, magnitude_deg, axis)
    # Covariance
    cov_max: float = 0.0
    cov_all_zero: bool = False


@dataclass
class BagReport:
    bag_dir: str
    duration_s: float = 0.0
    total_msgs: int = 0
    start_ns: int = 0
    start_iso: str = ""
    topics: dict = field(default_factory=dict)  # name -> TopicInfo
    pose_trajectories: dict = field(default_factory=dict)  # topic -> [(t, norm)] (legacy)
    pose_samples: dict = field(default_factory=dict)       # topic -> [PoseSample]
    pose_stats: dict = field(default_factory=dict)         # topic -> PoseTopicStats
    tf_jumps: list = field(default_factory=list)  # (t, parent, child, jump_m)
    sysmon: dict = field(default_factory=dict)  # parsed sysmon data


# ---------------------------------------------------------------------------
# Tier A: metadata.yaml parsing
# ---------------------------------------------------------------------------

def _parse_metadata(bag_dir: str) -> BagReport:
    """Parse bag metadata — tries metadata.yaml first, then MCAP file."""
    import yaml
    meta_path = os.path.join(bag_dir, "metadata.yaml")
    rep = BagReport(bag_dir=os.path.normpath(bag_dir))

    if os.path.isfile(meta_path):
        return _parse_metadata_yaml(meta_path, rep)
    else:
        print("[analyze_bag] metadata.yaml missing — falling back to MCAP file",
              file=sys.stderr)
        return _parse_metadata_mcap(bag_dir, rep)


def _parse_metadata_yaml(meta_path: str, rep: BagReport) -> BagReport:
    """Parse a rosbag2 metadata.yaml file."""
    import yaml
    with open(meta_path) as f:
        d = yaml.safe_load(f)
    info = d["rosbag2_bagfile_information"]

    rep.duration_s = info["duration"]["nanoseconds"] / 1e9
    rep.total_msgs = info["message_count"]
    rep.start_ns = info["starting_time"]["nanoseconds_since_epoch"]
    rep.start_iso = datetime.datetime.fromtimestamp(
        rep.start_ns / 1e9).strftime("%Y-%m-%d %H:%M:%S")

    for t in info["topics_with_message_count"]:
        name = t["topic_metadata"]["name"]
        ti = TopicInfo(
            name=name,
            msg_type=t["topic_metadata"]["type"],
            count=t["message_count"],
        )
        _compute_topic_rate(ti, rep.duration_s)
        rep.topics[name] = ti
    return rep


def _parse_metadata_mcap(bag_dir: str, rep: BagReport) -> BagReport:
    """Extract bag metadata from the MCAP file(s) when metadata.yaml is absent.

    Uses the low-level StreamReader (not make_reader) so we can gracefully
    handle truncated/corrupt MCAP files from interrupted recordings.
    """
    mcap_files = sorted(glob.glob(os.path.join(bag_dir, "*.mcap")))
    if not mcap_files:
        print(f"[analyze_bag] no .mcap file in {bag_dir} — report will be empty",
              file=sys.stderr)
        rep.start_iso = "unknown"
        return rep

    try:
        from mcap.stream_reader import StreamReader
        from mcap.records import Channel, Message, Schema
    except ImportError:
        print("[analyze_bag] mcap not installed — cannot read MCAP metadata",
              file=sys.stderr)
        rep.start_iso = "unknown (install mcap)"
        return rep

    # Collect topic message counts and find earliest/latest timestamps.
    # StreamReader reads raw records sequentially without needing a valid
    # summary, so it works with truncated files (we catch errors at EOF).
    topic_counts: dict = defaultdict(int)
    topic_types: dict = {}
    channels: dict = {}   # channel_id -> Channel
    schemas: dict = {}     # schema_id -> Schema
    earliest_ns: int = None
    latest_ns: int = None
    total_msgs = 0

    for mcap_path in mcap_files:
        with open(mcap_path, "rb") as f:
            reader = StreamReader(f)
            try:
                for rec in reader.records:
                    if isinstance(rec, Schema):
                        schemas[rec.id] = rec
                    elif isinstance(rec, Channel):
                        channels[rec.id] = rec
                    elif isinstance(rec, Message):
                        ch = channels.get(rec.channel_id)
                        if ch is not None:
                            topic = ch.topic
                            topic_counts[topic] = topic_counts.get(topic, 0) + 1
                            if topic not in topic_types:
                                s = schemas.get(ch.schema_id)
                                topic_types[topic] = s.name if s else "unknown"
                        stamp_ns = rec.publish_time
                        if earliest_ns is None or stamp_ns < earliest_ns:
                            earliest_ns = stamp_ns
                        if latest_ns is None or stamp_ns > latest_ns:
                            latest_ns = stamp_ns
                        total_msgs += 1
            except (struct.error, Exception) as e:
                # Truncated MCAP — the file was killed mid-write.
                print(f"[analyze_bag] MCAP read stopped early "
                      f"(corrupt/truncated file): {e}",
                      file=sys.stderr)

    rep.total_msgs = total_msgs
    if earliest_ns and latest_ns:
        rep.start_ns = earliest_ns
        rep.duration_s = (latest_ns - earliest_ns) / 1e9
        rep.start_iso = datetime.datetime.fromtimestamp(
            earliest_ns / 1e9).strftime("%Y-%m-%d %H:%M:%S")
    else:
        rep.start_iso = "unknown (no messages)"
        rep.duration_s = 0.0

    for name, count in topic_counts.items():
        ti = TopicInfo(
            name=name,
            msg_type=topic_types.get(name, "unknown"),
            count=count,
        )
        _compute_topic_rate(ti, rep.duration_s)
        rep.topics[name] = ti

    print(f"[analyze_bag] MCAP fallback: {total_msgs} msgs across "
          f"{len(topic_counts)} topics", file=sys.stderr)
    return rep


def _compute_topic_rate(ti: TopicInfo, duration_s: float):
    """Compute effective Hz for a topic from metadata counts."""
    if ti.count > 0 and duration_s > 0:
        ti.effective_hz = ti.count / duration_s


# ---------------------------------------------------------------------------
# Pose statistics computation
# ---------------------------------------------------------------------------

# Thresholds
TRANS_JUMP_THRESHOLD = 0.05    # m — matches diagnostics node
ROT_JUMP_THRESHOLD = 5.0       # degrees
VELOCITY_WARN_THRESHOLD = 2.0  # m/s — implausible for head/arm-mounted cameras


def _compute_pose_stats(topic: str, samples: list) -> PoseTopicStats:
    """Compute full 6-DOF statistics for a single pose topic."""
    stats = PoseTopicStats(topic=topic)
    if len(samples) < 2:
        return stats
    stats.n = len(samples)
    t0 = samples[0].t
    stats.duration_s = samples[-1].t - t0

    xs = [s.x for s in samples]
    ys = [s.y for s in samples]
    zs = [s.z for s in samples]
    rolls = [s.roll for s in samples]
    pitches = [s.pitch for s in samples]
    yaws = [s.yaw for s in samples]

    stats.x_range = (min(xs), max(xs))
    stats.y_range = (min(ys), max(ys))
    stats.z_range = (min(zs), max(zs))
    stats.roll_range = (min(rolls), max(rolls))
    stats.pitch_range = (min(pitches), max(pitches))
    stats.yaw_range = (min(yaws), max(yaws))

    # Frame-to-frame analysis
    for i in range(1, len(samples)):
        prev, cur = samples[i - 1], samples[i]
        dt = cur.t - prev.t
        t_rel = cur.t - t0

        # Translation jump
        dx, dy, dz = cur.x - prev.x, cur.y - prev.y, cur.z - prev.z
        trans_jump = math.sqrt(dx * dx + dy * dy + dz * dz)
        stats.path_length += trans_jump

        if trans_jump > stats.max_trans_jump:
            stats.max_trans_jump = trans_jump
            stats.max_trans_jump_t = t_rel
            # Which axis dominates?
            abs_d = {"X": abs(dx), "Y": abs(dy), "Z": abs(dz)}
            stats.max_trans_jump_axis = max(abs_d, key=abs_d.get)

        if trans_jump > TRANS_JUMP_THRESHOLD:
            abs_d = {"X": abs(dx), "Y": abs(dy), "Z": abs(dz)}
            axis = max(abs_d, key=abs_d.get)
            stats.trans_jumps.append((t_rel, trans_jump, axis))

        # Rotation jump (angular distance between quaternions)
        rot_jump = quat_angular_delta(
            (prev.qx, prev.qy, prev.qz, prev.qw),
            (cur.qx, cur.qy, cur.qz, cur.qw))
        if rot_jump > stats.max_rot_jump:
            stats.max_rot_jump = rot_jump
            stats.max_rot_jump_t = t_rel
            # Which euler axis changed most?
            d_roll = _angle_delta_deg(cur.roll, prev.roll)
            d_pitch = _angle_delta_deg(cur.pitch, prev.pitch)
            d_yaw = _angle_delta_deg(cur.yaw, prev.yaw)
            abs_d = {"roll": d_roll, "pitch": d_pitch, "yaw": d_yaw}
            stats.max_rot_jump_axis = max(abs_d, key=abs_d.get)

        if rot_jump > ROT_JUMP_THRESHOLD:
            d_roll = _angle_delta_deg(cur.roll, prev.roll)
            d_pitch = _angle_delta_deg(cur.pitch, prev.pitch)
            d_yaw = _angle_delta_deg(cur.yaw, prev.yaw)
            abs_d = {"roll": d_roll, "pitch": d_pitch, "yaw": d_yaw}
            axis = max(abs_d, key=abs_d.get)
            stats.rot_jumps.append((t_rel, rot_jump, axis))

        # Velocity
        if dt > 1e-9:
            vel = trans_jump / dt
            if vel > stats.max_velocity:
                stats.max_velocity = vel
                stats.max_velocity_t = t_rel
            if vel > VELOCITY_WARN_THRESHOLD:
                stats.velocity_warns += 1

    # Covariance analysis
    cov_vals = [s.cov_diag for s in samples if s.cov_diag is not None]
    if cov_vals:
        all_zero = True
        for cd in cov_vals:
            for v in cd:
                if v > 1e-12:
                    all_zero = False
                    stats.cov_max = max(stats.cov_max, v)
        stats.cov_all_zero = all_zero

    return stats


def _interpolate_pose(samples: list, t: float) -> PoseSample | None:
    """Linear interpolation of pose at time t. Returns None if out of range."""
    if not samples:
        return None
    if t <= samples[0].t:
        return samples[0]
    if t >= samples[-1].t:
        return samples[-1]
    # Binary search
    lo, hi = 0, len(samples) - 1
    while lo < hi - 1:
        mid = (lo + hi) // 2
        if samples[mid].t <= t:
            lo = mid
        else:
            hi = mid
    s0, s1 = samples[lo], samples[hi]
    dt = s1.t - s0.t
    if dt < 1e-9:
        return s0
    alpha = (t - s0.t) / dt
    return PoseSample(
        t=t,
        x=s0.x + alpha * (s1.x - s0.x),
        y=s0.y + alpha * (s1.y - s0.y),
        z=s0.z + alpha * (s1.z - s0.z),
        qx=s0.qx, qy=s0.qy, qz=s0.qz, qw=s0.qw,  # don't interp quaternion (short intervals)
        roll=s0.roll + alpha * (s1.roll - s0.roll),
        pitch=s0.pitch + alpha * (s1.pitch - s0.pitch),
        yaw=s0.yaw + alpha * (s1.yaw - s0.yaw),
    )


@dataclass
class PoseAgreement:
    """Odom→gtsam agreement analysis for a matched pair."""
    odom_topic: str
    gtsam_topic: str
    n_pairs: int = 0
    duration_s: float = 0.0
    # Translation delta series: (t_rel, delta_m)
    trans_deltas: list = field(default_factory=list)
    # Rotation delta series: (t_rel, delta_deg)
    rot_deltas: list = field(default_factory=list)
    trans_delta_max: float = 0.0
    trans_delta_mean: float = 0.0
    rot_delta_max: float = 0.0
    rot_delta_mean: float = 0.0
    # Divergence: linear regression slope of delta over time
    trans_divergence_rate: float = 0.0  # m/s (positive = diverging)
    rot_divergence_rate: float = 0.0    # deg/s


def _compute_pose_agreement(odom_samples: list, gtsam_samples: list,
                            odom_topic: str, gtsam_topic: str) -> PoseAgreement | None:
    """Compare odom vs gtsam at matching timestamps."""
    if len(odom_samples) < 2 or len(gtsam_samples) < 2:
        return None
    ag = PoseAgreement(odom_topic=odom_topic, gtsam_topic=gtsam_topic)
    t0 = max(odom_samples[0].t, gtsam_samples[0].t)
    t_end = min(odom_samples[-1].t, gtsam_samples[-1].t)
    if t_end <= t0:
        return None
    ag.duration_s = t_end - t0

    # Sample at odom timestamps within overlap
    for s_odom in odom_samples:
        if s_odom.t < t0 or s_odom.t > t_end:
            continue
        s_gtsam = _interpolate_pose(gtsam_samples, s_odom.t)
        if s_gtsam is None:
            continue
        t_rel = s_odom.t - t0
        dx = s_odom.x - s_gtsam.x
        dy = s_odom.y - s_gtsam.y
        dz = s_odom.z - s_gtsam.z
        d_trans = math.sqrt(dx * dx + dy * dy + dz * dz)
        d_rot = quat_angular_delta(
            (s_odom.qx, s_odom.qy, s_odom.qz, s_odom.qw),
            (s_gtsam.qx, s_gtsam.qy, s_gtsam.qz, s_gtsam.qw))
        ag.trans_deltas.append((t_rel, d_trans))
        ag.rot_deltas.append((t_rel, d_rot))
        ag.n_pairs += 1

    if ag.n_pairs < 2:
        return None

    ag.trans_delta_max = max(d for _, d in ag.trans_deltas)
    ag.trans_delta_mean = sum(d for _, d in ag.trans_deltas) / ag.n_pairs
    ag.rot_delta_max = max(d for _, d in ag.rot_deltas)
    ag.rot_delta_mean = sum(d for _, d in ag.rot_deltas) / ag.n_pairs

    # Divergence trend: simple linear regression slope of delta vs time
    if ag.duration_s > 0.5:
        ts = [t for t, _ in ag.trans_deltas]
        ds = [d for _, d in ag.trans_deltas]
        n = len(ts)
        t_mean = sum(ts) / n
        d_mean = sum(ds) / n
        denom = sum((t - t_mean) ** 2 for t in ts)
        if denom > 1e-9:
            ag.trans_divergence_rate = sum((t - t_mean) * (d - d_mean)
                                           for t, d in zip(ts, ds)) / denom
        rs = [d for _, d in ag.rot_deltas]
        r_mean = sum(rs) / n
        denom_r = sum((t - t_mean) ** 2 for t in ts)
        if denom_r > 1e-9:
            ag.rot_divergence_rate = sum((t - t_mean) * (r - r_mean)
                                         for t, r in zip(ts, rs)) / denom_r

    return ag


@dataclass
class RelativeDistance:
    """Head-arm relative distance analysis."""
    n: int = 0
    duration_s: float = 0.0
    distances: list = field(default_factory=list)  # (t_rel, dist_m)
    mean: float = 0.0
    std: float = 0.0
    min_val: float = 0.0
    max_val: float = 0.0
    max_rate_of_change: float = 0.0  # m/s — how fast the relative distance changed


def _compute_relative_distance(head_samples: list, arm_samples: list,
                               source_label: str) -> RelativeDistance | None:
    """Compute head-arm relative distance over time (for odom or gtsam)."""
    if len(head_samples) < 2 or len(arm_samples) < 2:
        return None
    rd = RelativeDistance()
    t0 = max(head_samples[0].t, arm_samples[0].t)
    t_end = min(head_samples[-1].t, arm_samples[-1].t)
    if t_end <= t0:
        return None
    rd.duration_s = t_end - t0

    for s_head in head_samples:
        if s_head.t < t0 or s_head.t > t_end:
            continue
        s_arm = _interpolate_pose(arm_samples, s_head.t)
        if s_arm is None:
            continue
        t_rel = s_head.t - t0
        dx = s_head.x - s_arm.x
        dy = s_head.y - s_arm.y
        dz = s_head.z - s_arm.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        rd.distances.append((t_rel, dist))
        rd.n += 1

    if rd.n < 2:
        return None

    vals = [d for _, d in rd.distances]
    rd.mean = sum(vals) / rd.n
    rd.std = (sum((v - rd.mean) ** 2 for v in vals) / rd.n) ** 0.5
    rd.min_val = min(vals)
    rd.max_val = max(vals)

    # Rate of change
    for i in range(1, len(rd.distances)):
        dt = rd.distances[i][0] - rd.distances[i - 1][0]
        if dt > 1e-9:
            rate = abs(rd.distances[i][1] - rd.distances[i - 1][1]) / dt
            rd.max_rate_of_change = max(rd.max_rate_of_change, rate)

    return rd


def _compute_all_agreements(pose_samples: dict) -> list:
    """Compute odom→gtsam agreement for all matching head/arm pairs."""
    pairs = [("/jetson/head/odom", "/gtsam/head_pose"),
             ("/jetson/arm/odom", "/gtsam/arm_pose")]
    agreements = []
    for odom_t, gtsam_t in pairs:
        odom_s = pose_samples.get(odom_t, [])
        gtsam_s = pose_samples.get(gtsam_t, [])
        if len(odom_s) < 2 or len(gtsam_s) < 2:
            continue
        ag = _compute_pose_agreement(odom_s, gtsam_s, odom_t, gtsam_t)
        if ag is not None:
            agreements.append(ag)
    return agreements


# ---------------------------------------------------------------------------
# Tier B: MCAP deep replay
# ---------------------------------------------------------------------------

def _deep_replay(rep: BagReport, max_msgs_per_topic: int = 5000):
    """Read the MCAP file to extract per-message sizes, timestamps, and pose data.

    Falls back to raw StreamReader if the MCAP summary/index is corrupt
    (e.g. from an interrupted recording), which skips pose/TF decoding but
    still collects per-message timestamps and sizes for Tier B stats.
    """
    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError:
        print("[analyze_bag] mcap/mcap-ros2-support not installed — skipping Tier B",
              file=sys.stderr)
        return False

    # Find the .mcap file
    mcap_files = sorted(glob.glob(os.path.join(rep.bag_dir, "*.mcap")))
    if not mcap_files:
        print(f"[analyze_bag] no .mcap file in {rep.bag_dir} — skipping Tier B",
              file=sys.stderr)
        return False

    # Try full decoded replay first (pose/TF extraction needs valid summary).
    # If the MCAP summary is corrupt, fall back to raw StreamReader.
    try:
        return _deep_replay_decoded(rep, mcap_files, max_msgs_per_topic)
    except Exception as e:
        print(f"[analyze_bag] MCAP summary corrupt, falling back to raw replay: {e}",
              file=sys.stderr)
        return _deep_replay_raw(rep, mcap_files, max_msgs_per_topic)


def _deep_replay_decoded(rep: BagReport, mcap_files: list,
                         max_msgs_per_topic: int) -> bool:
    """Full decoded replay — requires a valid MCAP summary/index."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    pose_norms = defaultdict(list)  # topic -> [(stamp_s, norm)]
    pose_samples = defaultdict(list)  # topic -> [PoseSample]
    tf_history = defaultdict(list)  # (parent, child) -> [(stamp_s, tx, ty, tz)]

    POSE_TOPICS = ("/jetson/head/odom", "/jetson/arm/odom",
                   "/gtsam/head_pose", "/gtsam/arm_pose",
                   "/vis/head_arm_pose")

    total = 0
    for mcap_path in mcap_files:
        with open(mcap_path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for schema, channel, message, ros_msg in reader.iter_decoded_messages():
                topic = channel.topic
                ti = rep.topics.get(topic)
                stamp_s = message.publish_time / 1e9
                size = len(message.data)

                if ti is not None:
                    if len(ti.sizes) < max_msgs_per_topic:
                        ti.sizes.append(size)
                        ti.stamps.append(message.publish_time)
                        ti.recv_times.append(message.log_time)

                # Full 6-DOF pose extraction
                if topic in POSE_TOPICS:
                    try:
                        pos = ros_msg.pose.pose.position
                        ori = ros_msg.pose.pose.orientation
                        px, py, pz = float(pos.x), float(pos.y), float(pos.z)
                        qx, qy, qz, qw = float(ori.x), float(ori.y), float(ori.z), float(ori.w)
                        roll, pitch, yaw = quat_to_euler(qx, qy, qz, qw)
                        norm = (px * px + py * py + pz * pz) ** 0.5
                        pose_norms[topic].append((stamp_s, norm))
                        # Covariance diagonal (if present — PoseWithCovarianceStamped)
                        cov_diag = None
                        cov = getattr(ros_msg.pose, "covariance", None)
                        if cov is not None and len(cov) >= 36:
                            cov_diag = (float(cov[0]), float(cov[7]), float(cov[14]),
                                        float(cov[21]), float(cov[28]), float(cov[35]))
                        pose_samples[topic].append(
                            PoseSample(stamp_s, px, py, pz, qx, qy, qz, qw,
                                       roll, pitch, yaw, cov_diag))
                    except (AttributeError, TypeError):
                        pass

                # TF jump detection from /tf
                if topic == "/tf":
                    try:
                        for tf in ros_msg.transforms:
                            child = tf.child_frame_id
                            parent = tf.header.frame_id
                            tx = float(tf.transform.translation.x)
                            ty = float(tf.transform.translation.y)
                            tz = float(tf.transform.translation.z)
                            key = (parent, child)
                            hist = tf_history[key]
                            if hist:
                                _, px, py, pz = hist[-1]
                                jump = ((tx - px) ** 2 + (ty - py) ** 2 +
                                        (tz - pz) ** 2) ** 0.5
                                if jump > 0.05:
                                    rep.tf_jumps.append(
                                        (stamp_s, parent, child, jump))
                            hist.append((stamp_s, tx, ty, tz))
                    except (AttributeError, TypeError):
                        pass

                total += 1

    # Compute Tier B stats per topic
    for ti in rep.topics.values():
        if len(ti.sizes) < 2:
            continue
        # Inter-message timing
        dts = [(ti.stamps[i] - ti.stamps[i - 1]) / 1e6
               for i in range(1, len(ti.stamps))]
        dts_ms = sorted(dts)
        ti.median_dt_ms = dts_ms[len(dts_ms) // 2] if dts_ms else None
        ti.min_gap_ms = dts_ms[0] if dts_ms else None
        med = ti.median_dt_ms or 1.0
        ti.max_gap_ms = max(dts) if dts else None
        ti.n_gaps = sum(1 for d in dts if d > med * GAP_WARN_FRAC)
        # Coefficient of variation (variability metric)
        if len(dts) > 1:
            mean_dt = sum(dts) / len(dts)
            if mean_dt > 1e-9:
                std_dt = (sum((d - mean_dt) ** 2 for d in dts) / len(dts)) ** 0.5
                ti.cv_dt = round(std_dt / mean_dt, 3)
        # Bandwidth
        ti.avg_size_kb = sum(ti.sizes) / len(ti.sizes) / 1024.0
        if ti.effective_hz:
            ti.bandwidth_mbs = ti.avg_size_kb * ti.effective_hz / 1024.0

    rep.pose_trajectories = dict(pose_norms)
    rep.pose_samples = dict(pose_samples)
    rep.pose_stats = {t: _compute_pose_stats(t, samples)
                      for t, samples in pose_samples.items()}
    print(f"[analyze_bag] Tier B: read {total} messages from {len(mcap_files)} file(s)",
          file=sys.stderr)
    return True


def _deep_replay_raw(rep: BagReport, mcap_files: list,
                     max_msgs_per_topic: int) -> bool:
    """Raw StreamReader fallback for corrupted MCAP files (no summary/index).

    Collects per-message sizes, publish times, and log times for Tier B stats
    (jitter, bandwidth).  Pose/TF decoding is skipped because ROS
    deserialization requires schemas from the summary.
    """
    try:
        from mcap.stream_reader import StreamReader
        from mcap.records import Channel, Message
    except ImportError:
        print("[analyze_bag] mcap StreamReader not available — no Tier B data",
              file=sys.stderr)
        return False

    total = 0
    channels: dict = {}   # channel_id -> Channel
    for mcap_path in mcap_files:
        with open(mcap_path, "rb") as f:
            reader = StreamReader(f)
            try:
                for rec in reader.records:
                    if isinstance(rec, Channel):
                        channels[rec.id] = rec
                    elif isinstance(rec, Message):
                        ch = channels.get(rec.channel_id)
                        if ch is None:
                            continue
                        topic = ch.topic
                        ti = rep.topics.get(topic)
                        size = len(rec.data)
                        if ti is not None:
                            if len(ti.sizes) < max_msgs_per_topic:
                                ti.sizes.append(size)
                                ti.stamps.append(rec.publish_time)
                                ti.recv_times.append(rec.log_time)
                        total += 1
            except (struct.error, Exception) as e:
                print(f"[analyze_bag] MCAP read stopped early: {e}",
                      file=sys.stderr)

    # Compute Tier B stats per topic (same as decoded path)
    for ti in rep.topics.values():
        if len(ti.sizes) < 2:
            continue
        dts = [(ti.stamps[i] - ti.stamps[i - 1]) / 1e6
               for i in range(1, len(ti.stamps))]
        dts_ms = sorted(dts)
        ti.median_dt_ms = dts_ms[len(dts_ms) // 2] if dts_ms else None
        ti.min_gap_ms = dts_ms[0] if dts_ms else None
        med = ti.median_dt_ms or 1.0
        ti.max_gap_ms = max(dts) if dts else None
        ti.n_gaps = sum(1 for d in dts if d > med * GAP_WARN_FRAC)
        # Coefficient of variation (variability metric)
        if len(dts) > 1:
            mean_dt = sum(dts) / len(dts)
            if mean_dt > 1e-9:
                std_dt = (sum((d - mean_dt) ** 2 for d in dts) / len(dts)) ** 0.5
                ti.cv_dt = round(std_dt / mean_dt, 3)
        ti.avg_size_kb = sum(ti.sizes) / len(ti.sizes) / 1024.0
        if ti.effective_hz:
            ti.bandwidth_mbs = ti.avg_size_kb * ti.effective_hz / 1024.0

    print(f"[analyze_bag] Tier B (raw): read {total} messages from "
          f"{len(mcap_files)} file(s) (pose/TF decoding skipped — "
          f"corrupt summary)",
          file=sys.stderr)
    return True


# ---------------------------------------------------------------------------
# Sysmon overlay
# ---------------------------------------------------------------------------

def _load_sysmon(bag_dir: str) -> dict | None:
    """Load sibling sysmon.jsonl if present."""
    path = os.path.join(bag_dir, "sysmon.jsonl")
    if not os.path.exists(path):
        return None
    host = []
    jetson = []
    meta = None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            src = obj.get("source")
            if src == "meta":
                meta = obj
            elif src == "host":
                host.append(obj)
            elif src == "jetson":
                jetson.append(obj)
    return {"meta": meta, "host": host, "jetson": jetson}


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def print_tier_a(rep: BagReport, out):
    p = lambda *a, **kw: print(*a, file=out, **kw)
    p("=" * 90)
    p(f"BAG ANALYSIS — {os.path.basename(rep.bag_dir)}")
    p("=" * 90)
    p(f"Directory:  {rep.bag_dir}")
    p(f"Duration:   {rep.duration_s:.1f}s")
    p(f"Messages:   {rep.total_msgs}")
    p(f"Started:    {rep.start_iso}")
    p("")
    p("-" * 90)
    p("TIER A — TOPIC OVERVIEW (from metadata.yaml)")
    p("-" * 90)
    p(f"  {'TOPIC':<36} {'TYPE':<20} {'COUNT':>7} {'EFF_HZ':>7}")
    p(f"  {'-'*36} {'-'*20} {'-'*7} {'-'*7}")
    for name in sorted(rep.topics):
        ti = rep.topics[name]
        eff = f"{ti.effective_hz:.1f}" if ti.effective_hz else "—"
        absent_flag = "  ABSENT" if ti.count == 0 else ""
        hf_flag = ""
        if ti.effective_hz and ti.effective_hz > 200:
            hf_flag = (f"  [WARNING: {ti.effective_hz:.0f} Hz — high-frequency "
                       f"topic may cause local CPU exhaustion and "
                       f"network stack choking]")
        p(f"  {name:<36} {ti.msg_type:<20.20} {ti.count:>7} {eff:>7}{absent_flag}{hf_flag}")
    p("")
    # Summary of absent topics
    absent = [t for t in rep.topics.values() if t.count == 0]
    if absent:
        p("  ABSENT TOPICS (0 messages):")
        for ti in absent:
            p(f"    {ti.name}")
        p("")


def print_tier_b(rep: BagReport, out):
    p = lambda *a, **kw: print(*a, file=out, **kw)
    p("-" * 90)
    p("TIER B — DEEP REPLAY (from MCAP)")
    p("-" * 90)
    has_data = any(t.median_dt_ms is not None for t in rep.topics.values())
    if not has_data:
        p("  (no Tier B data — run without --no-deep, or install mcap + mcap-ros2-support)")
        p("")
        return

    # Timing / jitter table
    p("  Inter-message timing — Hz stats & variability:")
    p(f"  {'TOPIC':<36} {'MIN_HZ':>7} {'MED_HZ':>7} {'MAX_HZ':>7} {'CV':>5} {'AVG_KB':>8} {'MB/s':>7}")
    p(f"  {'-'*36} {'-'*7} {'-'*7} {'-'*7} {'-'*5} {'-'*8} {'-'*7}")
    for name in sorted(rep.topics):
        ti = rep.topics[name]
        if ti.median_dt_ms is None:
            continue
        min_hz = 1000.0 / ti.max_gap_ms if ti.max_gap_ms and ti.max_gap_ms > 0 else 0
        med_hz = 1000.0 / ti.median_dt_ms if ti.median_dt_ms > 0 else 0
        max_hz = 1000.0 / ti.min_gap_ms if ti.min_gap_ms and ti.min_gap_ms > 0 else 0
        cv_str = f"{ti.cv_dt:.2f}" if ti.cv_dt is not None else "—"
        hf_flag = ""
        if max_hz > 200:
            hf_flag = (f"  [WARNING: peak {max_hz:.0f} Hz — high-frequency "
                       f"topic may cause local CPU exhaustion and "
                       f"network stack choking]")
        p(f"  {name:<36} {min_hz:>7.1f} {med_hz:>7.1f} {max_hz:>7.1f} {cv_str:>5} "
          f"{ti.avg_size_kb:>7.1f}K {ti.bandwidth_mbs or 0:>6.2f}{hf_flag}")
    p("  (CV = coefficient of variation of inter-message interval; K = KB)")

    # Full 6-DOF pose analysis
    if rep.pose_stats:
        p("")
        p("  POSE ANALYSIS — 6-DOF per topic:")
        for topic in sorted(rep.pose_stats):
            stats = rep.pose_stats[topic]
            if stats.n < 2:
                continue
            p(f"    {topic} ({stats.n} samples, {stats.duration_s:.1f}s):")
            p(f"      Translation: X=[{stats.x_range[0]:.3f},{stats.x_range[1]:.3f}] "
              f"Y=[{stats.y_range[0]:.3f},{stats.y_range[1]:.3f}] "
              f"Z=[{stats.z_range[0]:.3f},{stats.z_range[1]:.3f}]m")
            p(f"        path_length={stats.path_length:.3f}m")
            p(f"        max_frame_jump={stats.max_trans_jump:.4f}m ({stats.max_trans_jump_axis}-axis "
              f"at t+{stats.max_trans_jump_t:.1f}s)")
            if stats.trans_jumps:
                p(f"        {len(stats.trans_jumps)} trans jumps >{TRANS_JUMP_THRESHOLD}m")
            p(f"      Rotation:    roll=[{stats.roll_range[0]:.1f},{stats.roll_range[1]:.1f}] "
              f"pitch=[{stats.pitch_range[0]:.1f},{stats.pitch_range[1]:.1f}] "
              f"yaw=[{stats.yaw_range[0]:.1f},{stats.yaw_range[1]:.1f}]deg")
            p(f"        max_frame_jump={stats.max_rot_jump:.2f}deg ({stats.max_rot_jump_axis}-axis "
              f"at t+{stats.max_rot_jump_t:.1f}s)")
            if stats.rot_jumps:
                p(f"        {len(stats.rot_jumps)} rot jumps >{ROT_JUMP_THRESHOLD:.0f}deg")
            vel_flag = " [WARN: implausible]" if stats.velocity_warns else ""
            p(f"      Velocity:    max={stats.max_velocity:.2f}m/s at t+{stats.max_velocity_t:.1f}s"
              f"{vel_flag} ({stats.velocity_warns} warnings >{VELOCITY_WARN_THRESHOLD}m/s)")
            if stats.cov_max > 0 or stats.cov_all_zero:
                if stats.cov_all_zero:
                    p(f"      Covariance:  all zeros (GTSAM not populating)")
                else:
                    p(f"      Covariance:  max diagonal={stats.cov_max:.2e}")

        # Discontinuity timeline (bursts)
        all_jumps = []
        for topic, stats in rep.pose_stats.items():
            for t_rel, mag, axis in stats.trans_jumps:
                all_jumps.append((t_rel, mag, "trans", topic, axis))
            for t_rel, mag, axis in stats.rot_jumps:
                all_jumps.append((t_rel, mag, "rot", topic, axis))
        if all_jumps:
            all_jumps.sort()
            p("")
            p(f"  Discontinuity timeline ({len(all_jumps)} events):")
            # Burst detection: cluster events within 2s windows
            bursts = []
            current = [all_jumps[0]]
            for j in all_jumps[1:]:
                if j[0] - current[-1][0] <= 2.0:
                    current.append(j)
                else:
                    bursts.append(current)
                    current = [j]
            bursts.append(current)
            p(f"    {len(bursts)} burst(s) detected (2s clustering window)")
            for i, burst in enumerate(bursts[:10]):
                t_start = burst[0][0]
                t_end = burst[-1][0]
                n_trans = sum(1 for b in burst if b[2] == "trans")
                n_rot = sum(1 for b in burst if b[2] == "rot")
                max_mag = max(b[1] for b in burst)
                topics = set(b[3] for b in burst)
                p(f"      burst {i+1}: t+{t_start:.1f}s..{t_end:.1f}s  "
                  f"{n_trans} trans + {n_rot} rot  max={max_mag:.3f}  "
                  f"topics: {', '.join(sorted(topics))}")

    # Odom → GTSAM agreement
    agreements = _compute_all_agreements(rep.pose_samples)
    if agreements:
        p("")
        p("  ODOM → GTSAM AGREEMENT:")
        for ag in agreements:
            p(f"    {ag.odom_topic} vs {ag.gtsam_topic} ({ag.n_pairs} pairs, {ag.duration_s:.1f}s):")
            p(f"      Trans delta: mean={ag.trans_delta_mean:.4f}m  max={ag.trans_delta_max:.4f}m")
            p(f"      Rot delta:   mean={ag.rot_delta_mean:.2f}deg  max={ag.rot_delta_max:.2f}deg")
            div_flag = ""
            if abs(ag.trans_divergence_rate) > 0.001:
                direction = "DIVERGING" if ag.trans_divergence_rate > 0 else "converging"
                div_flag = f"  [{direction}: {abs(ag.trans_divergence_rate)*1000:.2f}mm/s]"
            p(f"      Divergence:  trans={ag.trans_divergence_rate*1000:.2f}mm/s  "
              f"rot={ag.rot_divergence_rate:.3f}deg/s{div_flag}")

    # Head-arm relative distance
    for source, head_topic, arm_topic in [("odom", "/jetson/head/odom", "/jetson/arm/odom"),
                                           ("gtsam", "/gtsam/head_pose", "/gtsam/arm_pose")]:
        head_s = rep.pose_samples.get(head_topic, [])
        arm_s = rep.pose_samples.get(arm_topic, [])
        if len(head_s) < 2 or len(arm_s) < 2:
            continue
        rd = _compute_relative_distance(head_s, arm_s, source)
        if rd is None or rd.n < 2:
            continue
        p("")
        p(f"  HEAD-ARM RELATIVE DISTANCE ({source}):")
        p(f"    {rd.n} samples over {rd.duration_s:.1f}s")
        p(f"    mean={rd.mean:.3f}m  std={rd.std:.3f}m  range=[{rd.min_val:.3f},{rd.max_val:.3f}]m")
        p(f"    max rate of change={rd.max_rate_of_change:.3f}m/s")
        instability = rd.std / rd.mean if rd.mean > 1e-9 else 0
        if instability > 0.1:
            p(f"    [INSTABILITY: std/mean={instability:.2f} — relative geometry is varying]")
        else:
            p(f"    [stable: std/mean={instability:.2f}]")
    p("")

    # TF jumps
    if rep.tf_jumps:
        p(f"  TF jumps reconstructed from /tf: {len(rep.tf_jumps)} total (>0.05m)")
        edges = defaultdict(int)
        for _, parent, child, _ in rep.tf_jumps:
            edges[f"{parent} -> {child}"] += 1
        for edge, c in sorted(edges.items(), key=lambda x: -x[1])[:5]:
            p(f"    {c:>5}x  {edge}")
        p("")
    elif any(t.name == "/tf" and t.count > 0 for t in rep.topics.values()):
        p("  TF jumps: none detected (>0.05m threshold)")
        p("")


def print_sysmon(rep: BagReport, out):
    p = lambda *a, **kw: print(*a, file=out, **kw)
    if not rep.sysmon:
        return
    p("-" * 90)
    p("SYSMON OVERLAY (from sibling sysmon.jsonl)")
    p("-" * 90)
    meta = rep.sysmon.get("meta")
    if meta:
        p(f"  Interval: {meta.get('interval_s', '?')}s   "
          f"Jetson: {'ON' if meta.get('jetson_enabled') else 'OFF'}   "
          f"DDS iface: {meta.get('host_dds_iface', 'n/a')}")
    host = rep.sysmon.get("host", [])
    jetson = rep.sysmon.get("jetson", [])

    def _stats(samples, key, subkey=None):
        vals = []
        for s in samples:
            v = s.get(key)
            if isinstance(v, dict) and subkey:
                v = v.get(subkey)
            if isinstance(v, (int, float)):
                vals.append(v)
        if not vals:
            return None
        return min(vals), sum(vals) / len(vals), max(vals)

    # Host local machine — collected via psutil directly on the analysis workstation
    if host:
        p("  ── HOST (local) ──")
        cpu = _stats(host, "cpu_pct")
        if cpu:
            p(f"  CPU:     min={cpu[0]:.0f}%  avg={cpu[1]:.0f}%  max={cpu[2]:.0f}%")
        mem = _stats(host, "mem_pct")
        if mem:
            p(f"  RAM:     min={mem[0]:.0f}%  avg={mem[1]:.0f}%  max={mem[2]:.0f}%")
        gpu = _stats(host, "gpu", "util_pct")
        if gpu:
            p(f"  GPU:     min={gpu[0]:.0f}%  avg={gpu[1]:.0f}%  max={gpu[2]:.0f}%")
        gpumem = _stats(host, "gpu", "mem_used_mb")
        if gpumem:
            p(f"  VRAM:    min={gpumem[0]:.0f}MB  avg={gpumem[1]:.0f}MB  max={gpumem[2]:.0f}MB")
        drift = _stats(host, "drift", "system_offset_s")
        if drift:
            p(f"  Drift:   min={drift[0]*1000:.1f}ms  avg={drift[1]*1000:.1f}ms  "
              f"max={drift[2]*1000:.1f}ms")
        # NIC stats on DDS interface
        iface = meta.get("host_dds_iface") if meta else None
        if iface:
            nic_rx = []
            nic_drop = []
            for s in host:
                net = s.get("net", {}).get(iface)
                if net:
                    nic_rx.append(net.get("rx_mbs", 0))
                    nic_drop.append(net.get("dropin", 0) + net.get("dropout", 0))
            if nic_rx:
                p(f"  NIC [{iface}]: rx avg={sum(nic_rx)/len(nic_rx):.1f} MB/s  "
                  f"max={max(nic_rx):.1f} MB/s  drops={sum(nic_drop)}")

    # Jetson — collected by host via SSH (remote psutil)
    if jetson:
        p("")
        p("  ── JETSON (collected by host via SSH) ──")
        cpu = _stats(jetson, "cpu_pct")
        if cpu:
            p(f"  CPU:     min={cpu[0]:.0f}%  avg={cpu[1]:.0f}%  max={cpu[2]:.0f}%")
        mem = _stats(jetson, "mem_pct")
        if mem:
            p(f"  RAM:     min={mem[0]:.0f}%  avg={mem[1]:.0f}%  max={mem[2]:.0f}%")
        gpu = _stats(jetson, "gpu", "gpu_util_pct")
        if gpu:
            p(f"  GPU:     min={gpu[0]:.0f}%  avg={gpu[1]:.0f}%  max={gpu[2]:.0f}%")
        drift = _stats(jetson, "drift", "system_offset_s")
        if drift:
            p(f"  Drift:   min={drift[0]*1000:.1f}ms  avg={drift[1]*1000:.1f}ms  "
              f"max={drift[2]*1000:.1f}ms")
    elif meta and meta.get("jetson_enabled"):
        p("  JETSON (via SSH): ENABLED but no samples received (Jetson was offline?)")
    p("")

    # Correlation callouts
    _correlate(rep, out)

    # ── OS KERNEL NETWORK HEALTH (from /proc/net/snmp deltas in sysmon) ──
    _print_kernel_net_health(rep, out)


def _print_kernel_net_health(rep: BagReport, out):
    """Evaluate /proc/net/snmp delta counters from host + Jetson sysmon samples."""
    p = lambda *a, **kw: print(*a, file=out, **kw)

    def _snmp_peak(samples, key):
        """Collect snmp.{key} deltas and return (peak_rps, total)."""
        vals = []
        for s in samples:
            snmp = s.get("snmp", {})
            v = snmp.get(key)
            if isinstance(v, (int, float)):
                vals.append(v)
        if not vals:
            return None, 0
        return max(vals), sum(vals)

    host = rep.sysmon.get("host", [])
    jetson = rep.sysmon.get("jetson", [])

    reasm_peak_h, reasm_total_h = _snmp_peak(host, "ip_reasm_fails_delta")
    reasm_peak_j, reasm_total_j = _snmp_peak(jetson, "ip_reasm_fails_delta")
    rcvbuf_peak_h, rcvbuf_total_h = _snmp_peak(host, "udp_rcvbuf_errors_delta")
    rcvbuf_peak_j, rcvbuf_total_j = _snmp_peak(jetson, "udp_rcvbuf_errors_delta")

    # Merge: take the worst peak across host and jetson
    reasm_peaks = [v for v in (reasm_peak_h, reasm_peak_j) if v is not None]
    rcvbuf_peaks = [v for v in (rcvbuf_peak_h, rcvbuf_peak_j) if v is not None]
    reasm_max = max(reasm_peaks) if reasm_peaks else None
    rcvbuf_max = max(rcvbuf_peaks) if rcvbuf_peaks else None
    reasm_total = (reasm_total_h or 0) + (reasm_total_j or 0)
    rcvbuf_total = (rcvbuf_total_h or 0) + (rcvbuf_total_j or 0)
    has_snmp = reasm_max is not None or rcvbuf_max is not None

    if has_snmp:
        p("-" * 90)
        p("OS KERNEL NETWORK HEALTH (from /proc/net/snmp deltas)")
        p("-" * 90)
        if rcvbuf_max is not None:
            labels = []
            if rcvbuf_peak_h is not None:
                labels.append(f"host={rcvbuf_peak_h:.1f}/s")
            if rcvbuf_peak_j is not None:
                labels.append(f"jetson={rcvbuf_peak_j:.1f}/s")
            p(f"  UDP RcvbufErrors:  peak={rcvbuf_max:.1f}/s  total={rcvbuf_total}  ({', '.join(labels)})")
        if reasm_max is not None:
            labels = []
            if reasm_peak_h is not None:
                labels.append(f"host={reasm_peak_h:.1f}/s")
            if reasm_peak_j is not None:
                labels.append(f"jetson={reasm_peak_j:.1f}/s")
            p(f"  IP  ReasmFails:    peak={reasm_max:.1f}/s  total={reasm_total}  ({', '.join(labels)})")
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


def _correlate(rep: BagReport, out):
    """Cross-correlate sysmon + bag data to surface likely root causes."""
    p = lambda *a, **kw: print(*a, file=out, **kw)
    callouts = []

    host = rep.sysmon.get("host", [])
    jetson = rep.sysmon.get("jetson", [])
    meta = rep.sysmon.get("meta")

    # ── 0. Latency vs Clock Drift Separation ──
    # Compute clock offset from bag data (recv_time - publish_time) and
    # cross-reference with true chrony drift from sysmon.
    clock_offs = []  # (topic, max_offset_s)
    for ti in rep.topics.values():
        if len(ti.stamps) < 2 or len(ti.recv_times) < 2:
            continue
        # Skip latched (TRANSIENT_LOCAL) topics — their frozen boot-time stamps
        # produce meaningless multi-second deltas that masquerade as clock skew.
        if ti.name in LATCHED_TOPICS:
            continue
        offsets = []
        for pub_ns, recv_ns in zip(ti.stamps, ti.recv_times):
            offsets.append(abs(recv_ns - pub_ns) / 1e9)
        if offsets:
            clock_offs.append((ti.name, max(offsets), sum(offsets) / len(offsets)))

    if clock_offs:
        peak_topic, peak_off, peak_avg = max(clock_offs, key=lambda x: x[1])
        # True chrony drift from sysmon
        true_drift_host_ms = None
        true_drift_jetson_ms = None
        if host:
            hd = [abs(s.get("drift", {}).get("system_offset_s", 0)) for s in host
                  if s.get("drift")]
            if hd:
                true_drift_host_ms = max(hd) * 1000
        if jetson:
            jd = [abs(s.get("drift", {}).get("system_offset_s", 0)) for s in jetson
                  if s.get("drift")]
            if jd:
                true_drift_jetson_ms = max(jd) * 1000

        # Determine effective true drift (worst of host/jetson)
        true_drift_ms = None
        if true_drift_host_ms is not None and true_drift_jetson_ms is not None:
            true_drift_ms = max(true_drift_host_ms, true_drift_jetson_ms)
        elif true_drift_host_ms is not None:
            true_drift_ms = true_drift_host_ms
        elif true_drift_jetson_ms is not None:
            true_drift_ms = true_drift_jetson_ms

        if peak_off > 0.05 and true_drift_ms is not None:
            if true_drift_ms < 50:
                callouts.append(
                    f"Peak message clock delta = {peak_off*1000:.0f}ms on '{peak_topic}' "
                    f"BUT true chrony drift is only {true_drift_ms:.1f}ms — "
                    f"the {peak_off*1000:.0f}ms delta is Message Transport / "
                    f"Serialization Latency, NOT clock skew.")
            else:
                callouts.append(
                    f"Peak message clock delta = {peak_off*1000:.0f}ms on '{peak_topic}' "
                    f"AND chrony drift = {true_drift_ms:.1f}ms — "
                    f"clock skew is a likely contributor.")

    # ── Bandwidth cross-reference: NIC util vs useful ROS payload ──
    total_payload_mbps = 0.0
    for ti in rep.topics.values():
        if ti.bandwidth_mbs is not None:
            total_payload_mbps += ti.bandwidth_mbs
    if total_payload_mbps > 0 and host:
        iface = meta.get("host_dds_iface") if meta else None
        if iface:
            nic_rx_all = [s.get("net", {}).get(iface, {}).get("rx_mbs", 0)
                          for s in host]
            if nic_rx_all:
                nic_rx_avg = sum(nic_rx_all) / len(nic_rx_all)
                nic_rx_peak = max(nic_rx_all)
                if nic_rx_avg > total_payload_mbps * 1.25:
                    overhead_pct = (nic_rx_avg / total_payload_mbps - 1) * 100
                    callouts.append(
                        f"[WARNING] Massive DDS/RTPS Network Overhead Detected. "
                        f"Expected ROS payload: ~{total_payload_mbps:.2f} MB/s, "
                        f"NIC [{iface}] rx avg: {nic_rx_avg:.1f} MB/s "
                        f"(peak {nic_rx_peak:.1f} MB/s). "
                        f"Overhead: {overhead_pct:.0f}% — check for UDP packet "
                        f"fragmentation dropouts or Reliable QoS retransmission storms.")

    # 1. Drift vs TF jumps
    if host:
        drifts = [abs(s.get("drift", {}).get("system_offset_s", 0)) for s in host
                  if s.get("drift")]
        if drifts and max(drifts) > 0.05 and rep.tf_jumps:
            callouts.append(
                f"Clock drift peaked at {max(drifts)*1000:.0f}ms AND {len(rep.tf_jumps)} "
                f"TF jumps recorded — drift is a likely contributor to TF instability.")

    # 2. NIC drops vs starved topics
    meta = rep.sysmon.get("meta")
    iface = meta.get("host_dds_iface") if meta else None
    if iface and host:
        total_drops = sum(
            s.get("net", {}).get(iface, {}).get("dropin", 0) +
            s.get("net", {}).get(iface, {}).get("dropout", 0)
            for s in host)
        low_rate = [t for t in rep.topics.values()
                     if t.effective_hz is not None and t.effective_hz < RATE_MIN_HZ and t.count > 0]
        if total_drops > 0 and low_rate:
            names = ", ".join(t.name for t in low_rate[:3])
            callouts.append(
                f"NIC [{iface}] dropped {total_drops} packets AND topics are low-rate "
                f"({names}) — packet loss is likely degrading delivery.")

    # 3. Jetson CPU saturation vs low-rate cloud topics
    jetson = rep.sysmon.get("jetson", [])
    if jetson:
        cpu_vals = [s.get("cpu_pct") for s in jetson if isinstance(s.get("cpu_pct"), (int, float))]
        low_cloud = [t for t in rep.topics.values()
                     if t.effective_hz is not None and t.effective_hz < RATE_MIN_HZ
                     and "points" in t.name and t.count > 0]
        if cpu_vals and max(cpu_vals) > 90 and low_cloud:
            callouts.append(
                f"Jetson CPU hit {max(cpu_vals):.0f}% AND pointcloud topics are low-rate — "
                f"Jetson compute saturation is likely throttling cloud publishing.")

    if callouts:
        p("  CORRELATION CALLOUTS:")
        for c in callouts:
            p(f"    ! {c}")
        p("")


# ---------------------------------------------------------------------------
# Feature 3: Host-Side Backprojection — Time Synchronization Health
# ---------------------------------------------------------------------------

def print_backprojection_sync(rep: BagReport, out):
    """Check host-side naive_pointcloud_assembler health by comparing
    input (depth/image) topic rates vs output (points) topic rates.

    The naive_pointcloud_assembler caches depth, RGB, and CameraInfo
    independently and publishes on a timer capped at ``max_rate_hz``
    (configured in pipeline.launch.py).  There is no timestamp
    synchronization — the output rate is the rate cap, not a sync result.
    A low output rate relative to the input is therefore expected and
    not a failure; the only real failure is a near-zero output rate
    (assembler not running or no input reaching it).
    """
    p = lambda *a, **kw: print(*a, file=out, **kw)

    # Build per-side input/output mapping
    results = []
    for side in ("head", "arm"):
        # Input topics: try raw first, fall back to compressed
        depth_t = rep.topics.get(f"/jetson/{side}/depth")
        image_t = rep.topics.get(f"/jetson/{side}/image")
        if depth_t is None:
            depth_t = rep.topics.get(f"/jetson/{side}/depth/compressed")
        if image_t is None:
            image_t = rep.topics.get(f"/jetson/{side}/image/compressed")
        # Output topic
        points_t = rep.topics.get(f"/jetson/{side}/points")

        # Need at least one input and the output to compare
        if not depth_t and not image_t:
            continue
        if not points_t:
            continue

        input_hzs = []
        label_parts = []
        if depth_t is not None and depth_t.effective_hz is not None:
            input_hzs.append(depth_t.effective_hz)
            label_parts.append(f"depth={depth_t.effective_hz:.1f}Hz")
        if image_t is not None and image_t.effective_hz is not None:
            input_hzs.append(image_t.effective_hz)
            label_parts.append(f"image={image_t.effective_hz:.1f}Hz")

        if not input_hzs:
            continue

        min_input_hz = min(input_hzs)
        points_hz = points_t.effective_hz if points_t.effective_hz is not None else 0.0

        # The naive assembler rate-caps the output (default max_rate_hz=5.0).
        # A drop below the input rate is expected, not a sync failure.
        # Only flag as a real problem if the output is near-zero (< 1.0 Hz),
        # which indicates the assembler is not running or receiving no input.
        entry = {
            "side": side,
            "inputs": ", ".join(label_parts),
            "min_input_hz": min_input_hz,
            "points_hz": points_hz,
            "points_absent": points_t.count == 0,
        }
        if points_hz < 1.0 or entry["points_absent"]:
            results.append(entry)

    if not results:
        return

    p("-" * 90)
    p("HOST-SIDE POINTCLOUD ASSEMBLER — OUTPUT HEALTH")
    p("-" * 90)
    p(f"  {'SIDE':<6} {'INPUTS':<36} {'MIN_INPUT_HZ':>12} {'POINTS_HZ':>10}")
    p(f"  {'-'*6} {'-'*36} {'-'*12} {'-'*10}")
    for r in results:
        p(f"  {r['side']:<6} {r['inputs']:<36} {r['min_input_hz']:>12.1f} "
          f"{r['points_hz']:>10.1f}")
    p("")

    for r in results:
        if r["points_hz"] < 1.0 and not r["points_absent"]:
            p(f"  [WARNING] Pointcloud assembler output near-zero ({r['side']}): "
              f"{r['points_hz']:.1f}Hz output from {r['inputs']}.")
            p(f"    The naive_pointcloud_assembler rate-caps output at max_rate_hz")
            p(f"    (default 5.0 Hz), so a rate below the input is expected.")
            p(f"    A near-zero rate indicates the assembler is not running,")
            p(f"    is not receiving input, or TF lookups are failing.")
            p(f"    Check [DIAG-ASM] telemetry for skip reasons.")
            p("")
        if r["points_absent"]:
            p(f"  [WARNING] Pointcloud assembler output absent ({r['side']}):")
            p(f"    Input: {r['inputs']} -> Output: 0 messages recorded.")
            p(f"    The naive_pointcloud_assembler may not be running or is")
            p(f"    failing to receive any input frames. Check node lifecycle.")
            p("")


# ---------------------------------------------------------------------------
# Feature 5: Compression Efficiency (CompressedImage topics)
# ---------------------------------------------------------------------------

def print_compression_efficiency(rep: BagReport, out):
    """Report compression ratios for Jetson relay compressed image streams.

    Compares CompressedImage wire format sizes against the expected raw
    frame sizes for the known downsampled resolution (320×240):
      - Depth Z16:  320×240×2 = 150 KiB raw
      - Color RGB8: 320×240×3 = 225 KiB raw
    """
    p = lambda *a, **kw: print(*a, file=out, **kw)

    # Expected raw sizes for 320×240 (after 2× downsampling)
    RAW_DEPTH_KB = 150.0   # 320×240×2 / 1024
    RAW_COLOR_KB = 225.0   # 320×240×3 / 1024

    compressed_topics = []
    for side in ("head", "arm"):
        for kind, raw_kb in [("depth", RAW_DEPTH_KB), ("image", RAW_COLOR_KB)]:
            ti = rep.topics.get(f"/jetson/{side}/{kind}/compressed")
            if ti is not None and ti.count > 0 and ti.avg_size_kb is not None:
                compressed_topics.append((ti.name, ti, raw_kb))

    if not compressed_topics:
        return

    p("-" * 90)
    p("COMPRESSION EFFICIENCY (CompressedImage wire format from Jetson relay)")
    p("-" * 90)
    p(f"  {'TOPIC':<38} {'#MSG':>6} {'AVG KB':>8} {'RAW KB':>8} {'RATIO':>7} {'SAVED':>7}")
    p(f"  {'-'*38} {'-'*6} {'-'*8} {'-'*8} {'-'*7} {'-'*7}")

    for name, ti, raw_kb in sorted(compressed_topics):
        ratio = raw_kb / ti.avg_size_kb if ti.avg_size_kb > 0 else 0
        saved_pct = (1 - ti.avg_size_kb / raw_kb) * 100 if raw_kb > 0 else 0
        p(f"  {name:<38} {ti.count:>6} {ti.avg_size_kb:>7.1f}K {raw_kb:>7.1f}K "
          f"{ratio:>6.1f}x {saved_pct:>6.0f}%")

    # Aggregate bandwidth comparison
    total_raw_kbs = 0.0
    total_cmp_kbs = 0.0
    for _, ti, raw_kb in compressed_topics:
        hz = ti.effective_hz or 0
        total_raw_kbs += raw_kb * hz
        total_cmp_kbs += ti.avg_size_kb * hz

    if total_raw_kbs > 0:
        p("")
        p(f"  Aggregate at recorded rates: raw={total_raw_kbs/1024:.3f} MB/s "
          f" compressed={total_cmp_kbs/1024:.3f} MB/s "
          f"(saved {(1 - total_cmp_kbs/total_raw_kbs)*100:.0f}%)")
    p("")

    # Flag poor compression
    for name, ti, raw_kb in compressed_topics:
        if ti.avg_size_kb > raw_kb * 0.8 and raw_kb > 0:
            p(f"  [WARNING] {name}: compressed size ({ti.avg_size_kb:.1f}K) is "
              f"near raw ({raw_kb:.0f}K) — compression may be ineffective "
              f"(check encoder params)")
        elif ti.avg_size_kb < raw_kb * 0.1 and raw_kb > 0:
            p(f"  [INFO] {name}: excellent compression — "
              f"{ti.avg_size_kb:.1f}K vs {raw_kb:.0f}K raw "
              f"({raw_kb/ti.avg_size_kb:.1f}x)")
    if compressed_topics:
        p("")


def print_plot(rep: BagReport, out_png: str):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[analyze_bag] matplotlib not available — skipping plot", file=sys.stderr)
        return

    has_data = any(t.median_dt_ms is not None for t in rep.topics.values())
    has_sysmon = bool(rep.sysmon and rep.sysmon.get("host"))
    has_pose = bool(rep.pose_samples)
    if not has_data and not has_sysmon and not has_pose:
        return

    n_rows = 1  # topic rates
    if has_pose:
        n_rows += 2  # XY trajectory + rotation timeline
    if has_sysmon:
        n_rows += 2  # host CPU/GPU, drift
    fig, axes = plt.subplots(n_rows, 1, figsize=(13, 3 * n_rows), sharex=False)
    if n_rows == 1:
        axes = [axes]
    row = 0

    # Topic effective rates (log scale — rates span 0.01 to >1000 Hz)
    ax = axes[row]
    row += 1
    topics_with_data = [(name, t) for name, t in sorted(rep.topics.items()) if t.count > 0]
    if topics_with_data:
        names = [n for n, _ in topics_with_data]
        # Clip to a small floor so log scale can render zero/near-zero rates
        log_floor = 0.01
        eff = [max(t.effective_hz or 0, log_floor) for _, t in topics_with_data]
        x = range(len(names))
        ax.bar(x, eff, color="steelblue", label="effective rate (Hz)")
        ax.set_xticks(list(x))
        ax.set_xticklabels([n.replace("/jetson/", "/j/").replace("/gtsam/", "/g/") for n in names],
                           rotation=45, ha="right", fontsize=6)
        ax.set_yscale("log")
        ax.set_ylabel("Rate (Hz, log)")
        ax.set_title(f"Topic rates — {os.path.basename(rep.bag_dir)}")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3, which="both")

    # XY trajectory (top-down) — odom vs gtsam overlaid
    if has_pose:
        ax = axes[row]
        row += 1
        colors = {"/jetson/head/odom": "steelblue", "/jetson/arm/odom": "dodgerblue",
                  "/gtsam/head_pose": "darkorange", "/gtsam/arm_pose": "red",
                  "/vis/head_arm_pose": "green"}
        for topic in sorted(rep.pose_samples):
            samples = rep.pose_samples[topic]
            if len(samples) < 2:
                continue
            xs = [s.x for s in samples]
            ys = [s.y for s in samples]
            ax.plot(xs, ys, label=topic, linewidth=1, color=colors.get(topic), alpha=0.8)
            # Mark jump points
            stats = rep.pose_stats.get(topic)
            if stats and stats.trans_jumps:
                jump_ts = set(round(jt[0], 2) for jt in stats.trans_jumps)
                t0 = samples[0].t
                jx = [s.x for s in samples if round(s.t - t0, 2) in jump_ts]
                jy = [s.y for s in samples if round(s.t - t0, 2) in jump_ts]
                if jx:
                    ax.scatter(jx, jy, color=colors.get(topic, "black"), s=8, zorder=5, alpha=0.6)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("XY trajectory (top-down) — jumps marked with dots")
        ax.legend(fontsize=6)
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal", adjustable="datalim")

    # Rotation timeline (roll/pitch/yaw over time)
    if has_pose:
        ax = axes[row]
        row += 1
        for topic in sorted(rep.pose_samples):
            samples = rep.pose_samples[topic]
            if len(samples) < 2:
                continue
            t0 = samples[0].t
            ts = [s.t - t0 for s in samples]
            # Only plot yaw for clarity (most unstable axis); use linestyle per topic
            ls = "--" if "gtsam" in topic else "-"
            lw = 0.8 if "gtsam" in topic else 1.0
            ax.plot(ts, [s.yaw for s in samples], label=f"{topic} yaw",
                    linewidth=lw, linestyle=ls, alpha=0.8)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Yaw (degrees)")
        ax.set_title("Rotation timeline — yaw (solid=odom, dashed=gtsam)")
        ax.legend(fontsize=5, loc="upper right")
        ax.grid(True, alpha=0.3)

    # Sysmon: host CPU + GPU
    if has_sysmon:
        host = rep.sysmon.get("host", [])
        ax = axes[row]
        row += 1
        if host:
            ts = [s["t"] - host[0]["t"] for s in host]
            cpu = [s.get("cpu_pct", 0) for s in host]
            ax.plot(ts, cpu, label="Host CPU %", color="steelblue", linewidth=1)
            gpu_data = [(s["t"] - host[0]["t"], s.get("gpu", {}).get("util_pct"))
                        for s in host if s.get("gpu")]
            if gpu_data:
                ax.plot([t for t, _ in gpu_data], [g for _, g in gpu_data],
                        label="Host GPU %", color="green", linewidth=1)
            jetson = rep.sysmon.get("jetson", [])
            if jetson:
                jt = [s["t"] - host[0]["t"] for s in jetson]
                jcpu = [s.get("cpu_pct", 0) for s in jetson]
                ax.plot(jt, jcpu, label="Jetson CPU %", color="red", linewidth=1, alpha=0.7)
            ax.set_ylabel("Utilization (%)")
            ax.legend(fontsize=7)
            ax.grid(True, alpha=0.3)

        # Drift
        ax = axes[row]
        row += 1
        if host:
            ts = [s["t"] - host[0]["t"] for s in host]
            drift = [s.get("drift", {}).get("system_offset_s", 0) * 1000 for s in host]
            ax.plot(ts, drift, label="Host drift (ms)", color="purple", linewidth=1)
            jetson = rep.sysmon.get("jetson", [])
            if jetson:
                jt = [s["t"] - host[0]["t"] for s in jetson]
                jdrift = [s.get("drift", {}).get("system_offset_s", 0) * 1000 for s in jetson]
                ax.plot(jt, jdrift, label="Jetson drift (ms)", color="red", linewidth=1, alpha=0.7)
            ax.axhline(0, color="gray", linewidth=0.5)
            ax.set_ylabel("Clock offset (ms)")
            ax.legend(fontsize=7)
            ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    print(f"[analyze_bag] plot written to {out_png}", file=sys.stderr)


def report(rep: BagReport, out, plot_path: str | None = None):
    print_tier_a(rep, out)
    print_tier_b(rep, out)
    print_sysmon(rep, out)
    print_backprojection_sync(rep, out)
    print_compression_efficiency(rep, out)
    if plot_path:
        print_plot(rep, plot_path)


# ---------------------------------------------------------------------------
# JSON metrics extraction (for replay-test regression comparison)
# ---------------------------------------------------------------------------

def to_metrics(rep: BagReport) -> dict:
    """Extract a flat dict of scalar metrics suitable for regression comparison.

    Only metrics that are meaningful to compare between runs are included.
    Timeseries and message-level data are omitted to keep the baseline small.
    """
    import datetime
    m: dict = {
        "bag": os.path.basename(os.path.normpath(rep.bag_dir)),
        "duration_s": round(rep.duration_s, 3),
        "total_msgs": rep.total_msgs,
        "start_iso": rep.start_iso,
        "captured_at": datetime.datetime.now().isoformat(timespec="seconds"),
    }

    # Topic counts + rates (Tier A — always available)
    topics: dict = {}
    for name, ti in rep.topics.items():
        topics[name] = {
            "count": ti.count,
            "effective_hz": round(ti.effective_hz, 3) if ti.effective_hz else None,
        }
    m["topics"] = topics

    # Pose statistics (Tier B — only if deep replay ran)
    pose: dict = {}
    for topic, stats in rep.pose_stats.items():
        if stats.n < 2:
            continue
        pose[topic] = {
            "n": stats.n,
            "path_length_m": round(stats.path_length, 4),
            "max_trans_jump_m": round(stats.max_trans_jump, 4),
            "max_rot_jump_deg": round(stats.max_rot_jump, 3),
            "max_velocity_mps": round(stats.max_velocity, 3),
            "n_trans_jumps": len(stats.trans_jumps),
            "n_rot_jumps": len(stats.rot_jumps),
            "n_velocity_warns": stats.velocity_warns,
        }
    if pose:
        m["pose_stats"] = pose

    # Odom → GTSAM agreement
    agreements = _compute_all_agreements(rep.pose_samples)
    agree: dict = {}
    for ag in agreements:
        key = f"{ag.odom_topic}__vs__{ag.gtsam_topic}"
        agree[key] = {
            "n_pairs": ag.n_pairs,
            "trans_delta_mean_m": round(ag.trans_delta_mean, 5),
            "trans_delta_max_m": round(ag.trans_delta_max, 5),
            "rot_delta_mean_deg": round(ag.rot_delta_mean, 4),
            "rot_delta_max_deg": round(ag.rot_delta_max, 4),
            "trans_divergence_rate_mmps": round(ag.trans_divergence_rate * 1000, 4),
        }
    if agree:
        m["odom_gtsam_agreement"] = agree

    # Head-arm relative distance
    rel_dist: dict = {}
    for source, head_topic, arm_topic in [
            ("odom", "/jetson/head/odom", "/jetson/arm/odom"),
            ("gtsam", "/gtsam/head_pose", "/gtsam/arm_pose")]:
        head_s = rep.pose_samples.get(head_topic, [])
        arm_s = rep.pose_samples.get(arm_topic, [])
        if len(head_s) < 2 or len(arm_s) < 2:
            continue
        rd = _compute_relative_distance(head_s, arm_s, source)
        if rd is None or rd.n < 2:
            continue
        rel_dist[source] = {
            "n": rd.n,
            "mean_m": round(rd.mean, 4),
            "std_m": round(rd.std, 4),
            "min_m": round(rd.min_val, 4),
            "max_m": round(rd.max_val, 4),
        }
    if rel_dist:
        m["head_arm_relative_distance"] = rel_dist

    # TF jumps
    if rep.tf_jumps:
        m["tf_jumps_total"] = len(rep.tf_jumps)

    # Sysmon summary (host only — jetson absent in replay tests)
    host = rep.sysmon.get("host", []) if rep.sysmon else []
    if host:
        def _minmax(key, subkey=None):
            vals = []
            for s in host:
                v = s.get(key)
                if isinstance(v, dict) and subkey:
                    v = v.get(subkey)
                if isinstance(v, (int, float)):
                    vals.append(v)
            if not vals:
                return None
            return {"min": round(min(vals), 1), "avg": round(sum(vals) / len(vals), 1),
                    "max": round(max(vals), 1)}
        sysmon_summary = {
            "host_cpu_pct": _minmax("cpu_pct"),
            "host_mem_pct": _minmax("mem_pct"),
            "host_gpu_util_pct": _minmax("gpu", "util_pct"),
            "host_gpu_mem_mb": _minmax("gpu", "mem_used_mb"),
        }
        sysmon_summary = {k: v for k, v in sysmon_summary.items() if v is not None}
        if sysmon_summary:
            m["sysmon"] = sysmon_summary

    return m


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _latest_bag():
    """Return the most-recently-modified bag directory, or None."""
    bags = glob.glob(os.path.join(BAGS_DIR, "v6_*"))
    dirs = [b for b in bags if os.path.isdir(b)]
    if not dirs:
        return None
    return max(dirs, key=os.path.getmtime)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", nargs="?", help="Path to bag dir (default: latest in data/bags/)")
    ap.add_argument("--no-deep", action="store_true",
                    help="Tier A only (metadata.yaml, no MCAP read)")
    ap.add_argument("--plot", action="store_true", help="Write PNG timelines (needs matplotlib)")
    ap.add_argument("--no-sysmon", action="store_true",
                    help="Skip sysmon.jsonl overlay even if present")
    ap.add_argument("-o", "--output", help="Write report to file (default: <bag>/<bag>_analysis.txt)")
    ap.add_argument("--json", metavar="PATH", help="Write scalar metrics to JSON (for replay-test comparison)")
    args = ap.parse_args()

    bag = args.bag or _latest_bag()
    if not bag:
        print("ERROR: no bag found in data/bags/. Pass a path or record a bag first.",
              file=sys.stderr)
        sys.exit(1)
    if not os.path.isdir(bag):
        print(f"ERROR: {bag} is not a directory", file=sys.stderr)
        sys.exit(1)

    # Default: save report alongside the bag
    output_path = args.output
    if output_path is None:
        bag_name = os.path.basename(os.path.normpath(bag))
        output_path = os.path.join(bag, bag_name + "_analysis.txt")
    out = open(output_path, "w")

    rep = _parse_metadata(bag)
    if not args.no_deep:
        _deep_replay(rep)
    if not args.no_sysmon:
        sm = _load_sysmon(bag)
        if sm:
            rep.sysmon = sm
        else:
            print(f"[analyze_bag] no sibling sysmon.jsonl found in {bag}",
                  file=sys.stderr)

    plot_path = None
    if args.plot:
        bag_name = os.path.basename(os.path.normpath(bag))
        plot_path = os.path.join(bag, bag_name + "_analysis.png")
    report(rep, out, plot_path)

    out.close()
    print(f"[analyze_bag] report written to {output_path}", file=sys.stderr)

    if args.json:
        metrics = to_metrics(rep)
        with open(args.json, "w") as jf:
            json.dump(metrics, jf, indent=2, sort_keys=True)
        print(f"[analyze_bag] metrics JSON written to {args.json}", file=sys.stderr)

    # Also echo to stdout for convenience
    with open(output_path) as f:
        print(f.read())


if __name__ == "__main__":
    main()
