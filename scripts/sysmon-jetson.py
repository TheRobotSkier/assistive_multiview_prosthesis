#!/usr/bin/env python3
"""sysmon-jetson.py — lightweight Jetson system telemetry to JSONL.

Runs on the Jetson alongside the pipeline. Polls /proc for CPU/mem/load/net
every INTERVAL seconds and optionally reads tegrastats for GPU temperature.

Usage:
  python3 scripts/sysmon-jetson.py --output /path/to/output.jsonl
  python3 scripts/sysmon-jetson.py --output foo.jsonl --interval 5.0

Signal handling:
  SIGTERM / SIGINT → flush remaining data and exit cleanly.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time


INTERVAL = 2.0  # seconds between samples

# ---------------------------------------------------------------------------
# /proc readers
# ---------------------------------------------------------------------------

_proc_stat_prev = None  # (total_jiffies, idle_jiffies)


def read_cpu() -> dict:
    """Return aggregate CPU percent since last call.

    Uses the idle jiffies column from /proc/stat to compute a
    true utilization percentage: 100 * (1 - Δidle/Δtotal).
    """
    global _proc_stat_prev
    with open("/proc/stat") as f:
        line = f.readline()  # first line = "cpu ..."
    parts = line.split()
    if len(parts) < 5:
        return {"cpu_pct": None}
    # /proc/stat: cpu user nice system idle iowait irq softirq steal ...
    vals = [int(x) for x in parts[1:]]
    total = sum(vals)
    idle = vals[3]  # idle jiffies

    result = {"cpu_pct": None}
    if _proc_stat_prev is not None:
        prev_total, prev_idle = _proc_stat_prev
        d_total = total - prev_total
        d_idle = idle - prev_idle
        if d_total > 0:
            result["cpu_pct"] = round(100.0 * (1.0 - d_idle / d_total), 1)

    _proc_stat_prev = (total, idle)
    return result


def read_memory() -> dict:
    with open("/proc/meminfo") as f:
        data = f.read()
    mem = {}
    for line in data.splitlines():
        key, _, val = line.partition(":")
        val = val.strip().split()[0]  # take first token (kB)
        try:
            mem[key] = int(val)
        except ValueError:
            pass

    total_kb = mem.get("MemTotal", 0)
    free_kb = mem.get("MemFree", 0)
    buffers_kb = mem.get("Buffers", 0)
    cached_kb = mem.get("Cached", 0)
    used_kb = total_kb - free_kb - buffers_kb - cached_kb
    return {
        "mem_total_mb": round(total_kb / 1024, 1),
        "mem_used_mb": round(used_kb / 1024, 1),
        "mem_pct": round(100.0 * used_kb / total_kb, 1) if total_kb else 0,
        "swap_total_mb": round(mem.get("SwapTotal", 0) / 1024, 1),
        "swap_used_mb": round((mem.get("SwapTotal", 0) - mem.get("SwapFree", 0)) / 1024, 1),
    }


def read_load() -> dict:
    with open("/proc/loadavg") as f:
        parts = f.read().strip().split()
    return {
        "load_1m": float(parts[0]) if parts else 0,
        "load_5m": float(parts[1]) if len(parts) > 1 else 0,
        "load_15m": float(parts[2]) if len(parts) > 2 else 0,
    }


_net_prev = None  # dict iface -> (rx_bytes, tx_bytes)


def read_network() -> dict:
    global _net_prev
    with open("/proc/net/dev") as f:
        lines = f.read().splitlines()

    # Skip header lines
    data_lines = [ln for ln in lines if ":" in ln]
    current = {}
    for ln in data_lines:
        iface, rest = ln.split(":")
        iface = iface.strip()
        parts = rest.strip().split()
        # bytes(0) packets(1) errs(2) drop(3) ...
        current[iface] = (int(parts[0]), int(parts[8]))  # rx_bytes, tx_bytes

    result = {}
    if _net_prev is not None:
        for iface, (rx, tx) in current.items():
            if iface in _net_prev:
                prx, ptx = _net_prev[iface]
                drx = max(0, rx - prx)
                dtx = max(0, tx - ptx)
                rx_mbps = round(drx * 8 / (INTERVAL * 1_000_000), 3)
                tx_mbps = round(dtx * 8 / (INTERVAL * 1_000_000), 3)
                if rx_mbps > 0 or tx_mbps > 0:
                    result[iface] = {"rx_mbps": rx_mbps, "tx_mbps": tx_mbps}
    _net_prev = current
    return result


def read_gpu_tegrastats() -> dict:
    """One-shot tegrastats query. Returns {} on failure."""
    try:
        out = subprocess.check_output(
            ["tegrastats", "--interval", "100", "--count", "1"],
            timeout=5, stderr=subprocess.DEVNULL,
        ).decode("utf-8", errors="replace")
    except (FileNotFoundError, subprocess.TimeoutExpired, subprocess.CalledProcessError):
        return {}

    result = {}
    # Parse: "RAM 3944/7851MB ... GPU@43C ... GR3D_FREQ 0%"
    import re
    m = re.search(r"GPU@(-?[\d.]+)C", out)
    if m:
        result["gpu_temp_c"] = float(m.group(1))
    m = re.search(r"GR3D_FREQ\s*(\d+)%", out)
    if m:
        result["gpu_util_pct"] = int(m.group(1))
    return result


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Jetson system telemetry collector")
    ap.add_argument("--output", "-o", required=True, help="Path to output JSONL file")
    ap.add_argument("--interval", type=float, default=INTERVAL, help=f"Poll interval in seconds (default: {INTERVAL})")
    args = ap.parse_args()
    interval = max(0.5, args.interval)

    # Ensure output directory exists
    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    # Signal handling
    stop = threading.Event()

    def _signal(signum, frame):
        stop.set()

    signal.signal(signal.SIGTERM, _signal)
    signal.signal(signal.SIGINT, _signal)

    with open(args.output, "w") as f:
        # First sample: prime /proc readers (no delta yet)
        read_cpu()
        read_network()

        while not stop.is_set():
            t = time.time()

            sample = {"t": round(t, 3), "source": "jetson"}
            sample.update(read_cpu())
            sample.update(read_memory())
            sample.update(read_load())
            net = read_network()
            if net:
                sample["net"] = net
            gpu = read_gpu_tegrastats()
            if gpu:
                sample.update(gpu)

            f.write(json.dumps(sample, sort_keys=True) + "\n")
            f.flush()

            # Wait for remaining interval time (check stop every 0.1s)
            deadline = t + interval
            while time.time() < deadline:
                if stop.is_set():
                    break
                time.sleep(0.1)

    # One last sample before exit
    with open(args.output, "a") as f:
        sample = {"t": round(time.time(), 3), "source": "jetson", "event": "shutdown"}
        f.write(json.dumps(sample, sort_keys=True) + "\n")


if __name__ == "__main__":
    import threading
    main()
