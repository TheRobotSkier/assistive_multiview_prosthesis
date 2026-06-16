#!/usr/bin/env python3
"""sysmon.py — capture host + Jetson system telemetry to a JSONL stream.

Runs on the HOST. Samples host metrics locally (psutil, pynvml, chronyc) and
opens ONE persistent SSH to the Jetson to stream its metrics (/proc, chronyc,
tegrastats). Everything is written to a single JSONL file (one JSON object per
line), tagged with ``"source"`` so the bag analyzer can overlay system state on
the topic timeline.

JSONL schema (one object per line):
  {"t": <epoch float>, "source": "host", "cpu_pct": float, "cpu_per_core": [...],
   "load_avg": [...], "mem_total_gb": float, "mem_used_gb": float, "mem_pct": float,
   "swap_pct": float, "gpu": {"util_pct": int, "mem_used_mb": int, "mem_total_mb": int,
   "temp_c": int}, "net": {iface: {"rx_mbs": float, "tx_mbs": float, "dropin": int, ...}}}
  {"t": <epoch float>, "source": "jetson", ...same shape, gpu best-effort...}
  {"t": <epoch float>, "source": "drift", "host": {...}, "jetson": {...}}

Usage:
  python3 scripts/sysmon.py --output data/bags/v6_<ts>/sysmon.jsonl
  python3 scripts/sysmon.py --output foo.jsonl --no-jetson   # host-only
  python3 scripts/sysmon.py --jetson-collector               # INTERNAL: run on jetson

Ctrl+C stops both sides cleanly and closes the SSH session.
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_INTERVAL = 1.0
JETSON_DEFAULT_HOST = os.environ.get("JETSON_HOST", "robotlab")
# Sudo password used elsewhere in this repo's Makefile (make timesync).
JETSON_SUDO_PW = os.environ.get("JETSON_SUDO_PW", "robotlab")
# Host DDS IP from config/cyclonedds_peer.xml — used to detect the active NIC.
HOST_DDS_IP = "10.42.0.1"
JETSON_DDS_IP = "10.42.0.2"

# ---------------------------------------------------------------------------
# Jetson-side collector (runs remotely over SSH via base64 heredoc)
# ---------------------------------------------------------------------------

JETSON_COLLECTOR = r'''
import json, os, re, subprocess, sys, time

INTERVAL = float(os.environ.get("SYSMON_INTERVAL", "1"))

def read_proc_stat():
    """Return (aggregate_jiff, per_core_jiffs) from /proc/stat."""
    with open("/proc/stat") as f:
        lines = f.read().splitlines()
    agg = None
    cores = []
    for ln in lines:
        parts = ln.split()
        if parts[0] == "cpu":
            agg = sum(int(x) for x in parts[1:])
        elif parts[0].startswith("cpu"):
            cores.append(sum(int(x) for x in parts[1:]))
    return agg, cores

def read_meminfo():
    d = {}
    with open("/proc/meminfo") as f:
        for ln in f:
            k, _, v = ln.partition(":")
            v = v.strip().split()[0]
            d[k] = int(v)
    return d

def read_netdev():
    """Return {iface: {rx_bytes, tx_bytes, dropin, dropout, errin, errout}}."""
    out = {}
    with open("/proc/net/dev") as f:
        for ln in f:
            if ":" not in ln:
                continue
            name, rest = ln.split(":", 1)
            name = name.strip()
            if name == "lo":
                continue
            f1 = rest.split()
            # recv: bytes packets errs drop ...  trans: bytes packets errs drop
            out[name] = {
                "rx_bytes": int(f1[0]), "tx_bytes": int(f1[8]),
                "errin": int(f1[2]), "dropin": int(f1[3]),
                "errout": int(f1[9]), "dropout": int(f1[10]),
            }
    return out

def read_chronyc():
    try:
        r = subprocess.run(["chronyc", "-c", "tracking"],
                           capture_output=True, text=True, timeout=2)
        if r.returncode != 0:
            return None
        cols = r.stdout.strip().split(",")
        # cols[4]=system offset, [5]=last offset, [6]=rms offset, [7]=freq ppm
        return {
            "system_offset_s": float(cols[4]),
            "last_offset_s": float(cols[5]),
            "rms_offset_s": float(cols[6]),
            "freq_ppm": float(cols[7]),
        }
    except Exception:
        return None

def start_tegrastats():
    """Best-effort: spawn sudo tegrastats, return Popen or None."""
    try:
        p = subprocess.Popen(
            ["sudo", "-S", "tegrastats", "--interval", str(int(INTERVAL * 1000))],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1)
        try:
            p.stdin.write(os.environ.get("JETSON_SUDO_PW", "robotlab") + "\n")
            p.stdin.flush()
        except Exception:
            pass
        return p
    except Exception:
        return None

def parse_tegrastats(line):
    """Extract GPU util %, EMC %, and thermal from a tegrastats line."""
    out = {}
    m = re.search(r"GR3D_FREQ\s+(\d+)%", line)
    if m:
        out["gpu_util_pct"] = int(m.group(1))
    m = re.search(r"EMC_FREQ\s+(\d+)%", line)
    if m:
        out["emc_pct"] = int(m.group(1))
    # thermal: find "GPU@45C" or similar
    m = re.search(r"GPU@(\d+)C", line)
    if m:
        out["gpu_temp_c"] = int(m.group(1))
    m = re.search(r"AUX@(\d+)C", line) or re.search(r"Tboard@(\d+)C", line)
    if m:
        out["board_temp_c"] = int(m.group(1))
    return out

def main():
    prev_agg, prev_cores = read_proc_stat()
    prev_net = read_netdev()
    teg = start_tegrastats()
    while True:
        time.sleep(INTERVAL)
        t = time.time()
        agg, cores = read_proc_stat()
        mem = read_meminfo()
        net = read_netdev()

        rec = {"t": t, "source": "jetson"}
        # CPU percent
        if prev_agg is not None:
            rec["cpu_pct"] = round(100.0 * (1.0 - ( (agg - prev_agg) / max(1, (INTERVAL * os.cpu_count() * 100.0)) )), 1) if False else None
        # Simpler: per-core utilization from jiffies
        if prev_cores and len(cores) == len(prev_cores):
            per = []
            for c, p in zip(cores, prev_cores):
                dt = c - p
                # each jiffy ~10ms; over INTERVAL seconds, max jiffies = INTERVAL*100
                pct = round(100.0 * dt / max(1.0, INTERVAL * 100.0), 1)
                per.append(min(100.0, max(0.0, pct)))
            rec["cpu_per_core"] = per
            rec["cpu_pct"] = round(sum(per) / max(1, len(per)), 1) if per else 0.0
        # Memory
        total_kb = mem.get("MemTotal", 0)
        avail_kb = mem.get("MemAvailable", mem.get("MemFree", 0))
        used_kb = total_kb - avail_kb
        rec["mem_total_mb"] = total_kb // 1024
        rec["mem_used_mb"] = used_kb // 1024
        rec["mem_pct"] = round(100.0 * used_kb / max(1, total_kb), 1)
        # Network deltas
        netrec = {}
        for iface, cur in net.items():
            prev = prev_net.get(iface)
            if prev:
                netrec[iface] = {
                    "rx_mbs": round((cur["rx_bytes"] - prev["rx_bytes"]) / INTERVAL / 1e6, 2),
                    "tx_mbs": round((cur["tx_bytes"] - prev["tx_bytes"]) / INTERVAL / 1e6, 2),
                    "dropin": cur["dropin"] - prev["dropin"],
                    "dropout": cur["dropout"] - prev["dropout"],
                    "errin": cur["errin"] - prev["errin"],
                    "errout": cur["errout"] - prev["errout"],
                }
        rec["net"] = netrec
        # Drift
        d = read_chronyc()
        if d:
            rec["drift"] = d
        # Tegrastats GPU/thermal (non-blocking read of latest line)
        if teg and teg.poll() is None:
            line = None
            try:
                # drain available output
                while True:
                    import select
                    r, _, _ = select.select([teg.stdout], [], [], 0)
                    if r:
                        ln = teg.stdout.readline()
                        if ln:
                            line = ln
                        else:
                            break
                    else:
                        break
            except Exception:
                pass
            if line:
                rec["gpu"] = parse_tegrastats(line)

        print(json.dumps(rec, separators=(",", ":")), flush=True)
        sys.stdout.flush()
        prev_agg, prev_cores = agg, cores
        prev_net = net

if __name__ == "__main__":
    main()
'''


# ---------------------------------------------------------------------------
# Host-side samplers
# ---------------------------------------------------------------------------

def _init_pynvml():
    try:
        import pynvml
        pynvml.nvmlInit()
        return pynvml, pynvml.nvmlDeviceGetHandleByIndex(0)
    except Exception:
        return None, None


def _gpu_sample(pynvml, handle):
    if not pynvml or not handle:
        return None
    try:
        util = pynvml.nvmlDeviceGetUtilizationRates(handle)
        mem = pynvml.nvmlDeviceGetMemoryInfo(handle)
        temp = pynvml.nvmlDeviceGetTemperature(handle, pynvml.NVML_TEMPERATURE_GPU)
        return {
            "util_pct": int(util.gpu),
            "mem_used_mb": int(mem.used // 1024 // 1024),
            "mem_total_mb": int(mem.total // 1024 // 1024),
            "temp_c": int(temp),
        }
    except Exception:
        return None


def _detect_dds_iface():
    """Find the NIC carrying the DDS traffic (the one with HOST_DDS_IP)."""
    try:
        import psutil
        stats = psutil.net_if_addrs()
        for name, addrs in stats.items():
            for a in addrs:
                if a.address == HOST_DDS_IP:
                    return name
    except Exception:
        pass
    return None


def _chronyc_tracking():
    try:
        r = subprocess.run(["chronyc", "-c", "tracking"],
                           capture_output=True, text=True, timeout=2)
        if r.returncode != 0:
            return None
        cols = r.stdout.strip().split(",")
        if len(cols) < 8:
            return None
        return {
            "system_offset_s": float(cols[4]),
            "last_offset_s": float(cols[5]),
            "rms_offset_s": float(cols[6]),
            "freq_ppm": float(cols[7]),
        }
    except Exception:
        return None


class HostSampler:
    def __init__(self, interval):
        import psutil
        self.psutil = psutil
        self.interval = interval
        self.pynvml, self.handle = _init_pynvml()
        self.prev_net = None
        self._net_all = True  # capture all non-lo ifaces

    def sample(self):
        ps = self.psutil
        t = time.time()
        rec = {"t": t, "source": "host"}
        rec["cpu_pct"] = ps.cpu_percent(interval=None)
        rec["cpu_per_core"] = ps.cpu_percent(interval=None, percpu=True)
        try:
            rec["load_avg"] = list(ps.getloadavg())
        except Exception:
            pass
        vm = ps.virtual_memory()
        rec["mem_total_gb"] = round(vm.total / 1e9, 2)
        rec["mem_used_gb"] = round(vm.used / 1e9, 2)
        rec["mem_pct"] = vm.percent
        sm = ps.swap_memory()
        rec["swap_pct"] = sm.percent
        # GPU
        g = _gpu_sample(self.pynvml, self.handle)
        if g:
            rec["gpu"] = g
        # Network
        net = ps.net_io_counters(pernic=True)
        netrec = {}
        for name, c in net.items():
            if name == "lo":
                continue
            p = self.prev_net.get(name) if self.prev_net else None
            if p:
                netrec[name] = {
                    "rx_mbs": round((c.bytes_recv - p.bytes_recv) / self.interval / 1e6, 2),
                    "tx_mbs": round((c.bytes_sent - p.bytes_sent) / self.interval / 1e6, 2),
                    "dropin": c.dropin - p.dropin,
                    "dropout": c.dropout - p.dropout,
                    "errin": c.errin - p.errin,
                    "errout": c.errout - p.errout,
                }
        rec["net"] = netrec
        self.prev_net = net
        # Drift
        d = _chronyc_tracking()
        if d:
            rec["drift"] = d
        return rec


# ---------------------------------------------------------------------------
# Jetson SSH streamer
# ---------------------------------------------------------------------------

class JetsonStreamer(threading.Thread):
    """Open one SSH to the Jetson running the collector; relay JSONL lines."""

    def __init__(self, jetson_host, interval, on_line):
        super().__init__(daemon=True)
        self.jetson_host = jetson_host
        self.interval = interval
        self.on_line = on_line
        self.proc = None
        self._stop_evt = threading.Event()

    def run(self):
        # Pre-flight probe: fail fast (within ~6s) if the Jetson is unreachable,
        # instead of letting SSH block indefinitely on a dead host. We use
        # BatchMode=yes so SSH never prompts interactively (which would hang a
        # non-TTY process). Key-based auth is assumed; if it fails, we disable
        # Jetson telemetry and continue host-only.
        probe = [
            "ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes",
            "-o", "StrictHostKeyChecking=accept-new",
            self.jetson_host, "echo SYSMON_OK",
        ]
        try:
            r = subprocess.run(probe, capture_output=True, text=True, timeout=8)
            if r.returncode != 0 or "SYSMON_OK" not in r.stdout:
                sys.stderr.write(
                    f"[sysmon] Jetson probe failed (rc={r.returncode}); "
                    f"continuing host-only. stderr: {r.stderr.strip()[:160]}\n")
                return
        except subprocess.TimeoutExpired:
            sys.stderr.write(
                "[sysmon] Jetson probe timed out (>8s); continuing host-only\n")
            return
        except FileNotFoundError:
            sys.stderr.write("[sysmon] ssh not found; Jetson telemetry disabled\n")
            return
        except Exception as e:
            sys.stderr.write(f"[sysmon] Jetson probe error: {e}; continuing host-only\n")
            return

        b64 = base64.b64encode(JETSON_COLLECTOR.encode()).decode()
        env = {
            "SYSMON_INTERVAL": str(self.interval),
            "JETSON_SUDO_PW": JETSON_SUDO_PW,
        }
        env_str = " ".join(f"{k}={v}" for k, v in env.items())
        cmd = [
            "ssh", "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=3",
            "-o", "ConnectTimeout=5", "-o", "BatchMode=yes",
            "-o", "StrictHostKeyChecking=accept-new",
            self.jetson_host,
            f"{env_str} sh -c 'echo {b64} | base64 -d | python3 -'",
        ]
        try:
            self.proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, bufsize=1)
        except FileNotFoundError:
            sys.stderr.write("[sysmon] ssh not found; Jetson telemetry disabled\n")
            return
        except Exception as e:
            sys.stderr.write(f"[sysmon] SSH to Jetson failed: {e}\n")
            return
        # Read stdout line by line until stopped or SSH dies.
        for line in self.proc.stdout:
            if self._stop_evt.is_set():
                break
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
                self.on_line(obj)
            except json.JSONDecodeError:
                sys.stderr.write(f"[sysmon] jetson parse error: {line[:120]}\n")
        # SSH exited — surface stderr for diagnosis.
        if not self._stop_evt.is_set():
            err = self.proc.stderr.read() if self.proc.stderr else ""
            sys.stderr.write(
                f"[sysmon] Jetson SSH stream ended"
                f"{f': {err.strip()[:200]}' if err and err.strip() else ' (jetson offline?)'}\n")

    def stop(self):
        self._stop_evt.set()
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except Exception:
                self.proc.kill()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--output", "-o", required=False,
                    help="Output JSONL path (default: stdout)")
    ap.add_argument("--interval", "-i", type=float, default=DEFAULT_INTERVAL,
                    help="Sample interval seconds (default: 1.0)")
    ap.add_argument("--jetson-host", default=JETSON_DEFAULT_HOST,
                    help=f"Jetson SSH host (default: {JETSON_DEFAULT_HOST})")
    ap.add_argument("--no-jetson", action="store_true",
                    help="Host-only mode (skip Jetson)")
    ap.add_argument("--jetson-collector", action="store_true",
                    help=argparse.SUPPRESS)  # internal remote mode
    args = ap.parse_args()

    if args.jetson_collector:
        # Run the embedded collector locally (used when re-invoked on the jetson).
        JETSON_NS = {}
        exec(compile(JETSON_COLLECTOR, "<jetson_collector>", "exec"), JETSON_NS)
        return

    out = open(args.output, "a") if args.output else sys.stdout
    stop_evt = threading.Event()

    def write_line(obj):
        out.write(json.dumps(obj, separators=(",", ":")) + "\n")
        out.flush()

    def handle_sig(signum, frame):
        stop_evt.set()

    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)

    header = {
        "t": time.time(), "source": "meta",
        "interval_s": args.interval,
        "jetson_enabled": not args.no_jetson,
        "jetson_host": args.jetson_host if not args.no_jetson else None,
        "host_dds_iface": _detect_dds_iface(),
    }
    write_line(header)

    # Start Jetson streamer (runs in its own thread; failure is non-fatal)
    jetson = None
    if not args.no_jetson:
        jetson = JetsonStreamer(args.jetson_host, args.interval, write_line)
        jetson.start()
        time.sleep(0.1)

    # Host sampling loop — always runs regardless of Jetson status
    host = HostSampler(args.interval)
    # Prime psutil cpu_percent (first call returns 0.0)
    host.psutil.cpu_percent(interval=None)
    host.psutil.cpu_percent(interval=None, percpu=True)
    time.sleep(min(args.interval, 0.2))

    jetson_status = ""
    if args.no_jetson:
        jetson_status = " (host-only)"
    elif jetson and jetson.is_alive():
        jetson_status = " + Jetson (probing...)"
    else:
        jetson_status = " (host-only; Jetson probe failed)"
    sys.stderr.write(
        f"[sysmon] capturing host metrics every {args.interval}s{jetson_status}"
        f"{f' -> {args.output}' if args.output else ' -> stdout'}\n")
    sys.stderr.write("[sysmon] press Ctrl+C to stop\n")

    while not stop_evt.is_set():
        try:
            rec = host.sample()
            write_line(rec)
        except Exception as e:
            sys.stderr.write(f"[sysmon] host sample error: {e}\n")
        stop_evt.wait(args.interval)

    # Cleanup
    if jetson:
        jetson.stop()
        jetson.join(timeout=4)
    if out is not sys.stdout:
        out.close()
    sys.stderr.write("[sysmon] stopped\n")


if __name__ == "__main__":
    main()
