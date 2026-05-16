#!/usr/bin/env python3
"""
analyze_imu_rate_test.py

Analyze CSV output from imu_i2c_rate_test_logger.py and summarize whether a
chosen rate looks stable enough for IMU use.
"""

import argparse
import math
from pathlib import Path

import numpy as np
import pandas as pd


def percentile_or_nan(series: pd.Series, pct: float) -> float:
    arr = series.dropna().to_numpy(dtype=np.float64)
    if arr.size == 0:
        return float("nan")
    return float(np.percentile(arr, pct))


def safe_float(value, default=float("nan")):
    try:
        return float(value)
    except Exception:
        return default


def verdict_text(metrics: dict) -> str:
    error_rate = metrics["read_error_fraction"]
    deadline_miss_fraction = metrics["deadline_miss_fraction"]
    late_half_fraction = metrics["late_half_fraction"]
    dt_std_ms = metrics["dt_std_ms"]
    dt_p99_ms = metrics["dt_p99_ms"]
    dt_max_ms = metrics["dt_max_ms"]
    requested_period_ms = metrics["requested_period_ms"]
    read_p99_ms = metrics["read_duration_p99_ms"]

    serious = []
    warning = []

    if error_rate > 0.0:
        serious.append("nonzero I2C read errors")
    if deadline_miss_fraction > 0.01:
        serious.append("more than 1% deadline misses")
    if late_half_fraction > 0.001:
        serious.append("frequent wakeups later than half a period")
    if requested_period_ms == requested_period_ms and dt_p99_ms > 1.5 * requested_period_ms:
        serious.append("dt p99 above 1.5x requested period")
    if requested_period_ms == requested_period_ms and dt_max_ms > 2.5 * requested_period_ms:
        serious.append("large worst-case dt outlier")

    if dt_std_ms > 0.20 * requested_period_ms:
        warning.append("dt jitter standard deviation is high")
    if read_p99_ms > 0.50 * requested_period_ms:
        warning.append("I2C transaction time consumes more than half the period at p99")

    if serious:
        return "UNSTABLE: " + "; ".join(serious)
    if warning:
        return "MARGINAL: " + "; ".join(warning)
    return "GOOD: no read errors, and timing jitter looks comfortably within the requested period"


def analyze_csv(csv_path: Path) -> dict:
    df = pd.read_csv(csv_path)

    numeric_cols = [
        "dt_sec",
        "requested_period_sec",
        "configured_sensor_period_sec",
        "schedule_lag_sec",
        "read_duration_sec",
        "deadline_missed",
        "late_by_more_than_half_period",
        "read_ok",
        "accel_x_mps2", "accel_y_mps2", "accel_z_mps2",
        "gyro_x_rps", "gyro_y_rps", "gyro_z_rps",
        "raw_accel_x", "raw_accel_y", "raw_accel_z",
        "raw_gyro_x", "raw_gyro_y", "raw_gyro_z",
    ]
    for col in numeric_cols:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    valid = df[df["read_ok"] == 1].copy() if "read_ok" in df.columns else df.copy()
    valid_dt = valid[valid["dt_sec"] > 0.0].copy()

    requested_period_ms = safe_float(valid["requested_period_sec"].dropna().iloc[0] * 1000.0) if len(valid) else float("nan")
    requested_rate_hz = 1000.0 / requested_period_ms if requested_period_ms == requested_period_ms and requested_period_ms > 0.0 else float("nan")
    configured_period_ms = safe_float(valid["configured_sensor_period_sec"].dropna().iloc[0] * 1000.0) if len(valid) else float("nan")
    configured_rate_hz = 1000.0 / configured_period_ms if configured_period_ms == configured_period_ms and configured_period_ms > 0.0 else float("nan")

    accel_norm = np.sqrt(valid["accel_x_mps2"]**2 + valid["accel_y_mps2"]**2 + valid["accel_z_mps2"]**2) if len(valid) else pd.Series(dtype=np.float64)
    gyro_norm = np.sqrt(valid["gyro_x_rps"]**2 + valid["gyro_y_rps"]**2 + valid["gyro_z_rps"]**2) if len(valid) else pd.Series(dtype=np.float64)

    metrics = {
        "file": str(csv_path),
        "scheduled_rows": int(len(df)),
        "successful_reads": int((df["read_ok"] == 1).sum()) if "read_ok" in df.columns else int(len(df)),
        "read_errors": int((df["read_ok"] == 0).sum()) if "read_ok" in df.columns else 0,
        "read_error_fraction": float((df["read_ok"] == 0).mean()) if "read_ok" in df.columns else 0.0,
        "requested_period_ms": requested_period_ms,
        "requested_rate_hz": requested_rate_hz,
        "configured_period_ms": configured_period_ms,
        "configured_rate_hz": configured_rate_hz,
        "dt_mean_ms": float(valid_dt["dt_sec"].mean() * 1000.0) if len(valid_dt) else float("nan"),
        "dt_std_ms": float(valid_dt["dt_sec"].std(ddof=0) * 1000.0) if len(valid_dt) else float("nan"),
        "dt_p99_ms": percentile_or_nan(valid_dt["dt_sec"], 99.0) * 1000.0,
        "dt_p999_ms": percentile_or_nan(valid_dt["dt_sec"], 99.9) * 1000.0,
        "dt_max_ms": float(valid_dt["dt_sec"].max() * 1000.0) if len(valid_dt) else float("nan"),
        "read_duration_mean_ms": float(valid["read_duration_sec"].mean() * 1000.0) if len(valid) else float("nan"),
        "read_duration_p99_ms": percentile_or_nan(valid["read_duration_sec"], 99.0) * 1000.0,
        "read_duration_p999_ms": percentile_or_nan(valid["read_duration_sec"], 99.9) * 1000.0,
        "read_duration_max_ms": float(valid["read_duration_sec"].max() * 1000.0) if len(valid) else float("nan"),
        "schedule_lag_p99_ms": percentile_or_nan(df["schedule_lag_sec"], 99.0) * 1000.0,
        "schedule_lag_p999_ms": percentile_or_nan(df["schedule_lag_sec"], 99.9) * 1000.0,
        "deadline_misses": int(df["deadline_missed"].sum()) if "deadline_missed" in df.columns else 0,
        "deadline_miss_fraction": float(df["deadline_missed"].mean()) if "deadline_missed" in df.columns else 0.0,
        "late_half_period_count": int(df["late_by_more_than_half_period"].sum()) if "late_by_more_than_half_period" in df.columns else 0,
        "late_half_fraction": float(df["late_by_more_than_half_period"].mean()) if "late_by_more_than_half_period" in df.columns else 0.0,
        "accel_norm_mean_mps2": float(accel_norm.mean()) if len(accel_norm) else float("nan"),
        "accel_norm_std_mps2": float(accel_norm.std(ddof=0)) if len(accel_norm) else float("nan"),
        "gyro_norm_mean_rps": float(gyro_norm.mean()) if len(gyro_norm) else float("nan"),
        "gyro_norm_std_rps": float(gyro_norm.std(ddof=0)) if len(gyro_norm) else float("nan"),
    }

    metrics["verdict"] = verdict_text(metrics)
    return metrics


def print_metrics(metrics: dict) -> None:
    print(f"File:                         {metrics['file']}")
    print(f"Requested rate:               {metrics['requested_rate_hz']:.3f} Hz")
    print(f"Configured sensor rate:       {metrics['configured_rate_hz']:.3f} Hz")
    print(f"Scheduled rows:               {metrics['scheduled_rows']}")
    print(f"Successful reads:             {metrics['successful_reads']}")
    print(f"Read errors:                  {metrics['read_errors']}")
    print(f"Deadline misses:              {metrics['deadline_misses']}")
    print(f"Late by > half period:        {metrics['late_half_period_count']}")
    print(f"dt mean:                      {metrics['dt_mean_ms']:.6f} ms")
    print(f"dt std:                       {metrics['dt_std_ms']:.6f} ms")
    print(f"dt p99:                       {metrics['dt_p99_ms']:.6f} ms")
    print(f"dt p99.9:                     {metrics['dt_p999_ms']:.6f} ms")
    print(f"dt max:                       {metrics['dt_max_ms']:.6f} ms")
    print(f"read duration mean:           {metrics['read_duration_mean_ms']:.6f} ms")
    print(f"read duration p99:            {metrics['read_duration_p99_ms']:.6f} ms")
    print(f"read duration p99.9:          {metrics['read_duration_p999_ms']:.6f} ms")
    print(f"read duration max:            {metrics['read_duration_max_ms']:.6f} ms")
    print(f"schedule lag p99:             {metrics['schedule_lag_p99_ms']:.6f} ms")
    print(f"schedule lag p99.9:           {metrics['schedule_lag_p999_ms']:.6f} ms")
    print(f"accel norm mean/std:          {metrics['accel_norm_mean_mps2']:.6f} / {metrics['accel_norm_std_mps2']:.6f} m/s^2")
    print(f"gyro norm mean/std:           {metrics['gyro_norm_mean_rps']:.6f} / {metrics['gyro_norm_std_rps']:.6f} rad/s")
    print(f"Verdict:                      {metrics['verdict']}")
    print()


def main():
    parser = argparse.ArgumentParser(description="Analyze IMU timing test CSV files.")
    parser.add_argument("csv_files", nargs="+", help="One or more CSV files from imu_i2c_rate_test_logger.py")
    args = parser.parse_args()

    all_metrics = []
    for csv_file in args.csv_files:
        metrics = analyze_csv(Path(csv_file))
        all_metrics.append(metrics)
        print_metrics(metrics)

    if len(all_metrics) > 1:
        print("Summary by requested rate:")
        all_metrics_sorted = sorted(all_metrics, key=lambda m: m["requested_rate_hz"])
        for m in all_metrics_sorted:
            print(
                f"  {m['requested_rate_hz']:7.3f} Hz | "
                f"errors={m['read_errors']:4d} | "
                f"misses={m['deadline_misses']:6d} | "
                f"dt_std={m['dt_std_ms']:.4f} ms | "
                f"dt_p99={m['dt_p99_ms']:.4f} ms | "
                f"read_p99={m['read_duration_p99_ms']:.4f} ms | "
                f"{m['verdict']}"
            )


if __name__ == "__main__":
    main()
