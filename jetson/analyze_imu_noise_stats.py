#!/usr/bin/env python3
"""
analyze_imu_noise_stats.py

Read a stationary IMU CSV log and compute noise statistics for EKF tuning.

What this script estimates well from stationary data:
- gyro white noise standard deviation [rad/s]
- accel white noise standard deviation [m/s^2]
- accel direction measurement noise for gravity update

What this script does NOT fully estimate yet:
- gyro bias random walk
- accel bias random walk

Those bias-drift terms are better estimated later with Allan deviation analysis.
This script is still the correct next step for setting the main EKF noise levels.

Expected CSV columns
--------------------
sample_index
timestamp_sec
time_monotonic_sec
dt_sec
accel_x_mps2
accel_y_mps2
accel_z_mps2
gyro_x_rps
gyro_y_rps
gyro_z_rps
temperature_c
raw_accel_x
raw_accel_y
raw_accel_z
raw_gyro_x
raw_gyro_y
raw_gyro_z
"""

import os
import json
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def save_plot(output_path):
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()


def vector_norm_rows(array_2d):
    return np.linalg.norm(array_2d, axis=1)


def compute_basic_stats(series):
    return {
        "mean": float(np.mean(series)),
        "std": float(np.std(series, ddof=1)),
        "min": float(np.min(series)),
        "max": float(np.max(series)),
    }


def main():
    parser = argparse.ArgumentParser(description="Analyze stationary IMU CSV log for EKF noise tuning.")
    parser.add_argument("csv_path", help="Path to IMU CSV file")
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Directory for analysis outputs. Default: <csv_basename>_analysis"
    )
    args = parser.parse_args()

    csv_path = args.csv_path
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    csv_dir = os.path.dirname(csv_path)
    csv_name = os.path.splitext(os.path.basename(csv_path))[0]

    if args.output_dir is None:
        output_dir = os.path.join(csv_dir, f"{csv_name}_analysis")
    else:
        output_dir = args.output_dir

    ensure_dir(output_dir)

    df = pd.read_csv(csv_path)

    required_columns = [
        "dt_sec",
        "accel_x_mps2", "accel_y_mps2", "accel_z_mps2",
        "gyro_x_rps", "gyro_y_rps", "gyro_z_rps",
        "temperature_c",
    ]
    missing = [col for col in required_columns if col not in df.columns]
    if missing:
        raise ValueError(f"Missing required CSV columns: {missing}")

    accel = df[["accel_x_mps2", "accel_y_mps2", "accel_z_mps2"]].to_numpy(dtype=np.float64)
    gyro = df[["gyro_x_rps", "gyro_y_rps", "gyro_z_rps"]].to_numpy(dtype=np.float64)
    dt = df["dt_sec"].to_numpy(dtype=np.float64)
    temperature = df["temperature_c"].to_numpy(dtype=np.float64)

    sample_count = len(df)
    duration_sec = float(np.sum(dt))
    mean_dt = float(np.mean(dt))
    std_dt = float(np.std(dt, ddof=1))
    achieved_rate_hz = 1.0 / mean_dt if mean_dt > 0.0 else float("nan")

    # ------------------------------------------------------------
    # Mean signals
    # ------------------------------------------------------------
    accel_mean = np.mean(accel, axis=0)
    gyro_mean = np.mean(gyro, axis=0)

    # Residuals around the mean
    accel_residual = accel - accel_mean
    gyro_residual = gyro - gyro_mean

    # Per-axis residual std
    accel_std_xyz = np.std(accel_residual, axis=0, ddof=1)
    gyro_std_xyz = np.std(gyro_residual, axis=0, ddof=1)

    # Conservative scalar suggestions for EKF white-noise std
    accel_noise_std_suggested = float(np.max(accel_std_xyz))
    gyro_noise_std_suggested = float(np.max(gyro_std_xyz))

    # ------------------------------------------------------------
    # Accelerometer norm statistics
    # ------------------------------------------------------------
    accel_norm = vector_norm_rows(accel)
    accel_norm_stats = compute_basic_stats(accel_norm)

    # ------------------------------------------------------------
    # Accelerometer direction statistics
    #
    # This is more relevant to accel_meas_std than raw accel std,
    # because the EKF gravity update uses direction.
    # ------------------------------------------------------------
    accel_dir = accel / accel_norm[:, None]
    accel_dir_mean = np.mean(accel_dir, axis=0)
    accel_dir_mean = accel_dir_mean / np.linalg.norm(accel_dir_mean)

    accel_dir_residual = accel_dir - accel_dir_mean[None, :]
    accel_dir_std_xyz = np.std(accel_dir_residual, axis=0, ddof=1)

    # A conservative scalar choice for the accelerometer measurement
    # noise in the gravity-direction update
    accel_meas_std_suggested = float(np.max(accel_dir_std_xyz))

    # ------------------------------------------------------------
    # Temperature
    # ------------------------------------------------------------
    temp_stats = compute_basic_stats(temperature)

    # ------------------------------------------------------------
    # Build report dictionary
    # ------------------------------------------------------------
    report = {
        "input_csv": csv_path,
        "sample_count": int(sample_count),
        "duration_sec": duration_sec,
        "timing": {
            "mean_dt_sec": mean_dt,
            "std_dt_sec": std_dt,
            "achieved_rate_hz": achieved_rate_hz,
            "min_dt_sec": float(np.min(dt)),
            "max_dt_sec": float(np.max(dt)),
        },
        "accelerometer": {
            "mean_mps2_xyz": accel_mean.tolist(),
            "residual_std_mps2_xyz": accel_std_xyz.tolist(),
            "suggested_accel_noise_std_mps2": accel_noise_std_suggested,
            "norm_stats_mps2": accel_norm_stats,
            "direction_mean_xyz": accel_dir_mean.tolist(),
            "direction_residual_std_xyz": accel_dir_std_xyz.tolist(),
            "suggested_accel_meas_std": accel_meas_std_suggested,
        },
        "gyroscope": {
            "mean_rps_xyz": gyro_mean.tolist(),
            "residual_std_rps_xyz": gyro_std_xyz.tolist(),
            "suggested_gyro_noise_std_rps": gyro_noise_std_suggested,
        },
        "temperature": {
            "stats_c": temp_stats,
        },
        "notes": [
            "suggested_accel_noise_std_mps2 is based on stationary residual std after subtracting mean accel vector",
            "suggested_gyro_noise_std_rps is based on stationary residual std after subtracting mean gyro vector",
            "suggested_accel_meas_std is based on normalized accelerometer direction residuals",
            "gyro_bias_random_walk_std and accel_bias_random_walk_std are not fully estimated here; use Allan deviation later",
        ],
    }

    # ------------------------------------------------------------
    # Save JSON report
    # ------------------------------------------------------------
    json_path = os.path.join(output_dir, "imu_noise_report.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)

    # ------------------------------------------------------------
    # Save readable text summary
    # ------------------------------------------------------------
    txt_path = os.path.join(output_dir, "imu_noise_report.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("IMU stationary noise analysis\n")
        f.write("=" * 60 + "\n\n")

        f.write(f"Input CSV: {csv_path}\n")
        f.write(f"Samples: {sample_count}\n")
        f.write(f"Duration [s]: {duration_sec:.3f}\n\n")

        f.write("Timing\n")
        f.write("-" * 60 + "\n")
        f.write(f"Mean dt [s]: {mean_dt:.9f}\n")
        f.write(f"Std dt  [s]: {std_dt:.9f}\n")
        f.write(f"Rate [Hz]:   {achieved_rate_hz:.6f}\n")
        f.write(f"Min dt [s]:  {np.min(dt):.9f}\n")
        f.write(f"Max dt [s]:  {np.max(dt):.9f}\n\n")

        f.write("Accelerometer\n")
        f.write("-" * 60 + "\n")
        f.write(f"Mean [m/s^2] xyz: {accel_mean}\n")
        f.write(f"Residual std [m/s^2] xyz: {accel_std_xyz}\n")
        f.write(f"Suggested accel_noise_std_mps2: {accel_noise_std_suggested:.9f}\n")
        f.write(f"Accel norm mean [m/s^2]: {accel_norm_stats['mean']:.9f}\n")
        f.write(f"Accel norm std  [m/s^2]: {accel_norm_stats['std']:.9f}\n")
        f.write(f"Direction mean xyz: {accel_dir_mean}\n")
        f.write(f"Direction residual std xyz: {accel_dir_std_xyz}\n")
        f.write(f"Suggested accel_meas_std: {accel_meas_std_suggested:.9f}\n\n")

        f.write("Gyroscope\n")
        f.write("-" * 60 + "\n")
        f.write(f"Mean [rad/s] xyz: {gyro_mean}\n")
        f.write(f"Residual std [rad/s] xyz: {gyro_std_xyz}\n")
        f.write(f"Suggested gyro_noise_std_rps: {gyro_noise_std_suggested:.9f}\n\n")

        f.write("Temperature\n")
        f.write("-" * 60 + "\n")
        f.write(f"Mean [C]: {temp_stats['mean']:.6f}\n")
        f.write(f"Std  [C]: {temp_stats['std']:.6f}\n")
        f.write(f"Min  [C]: {temp_stats['min']:.6f}\n")
        f.write(f"Max  [C]: {temp_stats['max']:.6f}\n\n")

        f.write("Suggested EKF values from this script\n")
        f.write("-" * 60 + "\n")
        f.write(f"self.gyro_noise_std_rps = {gyro_noise_std_suggested:.9f}\n")
        f.write(f"self.accel_noise_std_mps2 = {accel_noise_std_suggested:.9f}\n")
        f.write(f"self.accel_meas_std = {accel_meas_std_suggested:.9f}\n")
        f.write("\n")
        f.write("Bias random walk terms are not finalized here.\n")
        f.write("Estimate those later with Allan deviation.\n")

    # ------------------------------------------------------------
    # Plots
    # ------------------------------------------------------------
    time_from_start = np.cumsum(dt) - dt[0]

    # Residual accel plot
    plt.figure(figsize=(12, 6))
    plt.plot(time_from_start, accel_residual[:, 0], label="accel_x residual")
    plt.plot(time_from_start, accel_residual[:, 1], label="accel_y residual")
    plt.plot(time_from_start, accel_residual[:, 2], label="accel_z residual")
    plt.title("Accelerometer residuals vs time")
    plt.xlabel("time_from_start_sec")
    plt.ylabel("accel residual [m/s^2]")
    plt.legend(loc="upper right")
    plt.grid(True, alpha=0.3)
    save_plot(os.path.join(output_dir, "accel_residuals_vs_time.png"))

    # Residual gyro plot
    plt.figure(figsize=(12, 6))
    plt.plot(time_from_start, gyro_residual[:, 0], label="gyro_x residual")
    plt.plot(time_from_start, gyro_residual[:, 1], label="gyro_y residual")
    plt.plot(time_from_start, gyro_residual[:, 2], label="gyro_z residual")
    plt.title("Gyroscope residuals vs time")
    plt.xlabel("time_from_start_sec")
    plt.ylabel("gyro residual [rad/s]")
    plt.legend(loc="upper right")
    plt.grid(True, alpha=0.3)
    save_plot(os.path.join(output_dir, "gyro_residuals_vs_time.png"))

    # Accel norm histogram
    plt.figure(figsize=(10, 6))
    plt.hist(accel_norm, bins=100)
    plt.title("Accelerometer norm histogram")
    plt.xlabel("accel norm [m/s^2]")
    plt.ylabel("count")
    plt.grid(True, alpha=0.3)
    save_plot(os.path.join(output_dir, "accel_norm_histogram.png"))

    # dt histogram
    plt.figure(figsize=(10, 6))
    plt.hist(dt, bins=100)
    plt.title("dt histogram")
    plt.xlabel("dt [s]")
    plt.ylabel("count")
    plt.grid(True, alpha=0.3)
    save_plot(os.path.join(output_dir, "dt_histogram.png"))

    print(f"Saved analysis outputs in: {output_dir}")
    print()
    print("Suggested EKF values from this stationary analysis:")
    print(f"self.gyro_noise_std_rps   = {gyro_noise_std_suggested:.9f}")
    print(f"self.accel_noise_std_mps2 = {accel_noise_std_suggested:.9f}")
    print(f"self.accel_meas_std       = {accel_meas_std_suggested:.9f}")
    print()
    print("Bias random walk terms are not finalized here.")
    print("We should estimate those next from Allan deviation.")
    print()
    print(f"Gyro residual std xyz [rad/s]   : {gyro_std_xyz}")
    print(f"Accel residual std xyz [m/s^2]  : {accel_std_xyz}")
    print(f"Accel direction residual std xyz: {accel_dir_std_xyz}")


if __name__ == "__main__":
    main()