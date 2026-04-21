
#!/usr/bin/env python3
"""
analyze_imu_allan.py

Compute Allan deviation from a stationary IMU CSV log.

Purpose
-------
This script is intended for long stationary logs, for example:
- 45 min
- 60 min

It computes Allan deviation curves for:
- gyro_x_rps
- gyro_y_rps
- gyro_z_rps
- accel_x_mps2
- accel_y_mps2
- accel_z_mps2

It also:
- saves log-log Allan deviation plots
- saves a text report
- saves per-signal CSV tables with tau and Allan deviation values

Important note
--------------
This script helps characterize:
- white noise
- slow drift
- bias instability / long-term behavior

It does NOT automatically convert everything into final EKF parameters
perfectly. The main goal is to generate trustworthy Allan deviation curves
that we can interpret next.
"""

import os
import math
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


def mean_sample_rate_from_dt(dt_array):
    dt_array = np.asarray(dt_array, dtype=np.float64)
    dt_array = dt_array[dt_array > 0.0]
    if len(dt_array) == 0:
        raise ValueError("No positive dt values found in dt_sec.")
    return 1.0 / float(np.mean(dt_array))


def allan_deviation_from_signal(signal_array, sample_rate_hz, max_num_clusters=80):
    """
    Compute Allan deviation for a 1D time series sampled at a constant rate.
    """
    signal_array = np.asarray(signal_array, dtype=np.float64)
    N = len(signal_array)

    if N < 10:
        raise ValueError("Signal is too short for Allan deviation.")

    max_m = N // 4
    if max_m < 1:
        raise ValueError("Signal is too short for valid cluster sizing.")

    cluster_sizes = np.unique(
        np.logspace(0, math.log10(max_m), num=max_num_clusters).astype(int)
    )
    cluster_sizes = cluster_sizes[cluster_sizes >= 1]

    tau_values_sec = []
    allan_dev_values = []
    cluster_sizes_m = []

    for m in cluster_sizes:
        num_clusters = N // m
        if num_clusters < 2:
            continue

        trimmed = signal_array[:num_clusters * m]
        clusters = trimmed.reshape(num_clusters, m)
        cluster_means = np.mean(clusters, axis=1)

        diffs = np.diff(cluster_means)
        allan_var = 0.5 * np.mean(diffs ** 2)
        allan_dev = math.sqrt(allan_var)

        tau_values_sec.append(m / sample_rate_hz)
        allan_dev_values.append(allan_dev)
        cluster_sizes_m.append(m)

    return (
        np.asarray(tau_values_sec, dtype=np.float64),
        np.asarray(allan_dev_values, dtype=np.float64),
        np.asarray(cluster_sizes_m, dtype=np.int64),
    )


def local_log_slope(tau_values, allan_dev_values):
    """
    Estimate local slope on log-log axes using neighboring points.
    """
    log_tau = np.log10(tau_values)
    log_adev = np.log10(allan_dev_values)

    slopes = np.empty_like(log_tau)
    slopes[:] = np.nan

    if len(log_tau) < 3:
        if len(log_tau) == 2:
            slope = (log_adev[1] - log_adev[0]) / (log_tau[1] - log_tau[0])
            slopes[:] = slope
        return slopes

    for i in range(1, len(log_tau) - 1):
        dx = log_tau[i + 1] - log_tau[i - 1]
        if abs(dx) < 1e-15:
            continue
        slopes[i] = (log_adev[i + 1] - log_adev[i - 1]) / dx

    slopes[0] = (log_adev[1] - log_adev[0]) / (log_tau[1] - log_tau[0])
    slopes[-1] = (log_adev[-1] - log_adev[-2]) / (log_tau[-1] - log_tau[-2])
    return slopes


def find_minimum_allan_point(tau_values, allan_dev_values):
    index = int(np.argmin(allan_dev_values))
    return {
        "index": index,
        "tau_sec": float(tau_values[index]),
        "allan_dev": float(allan_dev_values[index]),
    }


def slope_near_tau(tau_values, slopes, tau_target):
    idx = int(np.argmin(np.abs(tau_values - tau_target)))
    return float(tau_values[idx]), float(slopes[idx])


def main():
    parser = argparse.ArgumentParser(description="Compute Allan deviation from stationary IMU CSV log.")
    parser.add_argument("csv_path", help="Path to IMU CSV file")
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Directory for Allan outputs. Default: <csv_basename>_allan"
    )
    parser.add_argument(
        "--max-num-clusters",
        type=int,
        default=80,
        help="Number of logarithmically spaced cluster sizes"
    )
    args = parser.parse_args()

    csv_path = args.csv_path
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    csv_dir = os.path.dirname(csv_path)
    csv_name = os.path.splitext(os.path.basename(csv_path))[0]

    if args.output_dir is None:
        output_dir = os.path.join(csv_dir, f"{csv_name}_allan")
    else:
        output_dir = args.output_dir

    ensure_dir(output_dir)

    df = pd.read_csv(csv_path)

    required_columns = [
        "dt_sec",
        "accel_x_mps2", "accel_y_mps2", "accel_z_mps2",
        "gyro_x_rps", "gyro_y_rps", "gyro_z_rps",
    ]
    missing = [col for col in required_columns if col not in df.columns]
    if missing:
        raise ValueError(f"Missing required CSV columns: {missing}")

    dt_array = df["dt_sec"].to_numpy(dtype=np.float64)
    sample_rate_hz = mean_sample_rate_from_dt(dt_array)

    signal_columns = [
        "gyro_x_rps",
        "gyro_y_rps",
        "gyro_z_rps",
        "accel_x_mps2",
        "accel_y_mps2",
        "accel_z_mps2",
    ]

    summary_lines = []
    summary_lines.append("IMU Allan deviation analysis")
    summary_lines.append("=" * 72)
    summary_lines.append("")
    summary_lines.append(f"Input CSV: {csv_path}")
    summary_lines.append(f"Estimated sample rate [Hz]: {sample_rate_hz:.9f}")
    summary_lines.append(f"Samples: {len(df)}")
    summary_lines.append(f"Duration [s]: {float(np.sum(dt_array)):.3f}")
    summary_lines.append("")

    all_curves = {}

    for column_name in signal_columns:
        signal_array = df[column_name].to_numpy(dtype=np.float64)

        tau_values_sec, allan_dev_values, cluster_sizes_m = allan_deviation_from_signal(
            signal_array=signal_array,
            sample_rate_hz=sample_rate_hz,
            max_num_clusters=args.max_num_clusters,
        )

        slopes = local_log_slope(tau_values_sec, allan_dev_values)
        min_point = find_minimum_allan_point(tau_values_sec, allan_dev_values)

        curve_df = pd.DataFrame({
            "tau_sec": tau_values_sec,
            "cluster_size_m": cluster_sizes_m,
            "allan_dev": allan_dev_values,
            "local_log_slope": slopes,
        })

        curve_csv_path = os.path.join(output_dir, f"{column_name}_allan.csv")
        curve_df.to_csv(curve_csv_path, index=False)

        all_curves[column_name] = {
            "tau_sec": tau_values_sec,
            "allan_dev": allan_dev_values,
        }

        summary_lines.append(f"{column_name}")
        summary_lines.append("-" * 72)
        summary_lines.append(f"Minimum Allan deviation at tau [s]: {min_point['tau_sec']:.6f}")
        summary_lines.append(f"Minimum Allan deviation value    : {min_point['allan_dev']:.12f}")
        summary_lines.append("Approx local slopes around selected taus:")
        for tau_target in [0.01, 0.1, 1.0, 10.0, 100.0, 300.0]:
            tau_used, slope_used = slope_near_tau(tau_values_sec, slopes, tau_target)
            summary_lines.append(
                f"  tau ~ {tau_used:10.6f} s : slope ~ {slope_used:+.4f}"
            )
        summary_lines.append("")

        plt.figure(figsize=(9, 6))
        plt.loglog(tau_values_sec, allan_dev_values, marker='o', markersize=3, linewidth=1)
        plt.title(f"Allan deviation: {column_name}")
        plt.xlabel("tau [s]")
        plt.ylabel("Allan deviation")
        plt.grid(True, which="both", alpha=0.3)
        save_plot(os.path.join(output_dir, f"{column_name}_allan.png"))

    plt.figure(figsize=(10, 7))
    for col in ["gyro_x_rps", "gyro_y_rps", "gyro_z_rps"]:
        plt.loglog(all_curves[col]["tau_sec"], all_curves[col]["allan_dev"], label=col)
    plt.title("Allan deviation: gyroscope")
    plt.xlabel("tau [s]")
    plt.ylabel("Allan deviation [rad/s]")
    plt.grid(True, which="both", alpha=0.3)
    plt.legend()
    save_plot(os.path.join(output_dir, "gyro_allan_combined.png"))

    plt.figure(figsize=(10, 7))
    for col in ["accel_x_mps2", "accel_y_mps2", "accel_z_mps2"]:
        plt.loglog(all_curves[col]["tau_sec"], all_curves[col]["allan_dev"], label=col)
    plt.title("Allan deviation: accelerometer")
    plt.xlabel("tau [s]")
    plt.ylabel("Allan deviation [m/s^2]")
    plt.grid(True, which="both", alpha=0.3)
    plt.legend()
    save_plot(os.path.join(output_dir, "accel_allan_combined.png"))

    txt_path = os.path.join(output_dir, "allan_summary.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(summary_lines))

    print(f"Saved Allan deviation outputs in: {output_dir}")
    print()
    print("Created:")
    print("- allan_summary.txt")
    print("- one CSV curve file per axis")
    print("- one Allan deviation plot per axis")
    print("- combined gyro and accel Allan plots")
    print()
    print("Next step:")
    print("Use the Allan curves to estimate gyro_bias_random_walk_std and accel_bias_random_walk_std.")


if __name__ == "__main__":
    main()
