#!/usr/bin/env python3
"""
plot_imu_csv_quicklook.py

Quick-look plots for a stationary IMU CSV log.

Purpose
-------
- verify the logger worked
- inspect timing stability
- inspect accelerometer and gyro traces
- catch obvious problems before running a long test
"""

import argparse
import os

import matplotlib.pyplot as plt
import pandas as pd


def make_plot(df, x_col, y_cols, title, ylabel, output_path):
    plt.figure(figsize=(12, 6))
    for y_col in y_cols:
        plt.plot(df[x_col], df[y_col], label=y_col)
    plt.xlabel(x_col)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()


def main():
    parser = argparse.ArgumentParser(description="Make quick-look plots from IMU CSV data.")
    parser.add_argument("csv_path", type=str, help="Input CSV path")
    parser.add_argument("--output-dir", type=str, default=None, help="Directory for generated plots")
    parser.add_argument("--max-seconds", type=float, default=None, help="Only plot the first N seconds")
    args = parser.parse_args()

    csv_path = args.csv_path
    output_dir = args.output_dir
    if output_dir is None:
        base_name = os.path.splitext(os.path.basename(csv_path))[0]
        output_dir = os.path.join(os.path.dirname(csv_path), f"{base_name}_quicklook")
    os.makedirs(output_dir, exist_ok=True)

    df = pd.read_csv(csv_path)

    if len(df) == 0:
        raise RuntimeError("CSV file is empty.")

    t0 = df["timestamp_monotonic_sec"].iloc[0]
    df["time_from_start_sec"] = df["timestamp_monotonic_sec"] - t0

    if args.max_seconds is not None:
        df = df[df["time_from_start_sec"] <= args.max_seconds].copy()

    # Timing summary
    dt_series = df["dt_sec"].copy()
    dt_series = dt_series[dt_series > 0.0]
    if len(dt_series) > 0:
        achieved_rate_hz = 1.0 / dt_series.mean()
        print(f"Samples plotted: {len(df)}")
        print(f"Mean dt: {dt_series.mean():.6f} s")
        print(f"Std dt : {dt_series.std():.6f} s")
        print(f"Approx achieved rate: {achieved_rate_hz:.2f} Hz")
    else:
        print(f"Samples plotted: {len(df)}")
        print("Not enough dt data to estimate rate.")

    print("\nAccelerometer means [m/s^2]:")
    print(df[["accel_x_mps2", "accel_y_mps2", "accel_z_mps2"]].mean())
    print("\nGyroscope means [rad/s]:")
    print(df[["gyro_x_rps", "gyro_y_rps", "gyro_z_rps"]].mean())

    make_plot(
        df,
        x_col="time_from_start_sec",
        y_cols=["accel_x_mps2", "accel_y_mps2", "accel_z_mps2"],
        title="Accelerometer vs time",
        ylabel="Acceleration [m/s^2]",
        output_path=os.path.join(output_dir, "accel_vs_time.png"),
    )

    make_plot(
        df,
        x_col="time_from_start_sec",
        y_cols=["gyro_x_rps", "gyro_y_rps", "gyro_z_rps"],
        title="Gyroscope vs time",
        ylabel="Angular velocity [rad/s]",
        output_path=os.path.join(output_dir, "gyro_vs_time.png"),
    )

    make_plot(
        df,
        x_col="time_from_start_sec",
        y_cols=["temperature_c"],
        title="IMU temperature vs time",
        ylabel="Temperature [deg C]",
        output_path=os.path.join(output_dir, "temperature_vs_time.png"),
    )

    plt.figure(figsize=(12, 5))
    dt_plot = df["dt_sec"][df["dt_sec"] > 0.0]
    plt.plot(dt_plot.to_numpy())
    plt.xlabel("Sample index")
    plt.ylabel("dt [s]")
    plt.title("Sample interval dt")
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "dt_vs_index.png"), dpi=150)
    plt.close()

    plt.figure(figsize=(8, 5))
    plt.hist(dt_plot.to_numpy(), bins=80)
    plt.xlabel("dt [s]")
    plt.ylabel("Count")
    plt.title("Histogram of sample interval dt")
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "dt_histogram.png"), dpi=150)
    plt.close()

    print(f"\nSaved quick-look plots in: {output_dir}")


if __name__ == "__main__":
    main()
