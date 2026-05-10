
#!/usr/bin/env python3
"""
interpret_imu_allan_for_ekf.py

Interpret Allan-deviation CSV curves and extract candidate EKF bias-drift parameters.

Purpose
-------
This script reads the per-axis Allan CSV files produced by analyze_imu_allan.py
and computes simple candidate parameters for EKF tuning.

What it tries to estimate
-------------------------
1. White-noise level from the region whose local slope is closest to -0.5
2. Bias-instability proxy from the flattest region whose local slope is closest to 0
3. Bias-random-walk candidate for EKF from the long-tau rising region whose
   local slope is closest to +0.5

Important note
--------------
This is still an engineering interpretation tool.
It gives grounded candidate values, not perfect final truth.

For your current EKF:
- the most important output here is a candidate for:
    gyro_bias_random_walk_std
    accel_bias_random_walk_std

Expected input
--------------
Run analyze_imu_allan.py first, then point this script at the generated folder,
for example:

python3 interpret_imu_allan_for_ekf.py csv_logging_files/imu_long_test_allan
"""

import os
import math
import json
import argparse
import numpy as np
import pandas as pd


def load_curve_csv(folder_path, signal_name):
    csv_path = os.path.join(folder_path, f"{signal_name}_allan.csv")
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"Missing Allan CSV: {csv_path}")
    return pd.read_csv(csv_path), csv_path


def pick_best_slope_region(curve_df, target_slope, tau_min=None):
    valid = curve_df.copy()

    if tau_min is not None:
        valid = valid[valid["tau_sec"] >= tau_min]

    valid = valid.replace([np.inf, -np.inf], np.nan).dropna(subset=["local_log_slope", "allan_dev", "tau_sec"])

    if len(valid) == 0:
        return None

    score = np.abs(valid["local_log_slope"].to_numpy(dtype=np.float64) - target_slope)
    idx_local = int(np.argmin(score))
    row = valid.iloc[idx_local]

    return {
        "tau_sec": float(row["tau_sec"]),
        "allan_dev": float(row["allan_dev"]),
        "local_log_slope": float(row["local_log_slope"]),
    }


def estimate_white_noise_from_minus_half_region(point):
    """
    For white noise, Allan deviation often behaves approximately like:
        sigma_A(tau) = N / sqrt(tau)

    So:
        N = sigma_A(tau) * sqrt(tau)

    We return N in the same physical units times sqrt(second).
    """
    tau = point["tau_sec"]
    adev = point["allan_dev"]
    return adev * math.sqrt(tau)


def estimate_bias_random_walk_from_plus_half_region(point):
    """
    For a +0.5 slope region, Allan deviation often behaves approximately like:
        sigma_A(tau) = K * sqrt(tau / 3)

    So:
        K = sigma_A(tau) * sqrt(3 / tau)

    We return K in the same physical units divided by sqrt(second).
    """
    tau = point["tau_sec"]
    adev = point["allan_dev"]
    return adev * math.sqrt(3.0 / tau)


def estimate_bias_instability_proxy_from_flat_region(point):
    """
    A simple proxy from the flattest Allan region.
    This is not a full metrology-grade conversion, just a practical indicator.
    """
    return point["allan_dev"]


def build_report_for_signal(signal_name, curve_df):
    white_point = pick_best_slope_region(curve_df, target_slope=-0.5, tau_min=None)
    flat_point = pick_best_slope_region(curve_df, target_slope=0.0, tau_min=None)
    rising_point = pick_best_slope_region(curve_df, target_slope=+0.5, tau_min=1.0)

    report = {
        "signal_name": signal_name,
        "white_noise_region": None,
        "flat_region": None,
        "rising_region": None,
        "candidates": {},
    }

    if white_point is not None:
        report["white_noise_region"] = white_point
        report["candidates"]["white_noise_coefficient"] = estimate_white_noise_from_minus_half_region(white_point)

    if flat_point is not None:
        report["flat_region"] = flat_point
        report["candidates"]["bias_instability_proxy"] = estimate_bias_instability_proxy_from_flat_region(flat_point)

    if rising_point is not None:
        report["rising_region"] = rising_point
        report["candidates"]["bias_random_walk_coefficient"] = estimate_bias_random_walk_from_plus_half_region(rising_point)

    return report


def choose_scalar_from_axes(values):
    """
    Conservative scalar choice for EKF tuning from multiple axes.
    """
    finite_values = [float(v) for v in values if v is not None and np.isfinite(v)]
    if not finite_values:
        return None
    return max(finite_values)


def main():
    parser = argparse.ArgumentParser(description="Interpret Allan CSV curves and extract candidate EKF parameters.")
    parser.add_argument("allan_folder", help="Folder created by analyze_imu_allan.py")
    args = parser.parse_args()

    allan_folder = args.allan_folder
    if not os.path.isdir(allan_folder):
        raise FileNotFoundError(f"Allan folder not found: {allan_folder}")

    gyro_signals = ["gyro_x_rps", "gyro_y_rps", "gyro_z_rps"]
    accel_signals = ["accel_x_mps2", "accel_y_mps2", "accel_z_mps2"]

    signal_reports = []

    for signal_name in gyro_signals + accel_signals:
        curve_df, _ = load_curve_csv(allan_folder, signal_name)
        signal_reports.append(build_report_for_signal(signal_name, curve_df))

    gyro_bias_rw_candidates = []
    accel_bias_rw_candidates = []

    for rep in signal_reports:
        coeff = rep["candidates"].get("bias_random_walk_coefficient", None)
        if rep["signal_name"].startswith("gyro_"):
            gyro_bias_rw_candidates.append(coeff)
        else:
            accel_bias_rw_candidates.append(coeff)

    suggested_gyro_bias_random_walk_std = choose_scalar_from_axes(gyro_bias_rw_candidates)
    suggested_accel_bias_random_walk_std = choose_scalar_from_axes(accel_bias_rw_candidates)

    # Fallback if +0.5 region is weak or ambiguous:
    # use a small fraction of white-noise coefficient as a conservative backup.
    if suggested_gyro_bias_random_walk_std is None:
        gyro_white = [
            rep["candidates"].get("white_noise_coefficient", None)
            for rep in signal_reports if rep["signal_name"].startswith("gyro_")
        ]
        finite = [v for v in gyro_white if v is not None and np.isfinite(v)]
        if finite:
            suggested_gyro_bias_random_walk_std = 0.01 * max(finite)

    if suggested_accel_bias_random_walk_std is None:
        accel_white = [
            rep["candidates"].get("white_noise_coefficient", None)
            for rep in signal_reports if rep["signal_name"].startswith("accel_")
        ]
        finite = [v for v in accel_white if v is not None and np.isfinite(v)]
        if finite:
            suggested_accel_bias_random_walk_std = 0.01 * max(finite)

    output = {
        "allan_folder": allan_folder,
        "signal_reports": signal_reports,
        "suggested_parameters": {
            "gyro_bias_random_walk_std": suggested_gyro_bias_random_walk_std,
            "accel_bias_random_walk_std": suggested_accel_bias_random_walk_std,
        },
        "notes": [
            "These are candidate EKF bias-drift parameters derived from Allan-curve interpretation.",
            "Use them as grounded starting values, then validate in stationary EKF tests.",
            "If the rising +0.5 region is weak, the script falls back to a conservative fraction of the white-noise coefficient.",
        ],
    }

    json_path = os.path.join(allan_folder, "ekf_allan_interpretation.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2)

    txt_path = os.path.join(allan_folder, "ekf_allan_interpretation.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("EKF candidate parameters from Allan interpretation\n")
        f.write("=" * 72 + "\n\n")

        for rep in signal_reports:
            f.write(f"{rep['signal_name']}\n")
            f.write("-" * 72 + "\n")

            if rep["white_noise_region"] is not None:
                p = rep["white_noise_region"]
                f.write(
                    f"white-noise region: tau={p['tau_sec']:.6f} s, "
                    f"adev={p['allan_dev']:.12e}, slope={p['local_log_slope']:+.4f}\n"
                )
                f.write(
                    f"white-noise coefficient: {rep['candidates']['white_noise_coefficient']:.12e}\n"
                )

            if rep["flat_region"] is not None:
                p = rep["flat_region"]
                f.write(
                    f"flat region:        tau={p['tau_sec']:.6f} s, "
                    f"adev={p['allan_dev']:.12e}, slope={p['local_log_slope']:+.4f}\n"
                )
                f.write(
                    f"bias-instability proxy: {rep['candidates']['bias_instability_proxy']:.12e}\n"
                )

            if rep["rising_region"] is not None:
                p = rep["rising_region"]
                f.write(
                    f"rising region:      tau={p['tau_sec']:.6f} s, "
                    f"adev={p['allan_dev']:.12e}, slope={p['local_log_slope']:+.4f}\n"
                )
                f.write(
                    f"bias-random-walk coefficient: {rep['candidates']['bias_random_walk_coefficient']:.12e}\n"
                )

            f.write("\n")

        f.write("Suggested EKF parameters\n")
        f.write("-" * 72 + "\n")
        if suggested_gyro_bias_random_walk_std is not None:
            f.write(f"self.gyro_bias_random_walk_std = {suggested_gyro_bias_random_walk_std:.12e}\n")
        else:
            f.write("self.gyro_bias_random_walk_std = <not found>\n")

        if suggested_accel_bias_random_walk_std is not None:
            f.write(f"self.accel_bias_random_walk_std = {suggested_accel_bias_random_walk_std:.12e}\n")
        else:
            f.write("self.accel_bias_random_walk_std = <not found>\n")

        f.write("\n")
        f.write("Interpretation note:\n")
        f.write("These are candidate starting values, not guaranteed final values.\n")
        f.write("Validate them in the stationary EKF test after updating the filter.\n")

    print(f"Saved EKF Allan interpretation in: {allan_folder}")
    print()
    if suggested_gyro_bias_random_walk_std is not None:
        print(f"Suggested self.gyro_bias_random_walk_std = {suggested_gyro_bias_random_walk_std:.12e}")
    else:
        print("Suggested self.gyro_bias_random_walk_std = <not found>")

    if suggested_accel_bias_random_walk_std is not None:
        print(f"Suggested self.accel_bias_random_walk_std = {suggested_accel_bias_random_walk_std:.12e}")
    else:
        print("Suggested self.accel_bias_random_walk_std = <not found>")

    print()
    print("Also wrote:")
    print("- ekf_allan_interpretation.txt")
    print("- ekf_allan_interpretation.json")


if __name__ == "__main__":
    main()
