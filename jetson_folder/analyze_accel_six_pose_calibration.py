
#!/usr/bin/env python3
"""
analyze_accel_six_pose_calibration.py

Estimate accelerometer bias and per-axis scale from six static pose logs:
    +X up, -X up, +Y up, -Y up, +Z up, -Z up
"""

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


REQUIRED_FILES = {
    "pos_x": "accel_calib_pos_x_up.csv",
    "neg_x": "accel_calib_neg_x_up.csv",
    "pos_y": "accel_calib_pos_y_up.csv",
    "neg_y": "accel_calib_neg_y_up.csv",
    "pos_z": "accel_calib_pos_z_up.csv",
    "neg_z": "accel_calib_neg_z_up.csv",
}


def load_pose_mean(csv_path: Path):
    df = pd.read_csv(csv_path)
    needed = ["accel_x_mps2", "accel_y_mps2", "accel_z_mps2"]
    missing = [c for c in needed if c not in df.columns]
    if missing:
        raise ValueError(f"Missing columns in {csv_path}: {missing}")

    accel = df[needed].to_numpy(dtype=np.float64)
    mean_vec = np.mean(accel, axis=0)
    std_vec = np.std(accel, axis=0)
    norm_mean = float(np.mean(np.linalg.norm(accel, axis=1)))
    return {
        "mean": mean_vec,
        "std": std_vec,
        "norm_mean": norm_mean,
        "samples": len(df),
    }


def corrected_vec(raw_vec, bias_vec, scale_vec):
    return (raw_vec - bias_vec) / scale_vec


def main():
    parser = argparse.ArgumentParser(description="Analyze six-pose accelerometer calibration logs.")
    parser.add_argument(
        "input_dir",
        help="Directory containing the six accelerometer calibration CSV files",
    )
    parser.add_argument(
        "--g",
        type=float,
        default=None,
        help="True local gravity magnitude [m/s^2]. If omitted, estimate from the six logs.",
    )
    args = parser.parse_args()

    input_dir = Path(args.input_dir).expanduser().resolve()
    if not input_dir.is_dir():
        raise FileNotFoundError(f"Input directory not found: {input_dir}")

    pose_stats = {}
    for pose_name, filename in REQUIRED_FILES.items():
        csv_path = input_dir / filename
        if not csv_path.is_file():
            raise FileNotFoundError(f"Missing required file: {csv_path}")
        pose_stats[pose_name] = load_pose_mean(csv_path)

    if args.g is None:
        g_true = float(np.mean([pose_stats[name]["norm_mean"] for name in pose_stats]))
        g_source = "estimated from mean norm across six logs"
    else:
        g_true = float(args.g)
        g_source = "user provided"

    mean_pos_x = pose_stats["pos_x"]["mean"]
    mean_neg_x = pose_stats["neg_x"]["mean"]
    mean_pos_y = pose_stats["pos_y"]["mean"]
    mean_neg_y = pose_stats["neg_y"]["mean"]
    mean_pos_z = pose_stats["pos_z"]["mean"]
    mean_neg_z = pose_stats["neg_z"]["mean"]

    bias_x = 0.5 * (mean_pos_x[0] + mean_neg_x[0])
    bias_y = 0.5 * (mean_pos_y[1] + mean_neg_y[1])
    bias_z = 0.5 * (mean_pos_z[2] + mean_neg_z[2])

    scale_x = 0.5 * (mean_pos_x[0] - mean_neg_x[0]) / g_true
    scale_y = 0.5 * (mean_pos_y[1] - mean_neg_y[1]) / g_true
    scale_z = 0.5 * (mean_pos_z[2] - mean_neg_z[2]) / g_true

    bias_vec = np.array([bias_x, bias_y, bias_z], dtype=np.float64)
    scale_vec = np.array([scale_x, scale_y, scale_z], dtype=np.float64)

    expected_pose_vectors = {
        "pos_x": np.array([+g_true, 0.0, 0.0]),
        "neg_x": np.array([-g_true, 0.0, 0.0]),
        "pos_y": np.array([0.0, +g_true, 0.0]),
        "neg_y": np.array([0.0, -g_true, 0.0]),
        "pos_z": np.array([0.0, 0.0, +g_true]),
        "neg_z": np.array([0.0, 0.0, -g_true]),
    }

    corrected_pose_means = {}
    raw_pose_means = {}

    for pose_name, stats in pose_stats.items():
        raw_pose_means[pose_name] = stats["mean"].tolist()
        corrected_mean = corrected_vec(stats["mean"], bias_vec, scale_vec)
        expected_mean = expected_pose_vectors[pose_name]
        error_vec = corrected_mean - expected_mean
        corrected_pose_means[pose_name] = {
            "corrected_mean": corrected_mean.tolist(),
            "expected_mean": expected_mean.tolist(),
            "error_vec": error_vec.tolist(),
            "error_norm": float(np.linalg.norm(error_vec)),
        }

    output_dir = input_dir / "accel_calib_analysis"
    output_dir.mkdir(parents=True, exist_ok=True)

    result = {
        "input_dir": str(input_dir),
        "gravity_magnitude_mps2": g_true,
        "gravity_source": g_source,
        "bias_mps2": bias_vec.tolist(),
        "scale_unitless": scale_vec.tolist(),
        "raw_pose_means_mps2": raw_pose_means,
        "corrected_pose_means_mps2": corrected_pose_means,
    }

    json_path = output_dir / "accel_calibration_result.json"
    json_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

    lines = []
    lines.append("Six-pose accelerometer calibration result")
    lines.append("=" * 72)
    lines.append("")
    lines.append(f"Input directory: {input_dir}")
    lines.append(f"Gravity magnitude used [m/s^2]: {g_true:.9f} ({g_source})")
    lines.append("")
    lines.append("Raw pose mean vectors [m/s^2]:")
    for pose_name in ["pos_x", "neg_x", "pos_y", "neg_y", "pos_z", "neg_z"]:
        vec = np.array(raw_pose_means[pose_name], dtype=np.float64)
        lines.append(f"  {pose_name:6s}: [{vec[0]: .6f}, {vec[1]: .6f}, {vec[2]: .6f}]")
    lines.append("")
    lines.append("Estimated accelerometer calibration:")
    lines.append(f"  accel_bias_vec_mps2 = np.array([{bias_vec[0]:.9f}, {bias_vec[1]:.9f}, {bias_vec[2]:.9f}], dtype=np.float64)")
    lines.append(f"  accel_scale_vec     = np.array([{scale_vec[0]:.9f}, {scale_vec[1]:.9f}, {scale_vec[2]:.9f}], dtype=np.float64)")
    lines.append("")
    lines.append("Corrected pose means and residuals:")
    for pose_name in ["pos_x", "neg_x", "pos_y", "neg_y", "pos_z", "neg_z"]:
        info = corrected_pose_means[pose_name]
        cm = np.array(info["corrected_mean"], dtype=np.float64)
        ev = np.array(info["expected_mean"], dtype=np.float64)
        er = np.array(info["error_vec"], dtype=np.float64)
        lines.append(
            f"  {pose_name:6s}: corrected=[{cm[0]: .6f}, {cm[1]: .6f}, {cm[2]: .6f}] "
            f" expected=[{ev[0]: .6f}, {ev[1]: .6f}, {ev[2]: .6f}] "
            f" error=[{er[0]: .6f}, {er[1]: .6f}, {er[2]: .6f}] "
            f"|error|={info['error_norm']:.6f}"
        )
    lines.append("")
    lines.append("How to apply in code:")
    lines.append("  corrected_accel = (raw_accel - accel_bias_vec_mps2) / accel_scale_vec")
    lines.append("")
    lines.append("Note:")
    lines.append("  This estimates per-axis bias and scale only.")
    lines.append("  It does not yet estimate cross-axis misalignment.")

    report_path = output_dir / "accel_calibration_report.txt"
    report_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Saved analysis outputs in: {output_dir}")
    print()
    print(f"Gravity magnitude used [m/s^2]: {g_true:.9f} ({g_source})")
    print()
    print("Suggested calibration values:")
    print(f"accel_bias_vec_mps2 = np.array([{bias_vec[0]:.9f}, {bias_vec[1]:.9f}, {bias_vec[2]:.9f}], dtype=np.float64)")
    print(f"accel_scale_vec     = np.array([{scale_vec[0]:.9f}, {scale_vec[1]:.9f}, {scale_vec[2]:.9f}], dtype=np.float64)")
    print()
    print("Also wrote:")
    print(f"- {report_path}")
    print(f"- {json_path}")


if __name__ == "__main__":
    main()
