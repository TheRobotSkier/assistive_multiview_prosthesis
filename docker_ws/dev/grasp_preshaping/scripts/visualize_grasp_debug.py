#!/usr/bin/env python3
"""Interactive 3D visualization of grasp preshaping debug dumps.

Usage:
    python visualize_grasp_debug.py <path_to_dump.npz> [--threshold T] [--no-tsdf]

Dependencies:
    pip install numpy pyvista

If pyvista is unavailable, falls back to a matplotlib-based viewer with
reduced interactivity.
"""

import argparse
import sys
import os
import textwrap

import numpy as np


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

GRASP_TYPE_NAMES = {1: "cylindrical", 2: "pinch", 3: "lateral"}
GRASP_TYPE_COLORS = {1: "#e76f51", 2: "#2a9d8f", 3: "#457b9d"}
F32_MAX = np.float32(np.finfo(np.float32).max)


def load_dump(path: str) -> dict:
    """Load a debug dump .npz and return a dict of parsed arrays."""
    data = np.load(path)

    meta = data["tsdf_metadata"].astype(np.float64)
    origin = meta[0:3]
    resolution = float(meta[3])
    shape = meta[4:7].astype(int)

    tsdf_flat = data["tsdf_volume"]
    tsdf = tsdf_flat.reshape(shape)

    pc = data["point_cloud"].reshape(-1, 3) if len(data["point_cloud"]) > 0 else np.zeros((0, 3))
    roi = data["roi_aabb"].reshape(2, 3)
    cams = data["cameras"].reshape(-1, 3) if len(data["cameras"]) > 0 else np.zeros((0, 3))
    pose = data["input_pose"]
    twist = data["input_twist"]

    grasps_raw = data["scored_grasps"].reshape(-1, 24) if len(data["scored_grasps"]) > 0 else np.zeros((0, 24))

    grasps = {
        "sample_index": grasps_raw[:, 0].astype(int),
        "grasp_type": grasps_raw[:, 1].astype(int),
        "closure": grasps_raw[:, 2],
        "alignment": grasps_raw[:, 3],
        "force_closure": grasps_raw[:, 4],
        "found_collision": grasps_raw[:, 5] > 0.5,
        "combined": grasps_raw[:, 6],
        "probability": grasps_raw[:, 7],
        "pose_4x4": grasps_raw[:, 8:24].reshape(-1, 4, 4),
    }

    return {
        "tsdf": tsdf,
        "tsdf_origin": origin,
        "tsdf_resolution": resolution,
        "point_cloud": pc.astype(np.float64),
        "roi": roi.astype(np.float64),
        "cameras": cams.astype(np.float64),
        "input_pose": pose,
        "input_twist": twist,
        "grasps": grasps,
    }


def print_summary(dump: dict, path: str):
    """Print a human-readable summary of the loaded dump."""
    tsdf = dump["tsdf"]
    grasps = dump["grasps"]
    n_total = len(grasps["combined"])
    n_collision = int(grasps["found_collision"].sum())

    best_idx = -1
    best_combined = -np.inf
    for i in range(n_total):
        if grasps["found_collision"][i] and grasps["combined"][i] > best_combined:
            best_combined = grasps["combined"][i]
            best_idx = i

    print("=" * 52)
    print(f"  Grasp Debug Dump: {os.path.basename(path)}")
    print("=" * 52)
    print(f"  TSDF grid:       {tsdf.shape[0]} x {tsdf.shape[1]} x {tsdf.shape[2]}"
          f" = {tsdf.size:,} voxels")
    print(f"  TSDF resolution: {dump['tsdf_resolution']:.4f} m")
    print(f"  Point cloud:     {len(dump['point_cloud']):,} points (pruned)")
    roi = dump["roi"]
    print(f"  ROI AABB:        [{roi[0,0]:.3f}, {roi[0,1]:.3f}, {roi[0,2]:.3f}]"
          f" -> [{roi[1,0]:.3f}, {roi[1,1]:.3f}, {roi[1,2]:.3f}] m")
    print(f"  Cameras:         {len(dump['cameras'])}")
    pose = dump["input_pose"]
    print(f"  Input pose:      pos=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})"
          f"  quat=({pose[3]:.3f}, {pose[4]:.3f}, {pose[5]:.3f}, {pose[6]:.3f})")
    twist = dump["input_twist"]
    print(f"  Input twist:     lin=({twist[0]:.3f}, {twist[1]:.3f}, {twist[2]:.3f})"
          f"  ang=({twist[3]:.3f}, {twist[4]:.3f}, {twist[5]:.3f})")
    print(f"  Grasp candidates: {n_total} total, {n_collision} with collision")
    if best_idx >= 0:
        gt = grasps["grasp_type"][best_idx]
        print(f"  Best grasp:      {GRASP_TYPE_NAMES.get(gt, '?')} (type {gt})")
        print(f"    closure={grasps['closure'][best_idx]:.4f}"
              f"  alignment={grasps['alignment'][best_idx]:.4f}"
              f"  force_closure={grasps['force_closure'][best_idx]:.4f}"
              f"  combined={grasps['combined'][best_idx]:.4f}"
              f"  prob={grasps['probability'][best_idx]:.4f}")
    print("=" * 52)


# ---------------------------------------------------------------------------
# PyVista visualization
# ---------------------------------------------------------------------------

def visualize_pyvista(dump: dict, args):
    """Full interactive 3D visualization using PyVista."""
    import pyvista as pv

    tsdf = dump["tsdf"]
    origin = dump["tsdf_origin"]
    res = dump["tsdf_resolution"]
    grasps = dump["grasps"]

    plotter = pv.Plotter(title="Grasp Preshaping Debug Viewer")
    plotter.set_background("white", top="lightgray")

    # --- TSDF isosurface ---
    if not args.no_tsdf and tsdf.size > 0:
        # Replace f32::MAX with NaN so contouring ignores unobserved voxels
        tsdf_vis = tsdf.astype(np.float64).copy()
        tsdf_vis[tsdf_vis > 1e10] = np.nan

        grid = pv.ImageData()
        grid.dimensions = np.array(tsdf_vis.shape) + 1
        grid.origin = origin
        grid.spacing = [res, res, res]
        grid.cell_data["distance"] = tsdf_vis.ravel(order="F")

        try:
            surface = grid.contour([0.0], scalars="distance")
            if surface.n_points > 0:
                plotter.add_mesh(
                    surface,
                    color="lightblue",
                    opacity=0.6,
                    smooth_shading=True,
                    label="TSDF isosurface (d=0)",
                )
        except Exception:
            # Contour may fail if no zero-crossing exists
            pass

    # --- Point cloud ---
    pc = dump["point_cloud"]
    if len(pc) > 0:
        cloud = pv.PolyData(pc)
        plotter.add_mesh(
            cloud,
            style="points",
            point_size=6,
            color="gray",
            opacity=0.35,
            render_points_as_spheres=True,
            label=f"Point cloud ({len(pc)} pts)",
        )

    # --- ROI bounding box ---
    roi = dump["roi"]
    bounds = [
        float(roi[0, 0]), float(roi[1, 0]),
        float(roi[0, 1]), float(roi[1, 1]),
        float(roi[0, 2]), float(roi[1, 2]),
    ]
    box = pv.Box(bounds=bounds)
    plotter.add_mesh(box, style="wireframe", color="orange", line_width=2, label="ROI AABB")

    # --- Camera positions ---
    for i, cam_pos in enumerate(dump["cameras"]):
        cam_sphere = pv.Sphere(radius=0.005, center=cam_pos)
        plotter.add_mesh(cam_sphere, color="purple", opacity=0.8, label=f"Camera {i}" if i < 3 else None)

    # --- Input hand pose ---
    pose = dump["input_pose"]
    hand_origin = pose[0:3]
    hand_axes_len = 0.015
    for axis_idx, (color, label_text) in enumerate(
        [("red", "X"), ("green", "Y"), ("blue", "Z")]
    ):
        direction = np.zeros(3)
        direction[axis_idx] = hand_axes_len
        # Rotate by quaternion (w, x, y, z) = (pose[6], pose[3], pose[4], pose[5])
        q_w, q_x, q_y, q_z = pose[6], pose[3], pose[4], pose[5]
        # Quaternion-rotate direction
        rotated = _quat_rotate(q_w, q_x, q_y, q_z, direction)
        arrow = pv.Arrow(start=hand_origin, direction=rotated, scale=1.0, shaft_radius=0.0006, tip_radius=0.0015)
        plotter.add_mesh(arrow, color=color, opacity=0.9)

    # --- Input twist arrow ---
    twist_lin = dump["input_twist"][0:3]
    if getattr(args, "show_twist", False) and np.linalg.norm(twist_lin) > 1e-6:
        twist_dir = twist_lin / np.linalg.norm(twist_lin) * 0.012
        twist_arrow = pv.Arrow(
            start=hand_origin,
            direction=twist_dir,
            scale=1.0,
            shaft_radius=0.0005,
            tip_radius=0.0012,
        )
        plotter.add_mesh(twist_arrow, color="yellow", opacity=0.7, label="Twist (linear)")

    # --- Grasp candidates ---
    n_grasps = len(grasps["combined"])
    threshold = args.threshold

    # Find best grasp
    best_idx = -1
    best_combined = -np.inf
    for i in range(n_grasps):
        if grasps["found_collision"][i] and grasps["combined"][i] > best_combined:
            best_combined = grasps["combined"][i]
            best_idx = i

    show_all_grasps = getattr(args, "show_all_grasps", False)
    grasp_indices = []
    if best_idx >= 0 and best_combined >= threshold:
        if show_all_grasps:
            grasp_indices = [
                i for i in range(n_grasps)
                if grasps["found_collision"][i] and grasps["combined"][i] >= threshold
            ]
        else:
            grasp_indices = [best_idx]

    displayed = 0
    for i in grasp_indices:
        gt = grasps["grasp_type"][i]
        color = GRASP_TYPE_COLORS.get(gt, "white")
        is_best = (i == best_idx)

        T = grasps["pose_4x4"][i]
        pos = T[:3, 3]

        # Draw small coordinate frame at grasp pose
        axes_len = 0.012 if is_best else 0.004
        opacity = 1.0 if is_best else 0.18
        shaft_r = 0.0012 if is_best else 0.00025
        tip_r = 0.003 if is_best else 0.0007

        if is_best:
            color = "gold"

        for axis_idx, ax_color in enumerate(["red", "green", "blue"]):
            direction = T[:3, axis_idx]
            if np.linalg.norm(direction) < 1e-10:
                continue
            arrow = pv.Arrow(
                start=pos,
                direction=direction * axes_len,
                scale=1.0,
                shaft_radius=shaft_r,
                tip_radius=tip_r,
            )
            plotter.add_mesh(arrow, color=ax_color, opacity=opacity)

        displayed += 1

    # --- Legend and controls ---
    plotter.add_legend(
        [
            ("TSDF surface", "lightblue"),
            ("Point cloud", "gray"),
            ("ROI box", "orange"),
            ("Best grasp", "gold"),
        ],
        size=(0.18, 0.15),
        loc="upper left",
    )

    plotter.add_text(
        f"Grasps shown: {displayed} (threshold >= {threshold:.2f})\n"
        f"Best: {GRASP_TYPE_NAMES.get(grasps['grasp_type'][best_idx], '?') if best_idx >= 0 else 'none'}"
        f"  combined={best_combined:.4f}" if best_idx >= 0 else "No valid grasps",
        position="upper_right",
        font_size=10,
        color="black",
    )

    plotter.add_axes()
    plotter.show()


def _quat_rotate(qw, qx, qy, qz, v):
    """Rotate vector v by quaternion (qw, qx, qy, qz)."""
    q = np.array([qx, qy, qz])
    t = 2.0 * np.cross(q, v)
    return v + qw * t + np.cross(q, t)


# ---------------------------------------------------------------------------
# Matplotlib fallback
# ---------------------------------------------------------------------------

def visualize_matplotlib(dump: dict, args):
    """Reduced 3D visualization using matplotlib (fallback)."""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")

    pc = dump["point_cloud"]
    if len(pc) > 0:
        ax.scatter(pc[:, 0], pc[:, 1], pc[:, 2], c="gray", s=2, alpha=0.3, label="Point cloud")

    roi = dump["roi"]
    _draw_box(ax, roi[0], roi[1], color="orange", label="ROI")

    grasps = dump["grasps"]
    threshold = args.threshold
    best_idx = -1
    best_combined = -np.inf
    for i in range(len(grasps["combined"])):
        if grasps["found_collision"][i] and grasps["combined"][i] > best_combined:
            best_combined = grasps["combined"][i]
            best_idx = i

    for i in range(len(grasps["combined"])):
        if not grasps["found_collision"][i] or grasps["combined"][i] < threshold:
            continue
        T = grasps["pose_4x4"][i]
        pos = T[:3, 3]
        is_best = (i == best_idx)
        gt = grasps["grasp_type"][i]
        color = "gold" if is_best else GRASP_TYPE_COLORS.get(gt, "blue")
        size = 40 if is_best else 10
        ax.scatter(*pos, c=color, s=size, marker="o" if is_best else ".")

    # Draw input pose
    pose = dump["input_pose"]
    ax.scatter(*pose[0:3], c="black", s=60, marker="^", label="Hand pose")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("Grasp Preshaping Debug Dump (matplotlib fallback)")
    ax.legend()
    plt.tight_layout()
    plt.show()


def _draw_box(ax, lo, hi, color="orange", label=None):
    """Draw a wireframe box on a matplotlib 3D axis."""
    xs = [lo[0], hi[0]]
    ys = [lo[1], hi[1]]
    zs = [lo[2], hi[2]]
    for x in xs:
        for y in ys:
            ax.plot([x, x], [y, y], zs, color=color, alpha=0.5, linewidth=0.8)
    for x in xs:
        for z in zs:
            ax.plot([x, x], ys, [z, z], color=color, alpha=0.5, linewidth=0.8)
    for y in ys:
        for z in zs:
            ax.plot(xs, [y, y], [z, z], color=color, alpha=0.5, linewidth=0.8)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Interactive 3D visualization of grasp preshaping debug dumps.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent("""\
            Examples:
              python visualize_grasp_debug.py data/debug/grasp_dump_20260424_153000.npz
              python visualize_grasp_debug.py dump.npz --threshold 0.5
              python visualize_grasp_debug.py dump.npz --no-tsdf
        """),
    )
    parser.add_argument("dump_path", help="Path to the .npz debug dump file")
    parser.add_argument(
        "--threshold", type=float, default=-np.inf,
        help="Minimum combined_score to display a grasp candidate (default: show all)",
    )
    parser.add_argument(
        "--show-all-grasps", action="store_true",
        help="Render all grasp candidates above the threshold instead of only the best one",
    )
    parser.add_argument(
        "--show-twist", action="store_true",
        help="Render the input linear twist as an arrow",
    )
    parser.add_argument(
        "--no-tsdf", action="store_true",
        help="Skip TSDF isosurface rendering (faster for large grids)",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.dump_path):
        print(f"Error: file not found: {args.dump_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Loading {args.dump_path} ...")
    dump = load_dump(args.dump_path)
    print_summary(dump, args.dump_path)

    try:
        import pyvista  # noqa: F401
        visualize_pyvista(dump, args)
    except ImportError:
        print("\npyvista not found — falling back to matplotlib.", file=sys.stderr)
        print("For the full interactive viewer: pip install pyvista\n", file=sys.stderr)
        visualize_matplotlib(dump, args)


if __name__ == "__main__":
    main()
