#!/usr/bin/env python3
"""Interactive 3D visualization of grasp preshaping debug dumps.

Usage:
    python visualize_grasp_debug.py <path_to_dump.npz> [options]

Dependencies:
    pip install numpy pyvista

If pyvista is unavailable, falls back to a matplotlib-based viewer with
reduced interactivity.

Keyboard shortcuts (PyVista viewer):
    t  - cycle TSDF mode: surface / points / off
    g  - toggle grasp display: best / all
    p  - toggle point cloud
    r  - toggle ROI box
    c  - toggle camera markers
    h  - toggle hand skeleton (best grasp, requires pinocchio + URDF)
    i  - toggle info text
    1  - filter grasps: cylindrical only
    2  - filter grasps: pinch only
    3  - filter grasps: lateral only
    0  - filter grasps: all types
    ?  - print help to console
"""

import argparse
import sys
import os
import textwrap

import numpy as np


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

GRASP_TYPE_NAMES = {1: "cylindrical", 2: "pinch", 3: "lateral"}
GRASP_TYPE_COLORS = {1: "#e76f51", 2: "#2a9d8f", 3: "#457b9d"}
GRASP_TYPE_SHORT = {1: "cyl", 2: "pinch", 3: "lat"}
F32_MAX = np.float32(np.finfo(np.float32).max)

# Truncation constant (must match config.rs TRUNCATION_CELLS).
TRUNCATION_CELLS = 4

# Hand skeleton: joint connectivity for the MIA hand (parent, child) pairs.
# Each pair defines a line segment between two joint frames.
HAND_JOINT_EDGES = [
    ("mia_palm", "mia_thumb_opp"),
    ("mia_thumb_opp", "mia_thumb_fle"),
    ("mia_palm", "mia_index_fle"),
    ("mia_index_fle", "mia_index_sensor"),
    ("mia_palm", "mia_middle_fle"),
    ("mia_middle_fle", "mia_middle_sensor"),
    ("mia_palm", "mia_ring_fle"),
    ("mia_palm", "mia_little_fle"),
]

# Finger group coloring for hand skeleton lines.
FINGER_COLORS = {
    "thumb": "#f4a261",
    "index": "#e76f51",
    "middle": "#2a9d8f",
    "ring": "#457b9d",
    "little": "#8d99ae",
}

# Map child joint name -> finger group for coloring.
_JOINT_TO_FINGER = {
    "mia_thumb_opp": "thumb",
    "mia_thumb_fle": "thumb",
    "mia_index_fle": "index",
    "mia_index_sensor": "index",
    "mia_middle_fle": "middle",
    "mia_middle_sensor": "middle",
    "mia_ring_fle": "ring",
    "mia_little_fle": "little",
}

# Try to import pinocchio + model for hand skeleton support.
_pin_hand_available = False
try:
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    if _script_dir not in sys.path:
        sys.path.insert(0, _script_dir)

    from model import (
        model as pin_model,
        data as pin_data,
        get_q_full,
        _q_full_with_thumb_mode,
        COLLISION_GEOMETRIES,
    )
    import pinocchio as pin
    _pin_hand_available = True
except (ImportError, ModuleNotFoundError, FileNotFoundError, Exception):
    pass


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_dump(path: str) -> dict:
    """Load a debug dump .npz and return a dict of parsed arrays."""
    data = np.load(path)

    meta = data["tsdf_metadata"].astype(np.float64)
    origin = meta[0:3]
    resolution = float(meta[3])
    shape = meta[4:7].astype(int)

    tsdf_flat = data["tsdf_volume"]
    # Rust stores TSDF flat as x + y*W + z*W*H (x-fastest).
    # Reconstruct with Fortran order so tsdf[x, y, z] matches Rust indexing.
    tsdf = tsdf_flat.reshape(shape, order="F")

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


def _quat_rotate(qw, qx, qy, qz, v):
    """Rotate vector v by quaternion (qw, qx, qy, qz)."""
    q = np.array([qx, qy, qz])
    t = 2.0 * np.cross(q, v)
    return v + qw * t + np.cross(q, t)


def _find_best_grasp(grasps, threshold=-np.inf):
    """Find the index and score of the best grasp above threshold."""
    n = len(grasps["combined"])
    best_idx = -1
    best_combined = -np.inf
    for i in range(n):
        if grasps["found_collision"][i] and grasps["combined"][i] >= threshold:
            if grasps["combined"][i] > best_combined:
                best_combined = grasps["combined"][i]
                best_idx = i
    return best_idx, best_combined


def _print_tsdf_index_diagnostics(tsdf, max_samples=6):
    """Print a few flat-index <-> (x,y,z) mappings for TSDF sanity checks."""
    w, h, d = tsdf.shape
    total = w * h * d
    if total == 0:
        return

    print("  TSDF index diagnostics (flat = x + y*W + z*W*H):")

    sample_indices = [0, min(total - 1, 1), min(total - 1, w), min(total - 1, w * h)]
    if total > 4:
        sample_indices += list(np.linspace(0, total - 1, num=max_samples, dtype=int))

    # Keep order stable while deduplicating.
    seen = set()
    unique_indices = []
    for idx in sample_indices:
        if idx not in seen:
            seen.add(idx)
            unique_indices.append(int(idx))

    for idx in unique_indices[:max_samples]:
        z = idx // (w * h)
        rem = idx - z * (w * h)
        y = rem // w
        x = rem % w
        v_xyz = float(tsdf[x, y, z])
        v_flat = float(tsdf.ravel(order="F")[idx])
        print(
            f"    flat[{idx:6d}] -> (x={x:4d}, y={y:4d}, z={z:4d})"
            f"  tsdf[x,y,z]={v_xyz:.4f}  ravelF[idx]={v_flat:.4f}"
        )


def print_summary(dump: dict, path: str):
    """Print a human-readable summary of the loaded dump."""
    tsdf = dump["tsdf"]
    grasps = dump["grasps"]
    n_total = len(grasps["combined"])
    n_collision = int(grasps["found_collision"].sum())

    best_idx, best_combined = _find_best_grasp(grasps)

    print("=" * 60)
    print(f"  Grasp Debug Dump: {os.path.basename(path)}")
    print("=" * 60)
    print(f"  TSDF grid:       {tsdf.shape[0]} x {tsdf.shape[1]} x {tsdf.shape[2]}"
          f" = {tsdf.size:,} voxels")
    print(f"  TSDF resolution: {dump['tsdf_resolution']:.4f} m")

    # TSDF statistics
    tsdf_vis = tsdf.astype(np.float64).copy()
    observed = tsdf_vis[tsdf_vis < 1e10]
    if len(observed) > 0:
        print(f"  TSDF observed:   {len(observed):,} voxels "
              f"(min={observed.min():.2f}, max={observed.max():.2f})")
        neg = (observed < 0).sum()
        zero = (np.abs(observed) < 0.5).sum()
        pos = (observed >= 0.5).sum()
        print(f"  TSDF sign:       {neg} negative, {zero} near-zero, {pos} positive")
    else:
        print(f"  TSDF observed:   NONE (all voxels unobserved)")

    print(f"  Point cloud:     {len(dump['point_cloud']):,} points (pruned)")
    roi = dump["roi"]
    print(f"  ROI AABB:        [{roi[0,0]:.3f}, {roi[0,1]:.3f}, {roi[0,2]:.3f}]"
          f" -> [{roi[1,0]:.3f}, {roi[1,1]:.3f}, {roi[1,2]:.3f}] m")
    print(f"  ROI size:        ({roi[1,0]-roi[0,0]:.3f}, {roi[1,1]-roi[0,1]:.3f},"
          f" {roi[1,2]-roi[0,2]:.3f}) m")
    print(f"  Cameras:         {len(dump['cameras'])}")
    pose = dump["input_pose"]
    print(f"  Input pose:      pos=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})"
          f"  quat=({pose[3]:.3f}, {pose[4]:.3f}, {pose[5]:.3f}, {pose[6]:.3f})")
    twist = dump["input_twist"]
    print(f"  Input twist:     lin=({twist[0]:.3f}, {twist[1]:.3f}, {twist[2]:.3f})"
          f"  ang=({twist[3]:.3f}, {twist[4]:.3f}, {twist[5]:.3f})")
    print(f"  Grasp candidates: {n_total} total, {n_collision} with collision")

    # Score distribution
    if n_collision > 0:
        collision_scores = grasps["combined"][grasps["found_collision"]]
        print(f"  Score range:     [{collision_scores.min():.4f}, {collision_scores.max():.4f}]")
        for gt_id in [1, 2, 3]:
            mask = grasps["grasp_type"] == gt_id
            col_mask = mask & grasps["found_collision"]
            n_type = col_mask.sum()
            if n_type > 0:
                print(f"    {GRASP_TYPE_NAMES[gt_id]:12s}: {n_type:4d} colliding,"
                      f" best={grasps['combined'][col_mask].max():.4f}")

    if best_idx >= 0:
        gt = grasps["grasp_type"][best_idx]
        print(f"  Best grasp:      {GRASP_TYPE_NAMES.get(gt, '?')} (type {gt})")
        print(f"    closure={grasps['closure'][best_idx]:.4f}"
              f"  alignment={grasps['alignment'][best_idx]:.4f}"
              f"  force_closure={grasps['force_closure'][best_idx]:.4f}"
              f"  combined={grasps['combined'][best_idx]:.4f}"
              f"  prob={grasps['probability'][best_idx]:.4f}")
    print("=" * 60)


# ---------------------------------------------------------------------------
# PyVista visualization
# ---------------------------------------------------------------------------

# Grasp display modes (simple toggle between best and all)
GRASP_MODES = ["best", "all"]

# TSDF display modes (cycle with 't' key)
TSDF_MODES = ["surface", "points", "off"]



def visualize_pyvista(dump: dict, args):
    """Full interactive 3D visualization using PyVista."""
    import pyvista as pv

    tsdf = dump["tsdf"]
    origin = dump["tsdf_origin"]
    res = dump["tsdf_resolution"]
    grasps = dump["grasps"]

    # ---- Scene scale estimation ----
    pc = dump["point_cloud"]
    roi = dump["roi"]
    roi_diag = float(np.linalg.norm(roi[1] - roi[0]))
    if len(pc) > 0:
        scene_extent = np.ptp(pc, axis=0).max()
    else:
        scene_extent = (roi[1] - roi[0]).max()
    scene_extent = max(scene_extent, 0.01)  # at least 1cm

    # Scale factors for markers proportional to scene.
    marker_size = min(scene_extent * 0.15, roi_diag * 0.08)
    marker_size = max(marker_size, 0.005)  # at least 5mm

    # ---- Mutable state for interactive toggles ----
    state = {
        "tsdf_mode": "surface" if not args.no_tsdf else "off",
        "grasp_mode": "best" if not args.show_all_grasps else "all",
        "show_pc": True,
        "show_roi": True,
        "show_cameras": True,
        "show_hand": False,
        "show_info": True,
        "grasp_type_filter": 0,  # 0 = all, 1/2/3 = specific type
    }

    plotter = pv.Plotter(title="Grasp Preshaping Debug Viewer")
    plotter.set_background("#1e1e2e", top="#2d2d44")

    # ---- Store actor references for toggling ----
    actor_groups = {
        "tsdf": [],
        "pc": [],
        "roi": [],
        "cameras": [],
        "hand_pose": [],
        "twist": [],
        "grasps": [],
        "hand_skeleton": [],
        "info": [],
    }

    # ==================================================================
    # TSDF voxel grid (multi-mode: surface / points / off)
    # ==================================================================
    def add_tsdf_actors():
        actor_groups["tsdf"].clear()
        mode = state["tsdf_mode"]
        if mode == "off" or tsdf.size == 0:
            return

        tsdf_vis = tsdf.astype(np.float64).copy()
        tsdf_vis[tsdf_vis > 1e10] = np.nan

        # Diagnostic info
        n_total = tsdf_vis.size
        n_observed = int(np.sum(~np.isnan(tsdf_vis)))
        observed_vals = tsdf_vis[~np.isnan(tsdf_vis)]
        if len(observed_vals) > 0:
            n_neg = int((observed_vals < 0).sum())
            n_zero = int((np.abs(observed_vals) < 0.5).sum())
            n_pos = int((observed_vals >= 0.5).sum())
            print(f"  TSDF [{mode}]: {tsdf_vis.shape} = {n_total} voxels, "
                  f"{n_observed} observed")
            print(f"    observed range: [{observed_vals.min():.2f}, {observed_vals.max():.2f}]")
            print(f"    sign: {n_neg} neg, {n_zero} near-zero, {n_pos} pos")
            if args.debug_tsdf_indexing:
                _print_tsdf_index_diagnostics(tsdf_vis)
        else:
            print(f"  TSDF [{mode}]: {tsdf_vis.shape} = {n_total} voxels, NONE observed")
            return

        # Build ImageData grid.
        # VTK expects x-fastest cell ordering for ImageData cell arrays.
        # TSDF was reconstructed as tsdf[x, y, z], so flatten in Fortran order
        # (first axis fastest) to match VTK's expected memory layout.
        grid = pv.ImageData()
        grid.dimensions = np.array(tsdf_vis.shape) + 1
        grid.origin = origin
        grid.spacing = [res, res, res]
        grid.cell_data["distance"] = tsdf_vis.ravel(order="F")

        if mode == "surface":
            # Show only voxels near the zero-crossing (thin surface band).
            clipped = grid.threshold(
                value=[-1.5, 1.5],
                scalars="distance",
            )
            if clipped.n_cells == 0:
                print("  TSDF [surface]: no voxels near zero-crossing")
                return
            print(f"  TSDF [surface]: rendering {clipped.n_cells} surface voxels")
            actor = plotter.add_mesh(
                clipped,
                scalars="distance",
                cmap="coolwarm",
                clim=[-TRUNCATION_CELLS, TRUNCATION_CELLS],
                opacity=0.7,
                show_scalar_bar=True,
                scalar_bar_args={
                    "title": "TSDF distance (cells)",
                    "position_x": 0.05,
                    "position_y": 0.05,
                    "width": 0.3,
                    "height": 0.05,
                },
                label="TSDF surface",
                show_edges=False,
            )
            actor_groups["tsdf"].append(actor)

        elif mode == "points":
            # Show all observed voxels as points — zero occlusion.
            clipped = grid.threshold(
                value=[-TRUNCATION_CELLS - 0.5, TRUNCATION_CELLS + 0.5],
                scalars="distance",
            )
            if clipped.n_cells == 0:
                print("  TSDF [points]: no observed voxels within truncation band")
                return
            print(f"  TSDF [points]: rendering {clipped.n_cells} voxels as points")
            # Extract cell centers for point rendering.
            centers = clipped.cell_centers()
            actor = plotter.add_mesh(
                centers,
                scalars="distance",
                cmap="coolwarm",
                clim=[-TRUNCATION_CELLS, TRUNCATION_CELLS],
                style="points",
                point_size=6,
                render_points_as_spheres=True,
                opacity=0.8,
                show_scalar_bar=True,
                scalar_bar_args={
                    "title": "TSDF distance (cells)",
                    "position_x": 0.05,
                    "position_y": 0.05,
                    "width": 0.3,
                    "height": 0.05,
                },
                label="TSDF points",
            )
            actor_groups["tsdf"].append(actor)

    if state["tsdf_mode"] != "off":
        add_tsdf_actors()

    # ==================================================================
    # Point cloud
    # ==================================================================
    def add_pc_actors():
        actor_groups["pc"].clear()
        if len(pc) > 0:
            cloud = pv.PolyData(pc)
            actor = plotter.add_mesh(
                cloud,
                style="points",
                point_size=8,
                color="white",
                opacity=0.7,
                render_points_as_spheres=True,
                label=f"Point cloud ({len(pc)} pts)",
            )
            actor_groups["pc"].append(actor)

    add_pc_actors()

    # ==================================================================
    # ROI bounding box
    # ==================================================================
    def add_roi_actors():
        actor_groups["roi"].clear()
        roi = dump["roi"]
        bounds = [
            float(roi[0, 0]), float(roi[1, 0]),
            float(roi[0, 1]), float(roi[1, 1]),
            float(roi[0, 2]), float(roi[1, 2]),
        ]
        box = pv.Box(bounds=bounds)
        actor = plotter.add_mesh(
            box, style="wireframe", color="orange",
            line_width=3, label="ROI AABB",
        )
        actor_groups["roi"].append(actor)

    add_roi_actors()

    # ==================================================================
    # Camera positions
    # ==================================================================
    def add_camera_actors():
        actor_groups["cameras"].clear()
        cam_radius = scene_extent * 0.05
        for i, cam_pos in enumerate(dump["cameras"]):
            cam_sphere = pv.Sphere(radius=cam_radius, center=cam_pos)
            actor = plotter.add_mesh(
                cam_sphere, color="mediumpurple", opacity=0.9,
                label=f"Camera {i}" if i < 3 else None,
            )
            actor_groups["cameras"].append(actor)

            # Draw a small direction indicator toward ROI center
            roi_center = (dump["roi"][0] + dump["roi"][1]) / 2.0
            direction = roi_center - cam_pos
            d_norm = np.linalg.norm(direction)
            if d_norm > 1e-6:
                direction = direction / d_norm * cam_radius * 4
                cone = pv.Cone(
                    center=cam_pos + direction,
                    direction=direction,
                    height=cam_radius * 2,
                    radius=cam_radius * 0.8,
                )
                actor2 = plotter.add_mesh(cone, color="mediumpurple", opacity=0.7)
                actor_groups["cameras"].append(actor2)

    add_camera_actors()

    # ==================================================================
    # Input hand pose (larger, more visible)
    # ==================================================================
    pose = dump["input_pose"]
    hand_origin = pose[0:3].astype(np.float64)
    hand_axes_len = scene_extent * 0.12
    q_w, q_x, q_y, q_z = pose[6], pose[3], pose[4], pose[5]

    arrow_shaft = marker_size * 0.06
    arrow_tip = marker_size * 0.12

    for axis_idx, (color, label_text) in enumerate(
        [("red", "X"), ("lime", "Y"), ("dodgerblue", "Z")]
    ):
        direction = np.zeros(3)
        direction[axis_idx] = hand_axes_len
        rotated = _quat_rotate(q_w, q_x, q_y, q_z, direction)
        arrow = pv.Arrow(
            start=hand_origin, direction=rotated, scale=1.0,
            shaft_radius=arrow_shaft * 0.5,
            tip_radius=arrow_tip * 0.5,
        )
        actor = plotter.add_mesh(arrow, color=color, opacity=0.9)
        actor_groups["hand_pose"].append(actor)

    # Hand origin marker
    hand_marker = pv.Sphere(radius=scene_extent * 0.015, center=hand_origin)
    actor = plotter.add_mesh(hand_marker, color="white", opacity=0.9)
    actor_groups["hand_pose"].append(actor)

    # ==================================================================
    # Input twist arrow
    # ==================================================================
    twist_lin = dump["input_twist"][0:3]
    if getattr(args, "show_twist", False) and np.linalg.norm(twist_lin) > 1e-6:
        twist_dir = twist_lin / np.linalg.norm(twist_lin) * marker_size
        twist_arrow = pv.Arrow(
            start=hand_origin, direction=twist_dir, scale=1.0,
            shaft_radius=arrow_shaft * 0.5,
            tip_radius=arrow_tip * 0.5,
        )
        actor = plotter.add_mesh(twist_arrow, color="yellow", opacity=0.8,
                                 label="Twist (linear)")
        actor_groups["twist"].append(actor)

    # ==================================================================
    # Grasp candidates
    # ==================================================================
    n_grasps = len(grasps["combined"])
    threshold = args.threshold

    best_idx, best_combined = _find_best_grasp(grasps, threshold)

    def get_grasp_indices():
        """Compute which grasp indices to show based on current state."""
        candidates = []
        for i in range(n_grasps):
            if not grasps["found_collision"][i]:
                continue
            if grasps["combined"][i] < threshold:
                continue
            # Type filter
            if state["grasp_type_filter"] != 0:
                if grasps["grasp_type"][i] != state["grasp_type_filter"]:
                    continue
            candidates.append(i)

        if state["grasp_mode"] == "best":
            if best_idx >= 0 and best_idx in candidates:
                return [best_idx]
            elif candidates:
                return [candidates[0]]  # best available
            return []

        # "all"
        return candidates

    def add_grasp_actors():
        actor_groups["grasps"].clear()

        indices = get_grasp_indices()
        if not indices:
            return

        # Compute score range for size scaling
        scores = [grasps["combined"][i] for i in indices]
        score_min = min(scores) if scores else 0
        score_max = max(scores) if scores else 1
        score_range = score_max - score_min if score_max > score_min else 1.0

        # Collect positions, colors, sizes, and Z-axis directions for batch rendering.
        positions = []
        z_dirs = []
        z_lens = []
        point_colors = []
        point_sizes = []

        for rank, i in enumerate(indices):
            gt = grasps["grasp_type"][i]
            is_best = (i == best_idx)
            T = grasps["pose_4x4"][i]
            pos = T[:3, 3]
            R = T[:3, :3]
            score = grasps["combined"][i]

            score_norm = (score - score_min) / score_range if score_range > 0 else 1.0

            positions.append(pos)

            # Z-axis direction (approach direction)
            z_dir = R[:, 2]
            z_dirs.append(z_dir)
            z_lens.append(marker_size * (1.2 if is_best else 0.4 + 0.6 * score_norm))

            # Color by grasp type, gold for best
            if is_best:
                point_colors.append([1.0, 0.84, 0.0])  # gold
                point_sizes.append(20)
            else:
                hex_color = GRASP_TYPE_COLORS.get(gt, "#ffffff")
                r_c = int(hex_color[1:3], 16) / 255.0
                g_c = int(hex_color[3:5], 16) / 255.0
                b_c = int(hex_color[5:7], 16) / 255.0
                point_colors.append([r_c, g_c, b_c])
                point_sizes.append(6 + 10 * score_norm)

        positions = np.array(positions)
        point_colors = np.array(point_colors)

        # Render all grasp positions as a single point cloud.
        grasp_cloud = pv.PolyData(positions)
        grasp_cloud["colors"] = point_colors
        actor = plotter.add_mesh(
            grasp_cloud,
            scalars="colors",
            rgb=True,
            style="points",
            point_size=12,
            render_points_as_spheres=True,
            label=f"Grasps ({len(indices)})",
        )
        actor_groups["grasps"].append(actor)

        # Render Z-axis arrows as lines.
        for idx in range(len(positions)):
            start = positions[idx]
            end = start + z_dirs[idx] * z_lens[idx]
            line = pv.Line(start, end)
            is_best = (indices[idx] == best_idx)
            line_color = "gold" if is_best else GRASP_TYPE_COLORS.get(
                grasps["grasp_type"][indices[idx]], "white"
            )
            actor = plotter.add_mesh(
                line, color=line_color,
                line_width=3 if is_best else 1.5,
                opacity=0.9 if is_best else 0.6,
            )
            actor_groups["grasps"].append(actor)

    add_grasp_actors()

    # ==================================================================
    # Hand skeleton for best grasp (URDF-based, requires pinocchio)
    # ==================================================================
    if _pin_hand_available:
        # Pre-resolve frame IDs from names.
        # Pinocchio preserves frames for all links (including fixed-joint links),
        # so we use getFrameId() instead of getJointId().
        _frame_edges_resolved = []
        for parent_name, child_name in HAND_JOINT_EDGES:
            pid = pin_model.getFrameId(parent_name)
            cid = pin_model.getFrameId(child_name)
            if pid < pin_model.nframes and cid < pin_model.nframes:
                _frame_edges_resolved.append((pid, cid, child_name))
            else:
                print(f"  WARNING: frame not found: parent={parent_name} (id={pid}), child={child_name} (id={cid})")
        print(f"  Hand skeleton: pinocchio + URDF loaded ({len(_frame_edges_resolved)} frame edges)")
    else:
        _frame_edges_resolved = []
        print("  Hand skeleton: unavailable (pinocchio or URDF not found)")

    def add_hand_skeleton():
        actor_groups["hand_skeleton"].clear()
        if not _pin_hand_available or best_idx < 0:
            return
        if not grasps["found_collision"][best_idx]:
            return

        gt = grasps["grasp_type"][best_idx]
        T = grasps["pose_4x4"][best_idx]
        closure = grasps["closure"][best_idx]

        # Build q_active from closure amount.
        # Cylindrical (1) and pinch (2) use thumb opposition mode 1 (abduction).
        # Lateral (3) uses thumb opposition mode 0 (adduction).
        q_active = np.array([closure, closure, closure], dtype=float)
        if gt == 3:  # lateral
            thumb_mode = 0.0
        else:  # cylindrical or pinch
            thumb_mode = 1.0

        q_full = _q_full_with_thumb_mode(q_active, thumb_mode)
        pin.forwardKinematics(pin_model, pin_data, q_full)
        pin.updateFramePlacements(pin_model, pin_data)

        # Get frame positions in local hand frame.
        frame_positions = {}
        for pid, cid, _ in _frame_edges_resolved:
            if pid not in frame_positions:
                frame_positions[pid] = pin_data.oMf[pid].translation.copy().astype(np.float64)
            if cid not in frame_positions:
                frame_positions[cid] = pin_data.oMf[cid].translation.copy().astype(np.float64)

        # Transform to world frame via grasp pose.
        for fid in frame_positions:
            p_local = frame_positions[fid]
            frame_positions[fid] = T[:3, :3] @ p_local + T[:3, 3]

        # Draw lines between connected frames and spheres at frame positions.
        drawn_frames = set()
        for pid, cid, child_name in _frame_edges_resolved:
            p_pos = frame_positions.get(pid)
            c_pos = frame_positions.get(cid)
            if p_pos is None or c_pos is None:
                continue

            finger = _JOINT_TO_FINGER.get(child_name, "index")
            line_color = FINGER_COLORS.get(finger, "#f4a261")

            # Line between parent and child
            line = pv.Line(p_pos, c_pos)
            actor = plotter.add_mesh(
                line, color=line_color, line_width=4, opacity=0.9,
            )
            actor_groups["hand_skeleton"].append(actor)

            # Sphere at child frame
            if cid not in drawn_frames:
                sphere = pv.Sphere(radius=scene_extent * 0.015, center=c_pos)
                actor = plotter.add_mesh(sphere, color=line_color, opacity=0.85)
                actor_groups["hand_skeleton"].append(actor)
                drawn_frames.add(cid)

            # Sphere at parent frame (if not drawn yet)
            if pid not in drawn_frames:
                sphere = pv.Sphere(radius=scene_extent * 0.015, center=p_pos)
                actor = plotter.add_mesh(sphere, color=line_color, opacity=0.85)
                actor_groups["hand_skeleton"].append(actor)
                drawn_frames.add(pid)

    # ==================================================================
    # Info text
    # ==================================================================
    def update_info_text():
        actor_groups["info"].clear()

        mode_str = state["grasp_mode"]
        if state["grasp_type_filter"] != 0:
            mode_str += f" ({GRASP_TYPE_SHORT.get(state['grasp_type_filter'], '?')})"

        indices = get_grasp_indices()
        n_shown = len(indices)

        tsdf_str = state["tsdf_mode"]

        lines = [
            f"Grasps: {n_shown} shown (mode={mode_str}, threshold>={threshold:.2f})",
        ]
        if best_idx >= 0:
            gt = grasps["grasp_type"][best_idx]
            lines.append(
                f"Best: {GRASP_TYPE_NAMES.get(gt, '?')}  combined={best_combined:.4f}"
            )
        else:
            lines.append("Best: none")

        lines.append(f"TSDF: {tsdf_str}")
        lines.append("Keys: t=TSDF  g=grasps  p=cloud  r=ROI  c=cam  h=hand  ?=help")

        text = "\n".join(lines)
        actor = plotter.add_text(
            text,
            position="upper_right",
            font_size=9,
            color="white",
            name="_info_text",
        )
        actor_groups["info"].append(actor)

    update_info_text()

    # ==================================================================
    # Legend
    # ==================================================================
    legend_entries = [
        ("TSDF voxels (coolwarm)", "cyan"),
        ("Point cloud", "white"),
        ("ROI box", "orange"),
        ("Best grasp", "gold"),
        ("Cylindrical", GRASP_TYPE_COLORS[1]),
        ("Pinch", GRASP_TYPE_COLORS[2]),
        ("Lateral", GRASP_TYPE_COLORS[3]),
        ("Hand pose", "white"),
    ]
    plotter.add_legend(legend_entries, size=(0.18, 0.22), loc="upper left",
                       face="rectangle")

    # ==================================================================
    # Auto-frame camera
    # ==================================================================
    if len(pc) > 0:
        center = pc.mean(axis=0)
    else:
        center = (dump["roi"][0] + dump["roi"][1]) / 2.0

    view_dist = scene_extent * 2.5
    plotter.camera_position = [
        center + np.array([view_dist * 0.5, -view_dist * 0.7, view_dist * 0.5]),
        center,
        [0, 0, 1],
    ]

    # ==================================================================
    # Keyboard interaction
    # ==================================================================
    def toggle_actors(group_name):
        for actor in actor_groups[group_name]:
            actor.SetVisibility(not actor.GetVisibility())
        plotter.render()

    def rebuild_grasps():
        """Remove and re-add grasp actors."""
        for actor in actor_groups["grasps"]:
            plotter.remove_actor(actor)
        actor_groups["grasps"].clear()
        add_grasp_actors()
        update_info_text()
        plotter.render()

    def rebuild_tsdf():
        """Remove and re-add TSDF actors."""
        for actor in actor_groups["tsdf"]:
            plotter.remove_actor(actor)
        actor_groups["tsdf"].clear()
        add_tsdf_actors()
        update_info_text()
        plotter.render()

    def on_key_t():
        """Cycle TSDF display mode: surface -> points -> off."""
        current = state["tsdf_mode"]
        current_idx = TSDF_MODES.index(current) if current in TSDF_MODES else 0
        next_idx = (current_idx + 1) % len(TSDF_MODES)
        state["tsdf_mode"] = TSDF_MODES[next_idx]
        print(f"  TSDF mode: {state['tsdf_mode']}")
        rebuild_tsdf()

    def on_key_g():
        """Toggle grasp display mode between best and all."""
        current = state["grasp_mode"]
        state["grasp_mode"] = "all" if current == "best" else "best"
        print(f"  Grasp mode: {state['grasp_mode']}")
        rebuild_grasps()

    def on_key_p():
        toggle_actors("pc")

    def on_key_r():
        toggle_actors("roi")

    def on_key_c():
        toggle_actors("cameras")

    def on_key_h():
        """Toggle hand skeleton."""
        if actor_groups["hand_skeleton"]:
            toggle_actors("hand_skeleton")
        else:
            add_hand_skeleton()
            plotter.render()

    def on_key_i():
        toggle_actors("info")

    def on_key_0():
        state["grasp_type_filter"] = 0
        print("  Filter: all grasp types")
        rebuild_grasps()

    def on_key_1():
        state["grasp_type_filter"] = 1
        print("  Filter: cylindrical only")
        rebuild_grasps()

    def on_key_2():
        state["grasp_type_filter"] = 2
        print("  Filter: pinch only")
        rebuild_grasps()

    def on_key_3():
        state["grasp_type_filter"] = 3
        print("  Filter: lateral only")
        rebuild_grasps()

    def on_key_help():
        print("\n  === Keyboard Shortcuts ===")
        print("  t  - cycle TSDF mode: surface / points / off")
        print("  g  - toggle grasp display: best / all")
        print("  p  - toggle point cloud")
        print("  r  - toggle ROI box")
        print("  c  - toggle camera markers")
        print("  h  - toggle hand skeleton (best grasp)")
        print("  i  - toggle info text")
        print("  1  - filter: cylindrical only")
        print("  2  - filter: pinch only")
        print("  3  - filter: lateral only")
        print("  0  - filter: all types")
        print("  ?  - this help\n")

    plotter.add_key_event("t", on_key_t)
    plotter.add_key_event("g", on_key_g)
    plotter.add_key_event("p", on_key_p)
    plotter.add_key_event("r", on_key_r)
    plotter.add_key_event("c", on_key_c)
    plotter.add_key_event("h", on_key_h)
    plotter.add_key_event("i", on_key_i)
    plotter.add_key_event("0", on_key_0)
    plotter.add_key_event("1", on_key_1)
    plotter.add_key_event("2", on_key_2)
    plotter.add_key_event("3", on_key_3)
    plotter.add_key_event("question", on_key_help)

    plotter.add_axes()
    plotter.show()


# ---------------------------------------------------------------------------
# Matplotlib fallback
# ---------------------------------------------------------------------------

def visualize_matplotlib(dump: dict, args):
    """Reduced 3D visualization using matplotlib (fallback)."""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection="3d")

    pc = dump["point_cloud"]
    if len(pc) > 0:
        ax.scatter(pc[:, 0], pc[:, 1], pc[:, 2], c="gray", s=3, alpha=0.5,
                   label="Point cloud")

    roi = dump["roi"]
    _draw_box(ax, roi[0], roi[1], color="orange", label="ROI")

    grasps = dump["grasps"]
    threshold = args.threshold
    best_idx, best_combined = _find_best_grasp(grasps)

    for i in range(len(grasps["combined"])):
        if not grasps["found_collision"][i] or grasps["combined"][i] < threshold:
            continue
        T = grasps["pose_4x4"][i]
        pos = T[:3, 3]
        is_best = (i == best_idx)
        gt = grasps["grasp_type"][i]
        color = "gold" if is_best else GRASP_TYPE_COLORS.get(gt, "blue")
        size = 60 if is_best else 15
        marker = "o" if is_best else "."
        ax.scatter(*pos, c=color, s=size, marker=marker, zorder=5 if is_best else 3)

        # Draw approach direction line
        approach = -T[:3, 2]  # -Z
        line_len = 0.02 if is_best else 0.008
        end = pos + approach * line_len
        ax.plot([pos[0], end[0]], [pos[1], end[1]], [pos[2], end[2]],
                color=color, linewidth=2 if is_best else 0.5, alpha=0.7)

    # Draw input pose
    pose = dump["input_pose"]
    ax.scatter(*pose[0:3], c="black", s=80, marker="^", label="Hand pose")

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
              python visualize_grasp_debug.py dump.npz --show-all-grasps

            Keyboard shortcuts (PyVista):
              t  cycle TSDF: surface/points/off
              g  toggle grasp mode (best/all)
              p  toggle point cloud      r  toggle ROI box
              c  toggle cameras          h  toggle hand skeleton
              i  toggle info text        0-3  filter grasp types
              ?  print help
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
        help="Skip TSDF voxel grid rendering",
    )
    parser.add_argument(
        "--debug-tsdf-indexing", action="store_true",
        help="Print TSDF flat-index to (x,y,z) diagnostics in the console",
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
        print("\npyvista not found - falling back to matplotlib.", file=sys.stderr)
        print("For the full interactive viewer: pip install pyvista\n", file=sys.stderr)
        visualize_matplotlib(dump, args)


if __name__ == "__main__":
    main()
