#!/usr/bin/env python3
"""Interactive 3D visualization of grasp preshaping debug dumps.

Usage:
    python visualize_grasp_debug.py <path_to_dump.npz> [options]

Dependencies:
    pip install numpy pyvista

If pyvista is unavailable, falls back to a matplotlib-based viewer with
reduced interactivity.

Keyboard shortcuts (PyVista viewer):
    t  - cycle TSDF mode: surface / points / signs / off
    g  - toggle grasp display: best / all
    p  - toggle point cloud
    r  - toggle ROI box
    c  - toggle camera markers
    h  - toggle hand skeleton (unified)
    k  - toggle LUT contact points
    b  - toggle superquadric mesh
    i  - toggle info text
    +  - next SMC iteration filter
    -  - previous SMC iteration filter
    *  - show all iterations (reset filter)
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

import warnings

warnings.filterwarnings("ignore", message=".*pickpoint.*")

import numpy as np


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

GRASP_TYPE_NAMES = {1: "cylindrical", 2: "pinch", 3: "lateral"}
GRASP_TYPE_COLORS = {1: "#e76f51", 2: "#2a9d8f", 3: "#457b9d"}
GRASP_TYPE_SHORT = {1: "cyl", 2: "pinch", 3: "lat"}
F32_MAX = np.float32(np.finfo(np.float32).max)

# Superquadric template names (must match superquadric.rs TEMPLATES order).
SQ_TEMPLATE_NAMES = ["sphere", "box", "cylinder"]

# Truncation constant (must match runtime_config.rs defaults::TRUNCATION_CELLS).
TRUNCATION_CELLS = 4

# Surface band width for TSDF surface mode (cells from zero-crossing).
# This is a visualization parameter, independent of the truncation distance.
TSDF_SURFACE_BAND = 2.5

# Unified hand skeleton: one compact topology for both best and all modes.
# Each entry is (finger name, base frame name, tip contact name).
_HAND_SKELETON_SPECS = [
    ("thumb", "mia_thumb_opp", "ThumbAddTip"),
    ("index", "mia_index_fle", "IndexTip"),
    ("middle", "mia_middle_fle", "MiddleTip"),
    ("ring", "mia_ring_fle", "RingTip"),
    ("little", "mia_little_fle", "LittleTip"),
]

# Finger group coloring for hand skeleton lines/markers.
FINGER_COLORS = {
    "thumb": "#f4a261",
    "index": "#e76f51",
    "middle": "#2a9d8f",
    "ring": "#457b9d",
    "little": "#8d99ae",
}

# Map finger name -> base frame for FK lookup.
_HAND_BASE_FRAMES = {finger: base for finger, base, _ in _HAND_SKELETON_SPECS}

# Map finger name -> contact tip name.
_HAND_TIP_CONTACTS = {finger: tip for finger, _, tip in _HAND_SKELETON_SPECS}

# Grasp-type-specific score contact names (from planner.rs grasp specs).
# These are the contacts used for scoring; all others are sweep-only.
_GRASP_SCORE_CONTACTS = {
    1: {  # cylindrical
        "ThumbAddPip", "ThumbAddDip", "ThumbAddTip",
        "IndexMcp", "IndexDip", "IndexPip", "IndexTip",
        "MiddleMcp", "MiddlePip", "MiddleDip", "MiddleTip",
        "RingDip", "RingPip", "RingTip",
        "LittleDip", "LittlePip", "LittleTip",
        "PalmProxUlna", "PalmProxRadi", "PalmDistUlna", "PalmDistRadi",
    },
    2: {  # pinch
        "ThumbAddTip", "IndexTip",
    },
    3: {  # lateral
        "ThumbAddTip",
        "IndexMcpSide", "IndexDipSide", "IndexPipSide", "IndexTipSide",
    },
}

# Finger group colors for contact points (same as hand skeleton).
_CONTACT_FINGER_COLORS = {
    "thumb": "#f4a261",
    "index": "#e76f51",
    "middle": "#2a9d8f",
    "ring": "#457b9d",
    "little": "#8d99ae",
    "palm": "#6a994e",
}

# Per-group contact line topology.  Each list defines the chain(s) of
# contact names that should be connected by lines within a finger group.
# The ordering follows the kinematic chain (proximal -> distal).
_CONTACT_LINES = {
    "index": [
        ["IndexMcp", "IndexPip", "IndexDip", "IndexTip"],       # palmar rail
        ["IndexMcpSide", "IndexPipSide", "IndexDipSide", "IndexTipSide"],  # lateral rail
        ["IndexMcp", "IndexMcpSide"],      # cross-links at each joint
        ["IndexDip", "IndexDipSide"],
        ["IndexPip", "IndexPipSide"],
        ["IndexTip", "IndexTipSide"],
    ],
    "middle": [
        ["MiddleMcp", "MiddlePip", "MiddleDip", "MiddleTip"],
    ],
    "ring": [
        ["RingPip", "RingDip", "RingTip"],
    ],
    "little": [
        ["LittlePip", "LittleDip", "LittleTip"],
    ],
    "thumb": [
        ["ThumbAddPip", "ThumbAddDip", "ThumbAddTip"],
    ],
    "palm": [
        ["PalmProxUlna", "PalmDistUlna"],     # ulnar rail
        ["PalmProxRadi", "PalmDistRadi"],     # radial rail
        ["PalmProxUlna", "PalmProxRadi"],     # cross-links
        ["PalmDistUlna", "PalmDistRadi"],
        ["PalmProxRadi", "ThumbAddPip"],      # Fingers
        ["PalmProxRadi", "IndexMcp"],
        ["PalmProxRadi", "MiddleMcp"],
        ["PalmProxUlna", "RingPip"],
        ["PalmProxUlna", "LittlePip"],
        ["LittlePip", "RingPip", "MiddleMcp", "IndexMcp", "ThumbAddPip"],
    ],
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
        get_sampled_contact_transforms,
        CONTACT_DEFINITIONS,
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

    # Backward-compatible: handle 24-col through 29-col formats
    raw_len = len(data["scored_grasps"])
    if raw_len > 0:
        if raw_len % 29 == 0:
            row_len = 29
        elif raw_len % 28 == 0:
            row_len = 28
        elif raw_len % 27 == 0:
            row_len = 27
        elif raw_len % 26 == 0:
            row_len = 26
        elif raw_len % 25 == 0:
            row_len = 25
        else:
            row_len = 24
        grasps_raw = data["scored_grasps"].reshape(-1, row_len)
    else:
        row_len = 29
        grasps_raw = np.zeros((0, row_len))

    if row_len == 29:
        grasps = {
            "sample_index": grasps_raw[:, 0].astype(int),
            "grasp_type": grasps_raw[:, 1].astype(int),
            "closure": grasps_raw[:, 2],
            "alignment": grasps_raw[:, 3],
            "force_closure": grasps_raw[:, 4],
            "contact_count_score": grasps_raw[:, 5],
            "contact_score": grasps_raw[:, 6],
            "active_contact_count": grasps_raw[:, 7].astype(int),
            "found_collision": grasps_raw[:, 8] > 0.5,
            "combined": grasps_raw[:, 9],
            "probability": grasps_raw[:, 10],
            "wrist_rotation": grasps_raw[:, 11],
            "smc_iteration": grasps_raw[:, 12].astype(int),
            "pose_4x4": grasps_raw[:, 13:29].reshape(-1, 4, 4),
        }
    elif row_len == 28:
        grasps = {
            "sample_index": grasps_raw[:, 0].astype(int),
            "grasp_type": grasps_raw[:, 1].astype(int),
            "closure": grasps_raw[:, 2],
            "alignment": grasps_raw[:, 3],
            "force_closure": grasps_raw[:, 4],
            "contact_count_score": grasps_raw[:, 5],
            "contact_score": grasps_raw[:, 6],
            "active_contact_count": grasps_raw[:, 7].astype(int),
            "found_collision": grasps_raw[:, 8] > 0.5,
            "combined": grasps_raw[:, 9],
            "probability": grasps_raw[:, 10],
            "wrist_rotation": grasps_raw[:, 11],
            "smc_iteration": np.zeros(len(grasps_raw), dtype=int),
            "pose_4x4": grasps_raw[:, 12:28].reshape(-1, 4, 4),
        }
    elif row_len == 27:
        grasps = {
            "sample_index": grasps_raw[:, 0].astype(int),
            "grasp_type": grasps_raw[:, 1].astype(int),
            "closure": grasps_raw[:, 2],
            "alignment": grasps_raw[:, 3],
            "force_closure": grasps_raw[:, 4],
            "contact_count_score": grasps_raw[:, 5],
            "contact_score": np.zeros(len(grasps_raw)),
            "active_contact_count": grasps_raw[:, 6].astype(int),
            "found_collision": grasps_raw[:, 7] > 0.5,
            "combined": grasps_raw[:, 8],
            "probability": grasps_raw[:, 9],
            "wrist_rotation": grasps_raw[:, 10],
            "smc_iteration": np.zeros(len(grasps_raw), dtype=int),
            "pose_4x4": grasps_raw[:, 11:27].reshape(-1, 4, 4),
        }
    elif row_len == 26:
        grasps = {
            "sample_index": grasps_raw[:, 0].astype(int),
            "grasp_type": grasps_raw[:, 1].astype(int),
            "closure": grasps_raw[:, 2],
            "alignment": grasps_raw[:, 3],
            "force_closure": grasps_raw[:, 4],
            "contact_count_score": grasps_raw[:, 5],
            "contact_score": np.zeros(len(grasps_raw)),
            "active_contact_count": np.zeros(len(grasps_raw), dtype=int),
            "found_collision": grasps_raw[:, 6] > 0.5,
            "combined": grasps_raw[:, 7],
            "probability": grasps_raw[:, 8],
            "wrist_rotation": grasps_raw[:, 9],
            "smc_iteration": np.zeros(len(grasps_raw), dtype=int),
            "pose_4x4": grasps_raw[:, 10:26].reshape(-1, 4, 4),
        }
    elif row_len == 25:
        grasps = {
            "sample_index": grasps_raw[:, 0].astype(int),
            "grasp_type": grasps_raw[:, 1].astype(int),
            "closure": grasps_raw[:, 2],
            "alignment": grasps_raw[:, 3],
            "force_closure": grasps_raw[:, 4],
            "contact_count_score": grasps_raw[:, 5],
            "contact_score": np.zeros(len(grasps_raw)),
            "active_contact_count": np.zeros(len(grasps_raw), dtype=int),
            "found_collision": grasps_raw[:, 6] > 0.5,
            "combined": grasps_raw[:, 7],
            "probability": grasps_raw[:, 8],
            "wrist_rotation": np.zeros(len(grasps_raw)),
            "smc_iteration": np.zeros(len(grasps_raw), dtype=int),
            "pose_4x4": grasps_raw[:, 9:25].reshape(-1, 4, 4),
        }
    else:
        # Legacy 24-column format
        grasps = {
            "sample_index": grasps_raw[:, 0].astype(int),
            "grasp_type": grasps_raw[:, 1].astype(int),
            "closure": grasps_raw[:, 2],
            "alignment": grasps_raw[:, 3],
            "force_closure": grasps_raw[:, 4],
            "contact_count_score": np.zeros(len(grasps_raw)),
            "contact_score": np.zeros(len(grasps_raw)),
            "active_contact_count": np.zeros(len(grasps_raw), dtype=int),
            "found_collision": grasps_raw[:, 5] > 0.5,
            "combined": grasps_raw[:, 6],
            "probability": grasps_raw[:, 7],
            "wrist_rotation": np.zeros(len(grasps_raw)),
            "smc_iteration": np.zeros(len(grasps_raw), dtype=int),
            "pose_4x4": grasps_raw[:, 8:24].reshape(-1, 4, 4),
        }

    # Superquadric params (backward-compatible: older dumps lack these).
    sq_params = None
    sq_meta = None
    if "sq_params" in data:
        sq_raw = data["sq_params"]
        if len(sq_raw) == 14:
            sq_params = {
                "epsilon1": float(sq_raw[0]),
                "epsilon2": float(sq_raw[1]),
                "a": float(sq_raw[2]),
                "b": float(sq_raw[3]),
                "c": float(sq_raw[4]),
                "translation": sq_raw[5:8].astype(np.float64),
                "rotation": sq_raw[8:14].reshape(2, 3).astype(np.float64),
            }
    if "sq_meta" in data:
        meta_raw = data["sq_meta"]
        if len(meta_raw) == 2:
            sq_meta = {
                "fit_error": float(meta_raw[0]),
                "template_index": int(meta_raw[1]),
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
        "sq_params": sq_params,
        "sq_meta": sq_meta,
    }


def _quat_rotate(qw, qx, qy, qz, v):
    """Rotate vector v by quaternion (qw, qx, qy, qz)."""
    q = np.array([qx, qy, qz])
    t = 2.0 * np.cross(q, v)
    return v + qw * t + np.cross(q, t)


def _find_best_grasp(grasps, threshold=-np.inf):
    """Find the index and score of the best grasp above threshold.

    With dense scoring, all grasps have real combined scores (no NEG_INFINITY).
    The found_collision flag is kept as a diagnostic but no longer used as a
    hard filter — Tier 2/3 results can be the "best" when no Tier 1 exists.
    """
    n = len(grasps["combined"])
    best_idx = -1
    best_combined = -np.inf
    for i in range(n):
        if grasps["combined"][i] >= threshold:
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
    print(f"  Input pose (TF): pos=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})"
          f"  quat=({pose[3]:.3f}, {pose[4]:.3f}, {pose[5]:.3f}, {pose[6]:.3f})")
    twist = dump["input_twist"]
    print(f"  Input twist:     lin=({twist[0]:.3f}, {twist[1]:.3f}, {twist[2]:.3f})"
          f"  ang=({twist[3]:.3f}, {twist[4]:.3f}, {twist[5]:.3f})")
    print(f"  Grasp candidates: {n_total} total, {n_collision} with collision")

    # Contact-score tier distribution (dense reward landscape)
    cs = grasps["contact_score"]
    n_tier1 = int((cs >= 0.8).sum())
    n_tier2 = int(((cs >= 0.3) & (cs < 0.8)).sum())
    n_tier3 = int(((cs > 0.0) & (cs < 0.3)).sum())
    n_tier4 = int((cs == 0.0).sum())
    print(f"  Contact tiers:   T1(valid)={n_tier1}  T2(soft)={n_tier2}"
          f"  T3(start-col)={n_tier3}  T4(no-col)={n_tier4}")

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

    # Superquadric info
    sq_params = dump.get("sq_params")
    sq_meta = dump.get("sq_meta")
    if sq_params is not None:
        template_names = ["sphere", "box", "cylinder"]
        tidx = sq_meta["template_index"] if sq_meta else 0
        tname = template_names[tidx] if tidx < len(template_names) else "?"
        fit_err = sq_meta["fit_error"] if sq_meta else float("nan")
        print(f"  Superquadric:    {tname} (template {tidx})"
              f"  fit_error={fit_err:.6f}")
        print(f"    eps=({sq_params['epsilon1']:.2f}, {sq_params['epsilon2']:.2f})"
              f"  scale=({sq_params['a']:.4f}, {sq_params['b']:.4f}, {sq_params['c']:.4f})")
        t = sq_params["translation"]
        print(f"    translation=({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f})")
    else:
        print(f"  Superquadric:    not available")

    if best_idx >= 0:
        gt = grasps["grasp_type"][best_idx]
        print(f"  Best grasp:      {GRASP_TYPE_NAMES.get(gt, '?')} (type {gt})")
        print(f"    contacts={grasps['active_contact_count'][best_idx]}"
              f"  closure={grasps['closure'][best_idx]:.4f}"
              f"  alignment={grasps['alignment'][best_idx]:.4f}"
              f"  force_closure={grasps['force_closure'][best_idx]:.4f}"
              f"  contact_score={grasps['contact_score'][best_idx]:.4f}"
              f"  combined={grasps['combined'][best_idx]:.4f}"
              f"  prob={grasps['probability'][best_idx]:.4f}")
    print("=" * 60)


# ---------------------------------------------------------------------------
# PyVista visualization
# ---------------------------------------------------------------------------

# Grasp display modes (simple toggle between best and all)
GRASP_MODES = ["best", "all"]

# TSDF display modes (cycle with 't' key)
TSDF_MODES = ["surface", "points", "signs", "off"]


def _build_sq_primitive_mesh(pv_mod, sq_params: dict, sq_meta: dict | None):
    """Build a PyVista mesh approximating the fitted superquadric as a primitive.

    Maps each winning template to a built-in PyVista primitive, then applies
    the fitted rotation + translation.  This is much cheaper than evaluating
    the implicit function on a dense grid and running marching cubes.

    Parameters
    ----------
    pv_mod : module
        The imported ``pyvista`` module.
    sq_params : dict
        Parsed superquadric parameters (epsilon1, epsilon2, a, b, c,
        translation, rotation).
    sq_meta : dict or None
        Parsed superquadric metadata (fit_error, template_index).

    Returns
    -------
    pyvista.PolyData or None
        The transformed primitive mesh, or None if the template is unknown.
    """
    template_index = sq_meta["template_index"] if sq_meta else 0
    a = max(sq_params["a"], 1e-4)
    b = max(sq_params["b"], 1e-4)
    c = max(sq_params["c"], 1e-4)

    # Build the primitive in local frame centered at origin, aligned with axes.
    if template_index == 0:
        # Sphere template -> ellipsoid
        mesh = pv_mod.Sphere(radius=1.0, theta_resolution=32, phi_resolution=32)
        mesh.points[:, 0] *= a
        mesh.points[:, 1] *= b
        mesh.points[:, 2] *= c
    elif template_index == 1:
        # Box template -> rectangular cuboid with half-extents a, b, c
        bounds = [-a, a, -b, b, -c, c]
        mesh = pv_mod.Box(bounds=bounds)
    elif template_index == 2:
        # Cylinder template -> radius=a (use max of a,b for roundness),
        # height=2*c.  Cylinder axis along Z by default.
        radius = max(a, b)
        mesh = pv_mod.Cylinder(
            center=(0, 0, 0),
            direction=(0, 0, 1),
            radius=radius,
            height=2 * c,
            resolution=32,
        )
        # If a != b, scale the non-axis directions to make an elliptical cross-section.
        if abs(a - b) > 1e-5:
            mesh.points[:, 0] *= (a / radius)
            mesh.points[:, 1] *= (b / radius)
    else:
        return None

    # Apply rotation + translation from fitted params.
    # sq_params["rotation"] is a (2, 3) array = first two rows of the 3x3 rotation.
    rot_flat = sq_params["rotation"]  # shape (2, 3)
    rot = np.eye(3)
    rot[0, :] = rot_flat[0]
    rot[1, :] = rot_flat[1]
    # Reconstruct third row via cross product to ensure a proper rotation matrix.
    rot[2, :] = np.cross(rot[0, :], rot[1, :])

    translation = sq_params["translation"]

    # Transform: rotate then translate.
    mesh.points = (rot @ mesh.points.T).T + translation

    return mesh


def visualize_pyvista(dump: dict, args):
    """Full interactive 3D visualization using PyVista."""
    import pyvista as pv

    pv.global_theme.font.color = 'white'

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

    # ---- Determine available SMC iterations (needed for default state) ----
    n_grasps = len(dump["grasps"]["combined"])
    if n_grasps > 0 and "smc_iteration" in dump["grasps"]:
        max_iteration = int(dump["grasps"]["smc_iteration"].max())
    else:
        max_iteration = 0

    # ---- Mutable state for interactive toggles ----
    state = {
        "tsdf_mode": "off",
        "grasp_mode": "all",
        "show_pc": True,
        "show_roi": True,
        "show_cameras": True,
        "show_hand": False,
        "show_contacts": False,
        "show_sq": False,
        "show_info": True,
        "grasp_type_filter": 0,  # 0 = all, 1/2/3 = specific type
        "iteration_filter": max_iteration if n_grasps > 0 else None,  # default to last iteration
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
        "contact_points": [],
        "sq_mesh": [],
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

        # Mask out voxels outside the truncation band (abs >= TRUNCATION_CELLS).
        # These are unobserved / SQ-filled boundary voxels that should not be
        # rendered in any mode.
        tsdf_vis[np.abs(tsdf_vis) >= TRUNCATION_CELLS] = np.nan

        grid = pv.ImageData()
        grid.dimensions = np.array(tsdf_vis.shape) + 1
        grid.origin = origin
        grid.spacing = [res, res, res]
        grid.cell_data["distance"] = tsdf_vis.ravel(order="F")

        if mode == "surface":
            # Show only voxels near the zero-crossing (thin surface band).
            clipped = grid.threshold(
                value=[-TSDF_SURFACE_BAND, TSDF_SURFACE_BAND],
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
                    "color": "white",
                    "position_x": 0.35,
                    "position_y": 0.02,
                    "width": 0.3,
                    "height": 0.05,
                },
                label="TSDF surface",
                show_edges=False,
            )
            actor_groups["tsdf"].append(actor)

        elif mode == "points":
            # Show observed voxels within the truncation band as points.
            clipped = grid.threshold(
                value=[-TRUNCATION_CELLS, TRUNCATION_CELLS],
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
                point_size=7,
                render_points_as_spheres=True,
                opacity=0.5,
                show_scalar_bar=True,
                scalar_bar_args={
                    "title": "TSDF distance (cells)",
                    "color": "white",
                    "position_x": 0.35,
                    "position_y": 0.02,
                    "width": 0.3,
                    "height": 0.05,
                },
                label="TSDF points",
            )
            actor_groups["tsdf"].append(actor)

        elif mode == "signs":
            # Show inside/outside classification as colored points.
            # Red = inside (negative), Blue = outside (positive), Green = surface (near-zero).
            clipped = grid.threshold(
                value=[-TRUNCATION_CELLS, TRUNCATION_CELLS],
                scalars="distance",
            )
            if clipped.n_cells == 0:
                print("  TSDF [signs]: no observed voxels within truncation band")
                return

            # Create a sign classification array: -1=inside, 0=surface, +1=outside.
            dist = clipped.cell_data["distance"]
            sign_arr = np.zeros_like(dist)
            sign_arr[dist < -0.5] = -1.0  # inside
            sign_arr[dist > 0.5] = 1.0   # outside
            # |dist| <= 0.5 stays 0.0 (surface)
            clipped.cell_data["sign"] = sign_arr

            n_inside = int((sign_arr < 0).sum())
            n_surface = int((sign_arr == 0).sum())
            n_outside = int((sign_arr > 0).sum())
            print(f"  TSDF [signs]: rendering {clipped.n_cells} voxels "
                  f"(in={n_inside}, surf={n_surface}, out={n_outside})")

            centers = clipped.cell_centers()
            actor = plotter.add_mesh(
                centers,
                scalars="sign",
                cmap=["red", "lime", "dodgerblue"],
                clim=[-1, 1],
                style="points",
                point_size=7,
                render_points_as_spheres=True,
                opacity=0.5,
                show_scalar_bar=True,
                scalar_bar_args={
                    "title": "Sign (red=in, green=surf, blue=out)",
                    "color": "white",
                    "position_x": 0.35,
                    "position_y": 0.02,
                    "width": 0.3,
                    "height": 0.05,
                },
                label="TSDF signs",
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
    # n_grasps and max_iteration already computed above (before state dict).

    threshold = args.threshold

    best_idx, best_combined = _find_best_grasp(grasps, threshold)

    # Iteration-filtered best grasp (for display when iteration filter is active).
    def _iteration_mask():
        """Return a boolean mask for the current iteration filter."""
        if state["iteration_filter"] is None:
            return np.ones(n_grasps, dtype=bool)
        return grasps["smc_iteration"] == state["iteration_filter"]

    def _filtered_best():
        """Find the best grasp within the current iteration filter."""
        mask = _iteration_mask()
        if not mask.any():
            return -1, -np.inf
        filtered_indices = np.where(mask)[0]
        scores = grasps["combined"][filtered_indices]
        best_local = int(np.argmax(scores))
        return int(filtered_indices[best_local]), float(scores[best_local])

    def get_grasp_indices():
        """Compute which grasp indices to show based on current state.
        Returns a tuple: (all_valid_indices, top_displayed_indices)

        With dense scoring, all grasps have real combined scores. We filter
        by threshold and contact_score > 0 (i.e., at least Tier 3) to avoid
        rendering Tier 4 (no object in reach) as clutter.
        """
        candidates = []
        iter_mask = _iteration_mask()
        for i in range(n_grasps):
            if not iter_mask[i]:
                continue
            if grasps["contact_score"][i] <= 0.0:
                # Tier 4: no collision at all — skip rendering.
                continue
            if grasps["combined"][i] < threshold:
                continue
            # Type filter
            if state["grasp_type_filter"] != 0:
                if grasps["grasp_type"][i] != state["grasp_type_filter"]:
                    continue
            candidates.append(i)

        # Sort descending by score for top 10 extraction
        candidates.sort(key=lambda i: grasps["combined"][i], reverse=True)

        # Use iteration-filtered best when filter is active.
        display_best = best_idx if state["iteration_filter"] is None else _filtered_best()[0]

        if state["grasp_mode"] == "best":
            if display_best >= 0 and display_best in candidates:
                return [display_best], [display_best]
            elif candidates:
                return [candidates[0]], [candidates[0]]  # best available
            return [], []

        # "all"
        return candidates, candidates[:10]

    def add_grasp_actors():
        actor_groups["grasps"].clear()

        all_indices, top_indices = get_grasp_indices()
        if not all_indices:
            return

        # Render the current grasp set as a single point cloud for context.
        positions = []
        point_colors = []

        # Use iteration-filtered best when filter is active.
        display_best = best_idx if state["iteration_filter"] is None else _filtered_best()[0]
        for i in all_indices:
            gt = grasps["grasp_type"][i]
            is_best = (i == display_best)
            T = grasps["pose_4x4"][i]
            positions.append(T[:3, 3])

            if is_best:
                point_colors.append([1.0, 0.84, 0.0])  # gold
            else:
                hex_color = GRASP_TYPE_COLORS.get(gt, "#ffffff")
                r_c = int(hex_color[1:3], 16) / 255.0
                g_c = int(hex_color[3:5], 16) / 255.0
                b_c = int(hex_color[5:7], 16) / 255.0
                point_colors.append([r_c, g_c, b_c])

        positions = np.array(positions)
        point_colors = np.array(point_colors)

        grasp_cloud = pv.PolyData(positions)
        grasp_cloud["colors"] = point_colors
        actor = plotter.add_mesh(
            grasp_cloud,
            scalars="colors",
            rgb=True,
            style="points",
            point_size=4 if state["grasp_mode"] == "all" else 10,
            render_points_as_spheres=True,
            label=f"Grasps ({len(all_indices)})",
        )
        actor_groups["grasps"].append(actor)

        if not top_indices:
            return

        # Compute score range for size scaling based on top displayed.
        scores = [grasps["combined"][i] for i in top_indices]
        score_min = min(scores) if scores else 0
        score_max = max(scores) if scores else 1
        score_range = score_max - score_min if score_max > score_min else 1.0

        # Render top grasps as separate actors, similar to the camera markers.
        # Use iteration-filtered best when filter is active.
        display_best = best_idx if state["iteration_filter"] is None else _filtered_best()[0]
        for idx in top_indices:
            gt = grasps["grasp_type"][idx]
            is_best = (idx == display_best)
            T = grasps["pose_4x4"][idx]
            pos = T[:3, 3]
            R = T[:3, :3]
            score = grasps["combined"][idx]

            score_norm = (score - score_min) / score_range if score_range > 0 else 1.0

            marker_radius = marker_size * (0.11 if is_best else (0.05 + 0.07 * score_norm))
            grasp_marker = pv.Sphere(radius=marker_radius, center=pos)
            marker_color = "gold" if is_best else GRASP_TYPE_COLORS.get(gt, "white")
            actor = plotter.add_mesh(
                grasp_marker,
                color=marker_color,
                opacity=0.95 if is_best else 0.75,
            )
            actor_groups["grasps"].append(actor)
            
            # Z-axis direction (approach direction)
            z_dir = R[:, 2]
            z_len = marker_size * (1.2 if is_best else 0.2 + 0.4 * score_norm)

            start = pos
            end = pos + z_dir * z_len
            line = pv.Line(start, end)
            line_color = "gold" if is_best else GRASP_TYPE_COLORS.get(gt, "white")
            actor = plotter.add_mesh(
                line,
                color=line_color,
                line_width=4 if is_best else 2,
                opacity=0.95 if is_best else 0.7,
            )
            actor_groups["grasps"].append(actor)

    add_grasp_actors()

    # ==================================================================
    # Hand skeleton (compact unified model, requires pinocchio)
    # One topology for both best/all modes: finger base marker + fingertip marker.
    # ==================================================================
    if _pin_hand_available:
        _hand_specs_resolved = []
        for finger, base_name, tip_name in _HAND_SKELETON_SPECS:
            base_id = pin_model.getFrameId(base_name)
            if base_id < pin_model.nframes:
                _hand_specs_resolved.append((finger, base_id, tip_name))
            else:
                print(f"  WARNING: frame not found: base={base_name} (id={base_id}), tip={tip_name}")

        print(f"  Hand skeleton: pinocchio + unified contact model loaded ({len(_hand_specs_resolved)} fingers)")
    else:
        _hand_specs_resolved = []
        print("  Hand skeleton: unavailable (pinocchio or URDF not found)")

    if _pin_hand_available:
        _contact_definitions_by_name = {contact["name"]: contact for contact in CONTACT_DEFINITIONS}
    else:
        _contact_definitions_by_name = {}

    def _contact_position_in_hand_frame(contact_name):
        """Return the contact position in the hand base frame for the current q."""
        contact = _contact_definitions_by_name.get(contact_name)
        if contact is None:
            return None

        geom_info = COLLISION_GEOMETRIES[contact["geom"]]
        m_joint = pin_data.oMi[geom_info["joint_id"]]
        m_geom_world = m_joint * geom_info["placement"]
        return (
            m_geom_world.translation + m_geom_world.rotation @ contact["local_offset"]
        ).astype(np.float64)

    def _compute_hand_positions(grasp_idx):
        """Run FK and return the compact hand markers in world coordinates."""
        gt = grasps["grasp_type"][grasp_idx]
        T = grasps["pose_4x4"][grasp_idx]
        closure = grasps["closure"][grasp_idx]
        thumb_mode = 0.0 if gt == 3 else 1.0
        q_active = np.array([closure, closure, closure], dtype=float)
        q_full = _q_full_with_thumb_mode(q_active, thumb_mode)
        pin.forwardKinematics(pin_model, pin_data, q_full)
        pin.updateFramePlacements(pin_model, pin_data)

        positions = {}
        for finger, base_id, tip_name in _hand_specs_resolved:
            positions[f"{finger}_base"] = pin_data.oMf[base_id].translation.copy().astype(np.float64)
            tip_pos = _contact_position_in_hand_frame(tip_name)
            if tip_pos is not None:
                positions[f"{finger}_tip"] = tip_pos

        # Transform to world frame via grasp pose.
        R, t = T[:3, :3], T[:3, 3]
        for fid in positions:
            positions[fid] = R @ positions[fid] + t

        return positions

    def _hex_to_rgb01(hex_color):
        hex_color = hex_color.lstrip("#")
        return np.array(
            [
                int(hex_color[0:2], 16) / 255.0,
                int(hex_color[2:4], 16) / 255.0,
                int(hex_color[4:6], 16) / 255.0,
            ],
            dtype=np.float32,
        )

    def _add_single_hand(frame_positions, opacity, line_width, point_size, render_points_as_spheres):
        """Render one compact hand skeleton with a single line/point actor pair."""
        points = []
        point_colors = []
        line_cells = []

        for finger, _, _ in _hand_specs_resolved:
            base_key = f"{finger}_base"
            tip_key = f"{finger}_tip"
            p_pos = frame_positions.get(base_key)
            c_pos = frame_positions.get(tip_key)
            if p_pos is None or c_pos is None:
                continue

            color = FINGER_COLORS.get(finger, "#f4a261")
            rgb = _hex_to_rgb01(color)

            start_idx = len(points)
            points.extend([p_pos, c_pos])
            point_colors.extend([rgb, rgb])
            line_cells.append([2, start_idx, start_idx + 1])

        if not points:
            return

        pts = pv.PolyData(np.asarray(points, dtype=np.float64))
        pts["colors"] = np.asarray(point_colors, dtype=np.float32)
        actor = plotter.add_mesh(
            pts,
            scalars="colors",
            rgb=True,
            style="points",
            point_size=point_size,
            render_points_as_spheres=render_points_as_spheres,
            opacity=opacity,
        )
        actor_groups["hand_skeleton"].append(actor)

        lines = pv.PolyData(np.asarray(points, dtype=np.float64))
        lines.lines = np.hstack(line_cells)
        actor = plotter.add_mesh(
            lines,
            color="#ffbc85",
            line_width=line_width,
            opacity=min(0.7, opacity),
            label="Hand skeleton",
        )
        actor_groups["hand_skeleton"].append(actor)

        thumb_base = frame_positions.get("thumb_base")
        palm_connectors = []
        if thumb_base is not None:
            thumb_pos = np.asarray(thumb_base, dtype=np.float64)
            connector_order = ["index", "middle", "ring", "little"]
            prev_pos = thumb_pos
            for finger in connector_order:
                base_pos = frame_positions.get(f"{finger}_base")
                if base_pos is None:
                    continue
                palm_connectors.extend([
                    prev_pos,
                    np.asarray(base_pos, dtype=np.float64),
                ])
                prev_pos = np.asarray(base_pos, dtype=np.float64)

            if thumb_pos is not None:
                for finger in connector_order:
                    base_pos = frame_positions.get(f"{finger}_base")
                    if base_pos is None:
                        continue
                    palm_connectors.extend([thumb_pos, np.asarray(base_pos, dtype=np.float64)])

        if palm_connectors:
            connector_points = np.asarray(palm_connectors, dtype=np.float64)
            connector_cells = np.hstack(
                [[2, i, i + 1] for i in range(0, len(connector_points), 2)]
            )
            connectors = pv.PolyData(connector_points)
            connectors.lines = connector_cells
            actor = plotter.add_mesh(
                connectors,
                color="#d8dee9",
                line_width=max(1, line_width - 1),
                opacity=min(0.28, opacity),
            )
            actor_groups["hand_skeleton"].append(actor)

    def add_hand_skeletons():
        """Render hand skeletons based on current grasp mode.

        Both modes use the same compact topology; only styling changes.
        """
        actor_groups["hand_skeleton"].clear()
        if not _pin_hand_available:
            return

        all_indices, top_indices = get_grasp_indices()
        if not top_indices:
            return

        # Use iteration-filtered best when filter is active.
        display_best = best_idx if state["iteration_filter"] is None else _filtered_best()[0]
        for i in top_indices:
            if grasps["contact_score"][i] <= 0.0:
                continue
            is_best = (i == display_best)
            positions = _compute_hand_positions(i)
            if is_best:
                _add_single_hand(
                    positions,
                    opacity=0.72,
                    line_width=4,
                    point_size=12,
                    render_points_as_spheres=True,
                )
            else:
                _add_single_hand(
                    positions,
                    opacity=0.35,
                    line_width=2,
                    point_size=8,
                    render_points_as_spheres=False,
                )

    if state["show_hand"]:
        add_hand_skeletons()

    # ==================================================================
    # LUT contact points (all 25 contacts per top grasp)
    # ==================================================================
    def _grasp_to_fk_params(grasp_type, closure):
        """Map grasp type + closure to (q_active, thumb_opp_mode) for FK.

        Mirrors the grasp specs in planner.rs:
          cylindrical: all fingers close, thumb in abduction (opposition)
          pinch: thumb + index close, MRL locked open, thumb in abduction
          lateral: thumb + index close, MRL locked open, thumb in adduction
        """
        if grasp_type == 1:  # cylindrical
            q_active = np.array([closure, closure, closure], dtype=float)
            thumb_mode = 1.0
        elif grasp_type == 2:  # pinch
            q_active = np.array([closure, closure, 0.0], dtype=float)
            thumb_mode = 1.0
        else:  # lateral (type 3)
            q_active = np.array([closure, closure, 0.0], dtype=float)
            thumb_mode = 0.0
        return q_active, thumb_mode

    def add_contact_points():
        """Render all 25 LUT contact points for the top grasps.

        Uses Pinocchio FK via get_sampled_contact_transforms() to compute
        contact positions in hand-local frame, then transforms to world frame
        using the grasp's 4x4 pose matrix. Score contacts are rendered larger
        than sweep-only contacts.  Lines connect contacts within each finger
        group to visualise the kinematic chain.
        """
        actor_groups["contact_points"].clear()
        if not _pin_hand_available:
            return

        all_indices, top_indices = get_grasp_indices()
        if not top_indices:
            return

        # Build a contact-name -> group lookup from model.py CONTACT_DEFINITIONS.
        contact_group_map = {c["name"]: c["group"] for c in CONTACT_DEFINITIONS}

        # Use iteration-filtered best when filter is active.
        display_best = best_idx if state["iteration_filter"] is None else _filtered_best()[0]

        for idx in top_indices:
            if grasps["contact_score"][idx] <= 0.0:
                continue

            gt = grasps["grasp_type"][idx]
            T = grasps["pose_4x4"][idx]
            closure = grasps["closure"][idx]
            R, t = T[:3, :3], T[:3, 3]
            is_best = (idx == display_best)

            q_active, thumb_mode = _grasp_to_fk_params(gt, closure)
            try:
                transforms = get_sampled_contact_transforms(q_active, thumb_opp_mode=thumb_mode)
            except Exception:
                continue

            score_contacts = _GRASP_SCORE_CONTACTS.get(gt, set())

            # ---- Collect per-contact world positions ----
            world_positions = {}  # contact_name -> world xyz
            for contact_name, contact_T in transforms.items():
                local_pos = contact_T[:3, 3]
                world_positions[contact_name] = R @ local_pos + t

            # ---- Render contact points ----
            points = []
            point_colors = []
            for contact_name in world_positions:
                points.append(world_positions[contact_name])
                group = contact_group_map.get(contact_name, "index")
                hex_color = _CONTACT_FINGER_COLORS.get(group, "#ffffff")
                r_c = int(hex_color[1:3], 16) / 255.0
                g_c = int(hex_color[3:5], 16) / 255.0
                b_c = int(hex_color[5:7], 16) / 255.0
                is_score = contact_name in score_contacts
                if is_score:
                    point_colors.append([r_c, g_c, b_c])
                else:
                    point_colors.append([r_c * 0.5, g_c * 0.5, b_c * 0.5])

            if not points:
                continue

            pt_size = 10 if is_best else 6
            pts = pv.PolyData(np.asarray(points, dtype=np.float64))
            pts["colors"] = np.asarray(point_colors, dtype=np.float32)
            actor = plotter.add_mesh(
                pts,
                scalars="colors",
                rgb=True,
                style="points",
                point_size=pt_size,
                render_points_as_spheres=True,
                opacity=0.85 if is_best else 0.55,
                label=f"Contacts ({len(points)} pts)" if is_best else None,
            )
            actor_groups["contact_points"].append(actor)

            # ---- Render per-group lines ----
            line_points = []
            line_cells = []
            for group, chains in _CONTACT_LINES.items():
                for chain in chains:
                    chain_positions = []
                    for cname in chain:
                        if cname in world_positions:
                            chain_positions.append(world_positions[cname])
                    if len(chain_positions) < 2:
                        continue
                    start_idx = len(line_points)
                    line_points.extend(chain_positions)
                    for j in range(len(chain_positions) - 1):
                        line_cells.append([2, start_idx + j, start_idx + j + 1])

            if line_points:
                lp = pv.PolyData(np.asarray(line_points, dtype=np.float64))
                lp.lines = np.hstack(line_cells)
                actor = plotter.add_mesh(
                    lp,
                    color="#a0c28d",
                    line_width=3 if is_best else 1,
                    opacity=0.6 if is_best else 0.3,
                )
                actor_groups["contact_points"].append(actor)

    if state["show_contacts"]:
        add_contact_points()

    # ==================================================================
    # Superquadric primitive mesh (toggleable)
    # ==================================================================
    def add_sq_actors():
        """Render the fitted superquadric as a semi-transparent primitive mesh."""
        actor_groups["sq_mesh"].clear()
        sq_params = dump.get("sq_params")
        sq_meta = dump.get("sq_meta")
        if sq_params is None:
            print("  SQ mesh: no superquadric data in dump")
            return

        mesh = _build_sq_primitive_mesh(pv, sq_params, sq_meta)
        if mesh is None:
            print("  SQ mesh: unknown template index")
            return

        tidx = sq_meta["template_index"] if sq_meta else 0
        tname = SQ_TEMPLATE_NAMES[tidx] if tidx < len(SQ_TEMPLATE_NAMES) else "?"
        n_cells = mesh.n_cells
        print(f"  SQ mesh: {tname} primitive, {n_cells} cells")

        actor = plotter.add_mesh(
            mesh,
            color="cyan",
            opacity=0.25,
            show_edges=True,
            edge_color="cyan",
            line_width=1,
            label=f"Superquadric ({tname})",
        )
        actor_groups["sq_mesh"].append(actor)

    if state["show_sq"]:
        add_sq_actors()

    # ==================================================================
    # Info text
    # ==================================================================
    def update_info_text():
        actor_groups["info"].clear()

        mode_str = state["grasp_mode"]
        if state["grasp_type_filter"] != 0:
            mode_str += f" ({GRASP_TYPE_SHORT.get(state['grasp_type_filter'], '?')})"

        all_indices, top_indices = get_grasp_indices()
        n_shown = len(all_indices)
        n_top = len(top_indices)

        tsdf_str = state["tsdf_mode"]
        hand_str = "unified"
        contacts_str = "on" if state["show_contacts"] else "off"
        sq_str = "on" if state["show_sq"] else "off"

        iter_str = f"iter={state['iteration_filter']}" if state['iteration_filter'] is not None else f"iter=all(0-{max_iteration})"
        lines = [
            f"Grasps: {n_shown} points, top {n_top} rendered (mode={mode_str}, threshold>={threshold:.2f}, {iter_str})",
        ]
        # Show iteration-filtered best when filter is active.
        display_best_idx = best_idx if state["iteration_filter"] is None else _filtered_best()[0]
        display_best_score = best_combined if state["iteration_filter"] is None else _filtered_best()[1]
        if display_best_idx >= 0:
            gt = grasps["grasp_type"][display_best_idx]
            lines.append(
                f"Best: {GRASP_TYPE_NAMES.get(gt, '?')}  combined={display_best_score:.4f}"
            )
        else:
            lines.append("Best: none")

        lines.append(f"TSDF: {tsdf_str}  Hand: {hand_str}  Contacts: {contacts_str}  SQ: {sq_str}")
        lines.append("Keys: t=TSDF  g=grasps  p=cloud  r=ROI  c=cam  h=hand  k=contacts  b=SQ  +/-/*=iter  ?=help")

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
        ("Point cloud", "white"),
        ("ROI box", "orange"),
        ("Best grasp", "gold"),
        ("Cylindrical", GRASP_TYPE_COLORS[1]),
        ("Pinch", GRASP_TYPE_COLORS[2]),
        ("Lateral", GRASP_TYPE_COLORS[3]),
        ("Hand", "#ffbc85"),
        ("Contact points", "#a0c28d"),
        ("Superquadric", "cyan"),
        ("TSDF: inside", "red"),
        ("TSDF: surface", "lime"),
        ("TSDF: outside", "dodgerblue"),
    ]
    plotter.add_legend(legend_entries, size=(0.18, 0.36), loc="upper left",
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
        """Remove and re-add grasp and hand actors when switching modes."""
        for actor in actor_groups["grasps"]:
            plotter.remove_actor(actor)
        actor_groups["grasps"].clear()
        add_grasp_actors()
        # Hand model stays unified across modes; rebuild it to refresh positions.
        for actor in actor_groups["hand_skeleton"]:
            plotter.remove_actor(actor)
        actor_groups["hand_skeleton"].clear()
        if state["show_hand"]:
            add_hand_skeletons()
        # Contact points also depend on which grasps are displayed.
        for actor in actor_groups["contact_points"]:
            plotter.remove_actor(actor)
        actor_groups["contact_points"].clear()
        if state["show_contacts"]:
            add_contact_points()
        update_info_text()
        plotter.render()

    def rebuild_tsdf():
        """Remove and re-add TSDF actors."""
        # Remove scalar bars first — they are separate actors from the meshes.
        for bar_name in ["TSDF distance (cells)", "Sign (red=in, green=surf, blue=out)"]:
            try:
                plotter.remove_scalar_bar(bar_name)
            except Exception:
                pass
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
        state["grasp_mode"] = "all" if state["grasp_mode"] == "best" else "best"
        state["show_hand"] = False  # Reset hand visibility to hidden on mode switch
        print(f"  Grasp mode: {state['grasp_mode']}")
        rebuild_grasps()

    def on_key_p():
        toggle_actors("pc")

    def on_key_r():
        toggle_actors("roi")

    def on_key_c():
        toggle_actors("cameras")

    def on_key_h():
        """Toggle hand skeleton(s)."""
        state["show_hand"] = not state["show_hand"]
        if state["show_hand"]:
            if not actor_groups["hand_skeleton"]:
                add_hand_skeletons()
            else:
                for actor in actor_groups["hand_skeleton"]:
                    actor.SetVisibility(True)
        else:
            for actor in actor_groups["hand_skeleton"]:
                actor.SetVisibility(False)
        update_info_text()
        plotter.render()

    def on_key_k():
        """Toggle LUT contact points."""
        state["show_contacts"] = not state["show_contacts"]
        if state["show_contacts"]:
            if not actor_groups["contact_points"]:
                add_contact_points()
            else:
                for actor in actor_groups["contact_points"]:
                    actor.SetVisibility(True)
        else:
            for actor in actor_groups["contact_points"]:
                actor.SetVisibility(False)
        update_info_text()
        plotter.render()

    def on_key_i():
        toggle_actors("info")

    def on_key_b():
        """Toggle superquadric primitive mesh."""
        state["show_sq"] = not state["show_sq"]
        if state["show_sq"]:
            if not actor_groups["sq_mesh"]:
                add_sq_actors()
            else:
                for actor in actor_groups["sq_mesh"]:
                    actor.SetVisibility(True)
        else:
            for actor in actor_groups["sq_mesh"]:
                actor.SetVisibility(False)
        update_info_text()
        plotter.render()

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

    def on_key_plus():
        """Advance to the next SMC iteration filter."""
        if max_iteration == 0:
            return
        if state["iteration_filter"] is None:
            state["iteration_filter"] = 0
        elif state["iteration_filter"] < max_iteration:
            state["iteration_filter"] += 1
        else:
            state["iteration_filter"] = None  # wrap to all
        iter_str = f"iter {state['iteration_filter']}" if state['iteration_filter'] is not None else "all"
        print(f"  Iteration filter: {iter_str}")
        rebuild_grasps()

    def on_key_minus():
        """Go back to the previous SMC iteration filter."""
        if max_iteration == 0:
            return
        if state["iteration_filter"] is None:
            state["iteration_filter"] = max_iteration
        elif state["iteration_filter"] > 0:
            state["iteration_filter"] -= 1
        else:
            state["iteration_filter"] = None  # wrap to all
        iter_str = f"iter {state['iteration_filter']}" if state['iteration_filter'] is not None else "all"
        print(f"  Iteration filter: {iter_str}")
        rebuild_grasps()

    def on_key_star():
        """Reset iteration filter to show all iterations."""
        state["iteration_filter"] = None
        print("  Iteration filter: all")
        rebuild_grasps()

    def on_key_help():
        print("\n  === Keyboard Shortcuts ===")
        print("  t  - cycle TSDF mode: surface / points / signs / off")
        print("  g  - toggle grasp display: best / all")
        print("  p  - toggle point cloud")
        print("  r  - toggle ROI box")
        print("  c  - toggle camera markers")
        print("  h  - toggle hand skeleton (unified)")
        print("  k  - toggle LUT contact points")
        print("  b  - toggle superquadric mesh")
        print("  i  - toggle info text")
        print("  +  - next SMC iteration filter")
        print("  -  - previous SMC iteration filter")
        print("  *  - show all iterations (reset filter)")
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
    plotter.add_key_event("k", on_key_k)
    plotter.add_key_event("b", on_key_b)
    plotter.add_key_event("i", on_key_i)
    plotter.add_key_event("0", on_key_0)
    plotter.add_key_event("1", on_key_1)
    plotter.add_key_event("2", on_key_2)
    plotter.add_key_event("3", on_key_3)
    plotter.add_key_event("plus", on_key_plus)
    plotter.add_key_event("minus", on_key_minus)
    plotter.add_key_event("asterisk", on_key_star)
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
        if grasps["contact_score"][i] <= 0.0 or grasps["combined"][i] < threshold:
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
    ax.scatter(*pose[0:3], c="black", s=80, marker="^", label="Hand pose (TF)")

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
              t  cycle TSDF: surface/points/signs/off
              g  toggle grasp mode (best/all)
              p  toggle point cloud      r  toggle ROI box
              c  toggle cameras          h  toggle hand skeleton (unified)
              k  toggle LUT contact points
              b  toggle superquadric mesh
              i  toggle info text        0-3  filter grasp types
              +/- cycle SMC iteration    * show all iterations
              ?  print help
        """),
    )
    parser.add_argument(
        "dump_path", nargs="?", default=None,
        help="Path to the .npz debug dump file (default: latest in data/debug/)",
    )
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

    # Resolve dump path: explicit or latest in data/debug/
    if args.dump_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_dir = os.path.join(script_dir, "..", "data", "debug")
        default_dir = os.path.normpath(default_dir)
        if not os.path.isdir(default_dir):
            print(f"Error: no dump path given and default dir not found: {default_dir}",
                  file=sys.stderr)
            sys.exit(1)
        npz_files = sorted(
            (f for f in os.listdir(default_dir) if f.endswith(".npz")),
            key=lambda f: os.path.getmtime(os.path.join(default_dir, f)),
        )
        if not npz_files:
            print(f"Error: no .npz files found in {default_dir}", file=sys.stderr)
            sys.exit(1)
        args.dump_path = os.path.join(default_dir, npz_files[-1])
        print(f"No path given — using latest dump:")

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
