#!/usr/bin/env python3
"""Generate synthetic point clouds for the software verification test.

Produces .npz files in objects/parametric/ and objects/ycb/ containing:
  - points: (N, 3) float32 surface-sampled point cloud
  - metadata: JSON string with object name, expected grasp, dimensions

Usage:
    python generate_objects.py               # generate parametric objects
    python generate_objects.py --ycb         # also process YCB meshes (requires trimesh)
    python generate_objects.py --all         # both
"""

import argparse
import json
import os
import sys

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARAMETRIC_DIR = os.path.join(SCRIPT_DIR, "objects", "parametric")
YCB_DIR = os.path.join(SCRIPT_DIR, "objects", "ycb")

N_POINTS = 30000  # target number of surface points per object
# Matches typical segmented object cloud size from the live D435i pipeline
# (dual-camera fused at 5mm voxel, then segmented). This ensures latency
# measurements are representative of real-world conditions.


def _save_object(path: str, name: str, points: np.ndarray,
                 expected_grasp: str, dimensions: dict):
    """Save an object as .npz with points + metadata."""
    points = np.ascontiguousarray(points, dtype=np.float32)
    meta = json.dumps({
        "name": name,
        "expected_grasp": expected_grasp,
        "dimensions": dimensions,
        "n_points": len(points),
    })
    np.savez(path, points=points, metadata=np.array([meta]))
    print(f"  saved {path}: {len(points)} points, expected_grasp={expected_grasp}")


# ---------------------------------------------------------------------------
# Parametric object generators
# ---------------------------------------------------------------------------

def _sample_surface(points: np.ndarray, n: int) -> np.ndarray:
    """Randomly sample n points from a dense point array."""
    if len(points) <= n:
        return points
    idx = np.random.default_rng(42).choice(len(points), size=n, replace=False)
    return points[idx]


def gen_cylinder_upright():
    """Cylinder: r=3cm, h=12cm, axis along Z, centered at origin."""
    rng = np.random.default_rng(42)
    r = 0.03
    h = 0.12
    n_side = int(N_POINTS * 0.9)
    n_cap = N_POINTS - n_side

    # Side surface
    theta = rng.uniform(0, 2 * np.pi, n_side)
    z = rng.uniform(-h / 2, h / 2, n_side)
    side = np.column_stack([r * np.cos(theta), r * np.sin(theta), z])

    # Top + bottom caps
    r_cap = rng.uniform(0, r, n_cap)
    theta_cap = rng.uniform(0, 2 * np.pi, n_cap)
    cap_x = r_cap * np.cos(theta_cap)
    cap_y = r_cap * np.sin(theta_cap)
    caps = np.column_stack([
        cap_x, cap_y,
        np.where(np.arange(n_cap) < n_cap // 2, h / 2, -h / 2),
    ])

    points = np.vstack([side, caps]).astype(np.float32)
    _save_object(
        os.path.join(PARAMETRIC_DIR, "cylinder_upright.npz"),
        "cylinder_upright", points, "cylindrical",
        {"radius_m": r, "height_m": h},
    )


def gen_cylinder_tilted():
    """Cylinder: r=3cm, h=12cm, tilted 30 degrees around Y axis."""
    rng = np.random.default_rng(42)
    r = 0.03
    h = 0.12
    angle = np.radians(30)

    n_side = int(N_POINTS * 0.9)
    n_cap = N_POINTS - n_side

    theta = rng.uniform(0, 2 * np.pi, n_side)
    z = rng.uniform(-h / 2, h / 2, n_side)
    side = np.column_stack([r * np.cos(theta), r * np.sin(theta), z])

    r_cap = rng.uniform(0, r, n_cap)
    theta_cap = rng.uniform(0, 2 * np.pi, n_cap)
    cap_x = r_cap * np.cos(theta_cap)
    cap_y = r_cap * np.sin(theta_cap)
    caps = np.column_stack([
        cap_x, cap_y,
        np.where(np.arange(n_cap) < n_cap // 2, h / 2, -h / 2),
    ])

    points = np.vstack([side, caps]).astype(np.float32)

    # Rotate 30 degrees around Y
    c, s = np.cos(angle), np.sin(angle)
    rot = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float32)
    points = points @ rot.T

    _save_object(
        os.path.join(PARAMETRIC_DIR, "cylinder_tilted.npz"),
        "cylinder_tilted", points, "cylindrical",
        {"radius_m": r, "height_m": h, "tilt_deg": 30},
    )


def gen_ellipsoid():
    """Ellipsoid: a=5cm, b=3cm, c=3cm."""
    rng = np.random.default_rng(42)
    a, b, c = 0.05, 0.03, 0.03

    # Uniform surface sampling via rejection
    # Generate points on unit sphere, then scale
    u = rng.standard_normal((N_POINTS * 3, 3))
    norms = np.linalg.norm(u, axis=1, keepdims=True)
    unit_sphere = u / norms

    # Scale to ellipsoid
    scale = np.array([a, b, c])
    points = (unit_sphere * scale).astype(np.float32)

    # Subsample to target count
    points = _sample_surface(points, N_POINTS)

    _save_object(
        os.path.join(PARAMETRIC_DIR, "ellipsoid.npz"),
        "ellipsoid", points, "cylindrical",
        {"a_m": a, "b_m": b, "c_m": c},
    )


def gen_tapered_bottle():
    """Tapered cylinder (bottle shape): r_bottom=3.5cm, r_top=2.5cm, h=18cm."""
    rng = np.random.default_rng(42)
    r_bot = 0.035
    r_top = 0.025
    h = 0.18
    n_side = int(N_POINTS * 0.9)
    n_cap = N_POINTS - n_side

    theta = rng.uniform(0, 2 * np.pi, n_side)
    z = rng.uniform(0, h, n_side)
    # Radius varies linearly with height
    r = r_bot + (r_top - r_bot) * (z / h)
    side = np.column_stack([r * np.cos(theta), r * np.sin(theta), z])

    # Bottom cap
    n_bot = n_cap // 2
    n_top = n_cap - n_bot
    r_bot_pts = rng.uniform(0, r_bot, n_bot)
    theta_bot = rng.uniform(0, 2 * np.pi, n_bot)
    bot = np.column_stack([
        r_bot_pts * np.cos(theta_bot),
        r_bot_pts * np.sin(theta_bot),
        np.zeros(n_bot),
    ])
    r_top_pts = rng.uniform(0, r_top, n_top)
    theta_top = rng.uniform(0, 2 * np.pi, n_top)
    top = np.column_stack([
        r_top_pts * np.cos(theta_top),
        r_top_pts * np.sin(theta_top),
        np.full(n_top, h),
    ])

    points = np.vstack([side, bot, top]).astype(np.float32)

    # Center at origin
    points[:, 2] -= h / 2

    _save_object(
        os.path.join(PARAMETRIC_DIR, "tapered_bottle.npz"),
        "tapered_bottle", points, "cylindrical",
        {"r_bottom_m": r_bot, "r_top_m": r_top, "height_m": h},
    )


def gen_l_block():
    """L-shaped block: 6x3x3 cm + 3x3x6 cm, self-occluding concave shape."""
    rng = np.random.default_rng(42)
    n_half = N_POINTS // 2

    # Block 1: 6x3x3 cm, centered at (0, 0, 0)
    b1 = rng.uniform(-0.03, 0.03, (n_half, 3))
    b1[:, 0] = rng.uniform(-0.03, 0.03, n_half)  # x: [-3, 3] cm
    b1[:, 1] = rng.uniform(-0.015, 0.015, n_half)  # y: [-1.5, 1.5] cm
    b1[:, 2] = rng.uniform(-0.015, 0.015, n_half)  # z: [-1.5, 1.5] cm

    # Block 2: 3x3x6 cm, attached to block 1 at x=3cm, extending in +z
    b2 = rng.uniform(-0.03, 0.03, (N_POINTS - n_half, 3))
    b2[:, 0] = rng.uniform(0.0, 0.03, N_POINTS - n_half)  # x: [0, 3] cm
    b2[:, 1] = rng.uniform(-0.015, 0.015, N_POINTS - n_half)  # y: [-1.5, 1.5] cm
    b2[:, 2] = rng.uniform(0.015, 0.045, N_POINTS - n_half)  # z: [1.5, 4.5] cm

    points = np.vstack([b1, b2]).astype(np.float32)

    # Only keep surface points (thin shell approximation)
    # For a box, surface points are those near any face
    def _box_surface_mask(pts, half_extents, thickness=0.003):
        """Keep points within `thickness` of any face of a box."""
        abs_pts = np.abs(pts)
        # Distance to nearest face along each axis
        dist_to_face = half_extents - abs_pts
        # A point is "surface" if it's close to any face
        on_surface = np.any(dist_to_face < thickness, axis=1)
        return on_surface

    # Generate dense interior + surface, then filter
    rng2 = np.random.default_rng(123)
    dense1 = rng2.uniform(-0.03, 0.03, (N_POINTS * 5, 3))
    dense1[:, 1] = rng2.uniform(-0.015, 0.015, N_POINTS * 5)
    dense1[:, 2] = rng2.uniform(-0.015, 0.015, N_POINTS * 5)
    mask1 = _box_surface_mask(dense1, np.array([0.03, 0.015, 0.015]))

    dense2 = rng2.uniform(0.0, 0.03, (N_POINTS * 5, 3))
    dense2[:, 1] = rng2.uniform(-0.015, 0.015, N_POINTS * 5)
    dense2[:, 2] = rng2.uniform(0.015, 0.045, N_POINTS * 5)
    mask2 = _box_surface_mask(dense2, np.array([0.03, 0.015, 0.015]))

    surf1 = dense1[mask1]
    surf2 = dense2[mask2]
    points = np.vstack([surf1, surf2]).astype(np.float32)
    points = _sample_surface(points, N_POINTS)

    _save_object(
        os.path.join(PARAMETRIC_DIR, "l_block.npz"),
        "l_block", points, "cylindrical",
        {"block1_m": [0.06, 0.03, 0.03], "block2_m": [0.03, 0.03, 0.06]},
    )


def gen_small_cube():
    """Small cube: 2.5cm side, pinch target."""
    rng = np.random.default_rng(42)
    half = 0.0125  # 1.25 cm half-side

    # Generate surface points for a cube
    dense = rng.uniform(-half, half, (N_POINTS * 5, 3))
    # Keep only surface (within 2mm of any face)
    abs_d = np.abs(dense)
    dist_to_face = half - abs_d
    surface_mask = np.any(dist_to_face < 0.002, axis=1)
    points = dense[surface_mask].astype(np.float32)
    points = _sample_surface(points, N_POINTS)

    _save_object(
        os.path.join(PARAMETRIC_DIR, "small_cube.npz"),
        "small_cube", points, "pinch",
        {"side_m": 0.025},
    )


def gen_thin_plate():
    """Thin plate: 8.5x5.4x0.1 cm, lateral target."""
    rng = np.random.default_rng(42)
    hx, hy, hz = 0.0425, 0.027, 0.0005

    # For a very thin plate, most surface is on the top/bottom faces
    n_face = int(N_POINTS * 0.85)
    n_edge = N_POINTS - n_face

    # Top and bottom faces
    face_x = rng.uniform(-hx, hx, n_face)
    face_y = rng.uniform(-hy, hy, n_face)
    face_z = np.where(np.arange(n_face) < n_face // 2, hz, -hz)
    faces = np.column_stack([face_x, face_y, face_z])

    # Edges (4 thin strips)
    edge_idx = np.arange(n_edge)
    edges = np.zeros((n_edge, 3))
    for i in range(n_edge):
        side = i % 4
        if side == 0:  # +x edge
            edges[i] = [hx, rng.uniform(-hy, hy), rng.uniform(-hz, hz)]
        elif side == 1:  # -x edge
            edges[i] = [-hx, rng.uniform(-hy, hy), rng.uniform(-hz, hz)]
        elif side == 2:  # +y edge
            edges[i] = [rng.uniform(-hx, hx), hy, rng.uniform(-hz, hz)]
        else:  # -y edge
            edges[i] = [rng.uniform(-hx, hx), -hy, rng.uniform(-hz, hz)]

    points = np.vstack([faces, edges]).astype(np.float32)

    _save_object(
        os.path.join(PARAMETRIC_DIR, "thin_plate.npz"),
        "thin_plate", points, "lateral",
        {"width_m": 0.085, "depth_m": 0.054, "thickness_m": 0.001},
    )


def gen_mug_with_handle():
    """Mug with handle: cylinder body + C-shaped handle on the side.
    Non-convex — handle interior is occluded from most viewpoints.
    """
    rng = np.random.default_rng(42)
    body_r, body_h = 0.035, 0.09
    handle_r = 0.008  # handle cross-section radius
    handle_center_r = 0.055  # distance from mug axis to handle center
    handle_z_range = (0.02, 0.07)

    # Body surface points (cylinder)
    body_pts = []
    n_body = int(N_POINTS * 0.65)
    for _ in range(n_body * 3):
        z = rng.uniform(0, body_h)
        theta = rng.uniform(0, 2 * np.pi)
        body_pts.append([body_r * np.cos(theta), body_r * np.sin(theta), z])
    body_pts = np.array(body_pts, dtype=np.float32)
    # Surface mask: keep points near the cylinder surface
    r_pts = np.sqrt(body_pts[:, 0]**2 + body_pts[:, 1]**2)
    surface_mask = np.abs(r_pts - body_r) < 0.002
    body_pts = body_pts[surface_mask][:n_body]

    # Handle: torus segment (C-shape)
    handle_pts = []
    n_handle = N_POINTS - len(body_pts)
    for _ in range(n_handle * 3):
        # Parametric torus: angle around the handle loop + angle around cross-section
        phi = rng.uniform(-np.pi * 0.7, np.pi * 0.7)  # partial torus (C-shape)
        theta = rng.uniform(0, 2 * np.pi)
        z_frac = rng.uniform(0, 1)
        z = handle_z_range[0] + z_frac * (handle_z_range[1] - handle_z_range[0])

        # Handle center in the +Y direction from mug axis
        cx = handle_r * np.cos(phi)
        cy = handle_center_r + handle_r * np.sin(phi)
        handle_pts.append([cx, cy, z])
    handle_pts = np.array(handle_pts, dtype=np.float32)
    handle_pts = _sample_surface(handle_pts, n_handle)

    points = np.vstack([body_pts, handle_pts]).astype(np.float32)
    points = _sample_surface(points, N_POINTS)

    _save_object(
        os.path.join(PARAMETRIC_DIR, "mug_with_handle.npz"),
        "mug_with_handle", points, "cylindrical",
        {"body_radius_m": body_r, "body_height_m": body_h,
         "handle_center_r_m": handle_center_r, "handle_r_m": handle_r},
    )


def gen_notched_box():
    """Box with a rectangular notch cut from one edge.
    Non-convex — the notch creates self-occlusion from oblique angles.
    """
    rng = np.random.default_rng(42)
    # Main box: 8x6x4 cm
    hx, hy, hz = 0.04, 0.03, 0.02
    # Notch: 3x6x2 cm cut from the +X side
    notch_hx, notch_hz = 0.015, 0.01

    dense = rng.uniform(
        np.array([-hx, -hy, -hz]),
        np.array([hx, hy, hz]),
        (N_POINTS * 5, 3),
    )

    # Surface mask: on any face of the outer box
    abs_d = np.abs(dense)
    dist_to_face = np.array([hx, hy, hz]) - abs_d
    on_outer_face = np.any(dist_to_face < 0.002, axis=1)

    # Notch mask: points inside the notch region
    in_notch = (
        (dense[:, 0] > (hx - notch_hx)) &
        (np.abs(dense[:, 1]) < hy) &
        (dense[:, 2] < (-hz + notch_hz))
    )

    # Keep surface points that are NOT inside the notch
    mask = on_outer_face & ~in_notch

    # Add notch interior surfaces
    notch_pts = rng.uniform(
        np.array([hx - notch_hx, -hy, -hz]),
        np.array([hx, hy, -hz + notch_hz]),
        (N_POINTS * 2, 3),
    )
    abs_n = np.abs(notch_pts)
    # Notch surfaces: on any face of the notch cavity
    notch_face_dist = np.array([
        notch_pts[:, 0] - (hx - notch_hx),  # left face
        hx - notch_pts[:, 0],  # right face (shared with outer)
        hy - abs_n[:, 1],  # front/back
        notch_pts[:, 2] - (-hz),  # bottom
        (-hz + notch_hz) - notch_pts[:, 2],  # top
    ]).T
    on_notch_face = np.any(notch_face_dist < 0.002, axis=1)
    notch_pts = notch_pts[on_notch_face]

    points = np.vstack([dense[mask], notch_pts]).astype(np.float32)
    points = _sample_surface(points, N_POINTS)

    _save_object(
        os.path.join(PARAMETRIC_DIR, "notched_box.npz"),
        "notched_box", points, "pinch",
        {"box_m": [0.08, 0.06, 0.04], "notch_m": [0.03, 0.06, 0.02]},
    )


def gen_cross_shape():
    """Cross/plus shape: two perpendicular rectangular bars.
    Non-convex — the junction creates self-occlusion from many angles.
    """
    rng = np.random.default_rng(42)
    # Bar 1: 10x3x3 cm along X
    # Bar 2: 3x10x3 cm along Y
    bar_len = 0.05  # half-length
    bar_thick = 0.015  # half-thickness

    # Generate surface points for bar 1
    dense1 = rng.uniform(
        np.array([-bar_len, -bar_thick, -bar_thick]),
        np.array([bar_len, bar_thick, bar_thick]),
        (N_POINTS * 3, 3),
    )
    abs_d1 = np.abs(dense1)
    face_dist1 = np.array([bar_len, bar_thick, bar_thick]) - abs_d1
    surf_mask1 = np.any(face_dist1 < 0.002, axis=1)

    # Generate surface points for bar 2
    dense2 = rng.uniform(
        np.array([-bar_thick, -bar_len, -bar_thick]),
        np.array([bar_thick, bar_len, bar_thick]),
        (N_POINTS * 3, 3),
    )
    abs_d2 = np.abs(dense2)
    face_dist2 = np.array([bar_thick, bar_len, bar_thick]) - abs_d2
    surf_mask2 = np.any(face_dist2 < 0.002, axis=1)

    points = np.vstack([dense1[surf_mask1], dense2[surf_mask2]]).astype(np.float32)
    points = _sample_surface(points, N_POINTS)

    _save_object(
        os.path.join(PARAMETRIC_DIR, "cross_shape.npz"),
        "cross_shape", points, "cylindrical",
        {"bar_length_m": 0.10, "bar_thickness_m": 0.03},
    )


# ---------------------------------------------------------------------------
# YCB mesh processing
# ---------------------------------------------------------------------------

YCB_OBJECTS = {
    "banana": {
        "expected_grasp": "cylindrical",
    },
    "mug": {
        "expected_grasp": "cylindrical",
    },
    "power_drill": {
        "expected_grasp": "cylindrical",
    },
}


def _find_mesh_files() -> dict:
    """Find .obj mesh files in the YCB directory.

    Handles two layouts:
      1. Flat: objects/ycb/banana.obj
      2. Subdirectory: objects/ycb/011_banana/textured.obj

    Returns dict mapping object_name -> mesh_path.
    """
    result = {}
    if not os.path.isdir(YCB_DIR):
        return result

    # Layout 1: flat .obj files directly in YCB_DIR
    for f in sorted(os.listdir(YCB_DIR)):
        if f.endswith(".obj"):
            name = os.path.splitext(f)[0]
            if name in YCB_OBJECTS:
                result[name] = os.path.join(YCB_DIR, f)

    # Layout 2: subdirectories (e.g. 011_banana/textured.obj)
    for entry in sorted(os.listdir(YCB_DIR)):
        subdir = os.path.join(YCB_DIR, entry)
        if not os.path.isdir(subdir):
            continue
        # Match subdirectory name to YCB_OBJECTS key
        for obj_name in YCB_OBJECTS:
            if obj_name in entry or entry.endswith(obj_name):
                # Find any .obj inside
                for f in os.listdir(subdir):
                    if f.endswith(".obj"):
                        result[obj_name] = os.path.join(subdir, f)
                        break

    return result


def gen_ycb_objects():
    """Process YCB mesh files into .npz point clouds."""
    try:
        import trimesh
    except ImportError:
        print("WARNING: trimesh not installed. Skipping YCB objects.")
        print("  Install with: pip install trimesh")
        return

    mesh_files = _find_mesh_files()

    if not mesh_files:
        print("  No YCB meshes found. Expected .obj files in:")
        print(f"    {YCB_DIR}/banana.obj")
        print(f"    {YCB_DIR}/mug.obj")
        print(f"    {YCB_DIR}/power_drill.obj")
        print("  Or subdirectories like 011_banana/textured.obj")
        return

    for obj_name, mesh_path in mesh_files.items():
        info = YCB_OBJECTS[obj_name]
        print(f"  Loading {mesh_path}...")
        mesh = trimesh.load(mesh_path, force="mesh")

        # Sample surface points
        points, _ = trimesh.sample.sample_surface(mesh, N_POINTS)
        points = points.astype(np.float32)

        # Center at origin
        points -= points.mean(axis=0)

        # Scale to reasonable size (YCB meshes are in mm or m depending on source)
        max_extent = np.abs(points).max()
        if max_extent > 1.0:
            # Likely in mm, convert to m
            points *= 0.001
        elif max_extent < 0.01:
            # Likely very small, scale up
            points *= 0.1

        dims = {
            "x_range": float(np.ptp(points[:, 0])),
            "y_range": float(np.ptp(points[:, 1])),
            "z_range": float(np.ptp(points[:, 2])),
        }

        out_path = os.path.join(YCB_DIR, f"{obj_name}.npz")
        _save_object(out_path, obj_name, points, info["expected_grasp"], dims)


# ---------------------------------------------------------------------------
# Parametric fallbacks for YCB objects (used when meshes are unavailable)
# ---------------------------------------------------------------------------

def gen_banana_fallback():
    """Curved cylinder approximating a banana."""
    rng = np.random.default_rng(42)
    r = 0.015  # 1.5 cm radius
    length = 0.12  # 12 cm
    curvature = 0.3  # bend factor

    n_side = int(N_POINTS * 0.95)
    n_cap = N_POINTS - n_side

    t = rng.uniform(0, 1, n_side)
    theta = rng.uniform(0, 2 * np.pi, n_side)

    # Center line curves along a circular arc
    cx = curvature * np.sin(t * np.pi * 0.8)
    cy = np.zeros(n_side)
    cz = (t - 0.5) * length

    # Radial offset
    dx = r * np.cos(theta)
    dy = r * np.sin(theta)

    side = np.column_stack([cx + dx, cy + dy, cz])

    # Caps
    r_cap = rng.uniform(0, r, n_cap)
    theta_cap = rng.uniform(0, 2 * np.pi, n_cap)
    t_cap = np.where(np.arange(n_cap) < n_cap // 2, 0.0, 1.0)
    cap = np.column_stack([
        curvature * np.sin(t_cap * np.pi * 0.8) + r_cap * np.cos(theta_cap),
        r_cap * np.sin(theta_cap),
        (t_cap - 0.5) * length,
    ])

    points = np.vstack([side, cap]).astype(np.float32)
    _save_object(
        os.path.join(PARAMETRIC_DIR, "banana_fallback.npz"),
        "banana_fallback", points, "cylindrical",
        {"radius_m": r, "length_m": length, "curvature": curvature},
    )


def gen_mug_fallback():
    """Cylinder + torus handle approximating a mug."""
    rng = np.random.default_rng(42)
    r_body = 0.035  # 3.5 cm
    h_body = 0.09  # 9 cm
    r_handle = 0.005  # handle tube radius
    r_handle_arc = 0.025  # handle arc radius

    n_body = int(N_POINTS * 0.7)
    n_handle = N_POINTS - n_body

    # Body (open-top cylinder)
    theta = rng.uniform(0, 2 * np.pi, n_body)
    z = rng.uniform(0, h_body, n_body)
    body = np.column_stack([r_body * np.cos(theta), r_body * np.sin(theta), z])

    # Handle: torus arc on one side
    t_arc = rng.uniform(-np.pi * 0.4, np.pi * 0.4, n_handle)
    theta_tube = rng.uniform(0, 2 * np.pi, n_handle)
    # Arc center is offset from body
    cx = r_body + r_handle_arc
    handle_x = cx + (r_handle_arc + r_handle * np.cos(theta_tube)) * np.cos(t_arc)
    handle_y = (r_handle_arc + r_handle * np.cos(theta_tube)) * np.sin(t_arc) * 0  # flatten
    handle_z = (h_body * 0.5) + (r_handle_arc + r_handle * np.cos(theta_tube)) * np.sin(t_arc)
    handle = np.column_stack([handle_x, handle_y, handle_z])

    points = np.vstack([body, handle]).astype(np.float32)
    points -= points.mean(axis=0)

    _save_object(
        os.path.join(PARAMETRIC_DIR, "mug_fallback.npz"),
        "mug_fallback", points, "cylindrical",
        {"body_radius_m": r_body, "body_height_m": h_body},
    )


def gen_drill_fallback():
    """Elongated box + perpendicular handle approximating a power drill."""
    rng = np.random.default_rng(42)
    n_body = int(N_POINTS * 0.65)
    n_handle = N_POINTS - n_body

    # Main body: 6x3x3 cm box
    body = rng.uniform(-1, 1, (n_body * 5, 3))
    body *= np.array([0.06, 0.015, 0.015])
    # Surface filter
    abs_b = np.abs(body)
    half = np.array([0.06, 0.015, 0.015])
    dist = half - abs_b
    mask = np.any(dist < 0.002, axis=1)
    body = body[mask][:n_body]

    # Handle: 3x2x6 cm box, attached at bottom of body, perpendicular
    handle = rng.uniform(-1, 1, ((N_POINTS - len(body)) * 5, 3))
    handle *= np.array([0.015, 0.01, 0.03])
    # Offset: attached to bottom of body, extending downward
    handle[:, 0] += 0.0  # centered
    handle[:, 2] -= 0.03 + 0.015  # below body
    abs_h = np.abs(handle - np.array([0, 0, -0.045]))
    half_h = np.array([0.015, 0.01, 0.03])
    dist_h = half_h - abs_h
    mask_h = np.any(dist_h < 0.002, axis=1)
    handle = handle[mask_h][:N_POINTS - len(body)]

    points = np.vstack([body, handle]).astype(np.float32)
    points = _sample_surface(points, N_POINTS)

    _save_object(
        os.path.join(PARAMETRIC_DIR, "drill_fallback.npz"),
        "drill_fallback", points, "cylindrical",
        {"body_m": [0.12, 0.03, 0.03], "handle_m": [0.03, 0.02, 0.06]},
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

PARAMETRIC_GENERATORS = [
    gen_cylinder_upright,
    gen_cylinder_tilted,
    gen_ellipsoid,
    gen_tapered_bottle,
    gen_l_block,
    gen_small_cube,
    gen_thin_plate,
    # Non-convex objects for multi-view advantage evaluation
    gen_mug_with_handle,
    gen_notched_box,
    gen_cross_shape,
]

FALLBACK_GENERATORS = [
    gen_banana_fallback,
    gen_mug_fallback,
    gen_drill_fallback,
]


def main():
    parser = argparse.ArgumentParser(description="Generate test object point clouds")
    parser.add_argument("--ycb", action="store_true", help="Process YCB meshes")
    parser.add_argument("--fallbacks", action="store_true",
                        help="Generate parametric fallbacks for YCB objects")
    parser.add_argument("--all", action="store_true", help="Generate everything")
    args = parser.parse_args()

    os.makedirs(PARAMETRIC_DIR, exist_ok=True)
    os.makedirs(YCB_DIR, exist_ok=True)

    print("Generating parametric objects...")
    for gen in PARAMETRIC_GENERATORS:
        gen()

    if args.ycb or args.all:
        print("\nProcessing YCB meshes...")
        gen_ycb_objects()

    if args.fallbacks or args.all:
        print("\nGenerating YCB fallbacks...")
        for gen in FALLBACK_GENERATORS:
            gen()

    print("\nDone. Object files in:", PARAMETRIC_DIR)
    if args.ycb or args.all:
        print("YCB files in:", YCB_DIR)


if __name__ == "__main__":
    main()
