#!/usr/bin/env python3
"""
Diagnostic inspector for grasp debug dump (.npz) files.

Usage:
    python inspect_dump.py <path/to/grasp_dump_XXXXXX_XXXXXX.npz>

Produces a multi-section report to determine why grasp markers appear far
from the point-cloud / TSDF when viewed in the PyVista debug visualizer.
No GUI dependencies -- only numpy is required.
"""

import sys
import os
import numpy as np

# ---------------------------------------------------------------------------
# Constants (must match runtime_config.rs / config.rs)
# ---------------------------------------------------------------------------
TRUNCATION_CELLS = 4

# Columns per scored_grasps row (see debug_export.rs:203-217)
G_COL = 29
G_SAMPLE_IDX      = 0
G_GRASP_TYPE      = 1   # 1=cyl, 2=pinch, 3=lat
G_CLOSURE         = 2
G_ALIGNMENT       = 3
G_FORCE_CLOSURE   = 4
G_CONTACT_COUNT   = 5
G_CONTACT_SCORE   = 6
G_ACTIVE_CONTACTS = 7
G_FOUND_COLLISION = 8   # 0.0 or 1.0
G_COMBINED        = 9
G_PROBABILITY     = 10
G_WRIST_ROT       = 11  # radians
G_SMC_ITER        = 12
G_POSE_START      = 13  # 16 f64 values -> row-major 4x4


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def load_dump(path: str) -> dict:
    """Load and validate a debug .npz dump."""
    if not os.path.exists(path):
        raise FileNotFoundError(f"File not found: {path}")
    dump = np.load(path, allow_pickle=False)
    required = ["point_cloud", "tsdf_volume", "tsdf_metadata",
                "roi_aabb", "input_pose", "input_twist", "scored_grasps"]
    for key in required:
        if key not in dump:
            raise KeyError(f"Missing required array '{key}' in {path}")
    return dump


def pose_pos_quat(pose7: np.ndarray) -> tuple:
    """Split input_pose [px,py,pz,qx,qy,qz,qw] -> (pos, quat_xyzw)."""
    return pose7[:3], pose7[3:]


def centroid_of_cloud(flat: np.ndarray) -> np.ndarray:
    """flat interleaved (N*3,) f32 -> centroid (3,)."""
    pts = flat.reshape(-1, 3)
    return pts.mean(axis=0)


def aabb_of_cloud(flat: np.ndarray):
    """Return (min_xyz, max_xyz) for flat N*3 cloud."""
    pts = flat.reshape(-1, 3)
    return pts.min(axis=0), pts.max(axis=0)


def tsdf_bounds(meta: np.ndarray):
    """Return ((ox,oy,oz), resolution, (W,H,D), (max_x,max_y,max_z))."""
    ox, oy, oz = float(meta[0]), float(meta[1]), float(meta[2])
    res = float(meta[3])
    w, h, d = int(meta[4]), int(meta[5]), int(meta[6])
    mx = ox + w * res
    my = oy + h * res
    mz = oz + d * res
    return (ox, oy, oz), res, (w, h, d), (mx, my, mz)


def tsdf_trilinear_sample(volume: np.ndarray, dims: tuple,
                          origin: tuple, res: float,
                          pos: np.ndarray) -> float:
    """Trilinear interpolation of TSDF value at world position pos (3,)."""
    w, h, d = dims
    ox, oy, oz = origin
    # Fractional voxel coordinates
    i = (pos[0] - ox) / res
    j = (pos[1] - oy) / res
    k = (pos[2] - oz) / res
    # Check bounds - need at least i0 and i1 to be valid
    if i < 0 or j < 0 or k < 0 or i >= w - 1 or j >= h - 1 or k >= d - 1:
        return np.nan
    i0, j0, k0 = int(np.floor(i)), int(np.floor(j)), int(np.floor(k))
    i1, j1, k1 = i0 + 1, j0 + 1, k0 + 1
    di, dj, dk = i - i0, j - j0, k - k0

    def idx(ii, jj, kk):
        return kk * w * h + jj * w + ii

    # 8 corners
    c000 = volume[idx(i0, j0, k0)]
    c100 = volume[idx(i1, j0, k0)]
    c010 = volume[idx(i0, j1, k0)]
    c110 = volume[idx(i1, j1, k0)]
    c001 = volume[idx(i0, j0, k1)]
    c101 = volume[idx(i1, j0, k1)]
    c011 = volume[idx(i0, j1, k1)]
    c111 = volume[idx(i1, j1, k1)]
    # Trilinear
    c00 = c000 * (1 - di) + c100 * di
    c01 = c001 * (1 - di) + c101 * di
    c10 = c010 * (1 - di) + c110 * di
    c11 = c011 * (1 - di) + c111 * di
    c0 = c00 * (1 - dj) + c10 * dj
    c1 = c01 * (1 - dj) + c11 * dj
    return c0 * (1 - dk) + c1 * dk


def grasp_type_name(gt: int) -> str:
    return {1: "cylindrical", 2: "pinch", 3: "lateral"}.get(gt, f"unknown({gt})")


def tier_label(cs: float) -> str:
    if cs >= 0.8:
        return "Tier1"
    elif cs >= 0.25:
        return "Tier2"
    else:
        return "Tier3/4"


def closest_cloud_dist(pos: np.ndarray, cloud_pts: np.ndarray) -> float:
    """Minimum Euclidean distance from pos to any point in cloud_pts (N,3)."""
    return float(np.linalg.norm(cloud_pts - pos, axis=1).min())


def quat_to_rot_matrix(q: np.ndarray) -> np.ndarray:
    """Convert quaternion [qx, qy, qz, qw] to 3x3 rotation matrix."""
    qx, qy, qz, qw = q
    R = np.array([
        [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,         1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,         2*qy*qz + 2*qx*qw,     1 - 2*qx**2 - 2*qy**2],
    ])
    return R


# ---------------------------------------------------------------------------
# Main report
# ---------------------------------------------------------------------------

def inspect(path: str) -> None:
    dump = load_dump(path)

    # -- unpack ------------------------------------------------------------------
    cloud_flat = dump["point_cloud"].astype(np.float64)          # (N*3,)
    tsdf_vol = dump["tsdf_volume"].astype(np.float64)            # (W*H*D,)
    tsdf_meta = dump["tsdf_metadata"]                            # (7,) f64
    roi_flat = dump["roi_aabb"].astype(np.float64)               # (6,)
    cameras_flat = dump["cameras"].astype(np.float64)            # (C*3,)
    input_pose = dump["input_pose"]                              # (7,) f64
    input_twist = dump["input_twist"]                            # (6,) f64
    grasps_flat = dump["scored_grasps"].astype(np.float64)        # (M*29,)
    has_sq = "sq_params" in dump and len(dump["sq_params"]) >= 14

    n_pts = len(cloud_flat) // 3
    n_grasps = len(grasps_flat) // G_COL
    cloud_pts = cloud_flat.reshape(n_pts, 3)
    grasps = grasps_flat.reshape(n_grasps, G_COL)
    cameras = cameras_flat.reshape(-1, 3) if len(cameras_flat) >= 3 else np.empty((0, 3))

    # --- Sort grasps by combined_score descending for "top N" queries ---
    sort_idx = np.argsort(grasps[:, G_COMBINED])[::-1]
    grasps_sorted = grasps[sort_idx]

    tsdf_origin, tsdf_res, tsdf_dims, tsdf_max = tsdf_bounds(tsdf_meta)
    roi_min, roi_max = roi_flat[:3], roi_flat[3:]
    ip_pos, ip_quat = pose_pos_quat(input_pose)
    cloud_centroid = centroid_of_cloud(cloud_flat)
    cloud_min, cloud_max = aabb_of_cloud(cloud_flat)

    # =========================================================================
    # SECTION 1 -- Coordinate frame consistency
    # =========================================================================
    print("=" * 72)
    print("SECTION 1 -- Coordinate Frame Consistency")
    print("=" * 72)
    dist_ip_to_cloud_centroid = float(np.linalg.norm(ip_pos - cloud_centroid))
    dist_ip_to_roi_center = float(np.linalg.norm(ip_pos - (roi_min + roi_max) / 2.0))
    print(f"  Input pose pos           = [{ip_pos[0]:.4f}, {ip_pos[1]:.4f}, {ip_pos[2]:.4f}]")
    print(f"  Point cloud centroid      = [{cloud_centroid[0]:.4f}, {cloud_centroid[1]:.4f}, {cloud_centroid[2]:.4f}]")
    print(f"  ROI box center            = [{(roi_min[0]+roi_max[0])/2:.4f}, {(roi_min[1]+roi_max[1])/2:.4f}, {(roi_min[2]+roi_max[2])/2:.4f}]")
    print(f"  Distance IP -> cloud ctr  = {dist_ip_to_cloud_centroid:.4f} m")
    print(f"  Distance IP -> ROI center = {dist_ip_to_roi_center:.4f} m")
    if dist_ip_to_cloud_centroid > 0.20:
        print("  *** WARNING: Input pose is > 0.20 m from point cloud centroid!")
        print("    -> Suspect coordinate frame mismatch (hand pose vs point cloud).")
        print("    -> Check that cloud.header.frame_id == pose.header.frame_id in the C++ bridge.")
    else:
        print("  OK: Input pose is near the point cloud -- frames likely consistent.")

    # =========================================================================
    # SECTION 2 -- Particle drift from input pose
    # =========================================================================
    print()
    print("=" * 72)
    print("SECTION 2 -- Particle Drift from Input Pose")
    print("=" * 72)
    top_n = min(10, n_grasps)
    distances = []
    for i in range(top_n):
        row = grasps_sorted[i]
        T = row[G_POSE_START:G_POSE_START + 16].reshape(4, 4)
        gpos = T[:3, 3]
        d = float(np.linalg.norm(gpos - ip_pos))
        distances.append(d)
    distances = np.array(distances)
    print(f"  Top {top_n} grasps (by combined_score) distance to input pose:")
    print(f"    min    = {distances.min():.4f} m")
    print(f"    median = {float(np.median(distances)):.4f} m")
    print(f"    max    = {distances.max():.4f} m")
    if float(np.median(distances)) > 0.15:
        print("  *** WARNING: Median particle drift > 0.15 m!")
        print("    -> Particles are drifting far from the current hand pose.")
        print("    -> Check twist units (rad/s vs deg/s, m/s vs cm/s) and displacement sampling.")
    else:
        print("  OK: Particles are close to the input pose -- sampling seems reasonable.")

    # =========================================================================
    # SECTION 2b -- Rotation of top grasps vs input pose
    # =========================================================================
    print()
    print("=" * 72)
    print("SECTION 2b -- Grasp Orientation vs Input Pose")
    print("=" * 72)
    R_ip = quat_to_rot_matrix(ip_quat)
    print(f"  Input pose rotation (from quaternion):")
    for row in R_ip:
        print(f"    [{row[0]:.4f}, {row[1]:.4f}, {row[2]:.4f}]")
    for i in range(min(3, n_grasps)):
        row = grasps_sorted[i]
        T = row[G_POSE_START:G_POSE_START + 16].reshape(4, 4)
        R_g = T[:3, :3]
        # Angle between input pose and grasp pose
        R_diff = R_g @ R_ip.T
        trace = np.trace(R_diff)
        angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))
        angle_deg = np.degrees(angle)
        print(f"  Grasp #{i+1} rotation:")
        for rrow in R_g:
            print(f"    [{rrow[0]:.4f}, {rrow[1]:.4f}, {rrow[2]:.4f}]")
        print(f"    Angle from input pose = {angle_deg:.1f} deg")
    # Check if top grasps have a consistent rotation offset
    angles = []
    for i in range(min(20, n_grasps)):
        row = grasps_sorted[i]
        T = row[G_POSE_START:G_POSE_START + 16].reshape(4, 4)
        R_g = T[:3, :3]
        R_diff = R_g @ R_ip.T
        trace = np.trace(R_diff)
        angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))
        angles.append(np.degrees(angle))
    angles = np.array(angles)
    print(f"  Top {len(angles)} grasps angle from input pose:")
    print(f"    min={angles.min():.1f} deg, median={float(np.median(angles)):.1f} deg, max={angles.max():.1f} deg")
    if float(np.median(angles)) > 45:
        print("  *** WARNING: Grasps are rotated >45 deg from input pose!")
        print("    -> May explain the 90 deg rotation observation.")
    print()
    print("=" * 72)
    print("SECTION 3 -- Top-Grasp Distance to Point Cloud")
    print("=" * 72)
    rng = np.random.default_rng(0)
    subset_size = min(2000, n_pts)
    cloud_subset = cloud_pts[rng.choice(n_pts, size=subset_size, replace=False)]
    cloud_dists = []
    for i in range(top_n):
        row = grasps_sorted[i]
        T = row[G_POSE_START:G_POSE_START + 16].reshape(4, 4)
        gpos = T[:3, 3]
        d = closest_cloud_dist(gpos, cloud_subset)
        cloud_dists.append(d)
    cloud_dists = np.array(cloud_dists)
    print(f"  Closest point-cloud distance for top {top_n} grasps:")
    print(f"    min    = {cloud_dists.min():.4f} m")
    print(f"    median = {float(np.median(cloud_dists)):.4f} m")
    print(f"    max    = {cloud_dists.max():.4f} m")
    if cloud_dists.min() > 0.10:
        print("  *** WARNING: Not a single grasp is within 0.10 m of the point cloud!")
        print("    -> Grasps are truly in empty space -- confirms visual observation.")
    else:
        print("  OK: At least some grasps are near the point cloud.")

    # =========================================================================
    # SECTION 4 -- TSDF lookup at grasp positions
    # =========================================================================
    print()
    print("=" * 72)
    print("SECTION 4 -- TSDF Lookup at Top-Grasp Positions")
    print("=" * 72)
    trunc_dist_m = TRUNCATION_CELLS * tsdf_res
    print(f"  TSDF origin         = [{tsdf_origin[0]:.4f}, {tsdf_origin[1]:.4f}, {tsdf_origin[2]:.4f}]")
    print(f"  TSDF resolution     = {tsdf_res:.4f} m/cell")
    print(f"  TSDF dims (W,H,D)   = {tsdf_dims}")
    print(f"  TSDF max corner     = [{tsdf_max[0]:.4f}, {tsdf_max[1]:.4f}, {tsdf_max[2]:.4f}]")
    print(f"  Truncation dist     = {trunc_dist_m:.4f} m ({TRUNCATION_CELLS} cells)")
    print(f"  Top {min(top_n, 5)} grasp positions:")
    for i in range(min(top_n, 5)):
        row = grasps_sorted[i]
        T = row[G_POSE_START:G_POSE_START + 16].reshape(4, 4)
        gpos = T[:3, 3]
        tsdf_val = tsdf_trilinear_sample(tsdf_vol, tsdf_dims, tsdf_origin, tsdf_res, gpos)
        combined = row[G_COMBINED]
        cs = row[G_CONTACT_SCORE]
        in_band = not np.isnan(tsdf_val) and abs(tsdf_val) < TRUNCATION_CELLS
        flag = "OK in-band" if in_band else "XX outside/truncated"
        print(f"    rank {i+1}: pos=[{gpos[0]:.3f}, {gpos[1]:.3f}, {gpos[2]:.3f}]  "
              f"TSDF={tsdf_val:+.3f}  combined={combined:.4f}  contact={cs:.4f}  {flag}")
    all_trunc = all(
        (lambda v: np.isnan(v) or abs(v) >= TRUNCATION_CELLS)(
            tsdf_trilinear_sample(tsdf_vol, tsdf_dims, tsdf_origin, tsdf_res,
                                  grasps_sorted[i, G_POSE_START:G_POSE_START+16].reshape(4,4)[:3,3])
        )
        for i in range(min(top_n, 5))
    )
    if all_trunc:
        print("  *** WARNING: All top grasps are at the TSDF truncation boundary!")
        print("    -> The hand base positions are completely outside the observed TSDF band.")

    # =========================================================================
    # SECTION 5 -- Wrist rotation across SMC iterations
    # =========================================================================
    print()
    print("=" * 72)
    print("SECTION 5 -- Wrist Rotation Across SMC Iterations")
    print("=" * 72)
    iterations = np.unique(grasps[:, G_SMC_ITER].astype(int))
    if len(iterations) == 0:
        print("  No grasps found.")
    else:
        print(f"  {'Iter':>6s}  {'Count':>8s}  {'Mean(deg)':>10s}  {'Median(deg)':>10s}  "
              f"{'Std(deg)':>10s}  {'Min(deg)':>10s}  {'Max(deg)':>10s}")
        print("  " + "-" * 64)
        for it in sorted(iterations):
            mask = grasps[:, G_SMC_ITER].astype(int) == it
            wr = grasps[mask, G_WRIST_ROT]
            wr_deg = np.degrees(wr)
            print(f"  {it:6d}  {mask.sum():8d}  {wr_deg.mean():10.2f}  "
                  f"{float(np.median(wr_deg)):10.2f}  {wr_deg.std():10.2f}  "
                  f"{wr_deg.min():10.2f}  {wr_deg.max():10.2f}")
        if len(iterations) >= 2:
            means = []
            for it in sorted(iterations):
                mask = grasps[:, G_SMC_ITER].astype(int) == it
                means.append(float(np.mean(np.abs(grasps[mask, G_WRIST_ROT]))))
            if all(m2 > m1 * 1.3 for m1, m2 in zip(means, means[1:])):
                print("  *** WARNING: Mean |wrist_rotation| grows with each SMC iteration!")
                print("    -> Confirms wrist rotation accumulation bug in resample_around_elites.")
            else:
                print("  OK: Wrist rotation does not appear to accumulate across iterations.")

    # =========================================================================
    # SECTION 6 -- Twist magnitude sanity
    # =========================================================================
    print()
    print("=" * 72)
    print("SECTION 6 -- Twist Magnitude Sanity Check")
    print("=" * 72)
    linear = input_twist[:3]
    angular = input_twist[3:]
    lin_norm = float(np.linalg.norm(linear))
    ang_norm = float(np.linalg.norm(angular))
    print(f"  Linear  = [{linear[0]:.6f}, {linear[1]:.6f}, {linear[2]:.6f}]  norm = {lin_norm:.4f} m/s")
    print(f"  Angular = [{angular[0]:.6f}, {angular[1]:.6f}, {angular[2]:.6f}]  norm = {ang_norm:.4f} rad/s ({np.degrees(ang_norm):.1f} deg/s)")
    if ang_norm > 10.0:
        print("  *** WARNING: Angular velocity norm > 10 rad/s (~573 deg/s) -- implausible!")
        print("    -> Suspect units mismatch: degrees/s instead of rad/s?")
    elif ang_norm > 5.0:
        print("  *** CAUTION: Angular velocity norm is high (> 5 rad/s). Verify units.")
    else:
        print("  OK: Angular velocity appears reasonable.")
    if lin_norm > 5.0:
        print("  *** WARNING: Linear velocity norm > 5.0 m/s -- implausible!")
        print("    -> Suspect units mismatch: cm/s instead of m/s?")
    else:
        print("  OK: Linear velocity appears reasonable.")

    # =========================================================================
    # SECTION 7 -- Score tier classification
    # =========================================================================
    print()
    print("=" * 72)
    print("SECTION 7 -- Score Tier Classification")
    print("=" * 72)
    tiers = np.array([tier_label(cs) for cs in grasps_sorted[:, G_CONTACT_SCORE]])
    unique_tiers, tier_counts = np.unique(tiers, return_counts=True)
    print("  All grasps tier distribution:")
    for t, c in zip(unique_tiers, tier_counts):
        print(f"    {t:>8s}: {c} ({100*c/len(tiers):.1f}%)")
    print(f"  Top {min(top_n, 10)} grasps (by combined_score):")
    print(f"    {'Rank':>5s}  {'Type':>12s}  {'Combined':>9s}  "
          f"{'Contact':>8s}  {'Align':>7s}  {'FC':>6s}  {'CC':>6s}  "
          f"{'Active':>6s}  {'Coll':>5s}  {'Tier':>7s}")
    print("    " + "-" * 72)
    for i in range(min(top_n, 10)):
        row = grasps_sorted[i]
        gt = int(row[G_GRASP_TYPE])
        print(f"    {i+1:5d}  {grasp_type_name(gt):>12s}  {row[G_COMBINED]:9.5f}  "
              f"{row[G_CONTACT_SCORE]:8.4f}  {row[G_ALIGNMENT]:7.4f}  "
              f"{row[G_FORCE_CLOSURE]:6.4f}  {row[G_CONTACT_COUNT]:6.4f}  "
              f"{int(row[G_ACTIVE_CONTACTS]):6d}  {int(row[G_FOUND_COLLISION]):5d}  "
              f"{tier_label(row[G_CONTACT_SCORE]):>7s}")
    tier1_count = int(np.sum(grasps[:, G_CONTACT_SCORE] >= 0.8))
    if tier1_count == 0:
        print("  *** WARNING: Zero Tier 1 grasps (contact_score >= 0.8)!")
        print("    -> The planner never found valid surface contacts.")
        print("    -> All scores are proximity-based fallback values <= 0.05.")
    else:
        print(f"  OK: Found {tier1_count} Tier 1 grasps with genuine surface contacts.")

    # =========================================================================
    # SECTION 8 -- ROI overlap with point cloud
    # =========================================================================
    print()
    print("=" * 72)
    print("SECTION 8 -- ROI vs Point Cloud Overlap")
    print("=" * 72)
    in_roi = np.all((cloud_pts >= roi_min) & (cloud_pts <= roi_max), axis=1)
    n_in_roi = int(in_roi.sum())
    pct_in_roi = 100.0 * n_in_roi / n_pts
    print(f"  ROI min  = [{roi_min[0]:.4f}, {roi_min[1]:.4f}, {roi_min[2]:.4f}]")
    print(f"  ROI max  = [{roi_max[0]:.4f}, {roi_max[1]:.4f}, {roi_max[2]:.4f}]")
    print(f"  ROI size = [{roi_max[0]-roi_min[0]:.3f}, {roi_max[1]-roi_min[1]:.3f}, {roi_max[2]-roi_min[2]:.3f}] m")
    print(f"  Points in cloud: {n_pts}")
    print(f"  Points inside ROI: {n_in_roi} ({pct_in_roi:.1f}%)")
    if pct_in_roi < 1.0:
        if dist_ip_to_cloud_centroid > 0.20:
            print("  *** CRITICAL: ROI and point cloud are in completely different regions!")
            print("    -> Combined with Section 1 finding, confirms frame mismatch.")
        else:
            print("  *** WARNING: Very few cloud points inside ROI despite input pose being near.")
            print("    -> ROI may be centered incorrectly (wrong motion model displacement?).")
    elif pct_in_roi < 50.0:
        print("  *** CAUTION: Less than 50% of points inside ROI -- possible partial coverage.")
    else:
        print("  OK: ROI covers a good portion of the point cloud.")

    # =========================================================================
    # SECTION 9 -- Spatial bounds comparison
    # =========================================================================
    print()
    print("=" * 72)
    print("SECTION 9 -- Spatial Bounds Comparison")
    print("=" * 72)
    print(f"  {'':>16s}  {'Min (m)':>36s}  {'Max (m)':>36s}")
    print(f"  {'Point cloud':>16s}  [{cloud_min[0]:.3f}, {cloud_min[1]:.3f}, {cloud_min[2]:.3f}]     "
          f"[{cloud_max[0]:.3f}, {cloud_max[1]:.3f}, {cloud_max[2]:.3f}]")
    print(f"  {'ROI':>16s}  [{roi_min[0]:.3f}, {roi_min[1]:.3f}, {roi_min[2]:.3f}]     "
          f"[{roi_max[0]:.3f}, {roi_max[1]:.3f}, {roi_max[2]:.3f}]")
    print(f"  {'TSDF':>16s}  [{tsdf_origin[0]:.3f}, {tsdf_origin[1]:.3f}, {tsdf_origin[2]:.3f}]     "
          f"[{tsdf_max[0]:.3f}, {tsdf_max[1]:.3f}, {tsdf_max[2]:.3f}]")
    print(f"  {'Input Pose':>16s}  [{ip_pos[0]:.3f}, {ip_pos[1]:.3f}, {ip_pos[2]:.3f}]")
    for i in range(min(3, n_grasps)):
        T = grasps_sorted[i][G_POSE_START:G_POSE_START+16].reshape(4,4)
        gpos = T[:3,3]
        print(f"  {'Grasp #'+str(i+1):>16s}  [{gpos[0]:.3f}, {gpos[1]:.3f}, {gpos[2]:.3f}]")
    tsdf_or = np.array(tsdf_origin)
    tsdf_mx = np.array(tsdf_max)
    cloud_in_tsdf = int(np.all((cloud_pts >= tsdf_or) & (cloud_pts <= tsdf_mx), axis=1).sum())
    roi_in_tsdf = (all(tsdf_or <= roi_min) and all(tsdf_mx >= roi_max))
    print()
    print(f"  Cloud points inside TSDF volume: {cloud_in_tsdf}/{n_pts} ({100*cloud_in_tsdf/n_pts:.1f}%)")
    print(f"  ROI fully inside TSDF volume: {'yes' if roi_in_tsdf else 'NO -- ROI extends beyond TSDF'}")

    # =========================================================================
    # SECTION 10 -- SQ proximity (optional)
    # =========================================================================
    if has_sq:
        print()
        print("=" * 72)
        print("SECTION 10 -- Superquadric Proximity at Grasp Positions")
        print("=" * 72)
        sq = dump["sq_params"].astype(np.float64)
        eps1, eps2, a, b, c = sq[0], sq[1], sq[2], sq[3], sq[4]
        tx, ty, tz = sq[5], sq[6], sq[7]
        # debug_export.rs only stores rows 0-1 of rotation (6 values: r00..r12).
        # Reconstruct row 2 via cross product to get a full 3x3 matrix.
        r0 = np.array([sq[8], sq[9], sq[10]])
        r1 = np.array([sq[11], sq[12], sq[13]])
        r2 = np.cross(r0, r1)
        R_sq = np.array([r0, r1, r2])  # 3x3
        sq_meta = dump.get("sq_meta", np.array([np.nan, np.nan]))
        print(f"  SQ shape: eps1={eps1:.3f}, eps2={eps2:.3f}, a={a:.4f}, b={b:.4f}, c={c:.4f}")
        print(f"  SQ center: [{tx:.4f}, {ty:.4f}, {tz:.4f}]")
        print(f"  SQ fit error: {sq_meta[0]:.6f}")
        print(f"  SQ template idx: {int(sq_meta[1])}")
        print(f"  SQ rotation det = {np.linalg.det(R_sq):.4f}")
        for i in range(min(5, n_grasps)):
            T = grasps_sorted[i][G_POSE_START:G_POSE_START+16].reshape(4,4)
            gpos = T[:3,3]
            p_local = R_sq.T @ (gpos - np.array([tx, ty, tz]))
            eps1_safe = max(float(eps1), 0.01)
            eps2_safe = max(float(eps2), 0.01)
            x_term = (abs(float(p_local[0])) / float(a)) ** (2.0/eps2_safe)
            y_term = (abs(float(p_local[1])) / float(b)) ** (2.0/eps2_safe)
            z_term = (abs(float(p_local[2])) / float(c)) ** (2.0/eps1_safe)
            F_val = (x_term + y_term) ** (eps2_safe/eps1_safe) + z_term - 1.0
            label = "(inside SQ)" if F_val < 0 else "(outside SQ)"
            print(f"    rank {i+1}: pos=[{gpos[0]:.3f}, {gpos[1]:.3f}, {gpos[2]:.3f}]  "
                  f"Taubin F = {F_val:+.4f}  {label}")

    # =========================================================================
    # SUMMARY VERDICT
    # =========================================================================
    print()
    print("=" * 72)
    print("SUMMARY VERDICT")
    print("=" * 72)

    issues = []

    if dist_ip_to_cloud_centroid > 0.20:
        issues.append(
            "COORDINATE FRAME MISMATCH: Input pose is {:.3f} m from point cloud centroid. "
            "The hand pose and point cloud are in different coordinate frames. "
            "Check that cloud.header.frame_id == pose.header.frame_id in the C++ bridge."
            .format(dist_ip_to_cloud_centroid))

    if len(iterations) >= 2:
        means_abs = []
        for it in sorted(iterations):
            mask = grasps[:, G_SMC_ITER].astype(int) == it
            means_abs.append(float(np.mean(np.abs(grasps[mask, G_WRIST_ROT]))))
        if all(m2 > m1 * 1.3 for m1, m2 in zip(means_abs, means_abs[1:])):
            issues.append(
                "WRIST ROTATION ACCUMULATION: Mean |wrist_rotation| grows across SMC iterations. "
                "Bug in resample_around_elites where wrist rotation is applied on top "
                "of the already-rotated elite pose. Explains ~90 deg orientation error.")

    if float(np.linalg.norm(input_twist[3:])) > 10.0:
        issues.append(
            "TWIST UNIT MISMATCH: Angular velocity norm = {:.1f} rad/s ({:.0f} deg/s). "
            "Likely degrees/s instead of rad/s."
            .format(float(np.linalg.norm(input_twist[3:])),
                    np.degrees(float(np.linalg.norm(input_twist[3:])))))
    if float(np.linalg.norm(input_twist[:3])) > 5.0:
        issues.append(
            "TWIST UNIT MISMATCH: Linear velocity norm = {:.1f} m/s. Likely cm/s instead of m/s."
            .format(float(np.linalg.norm(input_twist[:3]))))

    if tier1_count == 0 and dist_ip_to_cloud_centroid <= 0.20:
        issues.append(
            "NO SURFACE CONTACTS FOUND: Zero Tier 1 grasps despite input pose being near "
            "the point cloud. The planner never established valid surface contacts.")

    if cloud_dists.min() > 0.10 and dist_ip_to_cloud_centroid <= 0.20:
        issues.append(
            "GRASPS FAR FROM CLOUD DESPITE CORRECT FRAME: Input pose is near the cloud but "
            "all top grasps are >= {:.3f} m from it. Bug is in particle sampling or SMC, "
            "not in input coordinate frames."
            .format(cloud_dists.min()))

    if pct_in_roi < 1.0 and dist_ip_to_cloud_centroid <= 0.20:
        issues.append(
            "ROI MISPLACEMENT: Only {:.1f}% of cloud points inside ROI despite input pose "
            "being near the cloud."
            .format(pct_in_roi))

    if not issues:
        print("  OK: No clear issues detected from the dump file alone.")
        print("  The problem may be in the visualization script rendering or URDF FK chain.")
    else:
        for idx, issue in enumerate(issues, 1):
            print(f"  [{idx}] {issue}")

    print()
    print(f"  Dump file: {path}")
    print(f"  Point cloud: {n_pts} points")
    print(f"  Scored grasps: {n_grasps}")
    if len(cameras) > 0:
        print(f"  Cameras: {len(cameras)}")
    print(f"  TSDF volume: {tsdf_dims[0]}x{tsdf_dims[1]}x{tsdf_dims[2]} "
          f"= {tsdf_dims[0]*tsdf_dims[1]*tsdf_dims[2]} cells "
          f"@ {tsdf_res:.4f} m/cell = "
          f"[{tsdf_dims[0]*tsdf_res:.3f}x{tsdf_dims[1]*tsdf_res:.3f}x{tsdf_dims[2]*tsdf_res:.3f}] m")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <path/to/dump.npz>")
        sys.exit(1)
    inspect(sys.argv[1])
