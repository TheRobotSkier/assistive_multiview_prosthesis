#!/usr/bin/env python3
"""
Generate report figures from grasp preshaping debug dumps.

Usage:
    python report/generate_figures.py [--dump <path_to.npz>]

Outputs (in report/):
    tsdf_unsigned_cross_section.png   — Item 4: Unsigned TSDF cross-section
    tsdf_signed_cross_section.png     — Item 11: Signed TSDF + SQ overlay (+ ROI overlay)
    tsdf_signed_clean.png             — Item 11: Signed TSDF + SQ overlay (clean, no ROI)
    grasp_poses_overview.png          — Item 12: Best grasp poses + point cloud
"""

import argparse
import os
import sys

import numpy as np

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
TRUNCATION_CELLS = 8  # must match config.rs
SQ_TEMPLATE_NAMES = ["sphere", "box", "cylinder"]
GRASP_TYPE_COLORS = {1: "#e76f51", 2: "#2a9d8f", 3: "#457b9d"}
GRASP_TYPE_NAMES = {1: "Cylindrical", 2: "Pinch", 3: "Lateral"}
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------
def load_debug_dump(path: str) -> dict:
    data = np.load(path)
    meta = data["tsdf_metadata"].astype(np.float64)
    origin = meta[0:3]
    resolution = float(meta[3])
    shape = meta[4:7].astype(int)

    tsdf_flat = data["tsdf_volume"]
    tsdf = tsdf_flat.reshape(shape, order="F")

    pc = data["point_cloud"].reshape(-1, 3) if len(data["point_cloud"]) > 0 else np.zeros((0, 3))
    roi = data["roi_aabb"].reshape(2, 3)
    cams = data["cameras"].reshape(-1, 3) if len(data["cameras"]) > 0 else np.zeros((0, 3))

    # Parse grasps (29-column format expected)
    raw_len = len(data["scored_grasps"])
    if raw_len > 0:
        row_len = 29 if raw_len % 29 == 0 else 24
        grasps_raw = data["scored_grasps"].reshape(-1, row_len)
        grasps = {
            "grasp_type": grasps_raw[:, 1].astype(int),
            "closure": grasps_raw[:, 2],
            "combined": grasps_raw[:, 9],
            "contact_score": grasps_raw[:, 6],
            "pose_4x4": grasps_raw[:, 13:29].reshape(-1, 4, 4),
        }
        if row_len >= 29:
            grasps["smc_iteration"] = grasps_raw[:, 12].astype(int)
        else:
            grasps["smc_iteration"] = np.zeros(len(grasps_raw), dtype=int)
    else:
        grasps = {"grasp_type": np.zeros(0, dtype=int), "closure": np.zeros(0),
                  "combined": np.zeros(0), "contact_score": np.zeros(0),
                  "pose_4x4": np.zeros((0, 4, 4)),
                  "smc_iteration": np.zeros(0, dtype=int)}

    # Superquadric params
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
            sq_meta = {"fit_error": float(meta_raw[0]), "template_index": int(meta_raw[1])}

    return {
        "tsdf": tsdf,
        "tsdf_origin": origin,
        "tsdf_resolution": resolution,
        "tsdf_shape": shape,
        "point_cloud": pc,
        "roi": roi,
        "cameras": cams,
        "grasps": grasps,
        "sq_params": sq_params,
        "sq_meta": sq_meta,
    }


def _get_roi_samples(grasps, max_samples=1000):
    """Extract hand base translations from iteration-0 scored grasps.

    ``sample_initial_particles`` is documented as equivalent to
    ``sample_future_poses`` (predictor.rs:269), so iteration-0 scored
    grasp positions are an exact proxy for the distribution of motion
    model samples that define the ROI bounding box.
    Returns (N, 2) array of X,Y coordinates for cross-section projection.
    """
    n = len(grasps["combined"])
    if n == 0 or "smc_iteration" not in grasps:
        return np.zeros((0, 2))

    iter0_mask = grasps["smc_iteration"] == 0
    iter0_indices = np.where(iter0_mask)[0]
    n_iter0 = len(iter0_indices)
    if n_iter0 == 0:
        return np.zeros((0, 2))

    # Deterministic subset to avoid visual clutter
    n_show = min(n_iter0, max_samples)
    chosen = iter0_indices[np.linspace(0, n_iter0 - 1, n_show, dtype=int)]

    positions = grasps["pose_4x4"][chosen][:, :3, 3]
    return positions[:, :2]  # X,Y projection


def _find_best_grasp(grasps):
    n = len(grasps["combined"])
    best_idx = -1
    best_combined = -np.inf
    for i in range(n):
        if grasps["combined"][i] > best_combined:
            best_combined = grasps["combined"][i]
            best_idx = i
    return best_idx, best_combined


# ---------------------------------------------------------------------------
# Figure 1: Unsigned TSDF cross-section (Item 4)
# ---------------------------------------------------------------------------
def fig_tsdf_unsigned_cross_section(dump: dict):
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize

    tsdf = dump["tsdf"].astype(np.float64)
    origin = dump["tsdf_origin"]
    res = dump["tsdf_resolution"]
    W, H, D = dump["tsdf_shape"]

    unsigned = np.abs(tsdf)
    unsigned[tsdf > 1e10] = np.nan

    z_slice = D // 2
    slice_data = unsigned[:, :, z_slice]

    x_extent = np.array([origin[0], origin[0] + W * res])
    y_extent = np.array([origin[1], origin[1] + H * res])

    fig, ax = plt.subplots(1, 1, figsize=(8, 7))

    im = ax.imshow(
        slice_data.T, origin="lower",
        extent=[x_extent[0], x_extent[1], y_extent[0], y_extent[1]],
        cmap="viridis",
        norm=Normalize(vmin=0, vmax=TRUNCATION_CELLS),
        interpolation="nearest",
    )

    pc = dump["point_cloud"]
    z_val = origin[2] + z_slice * res
    pc_slice = pc[(np.abs(pc[:, 2] - z_val) < res * 2)]
    if len(pc_slice) > 0:
        ax.scatter(pc_slice[:, 0], pc_slice[:, 1], c="red", s=4, alpha=0.6, label="Point cloud")

    # Superquadric surface contour
    sq_params = dump["sq_params"]
    sq_meta = dump["sq_meta"]
    if sq_params is not None and sq_meta is not None:
        grid_res = 100
        xs = np.linspace(x_extent[0], x_extent[1], grid_res)
        ys = np.linspace(y_extent[0], y_extent[1], grid_res)
        X, Y = np.meshgrid(xs, ys)
        Z_fixed = np.full_like(X, z_val)
        F_vals = _sq_eval_grid(sq_params, X, Y, Z_fixed)
        ax.contour(X, Y, F_vals, levels=[0], colors="cyan", linewidths=2.5, linestyles="-")

    for ci, cam in enumerate(dump["cameras"]):
        if np.abs(cam[2] - z_val) < res * 3:
            ax.plot(cam[0], cam[1], marker="^", color="purple", markersize=10,
                    label=f"Camera {ci}" if ci == 0 else "")

    cbar = fig.colorbar(im, ax=ax, label="Unsigned distance (cells)", shrink=0.85)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title(f"Unsigned TSDF Cross-Section (z-slice {z_slice})")
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8)

    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, "tsdf_unsigned_cross_section.png")
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {path}")
    return path


# ---------------------------------------------------------------------------
# Figure 2: Signed TSDF cross-section with SQ (Item 11)
# ---------------------------------------------------------------------------
def _sq_eval_grid(sq_params, x_grid, y_grid, z_grid):
    eps1 = sq_params["epsilon1"]
    eps2 = sq_params["epsilon2"]
    a = max(sq_params["a"], 1e-6)
    b = max(sq_params["b"], 1e-6)
    c = max(sq_params["c"], 1e-6)

    rot_flat = sq_params["rotation"]
    rot = np.eye(3)
    rot[0, :] = rot_flat[0]
    rot[1, :] = rot_flat[1]
    rot[2, :] = np.cross(rot[0, :], rot[1, :])

    translation = sq_params["translation"]
    pts = np.stack([x_grid, y_grid, z_grid], axis=-1)
    pts_local = (pts - translation).dot(rot.T)

    x = pts_local[..., 0]
    y = pts_local[..., 1]
    z = pts_local[..., 2]

    term1 = (np.abs(x / a) ** (2 / eps1) + np.abs(y / b) ** (2 / eps1)) ** (eps1 / eps2)
    term2 = np.abs(z / c) ** (2 / eps2)
    F = (term1 + term2) ** eps2 - 1.0
    return F


def fig_tsdf_signed_with_sq(dump: dict, show_roi_overlay: bool = True):
    import matplotlib.pyplot as plt
    from matplotlib.colors import TwoSlopeNorm

    tsdf = dump["tsdf"].astype(np.float64)
    origin = dump["tsdf_origin"]
    res = dump["tsdf_resolution"]
    W, H, D = dump["tsdf_shape"]

    tsdf[tsdf > 1e10] = np.nan
    z_slice = D // 2
    slice_data = tsdf[:, :, z_slice]

    x_extent = np.array([origin[0], origin[0] + W * res])
    y_extent = np.array([origin[1], origin[1] + H * res])

    fig, ax = plt.subplots(1, 1, figsize=(8, 7))
    norm = TwoSlopeNorm(vmin=-TRUNCATION_CELLS, vcenter=0, vmax=TRUNCATION_CELLS)
    im = ax.imshow(
        slice_data.T, origin="lower",
        extent=[x_extent[0], x_extent[1], y_extent[0], y_extent[1]],
        cmap="coolwarm", norm=norm, interpolation="nearest",
    )

    pc = dump["point_cloud"]
    z_val = origin[2] + z_slice * res
    pc_slice = pc[(np.abs(pc[:, 2] - z_val) < res * 2)]
    if len(pc_slice) > 0:
        ax.scatter(pc_slice[:, 0], pc_slice[:, 1], c="lime", s=3, alpha=0.5, label="Point cloud")

    # ROI samples: iteration-0 scored grasp hand base positions
    # (equivalent to the motion-model samples that define the ROI)
    if show_roi_overlay:
        roi_samples_xy = _get_roi_samples(dump["grasps"], max_samples=1000)
        if len(roi_samples_xy) > 0:
            ax.scatter(
                roi_samples_xy[:, 0], roi_samples_xy[:, 1],
                c="#5099e9", s=1.2, alpha=0.30, label="ROI samples (iter 0)",
                rasterized=True,
            )

    sq_params = dump["sq_params"]
    sq_meta = dump["sq_meta"]
    if sq_params is not None and sq_meta is not None:
        grid_res = 100
        xs = np.linspace(x_extent[0], x_extent[1], grid_res)
        ys = np.linspace(y_extent[0], y_extent[1], grid_res)
        X, Y = np.meshgrid(xs, ys)
        Z_fixed = np.full_like(X, z_val)
        F_vals = _sq_eval_grid(sq_params, X, Y, Z_fixed)

        cs = ax.contour(X, Y, F_vals, levels=[0], colors="cyan", linewidths=2.5, linestyles="-")

        from matplotlib.lines import Line2D
        legend_proxy = Line2D([0], [0], color="cyan", linewidth=2.5, linestyle="-", label="SQ surface")
        ax.add_artist(legend_proxy)

        tidx = sq_meta["template_index"]
        tname = SQ_TEMPLATE_NAMES[tidx] if tidx < len(SQ_TEMPLATE_NAMES) else "?"
        ax.text(0.02, 0.98,
                f"SQ: {tname} (ε₁={sq_params['epsilon1']:.1f}, ε₂={sq_params['epsilon2']:.1f})\n"
                f"fit_error={sq_meta['fit_error']:.5f}",
                transform=ax.transAxes, fontsize=9, color="cyan",
                verticalalignment="top",
                bbox=dict(boxstyle="round,pad=0.3", facecolor="black", alpha=0.6))

    if show_roi_overlay:
        roi = dump["roi"]
        roi_rect = plt.Rectangle(
            (roi[0, 0], roi[0, 1]),
            roi[1, 0] - roi[0, 0], roi[1, 1] - roi[0, 1],
            fill=False, edgecolor="orange", linewidth=1.5, linestyle="--", label="ROI",
        )
        ax.add_patch(roi_rect)

    for ci, cam in enumerate(dump["cameras"]):
        if np.abs(cam[2] - z_val) < res * 3:
            ax.plot(cam[0], cam[1], marker="^", color="purple", markersize=10,
                    label=f"Camera {ci}" if ci == 0 else "")

    cbar = fig.colorbar(im, ax=ax, label="Signed distance (cells)", shrink=0.85)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Signed TSDF Cross-Section with Superquadric Overlay")
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8)

    plt.tight_layout()
    stem = "tsdf_signed_cross_section" if show_roi_overlay else "tsdf_signed_clean"
    path = os.path.join(OUTPUT_DIR, f"{stem}.png")
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {path}")
    return path


# ---------------------------------------------------------------------------
# Figure 3: Grasp pose overview (Item 12)
# ---------------------------------------------------------------------------
def fig_grasp_overview(dump: dict):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    grasps = dump["grasps"]
    pc = dump["point_cloud"]
    best_idx, best_combined = _find_best_grasp(grasps)

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection="3d")

    if len(pc) > 0:
        ax.scatter(pc[:, 0], pc[:, 1], pc[:, 2], c="lightgray", s=3, alpha=0.5,
                   label="Point cloud")

    roi = dump["roi"]
    xs = [roi[0, 0], roi[1, 0]]
    ys = [roi[0, 1], roi[1, 1]]
    zs = [roi[0, 2], roi[1, 2]]
    for x in xs:
        for y in ys:
            ax.plot([x, x], [y, y], zs, color="orange", alpha=0.4, linewidth=0.8)
    for x in xs:
        for z in zs:
            ax.plot([x, x], ys, [z, z], color="orange", alpha=0.4, linewidth=0.8)
    for y in ys:
        for z in zs:
            ax.plot(xs, [y, y], [z, z], color="orange", alpha=0.4, linewidth=0.8)

    for ci, cam in enumerate(dump["cameras"]):
        ax.scatter(*cam, c="purple", s=120, marker="^",
                   label=f"Camera {ci}" if ci == 0 else "")
        roi_center = (roi[0] + roi[1]) / 2
        ax.plot([cam[0], roi_center[0]], [cam[1], roi_center[1]], [cam[2], roi_center[2]],
                color="purple", alpha=0.2, linewidth=0.5)

    n_grasps = len(grasps["combined"])
    if n_grasps > 0:
        valid = grasps["contact_score"] > 0.0
        valid_indices = np.where(valid)[0]
        if len(valid_indices) > 0:
            sorted_idx = valid_indices[np.argsort(grasps["combined"][valid_indices])[::-1]]
            top_n = min(20, len(sorted_idx))

            for rank, idx in enumerate(sorted_idx[:top_n]):
                is_best = (idx == best_idx)
                T = grasps["pose_4x4"][idx]
                pos = T[:3, 3]
                gt = grasps["grasp_type"][idx]
                color = "gold" if is_best else GRASP_TYPE_COLORS.get(gt, "#888888")
                size = 80 if is_best else 20
                marker = "o" if is_best else "."
                ax.scatter(*pos, c=color, s=size, marker=marker, zorder=10 if is_best else 3)

                approach = T[:3, 2]
                line_len = 0.03 if is_best else 0.015
                end_pos = pos + approach * line_len
                ax.plot([pos[0], end_pos[0]], [pos[1], end_pos[1]], [pos[2], end_pos[2]],
                        color=color, linewidth=2.5 if is_best else 0.8, alpha=0.8)

            if best_idx >= 0:
                gt_best = grasps["grasp_type"][best_idx]
                closure = grasps["closure"][best_idx]
                best_pos = grasps["pose_4x4"][best_idx][:3, 3]
                ax.text(best_pos[0], best_pos[1], best_pos[2] + 0.015,
                        f"Best: {GRASP_TYPE_NAMES.get(gt_best, '?')}\n"
                        f"closure={closure:.2f}\nscore={best_combined:.3f}",
                        fontsize=8, color="white",
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="black", alpha=0.7))

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("Grasp Pose Overview — Best Candidates with Point Cloud")
    ax.legend(loc="upper left", fontsize=8, markerscale=0.7)
    ax.view_init(elev=25, azim=-45)

    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, "grasp_poses_overview.png")
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {path}")
    return path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Generate report figures from grasp preshaping debug dumps")
    parser.add_argument("--dump", default=None,
                        help="Path to debug .npz (default: latest in data/debug/)")
    parser.add_argument("--figures", nargs="+", default=["all"],
                        choices=["all", "unsigned", "signed", "signed_clean", "grasps"])
    args = parser.parse_args()

    if args.dump is None:
        debug_dir = os.path.normpath(
            os.path.join(os.path.dirname(__file__), "..",
                         "src/grasp_preshaping/data/debug"))
        if not os.path.isdir(debug_dir):
            print(f"Error: not found: {debug_dir}", file=sys.stderr)
            sys.exit(1)
        npz_files = sorted(
            (f for f in os.listdir(debug_dir) if f.endswith(".npz")),
            key=lambda f: os.path.getmtime(os.path.join(debug_dir, f)))
        if not npz_files:
            print(f"Error: no .npz in {debug_dir}", file=sys.stderr)
            sys.exit(1)
        args.dump = os.path.join(debug_dir, npz_files[-1])
        print(f"Using latest dump: {args.dump}")
    else:
        print(f"Using dump: {args.dump}")

    dump = load_debug_dump(args.dump)

    sq_info = dump["sq_params"]
    sq_meta = dump["sq_meta"]
    if sq_info and sq_meta:
        tidx = sq_meta["template_index"]
        tname = SQ_TEMPLATE_NAMES[tidx] if tidx < len(SQ_TEMPLATE_NAMES) else "?"
        print(f"  SQ: {tname}  fit_error={sq_meta['fit_error']:.5f}")
    print(f"  TSDF: {dump['tsdf_shape']}, {dump['tsdf_resolution']:.4f}m/vox")
    print(f"  Pts: {len(dump['point_cloud'])}, Grasps: {len(dump['grasps']['combined'])}, Cams: {len(dump['cameras'])}")

    want = args.figures
    if "all" in want:
        want = ["unsigned", "signed", "signed_clean", "grasps"]

    generated = []
    if "unsigned" in want:
        print("\n--- Unsigned TSDF cross-section ---")
        generated.append(fig_tsdf_unsigned_cross_section(dump))
    if "signed" in want:
        print("\n--- Signed TSDF with SQ overlay + ROI ---")
        generated.append(fig_tsdf_signed_with_sq(dump, show_roi_overlay=True))
    if "signed_clean" in want:
        print("\n--- Signed TSDF with SQ overlay (clean) ---")
        generated.append(fig_tsdf_signed_with_sq(dump, show_roi_overlay=False))
    if "grasps" in want:
        print("\n--- Grasp pose overview ---")
        generated.append(fig_grasp_overview(dump))

    print(f"\nDone. Generated {len(generated)} figures:")
    for g in generated:
        print(f"  {g}")


if __name__ == "__main__":
    main()
