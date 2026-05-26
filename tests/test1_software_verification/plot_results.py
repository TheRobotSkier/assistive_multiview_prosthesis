#!/usr/bin/env python3
"""Generate figures from Test 1 Software Verification results.

Reads CSV files from results/ and produces PDF figures in figures/.

Usage:
    python plot_results.py
    python plot_results.py --format png     # PNG instead of PDF
    python plot_results.py --dpi 300        # higher resolution
"""

import argparse
import csv
import json
import os
import sys
from collections import defaultdict

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(SCRIPT_DIR, "results")
FIGURES_DIR = os.path.join(SCRIPT_DIR, "figures")

def _load_csv(path: str) -> list[dict]:
    """Load a CSV file into a list of dicts."""
    if not os.path.isfile(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))

# Project color palette
COLORS = {
    "primary": "#5099e9",      # Blue accent
    "bg": "#fffdf6",           # Warm white background
    "panel_bg": "#f4fbf9",     # Light blue-green panel
    "single_view": "#50999e",  # Teal for single-view
    "multi_view": "#5099e9",   # Blue for multi-view
    "positive": "#27ae60",     # Green for positive/pass
    "negative": "#c0392b",     # Red for negative/fail
    "warning": "#e67e22",      # Orange for warnings
    "text": "#2c3e50",         # Dark text
    "grid": "#bdc3c7",         # Light grid
}

def _save_fig(fig, name: str, fmt: str = "pdf", dpi: int = 150):
    """Save a matplotlib figure with project styling."""
    import matplotlib
    matplotlib.use("Agg")
    fig.set_facecolor(COLORS["bg"])
    for ax in fig.get_axes():
        ax.set_facecolor(COLORS["panel_bg"])
        ax.tick_params(colors=COLORS["text"])
        ax.xaxis.label.set_color(COLORS["text"])
        ax.yaxis.label.set_color(COLORS["text"])
        ax.title.set_color(COLORS["text"])
        for spine in ax.spines.values():
            spine.set_color(COLORS["grid"])
    path = os.path.join(FIGURES_DIR, f"{name}.{fmt}")
    fig.savefig(path, dpi=dpi, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"  Saved: {path}")
    matplotlib.pyplot.close(fig)

# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# Figure 5: Cumulative wrist error CDF + grasp correctness
# Figure 5: Cumulative wrist error CDF
# ---------------------------------------------------------------------------

def plot_wrist_error_cdf(occlusion_rows: list[dict], fmt: str, dpi: int):
    """Cumulative distribution of wrist rotation errors (CDF)
    for single-view vs. multi-view.
    """
    import matplotlib.pyplot as plt

    if not occlusion_rows:
        print("  SKIP: No occlusion data for CDF plot")
        return

    # --- CDF of wrist errors ---
    fig, ax = plt.subplots(figsize=(8, 5))

    for cond, color, label in [("single_view", COLORS["single_view"], "Single-view"),
                                ("multi_view", COLORS["multi_view"], "Multi-view")]:
        errors = [float(r["orientation_error_deg"]) for r in occlusion_rows
                  if r.get("condition") == cond
                  and "orientation_error_deg" in r
                  and r.get("success", False)]
        if not errors:
            continue
        errors = np.sort(errors)
        cdf = np.arange(1, len(errors) + 1) / len(errors)
        ax.plot(errors, cdf, color=color, linewidth=2, label=label)

        # Annotate median and P90
        median = np.median(errors)
        p90 = np.percentile(errors, 90)
        ax.axvline(median, color=color, linestyle=":", alpha=0.5, linewidth=1)
        ax.annotate(f"median={median:.0f}°", xy=(median, 0.5),
                    fontsize=7, color=color, ha="left", va="bottom",
                    xytext=(5, 0), textcoords="offset points")

    ax.set_xlabel("Wrist Rotation Error (deg)")
    ax.set_ylabel("Cumulative Fraction")
    ax.set_title("Wrist Rotation Error CDF", fontweight="bold")
    ax.legend(loc="lower right")
    ax.set_xlim(0, None)
    ax.set_ylim(0, 1.05)
    ax.grid(True, alpha=0.3, color=COLORS["grid"])

    fig.tight_layout()
    _save_fig(fig, "fig5_wrist_error_cdf", fmt, dpi)

# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# Figure 7b: Convexity-based analysis
# ---------------------------------------------------------------------------

def plot_convexity_analysis(delta_rows: list[dict], summary_rows: list[dict],
                            fmt: str, dpi: int):
    """Analyse multi-view delta by object convexity.

    Left panel: grouped bar chart of grasp accuracy delta (multi - single)
    for convex vs non-convex objects.
    Right panel: scatter of head-camera coverage vs delta, coloured by convexity.
    """
    import matplotlib.pyplot as plt
    from object_registry import get_convexity

    if not delta_rows:
        print("  SKIP fig7b: No delta data")
        return

    # Classify objects
    convex_deltas = []
    nonconvex_deltas = []
    all_data = []

    for d in delta_rows:
        obj = d["object"]
        delta_val = float(d.get("delta_grasp_accuracy_pct",
                                 d.get("delta_fully_correct_pct", 0)))
        convex = get_convexity(obj)
        all_data.append({"object": obj, "delta": delta_val, "convex": convex == "convex"})
        if convex == "convex":
            convex_deltas.append(delta_val)
        else:
            nonconvex_deltas.append(delta_val)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5),
                                    gridspec_kw={"width_ratios": [1, 1.5]})

    # --- Left panel: Grouped bar chart ---
    categories = ["Convex", "Non-convex"]
    means = [np.mean(convex_deltas) if convex_deltas else 0,
             np.mean(nonconvex_deltas) if nonconvex_deltas else 0]
    stds = [np.std(convex_deltas) if convex_deltas else 0,
            np.std(nonconvex_deltas) if nonconvex_deltas else 0]
    counts = [len(convex_deltas), len(nonconvex_deltas)]

    bar_colors = [COLORS["single_view"], COLORS["multi_view"]]
    bars = ax1.bar(categories, means, yerr=stds, capsize=5,
                   color=bar_colors, edgecolor=COLORS["text"], linewidth=0.5,
                   alpha=0.85)

    ax1.axhline(y=0, color=COLORS["text"], linewidth=0.5)
    ax1.axhline(y=10, color=COLORS["positive"], linestyle="--", linewidth=1,
                label="IDE ($\\geq$10%)")
    ax1.set_ylabel("$\\Delta$ Grasp Accuracy (%)")
    ax1.set_title("Multi-view $\\Delta$ by Convexity", fontweight="bold")
    ax1.legend(fontsize=8)

    # Annotate counts
    for bar, count, mean in zip(bars, counts, means):
        ax1.annotate(f"n={count}", xy=(bar.get_x() + bar.get_width() / 2, 0),
                     xytext=(0, -15), textcoords="offset points",
                     ha="center", fontsize=9, color=COLORS["text"])
        ax1.annotate(f"{mean:+.1f}%", xy=(bar.get_x() + bar.get_width() / 2, mean),
                     xytext=(0, 5), textcoords="offset points",
                     ha="center", fontsize=10, fontweight="bold", color=COLORS["text"])

    ax1.grid(axis="y", alpha=0.3, color=COLORS["grid"])

    # --- Right panel: Per-object scatter with convexity colour ---
    # Sort by delta for display
    sorted_data = sorted(all_data, key=lambda x: x["delta"])
    x_labels = [d["object"].replace("_", "\n") for d in sorted_data]
    deltas = [d["delta"] for d in sorted_data]
    colors = [COLORS["single_view"] if d["convex"] else COLORS["multi_view"]
              for d in sorted_data]

    ax2.bar(range(len(sorted_data)), deltas, color=colors,
            edgecolor=COLORS["text"], linewidth=0.3)
    ax2.axhline(y=0, color=COLORS["text"], linewidth=0.5)
    ax2.axhline(y=10, color=COLORS["positive"], linestyle="--", linewidth=1)

    ax2.set_xticks(range(len(sorted_data)))
    ax2.set_xticklabels(x_labels, fontsize=6, rotation=45, ha="right")
    ax2.set_ylabel("$\\Delta$ Grasp Accuracy (%)")
    ax2.set_title("Per-object $\\Delta$ (blue=convex, green=non-convex)",
                  fontweight="bold", fontsize=10)
    ax2.grid(axis="y", alpha=0.3, color=COLORS["grid"])

    fig.tight_layout()
    _save_fig(fig, "fig7b_convexity_analysis", fmt, dpi)

# ---------------------------------------------------------------------------
# Figure 7c: Per-view point cloud coverage comparison
# ---------------------------------------------------------------------------

def plot_per_view_coverage(occlusion_rows: list[dict], fmt: str, dpi: int):
    """3D scatter of an example object showing which surfaces each camera sees.

    4-panel layout:
      1. Combined view (all colours) with camera positions
      2. Head camera visible surfaces only
      3. Wrist camera visible surfaces only
      4. Multi-view union (green = visible from either camera)

    Points are colour-coded:
      - #5099e9 (blue)     = head camera only
      - #ff9e4a (orange)   = wrist camera only
      - #55a868 (green)    = both cameras
      - #b0b0b0 (gray)     = neither
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    sys.path.insert(0, SCRIPT_DIR)
    from object_registry import load_object
    from hand_approaches import get_approach
    from view_geometry import get_camera_world_frames
    from occlusion import generate_view_cloud

    # Pick an interesting non-convex object
    obj_name = "power_drill"
    try:
        obj = load_object(obj_name)
        approach = get_approach(obj_name)
    except (KeyError, FileNotFoundError) as e:
        print(f"  SKIP per_view_coverage: {e}")
        return

    full = obj["points"]
    cam_frames = get_camera_world_frames(approach["pose"])

    # Generate camera-specific views
    head_vis = generate_view_cloud(full, cam_frames[0], use_depth_buffer=True)
    wrist_vis = generate_view_cloud(full, cam_frames[1], use_depth_buffer=True)

    # Dedup for membership check
    tol = 0.001  # 1 mm tolerance for membership test
    head_set = set(map(tuple, np.round(head_vis / tol).astype(int)))
    wrist_set = set(map(tuple, np.round(wrist_vis / tol).astype(int)))

    # Classify each point in the full cloud
    n = len(full)
    colors = np.full(n, 4, dtype=np.uint8)  # 4 = neither
    for i in range(n):
        key = tuple(np.round(full[i] / tol).astype(int))
        in_head = key in head_set
        in_wrist = key in wrist_set
        if in_head and in_wrist:
            colors[i] = 3  # both
        elif in_head:
            colors[i] = 1  # head only
        elif in_wrist:
            colors[i] = 2  # wrist only

    # Colour map: [unused, head-only, wrist-only, both, neither]
    cmap = np.array([
        [0, 0, 0, 0],                       # 0: unused
        [80/255, 153/255, 233/255, 0.9],    # 1: head only (blue #5099e9)
        [1, 159/255, 74/255, 0.9],          # 2: wrist only (orange #ff9e4a)
        [85/255, 168/255, 104/255, 0.9],    # 3: both (green #55a868)
        [176/255, 176/255, 176/255, 0.15],  # 4: neither (gray, faint)
    ])

    # Compute equal axis bounds from the full cloud (in metres)
    pad = 0.005
    x_min, x_max = full[:, 0].min() - pad, full[:, 0].max() + pad
    y_min, y_max = full[:, 1].min() - pad, full[:, 1].max() + pad
    z_min, z_max = full[:, 2].min() - pad, full[:, 2].max() + pad
    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min)
    x_mid = (x_min + x_max) / 2
    y_mid = (y_min + y_max) / 2
    z_mid = (z_min + z_max) / 2

    def _set_equal_axes(ax):
        ax.set_xlim(x_mid - max_range/2, x_mid + max_range/2)
        ax.set_ylim(y_mid - max_range/2, y_mid + max_range/2)
        ax.set_zlim(z_mid - max_range/2, z_mid + max_range/2)
        ax.set_box_aspect([1, 1, 1])
        ax.set_xlabel("X (m)", fontsize=7, labelpad=1)
        ax.set_ylabel("Y (m)", fontsize=7, labelpad=1)
        ax.set_zlabel("Z (m)", fontsize=7, labelpad=1)
        ax.tick_params(labelsize=6)

    # Count statistics for subplot titles
    n_head_only = (colors == 1).sum()
    n_wrist_only = (colors == 2).sum()
    n_both = (colors == 3).sum()
    n_neither = (colors == 4).sum()

    fig = plt.figure(figsize=(16, 10), facecolor=COLORS["bg"])

    # --- Subplot 1: Combined view (all colours) with cameras ---
    ax1 = fig.add_subplot(2, 2, 1, projection="3d", facecolor=COLORS["panel_bg"])
    ax1.scatter(full[:, 0], full[:, 1], full[:, 2],
                c=cmap[colors], s=0.3, depthshade=True)
    ax1.set_title(f"Combined — {obj_name}\n"
                  f"Head-only: {n_head_only}  Wrist-only: {n_wrist_only}  "
                  f"Both: {n_both}  Unseen: {n_neither}",
                  fontsize=9, fontweight="bold")
    ax1.view_init(elev=25, azim=-55)
    _set_equal_axes(ax1)

    # --- Subplot 2: Head camera view ---
    ax2 = fig.add_subplot(2, 2, 2, projection="3d", facecolor=COLORS["panel_bg"])
    head_labels = np.where(
        np.isin(np.arange(n), np.where(colors == 1)[0]), 1,
        np.where(np.isin(np.arange(n), np.where(colors == 3)[0]), 3, 4)
    )
    ax2.scatter(full[:, 0], full[:, 1], full[:, 2],
                c=cmap[head_labels], s=0.3, depthshade=True)
    ax2.set_title(f"Head Camera Visible\n"
                  f"{n_head_only + n_both} / {n} points "
                  f"({(n_head_only + n_both)/n*100:.0f}%)",
                  fontsize=9, fontweight="bold")
    ax2.view_init(elev=35, azim=-90)
    _set_equal_axes(ax2)

    # --- Subplot 3: Wrist camera view ---
    ax3 = fig.add_subplot(2, 2, 3, projection="3d", facecolor=COLORS["panel_bg"])
    wrist_labels = np.where(
        np.isin(np.arange(n), np.where(colors == 2)[0]), 2,
        np.where(np.isin(np.arange(n), np.where(colors == 3)[0]), 3, 4)
    )
    ax3.scatter(full[:, 0], full[:, 1], full[:, 2],
                c=cmap[wrist_labels], s=0.3, depthshade=True)
    ax3.set_title(f"Wrist Camera Visible\n"
                  f"{n_wrist_only + n_both} / {n} points "
                  f"({(n_wrist_only + n_both)/n*100:.0f}%)",
                  fontsize=9, fontweight="bold")
    ax3.view_init(elev=10, azim=-20)
    _set_equal_axes(ax3)

    # --- Subplot 4: Multi-view union ---
    ax4 = fig.add_subplot(2, 2, 4, projection="3d", facecolor=COLORS["panel_bg"])
    mv_labels = np.where(
        (colors == 1) | (colors == 2) | (colors == 3), 3, 4
    )
    ax4.scatter(full[:, 0], full[:, 1], full[:, 2],
                c=cmap[mv_labels], s=0.3, depthshade=True)
    n_visible = n - n_neither
    ax4.set_title(f"Multi-View Union (head + wrist)\n"
                  f"{n_visible} / {n} points ({n_visible/n*100:.0f}%)",
                  fontsize=9, fontweight="bold")
    ax4.view_init(elev=25, azim=-55)
    _set_equal_axes(ax4)

    # Add shared colour legend at the bottom of the figure
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor=cmap[1], label='Head camera only'),
        Patch(facecolor=cmap[2], label='Wrist camera only'),
        Patch(facecolor=cmap[3], label='Both cameras'),
        Patch(facecolor=cmap[4], label='Unseen'),
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=4, fontsize=9,
               framealpha=0.9, edgecolor='#cccccc')

    fig.tight_layout(rect=[0, 0.06, 1, 1])
    _save_fig(fig, "fig7c_per_view_coverage", fmt, dpi)

# ---------------------------------------------------------------------------
# Figure 7d-bench: Benchmark comparison (baseline vs SV vs MV per object)
# ---------------------------------------------------------------------------

def plot_benchmark_comparison(occlusion_rows: list[dict], summary_rows: list[dict], fmt: str, dpi: int):
    """Box plot: baseline vs single-view vs multi-view score distributions per object.

    Shows the full score distribution for each condition as box plots, with
    median markers and quartile ranges. Baseline uses all individual repetitions
    from baseline_all_scores.csv for a fair distribution comparison.
    """
    import numpy as np
    import matplotlib.pyplot as plt
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from object_registry import get_convexity

    # Collect per-object scores by condition
    objects = sorted(set(r['object'] for r in occlusion_rows))

    # SV and MV from occlusion_rows (per-trial data)
    sv_data = {}
    mv_data = {}
    for obj in objects:
        obj_rows = [r for r in occlusion_rows if r['object'] == obj]
        sv = [float(r['combined_score']) for r in obj_rows if r['condition'] == 'single_view']
        mv = [float(r['combined_score']) for r in obj_rows if r['condition'] == 'multi_view']
        if sv:
            sv_data[obj] = sv
        if mv:
            mv_data[obj] = mv

    # Baseline from baseline_all_scores.csv (individual reps for distribution)
    bl_data = {}
    baseline_csv = os.path.join(RESULTS_DIR, "baseline_all_scores.csv")
    if os.path.isfile(baseline_csv):
        import pandas as pd
        bl_df = pd.read_csv(baseline_csv)
        for obj in bl_df['object'].unique():
            scores = bl_df[bl_df['object'] == obj]['combined_score'].values.tolist()
            if scores:
                bl_data[obj] = scores
    else:
        # Fallback: single score per object from baseline_results.csv
        baseline_csv2 = os.path.join(RESULTS_DIR, "baseline_results.csv")
        if os.path.isfile(baseline_csv2):
            import pandas as pd
            bl_df2 = pd.read_csv(baseline_csv2)
            for _, row in bl_df2.iterrows():
                bl_data[row['object']] = [float(row['combined_score'])]

    # Only plot objects that have all three conditions
    plot_objects = [o for o in objects if o in bl_data and o in sv_data and o in mv_data]
    if not plot_objects:
        print("  [SKIP] fig7d_benchmark: no objects with all 3 conditions")
        return

    n = len(plot_objects)
    fig, ax = plt.subplots(figsize=(max(12, n * 1.2), 6))
    fig.patch.set_facecolor(COLORS["bg"])
    ax.set_facecolor(COLORS["panel_bg"])

    # Build boxplot data: for each object, 3 boxes (BL, SV, MV)
    box_data = []  # list of (position, [scores], color)
    box_positions = []
    box_scores = []
    box_colors = []

    for i, obj in enumerate(plot_objects):
        for j, (data, color, label) in enumerate([
            (bl_data[obj], '#8ecae6', 'Baseline (full cloud)'),
            (sv_data[obj], COLORS["primary"], 'Single-view (head)'),
            (mv_data[obj], '#55a868', 'Multi-view (head+wrist)'),
        ]):
            pos = i * 4 + j
            box_positions.append(pos)
            box_scores.append(data)
            box_colors.append(color)

    bp = ax.boxplot(box_scores, positions=box_positions, widths=2.4,
                    patch_artist=True, showfliers=False,
                    medianprops=dict(color='black', linewidth=1.5),
                    whiskerprops=dict(linewidth=0.8),
                    capprops=dict(linewidth=0.8),
                    boxprops=dict(linewidth=0.8))

    for i, patch in enumerate(bp['boxes']):
        patch.set_facecolor(box_colors[i])
        patch.set_alpha(0.7)
        patch.set_edgecolor('white')
        patch.set_linewidth(0.5)

    # Annotate convexity with colored dots below x-axis
    for i, obj in enumerate(plot_objects):
        c = get_convexity(obj)
        color = '#aaaaaa' if c == 'convex' else '#e6550d'
        ax.plot(i * 4 + 1, -0.08, 'o', color=color, markersize=5, clip_on=False,
                transform=ax.get_xaxis_transform())

    # X-axis labels (centered on each object group)
    ax.set_xticks([i * 4 + 1 for i in range(n)])
    ax.set_xticklabels([o.replace('_', '\n') for o in plot_objects],
                       fontsize=7, rotation=0, ha='center')
    ax.tick_params(axis='x', pad=15)
    ax.set_ylabel('Combined Score', fontsize=10)

    # Legend
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D
    legend_elements = [
        Patch(facecolor='#8ecae6', edgecolor='white', label='Baseline (full cloud)'),
        Patch(facecolor=COLORS["primary"], edgecolor='white', label='Single-view (head)'),
        Patch(facecolor='#55a868', edgecolor='white', label='Multi-view (head+wrist)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#aaaaaa',
               markersize=6, label='Convex'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#e6550d',
               markersize=6, label='Non-convex'),
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=8,
              framealpha=0.9, ncol=2)

    ax.set_ylim(-0.02, 1.05)
    ax.grid(axis='y', alpha=0.3, zorder=0)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    fig.tight_layout()
    path = os.path.join(FIGURES_DIR, f"fig7d_benchmark_comparison.{fmt}")
    fig.savefig(path, dpi=dpi, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  [OK] {os.path.basename(path)}")

# ---------------------------------------------------------------------------
# Figure 7e: Best score by grasp type (SV vs MV)
# ---------------------------------------------------------------------------

def plot_best_score_by_type(occlusion_rows: list[dict], fmt: str, dpi: int):
    """For each object, show the max combined_score attainable per grasp type,
    comparing single-view vs multi-view conditions.

    This answers: "Does multi-view let the SMC find higher-scoring grasps
    of each type?"
    """
    import matplotlib.pyplot as plt

    if not occlusion_rows:
        print("  SKIP: No occlusion data")
        return

    # Build: object -> condition -> grasp_type -> [scores]
    from collections import defaultdict
    data = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    for row in occlusion_rows:
        obj = row["object"]
        cond = row["condition"]
        gtype = row["grasp_type_name"]
        if gtype.lower() in ("unknown", ""):
            continue
        sc = float(row.get("combined_score", 0))
        data[obj][cond][gtype].append(sc)

    objects = sorted(data.keys())
    # Pick a representative subset (or all, arranged in a grid)
    # For clarity, show a focused 2x2 grid if > 4 objects
    n = len(objects)
    ncols = min(4, n)
    nrows = (n + ncols - 1) // ncols

    fig, axes = plt.subplots(nrows, ncols,
                             figsize=(ncols * 3.5, nrows * 3),
                             squeeze=False)

    # Collect all grasp types across objects for consistent ordering
    all_types = sorted(set(
        gt for obj_data in data.values()
        for cond_data in obj_data.values()
        for gt in cond_data
    ))

    for ax_idx, obj in enumerate(objects):
        ax = axes[ax_idx // ncols][ax_idx % ncols]

        types_present = sorted(set(
            gt for cond_data in data[obj].values()
            for gt in cond_data
        ))

        x = np.arange(len(types_present))
        width = 0.35

        sv_max = []
        mv_max = []
        for gt in types_present:
            sv_list = data[obj]["single_view"].get(gt, [])
            mv_list = data[obj]["multi_view"].get(gt, [])
            sv_max.append(max(sv_list) if sv_list else 0)
            mv_max.append(max(mv_list) if mv_list else 0)

        bars_sv = ax.bar(x - width / 2, sv_max, width,
                         color=COLORS["single_view"], alpha=0.8,
                         edgecolor=COLORS["text"], linewidth=0.3,
                         label="Single-view")
        bars_mv = ax.bar(x + width / 2, mv_max, width,
                         color=COLORS["multi_view"], alpha=0.8,
                         edgecolor=COLORS["text"], linewidth=0.3,
                         label="Multi-view")

        # Highlight where MV beats SV
        for i, gt in enumerate(types_present):
            sv_best = sv_max[i]
            mv_best = mv_max[i]
            if mv_best > sv_best:
                ax.annotate("+", xy=(i + width / 2, mv_best),
                            fontsize=10, ha="center", va="bottom",
                            color=COLORS["positive"], fontweight="bold")

        ax.set_xticks(x)
        ax.set_xticklabels(types_present, fontsize=7, rotation=30, ha="right")
        ax.set_title(obj, fontsize=9, fontweight="bold")
        ax.set_ylabel("Max Combined Score", fontsize=7)
        ax.tick_params(labelsize=7)
        ax.grid(axis="y", alpha=0.2, color=COLORS["grid"])

        if ax_idx == 0:
            ax.legend(fontsize=6, loc="upper left")

    # Hide unused subplots
    for ax_idx in range(len(objects), nrows * ncols):
        axes[ax_idx // ncols][ax_idx % ncols].set_visible(False)

    fig.tight_layout()
    _save_fig(fig, "fig7e_best_score_by_type", fmt, dpi)

# ---------------------------------------------------------------------------
# Figure 7f: Score vs wrist angle scatter (by condition, per object)
# ---------------------------------------------------------------------------

def plot_score_vs_wrist_angle(occlusion_rows: list[dict], fmt: str, dpi: int):
    """Scatter plot: combined_score vs wrist_rotation_deg for each object,
    colored by condition (single-view vs multi-view).

    This reveals whether multi-view helps the SMC converge to
    higher-scoring wrist angles more consistently.
    """
    import matplotlib.pyplot as plt

    if not occlusion_rows:
        print("  SKIP: No occlusion data")
        return

    # Group by object
    from collections import defaultdict
    rows_by_obj = defaultdict(list)
    for row in occlusion_rows:
        rows_by_obj[row["object"]].append(row)

    objects = sorted(rows_by_obj.keys())
    n = len(objects)
    ncols = min(4, n)
    nrows = (n + ncols - 1) // ncols

    fig, axes = plt.subplots(nrows, ncols,
                             figsize=(ncols * 3.5, nrows * 3),
                             squeeze=False)

    for ax_idx, obj in enumerate(objects):
        ax = axes[ax_idx // ncols][ax_idx % ncols]

        sv_rows = [r for r in rows_by_obj[obj] if r["condition"] == "single_view"]
        mv_rows = [r for r in rows_by_obj[obj] if r["condition"] == "multi_view"]

        # Single-view scatter
        if sv_rows:
            sv_angles = [float(r["wrist_rotation_deg"]) for r in sv_rows]
            sv_scores = [float(r["combined_score"]) for r in sv_rows]
            ax.scatter(sv_angles, sv_scores,
                       c=COLORS["single_view"], alpha=0.4, s=20,
                       marker="o", label=f"SV (n={len(sv_rows)})")

        # Multi-view scatter
        if mv_rows:
            mv_angles = [float(r["wrist_rotation_deg"]) for r in mv_rows]
            mv_scores = [float(r["combined_score"]) for r in mv_rows]
            ax.scatter(mv_angles, mv_scores,
                       c=COLORS["multi_view"], alpha=0.4, s=20,
                       marker="s", label=f"MV (n={len(mv_rows)})")

        # Show max score per condition with large marker
        if sv_rows:
            sv_best_idx = np.argmax(sv_scores)
            ax.scatter(sv_angles[sv_best_idx], sv_scores[sv_best_idx],
                       c=COLORS["single_view"], s=120, marker="*",
                       edgecolor="white", linewidth=0.8, zorder=5)
        if mv_rows:
            mv_best_idx = np.argmax(mv_scores)
            ax.scatter(mv_angles[mv_best_idx], mv_scores[mv_best_idx],
                       c=COLORS["multi_view"], s=120, marker="*",
                       edgecolor="white", linewidth=0.8, zorder=5)

        ax.set_title(obj, fontsize=9, fontweight="bold")
        ax.set_xlabel("Wrist Rotation (deg)", fontsize=7)
        ax.set_ylabel("Combined Score", fontsize=7)
        ax.tick_params(labelsize=7)
        ax.grid(alpha=0.2, color=COLORS["grid"])

        if ax_idx == 0:
            ax.legend(fontsize=6, loc="upper left", markerscale=0.5)

    # Hide unused subplots
    for ax_idx in range(len(objects), nrows * ncols):
        axes[ax_idx // ncols][ax_idx % ncols].set_visible(False)

    fig.tight_layout()
    _save_fig(fig, "fig7f_score_vs_wrist_angle", fmt, dpi)

# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# LaTeX table fragment
# ---------------------------------------------------------------------------

def generate_latex_table(latency_rows: list[dict], delta_rows: list[dict],
                         summary_rows: list[dict] = None):
    """Generate a LaTeX table fragment summarizing key results."""
    if not latency_rows:
        print("  SKIP: No latency data for LaTeX table")
        return

    by_object = defaultdict(list)
    for row in latency_rows:
        by_object[row["object"]].append(float(row["pipeline_time_ms"]))

    lines = [
        r"\begin{table}[H]",
        r"\centering",
        r"\small",
        r"\begin{tabular}{l r r r r}",
        r"\toprule",
        r"\textbf{Object} & \textbf{Mean (ms)} & \textbf{P95 (ms)} & \textbf{P99 (ms)} & \textbf{Result} \\",
        r"\midrule",
    ]

    for obj in sorted(by_object.keys()):
        times = np.array(by_object[obj])
        mean_t = np.mean(times)
        p95 = np.percentile(times, 95)
        p99 = np.percentile(times, 99)
        result = r"\textcolor{green!60!black}{\checkmark}" if p95 <= 400 else r"\textcolor{red}{\texttimes}"
        lines.append(
            f"{obj.replace('_', '\\_')} & {mean_t:.1f} & {p95:.1f} & {p99:.1f} & {result} \\\\"
        )

    lines.extend([
        r"\bottomrule",
        r"\end{tabular}",
        r"\caption{Test 1a: Pipeline latency results per object. MAR $\leq 400$\,ms.}",
        r"\label{tab:test1_latency}",
        r"\end{table}",
    ])

    # Add delta table if available
    if delta_rows:
        lines.extend([
            "",
            r"\begin{table}[H]",
            r"\centering",
            r"\small",
            r"\begin{tabular}{l r r r r}",
            r"\toprule",
            r"\textbf{Object} & \textbf{SV (\%)} & \textbf{MV (\%)} & \textbf{$\Delta$ (\%)} & \textbf{Wrist Err (deg)} \\",
            r"\midrule",
        ])

        # Build lookup for wrist errors from summary
        sv_wrist = {}
        mv_wrist = {}
        if summary_rows:
            for row in summary_rows:
                if row["condition"] == "single_view":
                    sv_wrist[row["object"]] = float(row.get("mean_orientation_error_deg", 0))
                elif row["condition"] == "multi_view":
                    mv_wrist[row["object"]] = float(row.get("mean_orientation_error_deg", 0))

        for d in delta_rows:
            obj = d["object"]
            sv_pct = float(d.get("single_view_grasp_accuracy_pct",
                                  d.get("single_view_grasp_correct_pct", 0)))
            mv_pct = float(d.get("multi_view_grasp_accuracy_pct",
                                  d.get("multi_view_grasp_correct_pct", 0)))
            delta_val = float(d.get("delta_grasp_accuracy_pct",
                                     d.get("delta_fully_correct_pct", 0)))
            wrist_sv = sv_wrist.get(obj, 0)
            wrist_mv = mv_wrist.get(obj, 0)
            lines.append(
                f"{obj.replace('_', '\\_')} & "
                f"{sv_pct:.1f} & "
                f"{mv_pct:.1f} & "
                f"{delta_val:+.1f} & "
                f"{wrist_sv:.0f} / {wrist_mv:.0f} \\\\"
            )
        mean_delta = np.mean([float(d.get("delta_grasp_accuracy_pct",
                                           d.get("delta_fully_correct_pct", 0)))
                              for d in delta_rows])
        lines.extend([
            r"\midrule",
            f"\\textbf{{Mean}} & & & {mean_delta:+.1f} \\\\",
            r"\bottomrule",
            r"\end{tabular}",
            r"\caption{Test 1b: Intent precision $\Delta$ (multi-view $-$ single-view). "
            r"Grasp type correctness. MAR $> 0$\,\%, IDE $\geq 10$\,\%. "
            r"Wrist Err shows SV / MV mean orientation error.}",
            r"\label{tab:test1_delta}",
            r"\end{table}",
        ])

    path = os.path.join(FIGURES_DIR, "test1_results_table.tex")
    os.makedirs(FIGURES_DIR, exist_ok=True)
    with open(path, "w") as f:
        f.write("\n".join(lines))
    print(f"  Saved: {path}")

# ---------------------------------------------------------------------------
# Figure 9: 3D Synthetic Setup Visualization
# ---------------------------------------------------------------------------

def plot_synthetic_setup(fmt: str, dpi: int):
    """3D rendered figure showing the synthetic test setup from multiple viewpoints.

    Displays a 2x2 panel with:
    - Top-left:  Overview (perspective)
    - Top-right: Top-down view
    - Bottom-left: Side view (along Y axis)
    - Bottom-right: Front view (along X axis, from behind hand)

    Each panel shows object cloud, camera frustums, approach pose, and
    camera-visible points colour-coded.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    sys.path.insert(0, SCRIPT_DIR)
    from view_geometry import (HEAD_CAMERA_LOCAL, WRIST_CAMERA_LOCAL,
                                get_camera_world_frames)
    from hand_approaches import get_approach
    from object_registry import load_object
    from occlusion import generate_view_cloud

    # Project colors
    BLUE = "#5099e9"
    ORANGE = "#e67e22"
    BG = "#fffdf6"
    LIGHT_BG = "#f4fbf9"

    # Use mug_with_handle as representative non-convex object
    obj_name = "mug_with_handle"
    try:
        obj = load_object(obj_name)
    except KeyError:
        obj_name = "cylinder_upright"
        obj = load_object(obj_name)
    approach = get_approach(obj_name)
    full_cloud = obj["points"]

    cam_frames = get_camera_world_frames(approach["pose"])
    head_frame = cam_frames[0]
    wrist_frame = cam_frames[1]

    head_cloud = generate_view_cloud(full_cloud, head_frame, use_depth_buffer=True)
    wrist_cloud = generate_view_cloud(full_cloud, wrist_frame, use_depth_buffer=True)

    # Find wrist-only points
    if len(wrist_cloud) > 0 and len(head_cloud) > 0:
        from scipy.spatial import cKDTree
        head_tree = cKDTree(head_cloud)
        dists, _ = head_tree.query(wrist_cloud, k=1)
        wrist_only = wrist_cloud[dists > 0.002]
    else:
        wrist_only = np.zeros((0, 3))

    # --- Helper: draw camera frustum ---
    def _draw_frustum(ax, cam_frame, color, label=None, alpha=0.12):
        pos = cam_frame["position"] * 100
        fwd = cam_frame["forward"]
        up = cam_frame["up"]
        right = np.cross(fwd, up)
        right /= np.linalg.norm(right) + 1e-8
        up = np.cross(right, fwd)
        up /= np.linalg.norm(up) + 1e-8

        fov_h = np.radians(cam_frame["fov_h_deg"]) / 2
        fov_v = np.radians(cam_frame["fov_v_deg"]) / 2
        near = cam_frame["near_m"] * 100
        far = min(cam_frame["far_m"] * 100, 40)

        nc = [
            pos + near * fwd + near * np.tan(fov_h) * s1 * right + near * np.tan(fov_v) * s2 * up
            for s1, s2 in [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        ]
        fc = [
            pos + far * fwd + far * np.tan(fov_h) * s1 * right + far * np.tan(fov_v) * s2 * up
            for s1, s2 in [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        ]

        for i in range(4):
            j = (i + 1) % 4
            ax.plot([nc[i][0], nc[j][0]], [nc[i][1], nc[j][1]], [nc[i][2], nc[j][2]],
                    color=color, linewidth=0.8, alpha=0.5)
            ax.plot([fc[i][0], fc[j][0]], [fc[i][1], fc[j][1]], [fc[i][2], fc[j][2]],
                    color=color, linewidth=0.8, alpha=0.5)
            ax.plot([nc[i][0], fc[i][0]], [nc[i][1], fc[i][1]], [nc[i][2], fc[i][2]],
                    color=color, linewidth=0.6, alpha=0.3)

        verts = [[(v[0], v[1], v[2]) for v in fc]]
        poly = Poly3DCollection(verts, alpha=alpha, facecolor=color, edgecolor=color,
                                linewidth=0.4)
        ax.add_collection3d(poly)

        ax.scatter([pos[0]], [pos[1]], [pos[2]], c=color, s=60, marker="^",
                   edgecolors="black", linewidth=0.4, zorder=10,
                   label=label if label else None)

    # --- Helper: draw scene content on an axis ---
    def _draw_scene(ax, show_labels=False):
        # Full cloud (faint)
        ax.scatter(full_cloud[:, 0] * 100, full_cloud[:, 1] * 100, full_cloud[:, 2] * 100,
                   c="#cccccc", s=1, alpha=0.2)

        # Head-visible (blue)
        if len(head_cloud) > 0:
            ax.scatter(head_cloud[:, 0] * 100, head_cloud[:, 1] * 100, head_cloud[:, 2] * 100,
                       c=BLUE, s=2, alpha=0.5,
                       label=f"Head visible ({len(head_cloud)})" if show_labels else None)

        # Wrist-only (orange)
        if len(wrist_only) > 0:
            ax.scatter(wrist_only[:, 0] * 100, wrist_only[:, 1] * 100, wrist_only[:, 2] * 100,
                       c=ORANGE, s=4, alpha=0.7,
                       label=f"Wrist only ({len(wrist_only)})" if show_labels else None)

        _draw_frustum(ax, head_frame, BLUE, "Head camera" if show_labels else None)
        _draw_frustum(ax, wrist_frame, ORANGE, "Wrist camera" if show_labels else None)

        # Hand approach
        hand_pos = np.array([approach["pose"]["px"],
                              approach["pose"]["py"],
                              approach["pose"]["pz"]]) * 100
        ax.scatter([hand_pos[0]], [hand_pos[1]], [hand_pos[2]],
                   c="#2c3e50", s=80, marker="s", edgecolors="black", linewidth=0.4,
                   zorder=10, label="Hand" if show_labels else None)
        ax.quiver(hand_pos[0], hand_pos[1], hand_pos[2],
                  8, 0, 0, color="#2c3e50", arrow_length_ratio=0.15, linewidth=1.5, alpha=0.7)

        # Origin axes
        al = 4
        ax.quiver(0, 0, 0, al, 0, 0, color="red", arrow_length_ratio=0.1, linewidth=1, alpha=0.4)
        ax.quiver(0, 0, 0, 0, al, 0, color="green", arrow_length_ratio=0.1, linewidth=1, alpha=0.4)
        ax.quiver(0, 0, 0, 0, 0, al, color="blue", arrow_length_ratio=0.1, linewidth=1, alpha=0.4)

        max_range = 35
        ax.set_xlim(-max_range, max_range)
        ax.set_ylim(-max_range, max_range)
        ax.set_zlim(-5, max_range * 1.5)
        try:
            ax.set_box_aspect([1, 1, 1])
        except AttributeError:
            pass
        ax.set_xlabel("X (cm)", fontsize=7, labelpad=2)
        ax.set_ylabel("Y (cm)", fontsize=7, labelpad=2)
        ax.set_zlabel("Z (cm)", fontsize=7, labelpad=2)
        ax.tick_params(labelsize=6)

    # --- 4-panel figure ---
    fig = plt.figure(figsize=(16, 14), facecolor=BG)

    viewpoints = [
        (25, -55, "Perspective Overview"),
        (90, -90, "Top-Down View"),
        (0, -90, "Side View (from \u2013Y)"),
        (10, 180, "Rear View (from \u2013X)"),
    ]

    for idx, (elev, azim, title) in enumerate(viewpoints):
        ax = fig.add_subplot(2, 2, idx + 1, projection="3d", facecolor=LIGHT_BG)
        _draw_scene(ax, show_labels=(idx == 0))
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title, fontsize=11, fontweight="bold", pad=8)

    # Add legend to first panel
    axes = fig.axes
    axes[0].legend(loc="upper left", fontsize=8, framealpha=0.9, edgecolor="#cccccc")

    # Camera info annotation
    info_text = (
        f"Object: {obj_name.replace('_', ' ').title()} | "
        f"Approach: {abs(approach['pose']['px'])*100:.0f} cm | "
        f"Points: {len(full_cloud)//1000}K\n"
        f"Head: ({head_frame['position'][0]*100:.1f}, {head_frame['position'][1]*100:.1f}, "
        f"{head_frame['position'][2]*100:.1f}) cm  |  "
        f"Wrist: ({wrist_frame['position'][0]*100:.1f}, {wrist_frame['position'][1]*100:.1f}, "
        f"{wrist_frame['position'][2]*100:.1f}) cm  (15° pitch)"
    )
    fig.text(0.5, 0.02, info_text, ha="center", fontsize=9, fontfamily="monospace",
             bbox=dict(boxstyle="round,pad=0.5", facecolor="white", alpha=0.9,
                       edgecolor="#cccccc"))

    fig.tight_layout(rect=[0, 0.06, 1, 1])
    _save_fig(fig, "fig9_synthetic_setup_3d", fmt, dpi)

# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# Figure 11: Per-stage latency waterfall chart
# ---------------------------------------------------------------------------

def plot_per_stage_waterfall(fmt: str = "pdf", dpi: int = 150):
    """Waterfall chart showing cumulative latency from EMG to command."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = _load_csv(os.path.join(RESULTS_DIR, "latency_per_stage_results.csv"))
    if not rows:
        print("  Skipping per-stage waterfall figure (no data)")
        return

    import numpy as np

    ok_rows = [r for r in rows if r.get("status") == "ok"]
    if not ok_rows:
        return

    # Combine Pipeline Manager + PM Cloud Handling into "ROS Overhead"
    for r in ok_rows:
        pm = float(r.get("pipeline_manager_ms", 0) or 0)
        pc = float(r.get("pm_cloud_handling_ms", 0) or 0)
        r["ros_overhead_ms"] = pm + pc

    stage_fields = [
        ("ros_overhead_ms", "ROS Overhead", "#3498db"),
        ("twist_propagation_ms", "Twist Propagation", "#2ecc71"),
        ("segmentation_ms", "Segmentation", "#e67e22"),
        ("preshaping_ms", "Grasp Preshaping", "#e74c3c"),
    ]

    # Compute overall mean per stage
    stage_means = []
    for field, label, color in stage_fields:
        values = [float(r[field]) for r in ok_rows
                  if not _is_nan(r.get(field))]
        stage_means.append((label, np.mean(values) if values else 0, color))

    # Waterfall plot
    fig, ax = plt.subplots(figsize=(8, 5))
    cumulative = 0
    labels = []
    for label, mean_val, color in stage_means:
        ax.bar(label, mean_val, bottom=cumulative, color=color)
        if mean_val > 0:
            ax.text(label, cumulative + mean_val / 2, f"{mean_val:.1f}",
                    ha="center", va="center", fontsize=8, color="white",
                    fontweight="bold")
        cumulative += mean_val
        labels.append(label)

    # Add total bar
    ax.bar("TOTAL", cumulative, color="#2c3e50")
    ax.text("TOTAL", cumulative / 2, f"{cumulative:.1f}",
            ha="center", va="center", fontsize=9, color="white",
            fontweight="bold")

    # Threshold lines
    ax.axhline(y=400, color=COLORS["negative"], linestyle="--", linewidth=1.5,
               label="MAR (400 ms)")
    ax.axhline(y=100, color=COLORS["warning"], linestyle=":", linewidth=1.0,
               label="IDE (100 ms)")

    ax.set_ylabel("Cumulative Latency (ms)")
    ax.set_title("Pipeline Latency Waterfall (Mean Across All Objects)")
    ax.legend(loc="upper left", fontsize=8)

    # Rotate x-axis labels to prevent overlap
    for label in ax.get_xticklabels():
        label.set_ha('center')
        label.set_rotation(25)

    _save_fig(fig, "fig11_latency_waterfall", fmt, dpi)

# ---------------------------------------------------------------------------
# Figure 12: Object Gallery (with superquadric estimation overlays)
# ---------------------------------------------------------------------------

# Cached debug dumps for SQ lookups (lazy-loaded on first gallery render).
_SQ_CACHE: dict | None = None

def _load_sq_cache() -> dict:
    """Scan known debug-dump directories and return {object_name_or_idx: sq_params}.

    The debug dumps don't embed the object name directly, so we match by
    point-cloud centroid + extent to gallery objects loaded later.
    """
    global _SQ_CACHE
    if _SQ_CACHE is not None:
        return _SQ_CACHE

    _SQ_CACHE = {}
    search_dirs = [
        os.path.join(SCRIPT_DIR, "results", "debug_dumps"),
        os.path.join(SCRIPT_DIR, "..", "..", "src", "grasp_preshaping", "data", "debug"),
    ]

    for sdir in search_dirs:
        if not os.path.isdir(sdir):
            continue
        for fname in sorted(os.listdir(sdir)):
            if not fname.endswith(".npz"):
                continue
            path = os.path.join(sdir, fname)
            try:
                data = np.load(path, allow_pickle=True)
                if "sq_params" not in data:
                    continue
                sq_raw = data["sq_params"]
                if len(sq_raw) != 14:
                    continue

                sq = {
                    "epsilon1": max(float(sq_raw[0]), 1e-3),
                    "epsilon2": max(float(sq_raw[1]), 1e-3),
                    "a": float(sq_raw[2]),
                    "b": float(sq_raw[3]),
                    "c": float(sq_raw[4]),
                    "translation": sq_raw[5:8].astype(np.float64),
                    "rotation": sq_raw[8:14].reshape(2, 3).astype(np.float64),
                }

                # Also store the point cloud centroid for matching
                if "point_cloud" in data:
                    pc_raw = data["point_cloud"]
                    pc_centroid = np.mean(pc_raw.reshape(-1, 3), axis=0)
                else:
                    pc_centroid = None

                _SQ_CACHE[fname] = {"sq": sq, "centroid": pc_centroid, "path": path}
            except Exception:
                continue

    return _SQ_CACHE

def _generate_sq_surface(sq_params: dict, n_eta: int = 24, n_omega: int = 24):
    """Generate a superquadric surface mesh from fitted parameters.

    Returns:
        (x, y, z) arrays in metres suitable for ax.plot_wireframe().
    """
    e1 = sq_params["epsilon1"]
    e2 = sq_params["epsilon2"]
    a, b, c = sq_params["a"], sq_params["b"], sq_params["c"]
    translation = sq_params["translation"]
    rot_flat = sq_params["rotation"]

    eta = np.linspace(-np.pi / 2, np.pi / 2, n_eta)
    omega = np.linspace(-np.pi, np.pi, n_omega)
    E, O = np.meshgrid(eta, omega)

    # Parametric superquadric (local frame)
    cosE = np.cos(E)
    sinE = np.sin(E)
    cosO = np.cos(O)
    sinO = np.sin(O)

    sgn_cosE = np.sign(cosE)
    sgn_sinE = np.sign(sinE)
    sgn_cosO = np.sign(cosO)
    sgn_sinO = np.sign(sinO)

    abs_cosE = np.abs(cosE)
    abs_sinE = np.abs(sinE)

    x_local = a * sgn_cosE * (abs_cosE ** e1) * sgn_cosO * (np.abs(cosO) ** e2)
    y_local = b * sgn_cosE * (abs_cosE ** e1) * sgn_sinO * (np.abs(sinO) ** e2)
    z_local = c * sgn_sinE * (abs_sinE ** e1)

    # Build rotation matrix from first two rows (third = cross product)
    rot = np.eye(3)
    rot[0, :] = rot_flat[0]
    rot[1, :] = rot_flat[1]
    rot[2, :] = np.cross(rot_flat[0], rot_flat[1])
    # Ensure orthonormal
    rot[2, :] = np.cross(rot[0, :], rot[1, :])
    norm = np.linalg.norm(rot[2, :])
    if norm > 0:
        rot[2, :] /= norm

    shape = x_local.shape
    pts_local = np.stack([x_local.ravel(), y_local.ravel(), z_local.ravel()], axis=0)  # (3, N)
    pts_world = rot @ pts_local + translation.reshape(3, 1)

    return (
        pts_world[0].reshape(shape),
        pts_world[1].reshape(shape),
        pts_world[2].reshape(shape),
    )

def _find_sq_for_object(obj_name: str, pts: np.ndarray) -> dict | None:
    """Try to find SQ parameters matching this gallery object.

    Matches by point-cloud centroid against cached debug dumps.
    Returns sq_params dict or None.
    """
    cache = _load_sq_cache()
    if not cache:
        return None

    obj_centroid = np.mean(pts, axis=0)

    best_match = None
    best_dist = float("inf")
    for key, entry in cache.items():
        if entry["centroid"] is None:
            continue
        dist = np.linalg.norm(entry["centroid"] - obj_centroid)
        if dist < best_dist and dist < 0.3:  # within 30cm
            best_dist = dist
            best_match = entry["sq"]

    return best_match

def _draw_obb_wireframe(ax, pts_cm: np.ndarray, color: str = "#c0392b", alpha: float = 0.4):
    """Draw an oriented bounding box (OBB) wireframe from PCA of the point cloud.

    Args:
        ax: 3D matplotlib axis.
        pts_cm: (N, 3) point cloud already in cm.
        color: Wireframe colour.
        alpha: Transparency.
    """
    if len(pts_cm) < 4:
        return

    # PCA to find principal axes
    mean = np.mean(pts_cm, axis=0)
    centered = pts_cm - mean
    cov = np.cov(centered.T)
    try:
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
    except np.linalg.LinAlgError:
        return

    # Sort by eigenvalue descending
    idx = np.argsort(eigenvalues)[::-1]
    eigenvectors = eigenvectors[:, idx]

    # Project points onto principal axes to find extents
    proj = centered @ eigenvectors
    mins = np.min(proj, axis=0)
    maxs = np.max(proj, axis=0)

    # 8 corners of the OBB
    corners_local = np.array([
        [mins[0], mins[1], mins[2]],
        [maxs[0], mins[1], mins[2]],
        [maxs[0], maxs[1], mins[2]],
        [mins[0], maxs[1], mins[2]],
        [mins[0], mins[1], maxs[2]],
        [maxs[0], mins[1], maxs[2]],
        [maxs[0], maxs[1], maxs[2]],
        [mins[0], maxs[1], maxs[2]],
    ])

    corners_world = corners_local @ eigenvectors.T + mean

    # 12 edges of the box
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # bottom face
        (4, 5), (5, 6), (6, 7), (7, 4),  # top face
        (0, 4), (1, 5), (2, 6), (3, 7),  # vertical edges
    ]

    for i, j in edges:
        ax.plot(
            [corners_world[i, 0], corners_world[j, 0]],
            [corners_world[i, 1], corners_world[j, 1]],
            [corners_world[i, 2], corners_world[j, 2]],
            color=color, linewidth=0.5, alpha=alpha,
        )

def plot_object_gallery(fmt: str, dpi: int):
    """Show all test objects as 3D point clouds in a grid, coloured by convexity.

    Overlays superquadric (SQ) wireframe meshes where debug-dump data is
    available, falling back to an oriented bounding box (OBB) wireframe
    computed via PCA.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    sys.path.insert(0, SCRIPT_DIR)
    from object_registry import load_object, list_objects, get_convexity

    objects = sorted(list_objects())
    n_obj = len(objects)
    ncols = 5
    nrows = int(np.ceil(n_obj / ncols))

    fig, axes = plt.subplots(nrows, ncols, figsize=(20, 4 * nrows),
                              subplot_kw={"projection": "3d"},
                              facecolor=COLORS["bg"])

    convex_color = COLORS["primary"]
    nonconvex_color = COLORS["warning"]
    sq_wire_color = COLORS["negative"]
    obb_wire_color = "#8e44ad"  # purple for fallback OBB
    sq_found_count = 0

    for idx, ax in enumerate(axes.flat):
        if idx < n_obj:
            obj_name = objects[idx]
            try:
                obj = load_object(obj_name)
            except Exception:
                ax.axis("off")
                continue

            pts = obj["points"]
            pts_cm = pts * 100  # convert to cm for display
            c = get_convexity(obj_name)
            color = convex_color if c == "convex" else nonconvex_color

            ax.scatter(pts_cm[:, 0], pts_cm[:, 1], pts_cm[:, 2],
                       c=color, s=0.3, alpha=0.5, depthshade=True)

            # --- SQ / OBB overlay ---
            sq = _find_sq_for_object(obj_name, pts)
            if sq is not None:
                # Superquadric wireframe from fitted params
                try:
                    xs, ys, zs = _generate_sq_surface(sq)
                    ax.plot_wireframe(
                        xs * 100, ys * 100, zs * 100,
                        color=sq_wire_color, alpha=0.35, linewidth=0.4,
                        rstride=2, cstride=2,
                    )
                    sq_found_count += 1
                except Exception:
                    _draw_obb_wireframe(ax, pts_cm, color=obb_wire_color, alpha=0.3)
            else:
                # Fallback: oriented bounding box from PCA
                _draw_obb_wireframe(ax, pts_cm, color=obb_wire_color, alpha=0.3)

            # Equal axis scaling
            ranges = np.array([
                pts[:, 0].max() - pts[:, 0].min(),
                pts[:, 1].max() - pts[:, 1].min(),
                pts[:, 2].max() - pts[:, 2].min(),
            ]) * 100
            max_range = ranges.max()
            mid = np.array([
                (pts[:, 0].max() + pts[:, 0].min()) / 2 * 100,
                (pts[:, 1].max() + pts[:, 1].min()) / 2 * 100,
                (pts[:, 2].max() + pts[:, 2].min()) / 2 * 100,
            ])
            ax.set_xlim(mid[0] - max_range / 2, mid[0] + max_range / 2)
            ax.set_ylim(mid[1] - max_range / 2, mid[1] + max_range / 2)
            ax.set_zlim(mid[2] - max_range / 2, mid[2] + max_range / 2)
            try:
                ax.set_box_aspect([1, 1, 1])
            except Exception:
                pass

            tag = "convex" if c == "convex" else "non-convex"
            ax.set_title(f"{obj_name}\n({tag})", fontsize=9, color=COLORS["text"])
            ax.set_xlabel("X (cm)", fontsize=7, labelpad=-2)
            ax.set_ylabel("Y (cm)", fontsize=7, labelpad=-2)
            ax.set_zlabel("Z (cm)", fontsize=7, labelpad=-2)
            ax.tick_params(labelsize=6)
            ax.view_init(elev=25, azim=-55)
        else:
            ax.axis("off")

    # Summary line
    if sq_found_count > 0:
        print(f"  [SQ]  {sq_found_count}/{n_obj} objects matched debug-dump SQ params")
    else:
        print(f"  [OBB] No SQ debug dumps matched — showing PCA bounding boxes "
              f"(run 'python run.py --debug' to generate SQ data)")

    # Legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor=convex_color,
               markersize=10, label='Convex'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor=nonconvex_color,
               markersize=10, label='Non-convex'),
    ]
    if sq_found_count > 0:
        legend_elements.append(
            Line2D([0], [0], color=sq_wire_color, linewidth=1.5, label='SQ fit'))
    legend_elements.append(
        Line2D([0], [0], color=obb_wire_color, linewidth=1.5, label='OBB (PCA)'))
    fig.legend(handles=legend_elements, loc='lower center', ncol=len(legend_elements),
               fontsize=11, frameon=False)

    fig.tight_layout(rect=[0, 0.03, 1, 1])
    _save_fig(fig, "fig12_object_gallery", fmt, dpi)

# ---------------------------------------------------------------------------
# Figure 13: Score vs Samples Sweep
# ---------------------------------------------------------------------------

def plot_score_vs_samples(fmt: str, dpi: int):
    """Plot how combined_score converges as prediction_samples increases.

    Requires results/score_sweep_results.csv generated by run.py --sweep-samples.
    If the file doesn't exist, prints a message and returns.
    """
    import matplotlib.pyplot as plt

    sweep_path = os.path.join(RESULTS_DIR, "score_sweep_results.csv")
    if not os.path.isfile(sweep_path):
        print("  Skipping fig13_score_vs_samples: no sweep data. "
              "Run: python run.py --sweep-samples")
        return

    import pandas as pd
    sweep = pd.read_csv(sweep_path)

    if len(sweep) == 0:
        print("  Skipping fig13_score_vs_samples: sweep data is empty.")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), facecolor=COLORS["bg"])
    fig.set_facecolor(COLORS["bg"])

    # Left panel: Score vs samples (accumulated across objects)
    sample_counts = sorted(sweep["prediction_samples"].unique())

    sv_means = []
    mv_means = []
    sv_stds = []
    mv_stds = []

    for n_samples in sample_counts:
        subset = sweep[sweep["prediction_samples"] == n_samples]
        sv = subset[subset["condition"] == "single_view"]["combined_score"]
        mv = subset[subset["condition"] == "multi_view"]["combined_score"]
        sv_means.append(sv.mean() if len(sv) > 0 else np.nan)
        mv_means.append(mv.mean() if len(mv) > 0 else np.nan)
        sv_stds.append(sv.std() if len(sv) > 0 else np.nan)
        mv_stds.append(mv.std() if len(mv) > 0 else np.nan)

    ax1.errorbar(sample_counts, sv_means, yerr=sv_stds,
                  fmt='o-', color=COLORS["single_view"], label="Single-view",
                  capsize=3, markersize=5, linewidth=2)
    ax1.errorbar(sample_counts, mv_means, yerr=mv_stds,
                  fmt='s-', color=COLORS["multi_view"], label="Multi-view",
                  capsize=3, markersize=5, linewidth=2)

    # Baseline reference
    baseline_path = os.path.join(RESULTS_DIR, "baseline_results.csv")
    if os.path.isfile(baseline_path):
        bl = pd.read_csv(baseline_path)
        bl_mean = bl["combined_score"].mean()
        ax1.axhline(bl_mean, color=COLORS["positive"], linestyle='--', linewidth=1.5,
                     label=f"Baseline (mean={bl_mean:.3f})")

    ax1.set_xlabel("Prediction Samples", color=COLORS["text"])
    ax1.set_ylabel("Mean Combined Score", color=COLORS["text"])
    ax1.set_title("Score vs Sample Count", color=COLORS["text"])
    ax1.legend(fontsize=9, frameon=True)
    ax1.set_xscale("log")
    ax1.grid(True, alpha=0.3, color=COLORS["grid"])
    ax1.set_facecolor(COLORS["panel_bg"])

    # Right panel: Latency vs samples
    lat_means = []
    lat_stds = []
    for n_samples in sample_counts:
        subset = sweep[sweep["prediction_samples"] == n_samples]
        lats = subset["latency_ms"]
        lat_means.append(lats.mean() if len(lats) > 0 else np.nan)
        lat_stds.append(lats.std() if len(lats) > 0 else np.nan)

    ax2.errorbar(sample_counts, lat_means, yerr=lat_stds,
                  fmt='o-', color=COLORS["warning"], capsize=3, markersize=5, linewidth=2)
    ax2.axhline(100, color=COLORS["negative"], linestyle='--', linewidth=1.5,
                 label="IDE limit (100ms)")
    ax2.axhline(400, color=COLORS["negative"], linestyle=':', linewidth=1.5,
                 label="MAR limit (400ms)")

    ax2.set_xlabel("Prediction Samples", color=COLORS["text"])
    ax2.set_ylabel("Pipeline Latency (ms)", color=COLORS["text"])
    ax2.set_title("Latency vs Sample Count", color=COLORS["text"])
    ax2.legend(fontsize=9, frameon=True)
    ax2.set_xscale("log")
    ax2.grid(True, alpha=0.3, color=COLORS["grid"])
    ax2.set_facecolor(COLORS["panel_bg"])

    fig.tight_layout()
    _save_fig(fig, "fig13_score_vs_samples", fmt, dpi)

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Generate Test 1 figures")
    parser.add_argument("--format", default="pdf", choices=["pdf", "png"],
                        help="Output format (default: pdf)")
    parser.add_argument("--dpi", type=int, default=150,
                        help="Resolution for raster formats (default: 150)")
    args = parser.parse_args()

    os.makedirs(FIGURES_DIR, exist_ok=True)

    print("Loading results...")
    latency_rows = _load_csv(os.path.join(RESULTS_DIR, "latency_results.csv"))
    occlusion_rows = _load_csv(os.path.join(RESULTS_DIR, "occlusion_results.csv"))
    summary_rows = _load_csv(os.path.join(RESULTS_DIR, "intent_precision_summary.csv"))
    delta_rows = _load_csv(os.path.join(RESULTS_DIR, "intent_precision_delta.csv"))

    print(f"  Latency rows: {len(latency_rows)}")
    print(f"  Occlusion rows: {len(occlusion_rows)}")
    print(f"  Summary rows: {len(summary_rows)}")
    print(f"  Delta rows: {len(delta_rows)}")

    print("\nGenerating figures...")
    plot_wrist_error_cdf(occlusion_rows, args.format, args.dpi)
    plot_convexity_analysis(delta_rows, summary_rows, args.format, args.dpi)
    plot_per_view_coverage(occlusion_rows, args.format, args.dpi)

    # Score-based analysis figures (Figures 7d-7f)
    plot_benchmark_comparison(occlusion_rows, summary_rows, args.format, args.dpi)
    plot_best_score_by_type(occlusion_rows, args.format, args.dpi)
    plot_score_vs_wrist_angle(occlusion_rows, args.format, args.dpi)

    plot_per_stage_waterfall(args.format, args.dpi)
    generate_latex_table(latency_rows, delta_rows, summary_rows)

    # Figure 9: 3D synthetic setup (always generated, no data dependencies)
    plot_synthetic_setup(args.format, args.dpi)

    # Figure 12: Object gallery (no data dependencies)
    plot_object_gallery(args.format, args.dpi)

    # Figure 13: Score vs samples sweep (requires sweep data)
    plot_score_vs_samples(args.format, args.dpi)

    print(f"\nDone. Figures in: {FIGURES_DIR}")

if __name__ == "__main__":
    main()
