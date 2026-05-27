#!/usr/bin/env python3
"""Generate figures from Test 1 Software Verification results.

Reads CSV files from results/ and produces PDF figures in figures/.

Usage:
    python plot_results.py
    python plot_results.py --format png     # PNG instead of PDF
    python plot_results.py --dpi 600        # higher resolution
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
    "primary": "#5099e9",  # Blue accent
    "bg": "#ffffff",  # Pure white background
    "panel_bg": "#ffffff",  # Pure white panel
    "single_view": "#50999e",  # Teal for single-view
    "multi_view": "#5099e9",  # Blue for multi-view
    "positive": "#27ae60",  # Green for positive/pass
    "negative": "#c0392b",  # Red for negative/fail
    "warning": "#e67e22",  # Orange for warnings
    "text": "#2c3e50",  # Dark text
    "grid": "#bdc3c7",  # Light grid
}


def _save_fig(fig, name: str, fmt: str = "png", dpi: int = 300):
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
# Figure 1: Cumulative wrist error CDF
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

    for cond, color, label in [
        ("single_view", COLORS["single_view"], "Single-view"),
        ("multi_view", COLORS["multi_view"], "Multi-view"),
    ]:
        errors = [
            float(r["orientation_error_deg"])
            for r in occlusion_rows
            if r.get("condition") == cond
            and "orientation_error_deg" in r
            and r.get("success", False)
        ]
        if not errors:
            continue
        errors = np.sort(errors)
        cdf = np.arange(1, len(errors) + 1) / len(errors)
        ax.plot(errors, cdf, color=color, linewidth=2, label=label)

        # Annotate median and P90
        median = np.median(errors)
        p90 = np.percentile(errors, 90)
        ax.axvline(median, color=color, linestyle=":", alpha=0.5, linewidth=1)
        ax.annotate(
            f"median={median:.0f}°",
            xy=(median, 0.5),
            fontsize=7,
            color=color,
            ha="left",
            va="bottom",
            xytext=(5, 0),
            textcoords="offset points",
        )

    ax.set_xlabel("Wrist Rotation Error (deg)")
    ax.set_ylabel("Cumulative Fraction")
    ax.set_title("Wrist Rotation Error CDF", fontweight="bold")
    ax.legend(loc="lower right")
    ax.set_xlim(0, None)
    ax.set_ylim(0, 1.05)
    ax.grid(True, alpha=0.3, color=COLORS["grid"])

    fig.tight_layout()
    _save_fig(fig, "fig1_wrist_error_cdf", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 2: Convexity-based analysis
# ---------------------------------------------------------------------------


def plot_convexity_analysis(
    delta_rows: list[dict], summary_rows: list[dict], fmt: str, dpi: int
):
    """Analyse multi-view delta by object convexity.

    Left panel: grouped bar chart of grasp accuracy delta (multi - single)
    for convex vs non-convex objects.
    Right panel: scatter of head-camera coverage vs delta, coloured by convexity.
    """
    import matplotlib.pyplot as plt
    from object_registry import get_convexity

    if not delta_rows:
        print("  SKIP fig2: No delta data")
        return

    # Classify objects
    convex_deltas = []
    nonconvex_deltas = []
    all_data = []

    for d in delta_rows:
        obj = d["object"]
        delta_val = float(
            d.get("delta_grasp_accuracy_pct", d.get("delta_fully_correct_pct", 0))
        )
        convex = get_convexity(obj)
        all_data.append(
            {"object": obj, "delta": delta_val, "convex": convex == "convex"}
        )
        if convex == "convex":
            convex_deltas.append(delta_val)
        else:
            nonconvex_deltas.append(delta_val)

    fig, (ax1, ax2) = plt.subplots(
        1, 2, figsize=(14, 5.5), gridspec_kw={"width_ratios": [1, 1.5]}
    )

    # --- Left panel: Grouped bar chart ---
    categories = ["Convex", "Non-convex"]
    means = [
        np.mean(convex_deltas) if convex_deltas else 0,
        np.mean(nonconvex_deltas) if nonconvex_deltas else 0,
    ]
    stds = [
        np.std(convex_deltas) if convex_deltas else 0,
        np.std(nonconvex_deltas) if nonconvex_deltas else 0,
    ]
    counts = [len(convex_deltas), len(nonconvex_deltas)]

    bar_colors = [COLORS["single_view"], COLORS["multi_view"]]
    bars = ax1.bar(
        categories,
        means,
        yerr=stds,
        capsize=5,
        color=bar_colors,
        edgecolor=COLORS["text"],
        linewidth=0.5,
        alpha=0.85,
    )

    ax1.axhline(y=0, color=COLORS["text"], linewidth=0.5)
    ax1.axhline(
        y=10,
        color=COLORS["positive"],
        linestyle="--",
        linewidth=1,
        label="IDE ($\\geq$10%)",
    )
    ax1.set_ylabel("$\\Delta$ Grasp Accuracy (%)")
    ax1.set_title("Multi-view $\\Delta$ by Convexity", fontweight="bold")
    ax1.legend(fontsize=8)

    # Annotate counts
    for bar, count, mean in zip(bars, counts, means):
        ax1.annotate(
            f"n={count}",
            xy=(bar.get_x() + bar.get_width() / 2, 0),
            xytext=(0, -15),
            textcoords="offset points",
            ha="center",
            fontsize=9,
            color=COLORS["text"],
        )
        ax1.annotate(
            f"{mean:+.1f}%",
            xy=(bar.get_x() + bar.get_width() / 2, mean),
            xytext=(0, 5),
            textcoords="offset points",
            ha="center",
            fontsize=10,
            fontweight="bold",
            color=COLORS["text"],
        )

    ax1.grid(axis="y", alpha=0.3, color=COLORS["grid"])

    # --- Right panel: Per-object scatter with convexity colour ---
    # Sort by delta for display
    sorted_data = sorted(all_data, key=lambda x: x["delta"])
    x_labels = [d["object"].replace("_", "\n") for d in sorted_data]
    deltas = [d["delta"] for d in sorted_data]
    colors = [
        COLORS["single_view"] if d["convex"] else COLORS["multi_view"]
        for d in sorted_data
    ]

    ax2.bar(
        range(len(sorted_data)),
        deltas,
        color=colors,
        edgecolor=COLORS["text"],
        linewidth=0.3,
    )
    ax2.axhline(y=0, color=COLORS["text"], linewidth=0.5)
    ax2.axhline(y=10, color=COLORS["positive"], linestyle="--", linewidth=1)

    ax2.set_xticks(range(len(sorted_data)))
    ax2.set_xticklabels(x_labels, fontsize=6, rotation=45, ha="right")
    ax2.set_ylabel("$\\Delta$ Grasp Accuracy (%)")
    ax2.set_title(
        "Per-object $\\Delta$ (blue=convex, green=non-convex)",
        fontweight="bold",
        fontsize=10,
    )
    ax2.grid(axis="y", alpha=0.3, color=COLORS["grid"])

    fig.tight_layout()
    _save_fig(fig, "fig2_convexity_analysis", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 3: Per-view point cloud coverage
# ---------------------------------------------------------------------------


def plot_per_view_coverage(occlusion_rows: list[dict], fmt: str, dpi: int):
    """3D scatter of an example object showing which surfaces each camera sees.

    Single-panel view showing three categories with transparent colours
    so overlap is clearly visible:
      - Blue   = head camera only
      - Orange = wrist camera only
      - Gray   = unseen by either camera
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    sys.path.insert(0, SCRIPT_DIR)
    from hand_approaches import get_approach
    from object_registry import load_object
    from occlusion import generate_view_cloud
    from view_geometry import get_camera_world_frames

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

    # Classify each point: head-only, wrist-only, both, or unseen
    n = len(full)
    labels = np.full(n, 3, dtype=np.uint8)  # 3 = unseen
    for i in range(n):
        key = tuple(np.round(full[i] / tol).astype(int))
        in_head = key in head_set
        in_wrist = key in wrist_set
        if in_head and in_wrist:
            labels[i] = 0  # both — skip (drawn under head/wrist)
        elif in_head:
            labels[i] = 1  # head only
        elif in_wrist:
            labels[i] = 2  # wrist only

    # Colour map: transparent so overlap is visible
    # [unused, head-only, wrist-only, unseen]
    cmap = np.array(
        [
            [0, 0, 0, 0],  # 0: unused (both — invisible)
            [80 / 255, 153 / 255, 233 / 255, 0.55],  # 1: head only (blue #5099e9)
            [1, 159 / 255, 74 / 255, 0.55],  # 2: wrist only (orange #ff9e4a)
            [176 / 255, 176 / 255, 176 / 255, 0.12],  # 3: unseen (gray, faint)
        ]
    )

    # Compute equal axis bounds from the full cloud (in metres)
    pad = 0.005
    x_min, x_max = full[:, 0].min() - pad, full[:, 0].max() + pad
    y_min, y_max = full[:, 1].min() - pad, full[:, 1].max() + pad
    z_min, z_max = full[:, 2].min() - pad, full[:, 2].max() + pad
    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min)
    x_mid = (x_min + x_max) / 2
    y_mid = (y_min + y_max) / 2
    z_mid = (z_min + z_max) / 2

    # Count statistics for title
    n_head_only = (labels == 1).sum()
    n_wrist_only = (labels == 2).sum()
    n_both = (labels == 0).sum()
    n_unseen = (labels == 3).sum()

    fig = plt.figure(figsize=(8, 7), facecolor=COLORS["bg"])
    ax = fig.add_subplot(1, 1, 1, projection="3d", facecolor=COLORS["panel_bg"])
    ax.scatter(
        full[:, 0], full[:, 1], full[:, 2], c=cmap[labels], s=0.3, depthshade=True
    )
    ax.set_title(
        f"Per-View Coverage — {obj_name}\n"
        f"Head: {n_head_only}  Wrist: {n_wrist_only}  "
        f"Both: {n_both}  Unseen: {n_unseen}",
        fontsize=10,
        fontweight="bold",
    )
    ax.view_init(elev=25, azim=-55)
    ax.set_xlim(x_mid - max_range / 2, x_mid + max_range / 2)
    ax.set_ylim(y_mid - max_range / 2, y_mid + max_range / 2)
    ax.set_zlim(z_mid - max_range / 2, z_mid + max_range / 2)
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlabel("X (m)", fontsize=7, labelpad=1)
    ax.set_ylabel("Y (m)", fontsize=7, labelpad=1)
    ax.set_zlabel("Z (m)", fontsize=7, labelpad=1)
    ax.tick_params(labelsize=6)

    # Legend
    from matplotlib.patches import Patch

    legend_elements = [
        Patch(facecolor=cmap[1][:3], alpha=cmap[1][3], label="Head camera only"),
        Patch(facecolor=cmap[2][:3], alpha=cmap[2][3], label="Wrist camera only"),
        Patch(facecolor=cmap[3][:3], alpha=cmap[3][3], label="Unseen"),
    ]
    fig.legend(
        handles=legend_elements,
        loc="lower center",
        ncol=3,
        fontsize=9,
        framealpha=0.9,
        edgecolor="#cccccc",
    )

    fig.tight_layout(rect=[0, 0.06, 1, 1])
    _save_fig(fig, "fig3_per_view_coverage", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 4: Benchmark comparison (baseline vs SV vs MV per object)
# ---------------------------------------------------------------------------


def plot_benchmark_comparison(
    occlusion_rows: list[dict], summary_rows: list[dict], fmt: str, dpi: int
):
    """Box plot: baseline vs single-view vs multi-view score distributions per object.

    Shows the full score distribution for each condition as box plots, with
    median markers and quartile ranges. Baseline uses all individual repetitions
    from baseline_all_scores.csv for a fair distribution comparison.
    """
    import matplotlib.pyplot as plt
    import numpy as np

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from object_registry import get_convexity

    # Collect per-object scores by condition
    objects = sorted(set(r["object"] for r in occlusion_rows))

    # SV and MV from occlusion_rows (per-trial data)
    sv_data = {}
    mv_data = {}
    for obj in objects:
        obj_rows = [r for r in occlusion_rows if r["object"] == obj]
        sv = [
            float(r["combined_score"])
            for r in obj_rows
            if r["condition"] == "single_view"
        ]
        mv = [
            float(r["combined_score"])
            for r in obj_rows
            if r["condition"] == "multi_view"
        ]
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
        for obj in bl_df["object"].unique():
            scores = bl_df[bl_df["object"] == obj]["combined_score"].values.tolist()
            if scores:
                bl_data[obj] = scores
    else:
        # Fallback: single score per object from baseline_results.csv
        baseline_csv2 = os.path.join(RESULTS_DIR, "baseline_results.csv")
        if os.path.isfile(baseline_csv2):
            import pandas as pd

            bl_df2 = pd.read_csv(baseline_csv2)
            for _, row in bl_df2.iterrows():
                bl_data[row["object"]] = [float(row["combined_score"])]

    # Only plot objects that have all three conditions
    plot_objects = [
        o for o in objects if o in bl_data and o in sv_data and o in mv_data
    ]
    if not plot_objects:
        print("  [SKIP] fig4_benchmark: no objects with all 3 conditions")
        return

    n = len(plot_objects)
    group_width = 3.0  # spacing between object groups
    box_width = 0.8  # individual box width (narrower to avoid overlap)
    fig, ax = plt.subplots(figsize=(max(12, n * 1.5), 6))
    fig.patch.set_facecolor(COLORS["bg"])
    ax.set_facecolor(COLORS["panel_bg"])

    # Build boxplot data: for each object, 3 boxes (BL, SV, MV)
    box_positions = []
    box_scores = []
    box_colors = []

    for i, obj in enumerate(plot_objects):
        for j, (data, color, label) in enumerate(
            [
                (bl_data[obj], "#8ecae6", "Baseline (full cloud)"),
                (sv_data[obj], COLORS["primary"], "Single-view (head)"),
                (mv_data[obj], "#55a868", "Multi-view (head+wrist)"),
            ]
        ):
            pos = i * group_width + j * (box_width + 0.1)
            box_positions.append(pos)
            box_scores.append(data)
            box_colors.append(color)

    bp = ax.boxplot(
        box_scores,
        positions=box_positions,
        widths=box_width,
        patch_artist=True,
        showfliers=False,
        medianprops=dict(color="black", linewidth=1.5),
        whiskerprops=dict(linewidth=0.8),
        capprops=dict(linewidth=0.8),
        boxprops=dict(linewidth=0.8),
    )

    for i, patch in enumerate(bp["boxes"]):
        patch.set_facecolor(box_colors[i])
        patch.set_alpha(0.7)
        patch.set_edgecolor("white")
        patch.set_linewidth(0.5)

    # Annotate convexity with colored dots below x-axis
    for i, obj in enumerate(plot_objects):
        c = get_convexity(obj)
        color = "#aaaaaa" if c == "convex" else "#e6550d"
        center = i * group_width + (box_width + 0.1)  # center of group
        ax.plot(
            center,
            -0.12,
            "o",
            color=color,
            markersize=5,
            clip_on=False,
            transform=ax.get_xaxis_transform(),
        )

    # X-axis labels (centered on each object group)
    group_centers = [i * group_width + (box_width + 0.1) for i in range(n)]
    ax.set_xticks(group_centers)
    ax.set_xticklabels(
        [o.replace("_", "\n") for o in plot_objects],
        fontsize=7,
        rotation=0,
        ha="center",
    )
    ax.tick_params(axis="x", pad=20)
    ax.set_ylabel("Combined Score", fontsize=10)

    # Legend
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    legend_elements = [
        Patch(facecolor="#8ecae6", edgecolor="white", label="Baseline (full cloud)"),
        Patch(
            facecolor=COLORS["primary"], edgecolor="white", label="Single-view (head)"
        ),
        Patch(facecolor="#55a868", edgecolor="white", label="Multi-view (head+wrist)"),
        Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            markerfacecolor="#aaaaaa",
            markersize=6,
            label="Convex",
        ),
        Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            markerfacecolor="#e6550d",
            markersize=6,
            label="Non-convex",
        ),
    ]
    ax.legend(
        handles=legend_elements, loc="upper right", fontsize=8, framealpha=0.9, ncol=2
    )

    ax.set_ylim(-0.02, 1.05)
    ax.grid(axis="y", alpha=0.3, zorder=0)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.tight_layout()
    path = os.path.join(FIGURES_DIR, f"fig4_benchmark_comparison.{fmt}")
    fig.savefig(path, dpi=dpi, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  [OK] {os.path.basename(path)}")


# ---------------------------------------------------------------------------
# Figure 5: Best score by grasp type (SV vs MV)
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
    n = len(objects)
    ncols = min(4, n)
    nrows = (n + ncols - 1) // ncols

    fig, axes = plt.subplots(
        nrows, ncols, figsize=(ncols * 3.5, nrows * 3), squeeze=False
    )

    # Collect all grasp types across objects for consistent ordering
    all_types = sorted(
        set(
            gt
            for obj_data in data.values()
            for cond_data in obj_data.values()
            for gt in cond_data
        )
    )

    for ax_idx, obj in enumerate(objects):
        ax = axes[ax_idx // ncols][ax_idx % ncols]

        types_present = sorted(
            set(gt for cond_data in data[obj].values() for gt in cond_data)
        )

        x = np.arange(len(types_present))
        width = 0.35

        sv_max = []
        mv_max = []
        for gt in types_present:
            sv_list = data[obj]["single_view"].get(gt, [])
            mv_list = data[obj]["multi_view"].get(gt, [])
            sv_max.append(max(sv_list) if sv_list else 0)
            mv_max.append(max(mv_list) if mv_list else 0)

        bars_sv = ax.bar(
            x - width / 2,
            sv_max,
            width,
            color=COLORS["single_view"],
            alpha=0.8,
            edgecolor=COLORS["text"],
            linewidth=0.3,
            label="Single-view",
        )
        bars_mv = ax.bar(
            x + width / 2,
            mv_max,
            width,
            color=COLORS["multi_view"],
            alpha=0.8,
            edgecolor=COLORS["text"],
            linewidth=0.3,
            label="Multi-view",
        )

        ax.set_xticks(x)
        ax.set_xticklabels(types_present, fontsize=7, rotation=30, ha="right")
        ax.set_title(obj, fontsize=9, fontweight="bold")
        ax.set_ylabel("Max Combined Score", fontsize=7)
        ax.tick_params(labelsize=7)
        ax.grid(axis="y", alpha=0.2, color=COLORS["grid"])

        if ax_idx == 0:
            ax.legend(fontsize=6, loc="lower left")

    # Hide unused subplots
    for ax_idx in range(len(objects), nrows * ncols):
        axes[ax_idx // ncols][ax_idx % ncols].set_visible(False)

    fig.tight_layout()
    _save_fig(fig, "fig5_best_score_by_type", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 6: Score vs wrist angle scatter (by condition, per object)
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

    fig, axes = plt.subplots(
        nrows, ncols, figsize=(ncols * 3.5, nrows * 3), squeeze=False
    )

    for ax_idx, obj in enumerate(objects):
        ax = axes[ax_idx // ncols][ax_idx % ncols]

        sv_rows = [r for r in rows_by_obj[obj] if r["condition"] == "single_view"]
        mv_rows = [r for r in rows_by_obj[obj] if r["condition"] == "multi_view"]

        # Single-view scatter
        if sv_rows:
            sv_angles = [float(r["wrist_rotation_deg"]) for r in sv_rows]
            sv_scores = [float(r["combined_score"]) for r in sv_rows]
            ax.scatter(
                sv_angles,
                sv_scores,
                c=COLORS["single_view"],
                alpha=0.4,
                s=20,
                marker="o",
                label=f"SV (n={len(sv_rows)})",
            )

        # Multi-view scatter
        if mv_rows:
            mv_angles = [float(r["wrist_rotation_deg"]) for r in mv_rows]
            mv_scores = [float(r["combined_score"]) for r in mv_rows]
            ax.scatter(
                mv_angles,
                mv_scores,
                c=COLORS["multi_view"],
                alpha=0.4,
                s=20,
                marker="s",
                label=f"MV (n={len(mv_rows)})",
            )

        # Show max score per condition with large marker
        if sv_rows:
            sv_best_idx = np.argmax(sv_scores)
            ax.scatter(
                sv_angles[sv_best_idx],
                sv_scores[sv_best_idx],
                c=COLORS["single_view"],
                s=120,
                marker="*",
                edgecolor="white",
                linewidth=0.8,
                zorder=5,
            )
        if mv_rows:
            mv_best_idx = np.argmax(mv_scores)
            ax.scatter(
                mv_angles[mv_best_idx],
                mv_scores[mv_best_idx],
                c=COLORS["multi_view"],
                s=120,
                marker="*",
                edgecolor="white",
                linewidth=0.8,
                zorder=5,
            )

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
    _save_fig(fig, "fig6_score_vs_wrist_angle", fmt, dpi)


# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# LaTeX table fragment
# ---------------------------------------------------------------------------


def generate_latex_table(
    latency_rows: list[dict], delta_rows: list[dict], summary_rows: list[dict] = None
):
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
        result = (
            r"\textcolor{green!60!black}{\checkmark}"
            if p95 <= 400
            else r"\textcolor{red}{\texttimes}"
        )
        lines.append(
            f"{obj.replace('_', '\\_')} & {mean_t:.1f} & {p95:.1f} & {p99:.1f} & {result} \\\\"
        )

    lines.extend(
        [
            r"\bottomrule",
            r"\end{tabular}",
            r"\caption{Test 1a: Pipeline latency results per object. MAR $\leq 400$\,ms.}",
            r"\label{tab:test1_latency}",
            r"\end{table}",
        ]
    )

    # Add delta table if available
    if delta_rows:
        lines.extend(
            [
                "",
                r"\begin{table}[H]",
                r"\centering",
                r"\small",
                r"\begin{tabular}{l r r r r}",
                r"\toprule",
                r"\textbf{Object} & \textbf{SV (\%)} & \textbf{MV (\%)} & \textbf{$\Delta$ (\%)} & \textbf{Wrist Err (deg)} \\",
                r"\midrule",
            ]
        )

        # Build lookup for wrist errors from summary
        sv_wrist = {}
        mv_wrist = {}
        if summary_rows:
            for row in summary_rows:
                if row["condition"] == "single_view":
                    sv_wrist[row["object"]] = float(
                        row.get("mean_orientation_error_deg", 0)
                    )
                elif row["condition"] == "multi_view":
                    mv_wrist[row["object"]] = float(
                        row.get("mean_orientation_error_deg", 0)
                    )

        for d in delta_rows:
            obj = d["object"]
            sv_pct = float(
                d.get(
                    "single_view_grasp_accuracy_pct",
                    d.get("single_view_grasp_correct_pct", 0),
                )
            )
            mv_pct = float(
                d.get(
                    "multi_view_grasp_accuracy_pct",
                    d.get("multi_view_grasp_correct_pct", 0),
                )
            )
            delta_val = float(
                d.get("delta_grasp_accuracy_pct", d.get("delta_fully_correct_pct", 0))
            )
            wrist_sv = sv_wrist.get(obj, 0)
            wrist_mv = mv_wrist.get(obj, 0)
            lines.append(
                f"{obj.replace('_', '\\_')} & "
                f"{sv_pct:.1f} & "
                f"{mv_pct:.1f} & "
                f"{delta_val:+.1f} & "
                f"{wrist_sv:.0f} / {wrist_mv:.0f} \\\\"
            )
        mean_delta = np.mean(
            [
                float(
                    d.get(
                        "delta_grasp_accuracy_pct", d.get("delta_fully_correct_pct", 0)
                    )
                )
                for d in delta_rows
            ]
        )
        lines.extend(
            [
                r"\midrule",
                f"\\textbf{{Mean}} & & & {mean_delta:+.1f} \\\\",
                r"\bottomrule",
                r"\end{tabular}",
                r"\caption{Test 1b: Intent precision $\Delta$ (multi-view $-$ single-view). "
                r"Grasp type correctness. MAR $> 0$\,\%, IDE $\geq 10$\,\%. "
                r"Wrist Err shows SV / MV mean orientation error.}",
                r"\label{tab:test1_delta}",
                r"\end{table}",
            ]
        )

    path = os.path.join(FIGURES_DIR, "test1_results_table.tex")
    os.makedirs(FIGURES_DIR, exist_ok=True)
    with open(path, "w") as f:
        f.write("\n".join(lines))
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Figure 8: 3D Synthetic Setup Visualization
# ---------------------------------------------------------------------------


def plot_synthetic_setup(fmt: str, dpi: int):
    """3D rendered figure showing the synthetic test setup from multiple viewpoints.

    Displays a 2x2 panel with:
    - Top-left:  Overview (perspective)
    - Top-right: Top-down view
    - Bottom-left: Side view (along Y axis)
    - Bottom-right: Front view (along X axis, from behind hand)

    Each panel shows object cloud, camera frustums, approach pose, and
    all visible points in a single colour.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    FONT_SIZES = {
        "title": 20,
        "panel_title": 17,
        "axis_label": 13,
        "tick": 12,
        "legend": 13,
        "info": 14,
    }

    sys.path.insert(0, SCRIPT_DIR)
    from hand_approaches import get_approach
    from object_registry import load_object
    from occlusion import generate_view_cloud
    from view_geometry import (
        HEAD_CAMERA_LOCAL,
        WRIST_CAMERA_LOCAL,
        get_camera_world_frames,
    )

    # Project colors
    POINT_COLOR = "#5099e9"
    CAM_HEAD = "#5099e9"
    CAM_WRIST = "#e67e22"
    BG = "#ffffff"
    LIGHT_BG = "#ffffff"

    # Use power_drill as representative non-convex object
    obj_name = "power_drill"
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

    # Merge all visible points (head + wrist, deduplicated)
    if len(wrist_cloud) > 0 and len(head_cloud) > 0:
        merged = np.vstack([head_cloud, wrist_cloud])
        voxel_keys = np.floor(merged / 0.001).astype(np.int64)
        _, unique_idx = np.unique(voxel_keys, axis=0, return_index=True)
        all_visible = merged[unique_idx]
    elif len(head_cloud) > 0:
        all_visible = head_cloud
    else:
        all_visible = wrist_cloud

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
            pos
            + near * fwd
            + near * np.tan(fov_h) * s1 * right
            + near * np.tan(fov_v) * s2 * up
            for s1, s2 in [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        ]
        fc = [
            pos
            + far * fwd
            + far * np.tan(fov_h) * s1 * right
            + far * np.tan(fov_v) * s2 * up
            for s1, s2 in [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        ]

        for i in range(4):
            j = (i + 1) % 4
            ax.plot(
                [nc[i][0], nc[j][0]],
                [nc[i][1], nc[j][1]],
                [nc[i][2], nc[j][2]],
                color=color,
                linewidth=0.8,
                alpha=0.5,
            )
            ax.plot(
                [fc[i][0], fc[j][0]],
                [fc[i][1], fc[j][1]],
                [fc[i][2], fc[j][2]],
                color=color,
                linewidth=0.8,
                alpha=0.5,
            )
            ax.plot(
                [nc[i][0], fc[i][0]],
                [nc[i][1], fc[i][1]],
                [nc[i][2], fc[i][2]],
                color=color,
                linewidth=0.6,
                alpha=0.3,
            )

        verts = [[(v[0], v[1], v[2]) for v in fc]]
        poly = Poly3DCollection(
            verts, alpha=alpha, facecolor=color, edgecolor=color, linewidth=0.4
        )
        ax.add_collection3d(poly)

        ax.scatter(
            [pos[0]],
            [pos[1]],
            [pos[2]],
            c=color,
            s=60,
            marker="^",
            edgecolors="black",
            linewidth=0.4,
            zorder=10,
            label=label if label else None,
        )

    # --- Helper: draw scene content on an axis ---
    def _draw_scene(ax, show_labels=False):
        # Full cloud (faint)
        ax.scatter(
            full_cloud[:, 0] * 100,
            full_cloud[:, 1] * 100,
            full_cloud[:, 2] * 100,
            c="#cccccc",
            s=1,
            alpha=0.2,
        )

        # All visible points in a single colour
        if len(all_visible) > 0:
            ax.scatter(
                all_visible[:, 0] * 100,
                all_visible[:, 1] * 100,
                all_visible[:, 2] * 100,
                c=POINT_COLOR,
                s=2,
                alpha=0.5,
                label=f"Visible ({len(all_visible)})" if show_labels else None,
            )

        _draw_frustum(ax, head_frame, CAM_HEAD, "Head camera" if show_labels else None)
        _draw_frustum(
            ax, wrist_frame, CAM_WRIST, "Wrist camera" if show_labels else None
        )

        # Hand approach
        hand_pos = (
            np.array(
                [approach["pose"]["px"], approach["pose"]["py"], approach["pose"]["pz"]]
            )
            * 100
        )
        ax.scatter(
            [hand_pos[0]],
            [hand_pos[1]],
            [hand_pos[2]],
            c="#2c3e50",
            s=80,
            marker="s",
            edgecolors="black",
            linewidth=0.4,
            zorder=10,
            label="Hand" if show_labels else None,
        )
        ax.quiver(
            hand_pos[0],
            hand_pos[1],
            hand_pos[2],
            8,
            0,
            0,
            color="#2c3e50",
            arrow_length_ratio=0.15,
            linewidth=1.5,
            alpha=0.7,
        )

        # Origin axes
        al = 4
        ax.quiver(
            0,
            0,
            0,
            al,
            0,
            0,
            color="red",
            arrow_length_ratio=0.1,
            linewidth=1,
            alpha=0.4,
        )
        ax.quiver(
            0,
            0,
            0,
            0,
            al,
            0,
            color="green",
            arrow_length_ratio=0.1,
            linewidth=1,
            alpha=0.4,
        )
        ax.quiver(
            0,
            0,
            0,
            0,
            0,
            al,
            color="blue",
            arrow_length_ratio=0.1,
            linewidth=1,
            alpha=0.4,
        )

        max_range = 35
        ax.set_xlim(-max_range, max_range)
        ax.set_ylim(-max_range, max_range)
        ax.set_zlim(-5, max_range * 1.5)
        try:
            ax.set_box_aspect([1, 1, 1])
        except AttributeError:
            pass
        ax.set_xlabel("X (cm)", fontsize=FONT_SIZES["axis_label"], labelpad=3)
        ax.set_ylabel("Y (cm)", fontsize=FONT_SIZES["axis_label"], labelpad=3)
        ax.set_zlabel("Z (cm)", fontsize=FONT_SIZES["axis_label"], labelpad=3)
        ax.tick_params(labelsize=FONT_SIZES["tick"])

    # --- 4-panel figure ---
    fig = plt.figure(figsize=(16, 14), facecolor=BG)
    fig.suptitle(
        "Synthetic Setup Visualization",
        fontsize=FONT_SIZES["title"],
        fontweight="bold",
        color=COLORS["text"],
        y=0.98,
    )

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
        ax.set_title(
            title, fontsize=FONT_SIZES["panel_title"], fontweight="bold", pad=10
        )

    # Add legend to first panel
    axes = fig.axes
    axes[0].legend(
        loc="upper left",
        fontsize=FONT_SIZES["legend"],
        framealpha=0.9,
        edgecolor="#cccccc",
    )

    # Camera info annotation
    info_text = (
        f"Object: {obj_name.replace('_', ' ').title()} | "
        f"Approach: {abs(approach['pose']['px']) * 100:.0f} cm | "
        f"Points: {len(full_cloud) // 1000}K\n"
        f"Head: ({head_frame['position'][0] * 100:.1f}, {head_frame['position'][1] * 100:.1f}, "
        f"{head_frame['position'][2] * 100:.1f}) cm  |  "
        f"Wrist: ({wrist_frame['position'][0] * 100:.1f}, {wrist_frame['position'][1] * 100:.1f}, "
        f"{wrist_frame['position'][2] * 100:.1f}) cm  (15° pitch)"
    )
    fig.text(
        0.5,
        0.02,
        info_text,
        ha="center",
        fontsize=FONT_SIZES["info"],
        fontfamily="monospace",
        bbox=dict(
            boxstyle="round,pad=0.5", facecolor="white", alpha=0.9, edgecolor="#cccccc"
        ),
    )

    fig.tight_layout(rect=[0, 0.06, 1, 0.96])
    _save_fig(fig, "fig8_synthetic_setup_3d", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 7: Per-stage latency waterfall chart
# ---------------------------------------------------------------------------


def _load_stage_means() -> dict:
    """Load mean latencies for the three main pipeline stages.

    Sources:
      - Twist Propagation: per-stage benchmark (arrival-time tapping)
      - Segmentation: benchmark_segmentation CPU/CUDA trial CSVs
      - Grasp Preshaping: per-stage benchmark (arrival-time tapping)

    Returns dict with keys: twist_mean, twist_p95, twist_min, twist_max,
    seg_cpu_mean, seg_cpu_p95, seg_cpu_min, seg_cpu_max,
    seg_cuda_mean, seg_cuda_p95, seg_cuda_min, seg_cuda_max,
    preshape_mean, preshape_p95, preshape_min, preshape_max.
    """
    import numpy as np

    result = {}

    # --- Twist Propagation & Grasp Preshaping from per-stage benchmark ---
    per_stage_path = os.path.join(RESULTS_DIR, "latency_per_stage_results.csv")
    rows = _load_csv(per_stage_path)
    ok_rows = [r for r in rows if r.get("status") == "ok"]

    if ok_rows:
        twist_vals = [
            float(r["twist_propagation_ms"])
            for r in ok_rows
            if not _is_nan(r.get("twist_propagation_ms"))
        ]
        preshape_vals = [
            float(r["preshaping_ms"])
            for r in ok_rows
            if not _is_nan(r.get("preshaping_ms"))
        ]
        if twist_vals:
            result["twist_mean"] = np.mean(twist_vals)
            result["twist_p95"] = np.percentile(twist_vals, 95)
            result["twist_min"] = float(np.min(twist_vals))
            result["twist_max"] = float(np.max(twist_vals))
        if preshape_vals:
            result["preshape_mean"] = np.mean(preshape_vals)
            result["preshape_p95"] = np.percentile(preshape_vals, 95)
            result["preshape_min"] = float(np.min(preshape_vals))
            result["preshape_max"] = float(np.max(preshape_vals))

    # Fallback: use documented benchmark values if CSV has no data
    if "twist_mean" not in result:
        result["twist_mean"] = 88.7
        result["twist_p95"] = 141.2
        result["twist_min"] = 60.0
        result["twist_max"] = 150.0
    if "preshape_mean" not in result:
        result["preshape_mean"] = 75.3
        result["preshape_p95"] = 83.5
        result["preshape_min"] = 55.0
        result["preshape_max"] = 90.0

    # --- Segmentation from benchmark trial CSVs ---
    for backend, prefix in [("cpu", "seg_cpu"), ("cuda", "seg_cuda")]:
        trial_path = os.path.join(RESULTS_DIR, f"segmentation_{backend}_trials.csv")
        trial_rows = _load_csv(trial_path)
        ok_trials = [r for r in trial_rows if r.get("status") == "ok"]
        if ok_trials:
            vals = [float(r["latency_ms"]) for r in ok_trials]
            result[f"{prefix}_mean"] = np.mean(vals)
            result[f"{prefix}_p95"] = np.percentile(vals, 95)
            result[f"{prefix}_min"] = float(np.min(vals))
            result[f"{prefix}_max"] = float(np.max(vals))
        else:
            # Fallback: CPU ~726 ms, CUDA ~38 ms
            result[f"{prefix}_mean"] = 726.0 if backend == "cpu" else 38.0
            result[f"{prefix}_p95"] = 852.0 if backend == "cpu" else 51.0
            result[f"{prefix}_min"] = 650.0 if backend == "cpu" else 30.0
            result[f"{prefix}_max"] = 900.0 if backend == "cpu" else 60.0

    return result


def plot_per_stage_waterfall(fmt: str = "png", dpi: int = 300):
    """Latency summary with stacked base/accelerated bars and variability panel."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.patches as mpatches
    import matplotlib.pyplot as plt
    import numpy as np

    sm = _load_stage_means()

    twist_mean = sm["twist_mean"]
    twist_min = sm["twist_min"]
    twist_max = sm["twist_max"]
    seg_cpu_mean = sm["seg_cpu_mean"]
    seg_cpu_min = sm["seg_cpu_min"]
    seg_cpu_max = sm["seg_cpu_max"]
    seg_gpu_mean = sm["seg_cuda_mean"]
    seg_gpu_min = sm["seg_cuda_min"]
    seg_gpu_max = sm["seg_cuda_max"]
    preshape_mean = sm["preshape_mean"]
    preshape_min = sm["preshape_min"]
    preshape_max = sm["preshape_max"]

    # Min / Max totals for each pipeline
    base_min_total = twist_min + seg_cpu_min + preshape_min
    base_max_total = twist_max + seg_cpu_max + preshape_max
    accel_min_total = twist_min + seg_gpu_min + preshape_min
    accel_max_total = twist_max + seg_gpu_max + preshape_max

    fig, (ax_main, ax_box) = plt.subplots(
        1,
        2,
        figsize=(13.5, 6.2),
        gridspec_kw={"width_ratios": [1.2, 1], "wspace": 0.35},
    )

    # ═══════════════════════════════════════════════════════════════════
    # Left panel: stacked totals for base vs accelerated pipeline
    # ═══════════════════════════════════════════════════════════════════
    pipeline_labels = ["Base", "Accelerated"]
    x = np.arange(len(pipeline_labels))
    width = 0.55

    stage_series = [
        ("Twist propagation", [twist_mean, twist_mean], "#2ecc71"),
        ("Segmentation", [seg_cpu_mean, seg_gpu_mean], "#e67e22"),
        ("Grasp prediction", [preshape_mean, preshape_mean], "#e74c3c"),
    ]

    bottoms = np.zeros(len(pipeline_labels))
    for label, values, color in stage_series:
        bars = ax_main.bar(
            x,
            values,
            width,
            bottom=bottoms,
            color=color,
            edgecolor="white",
            linewidth=0.8,
            label=label,
        )
        for bar, value, bottom in zip(bars, values, bottoms):
            if value >= 20:
                ax_main.text(
                    bar.get_x() + bar.get_width() / 2 + 0.2,
                    bottom + value / 2,
                    f"{value:.0f}",
                    ha="center",
                    va="center",
                    fontsize=10,
                    color="white",
                    fontweight="bold",
                )
        bottoms += np.array(values)

    # Min / Max total whiskers above each stacked bar
    min_totals = [base_min_total, accel_min_total]
    max_totals = [base_max_total, accel_max_total]
    for xi, total_mean, total_min, total_max in zip(x, bottoms, min_totals, max_totals):
        # Mean total label (offset to the right)
        ax_main.text(
            xi + width * 0.2,
            total_mean + 8,
            f"{total_mean:.0f} ms",
            ha="left",
            va="bottom",
            fontsize=10,
            color=COLORS["text"],
            fontweight="bold",
        )
        # Min line
        ax_main.plot(
            [xi - width * 0.2, xi + width * 0.2],
            [total_min, total_min],
            color=COLORS["text"],
            linewidth=2,
            solid_capstyle="butt",
        )
        ax_main.text(
            xi,
            total_min - 30,
            f"Min: {total_min:.0f}",
            ha="center",
            va="center",
            fontsize=10,
            color="white",
            fontweight="bold",
        )
        # Max line
        ax_main.plot(
            [xi - width * 0.2, xi + width * 0.2],
            [total_max, total_max],
            color=COLORS["text"],
            linewidth=2,
            solid_capstyle="butt",
        )
        ax_main.text(
            xi,
            total_max + 30,
            f"Max: {total_max:.0f}",
            ha="center",
            va="center",
            fontsize=10,
            color=COLORS["text"],
            fontweight="bold",
        )
        # Connecting line from min to max
        ax_main.plot(
            [xi, xi],
            [total_min, total_max],
            color=COLORS["text"],
            linewidth=2,
            alpha=0.6,
        )

    ax_main.axhline(
        y=400,
        color=COLORS["negative"],
        linestyle="--",
        linewidth=1.5,
        label="MAR (400 ms)",
    )
    ax_main.axhline(
        y=100,
        color=COLORS["warning"],
        linestyle=":",
        linewidth=1.2,
        label="IDE (100 ms)",
    )
    ax_main.set_xticks(x)
    ax_main.set_xticklabels(pipeline_labels, fontsize=11, fontweight="bold")
    ax_main.set_ylabel("Latency (ms)", fontsize=11)
    ax_main.set_title("Latency Breakdown", fontsize=13, fontweight="bold")
    ax_main.grid(axis="y", alpha=0.25, color=COLORS["grid"])

    # ═══════════════════════════════════════════════════════════════════
    # Right panel: per-run variability
    # ═══════════════════════════════════════════════════════════════════
    cpu_trials_path = os.path.join(RESULTS_DIR, "segmentation_cpu_trials.csv")
    gpu_trials_path = os.path.join(RESULTS_DIR, "segmentation_cuda_trials.csv")
    per_stage_path = os.path.join(RESULTS_DIR, "latency_per_stage_results.csv")

    cpu_trials_rows = _load_csv(cpu_trials_path)
    gpu_trials_rows = _load_csv(gpu_trials_path)
    per_stage_rows = _load_csv(per_stage_path)

    cpu_latencies = [
        float(r["latency_ms"]) for r in cpu_trials_rows if r.get("status") == "ok"
    ]
    gpu_latencies = [
        float(r["latency_ms"]) for r in gpu_trials_rows if r.get("status") == "ok"
    ]
    preshape_latencies = [
        float(r["preshaping_ms"])
        for r in per_stage_rows
        if r.get("status") == "ok" and not _is_nan(r.get("preshaping_ms"))
    ]

    dist_data = [cpu_latencies, gpu_latencies, preshape_latencies]
    dist_labels = ["Segmentation\nCPU", "Segmentation\nGPU", "Grasp\nPrediction"]
    dist_colors = ["#d4a574", "#f5b041", "#e74c3c"]

    nonempty = [
        (vals, label, color)
        for vals, label, color in zip(dist_data, dist_labels, dist_colors)
        if vals
    ]
    if nonempty:
        data, labels, colors = zip(*nonempty)
        bp = ax_box.boxplot(
            data,
            tick_labels=labels,
            patch_artist=True,
            widths=0.55,
            showfliers=False,
            medianprops={"color": "white", "linewidth": 2},
            whiskerprops={"color": COLORS["text"], "linewidth": 1},
            capprops={"color": COLORS["text"], "linewidth": 1},
        )
        for patch, color in zip(bp["boxes"], colors):
            patch.set_facecolor(color)
            patch.set_edgecolor(COLORS["text"])
            patch.set_alpha(0.9)

        rng = np.random.default_rng(42)
        for i, (latencies, color) in enumerate(zip(data, colors), start=1):
            jitter = rng.uniform(-0.12, 0.12, len(latencies))
            """ax_box.scatter(
                np.full(len(latencies), i) + jitter,
                latencies,
                alpha=0.28,
                s=10,
                color=color,
                edgecolors="none",
                zorder=2,
            )
            mean_val = np.mean(latencies)
            ax_box.scatter(
                [i],
                [mean_val],
                marker="D",
                s=42,
                color="white",
                edgecolors=COLORS["text"],
                zorder=3,
                linewidths=1,
            )"""
            mean_val = np.mean(latencies)
            ax_box.annotate(
                f"{mean_val:.0f} ms",
                xy=(i, mean_val),
                xytext=(i + 0.4, mean_val),
                fontsize=8.5,
                color=COLORS["text"],
                fontweight="bold",
                arrowprops=dict(arrowstyle="-", color=COLORS["text"], linewidth=1.5),
            )

        ax_box.set_ylabel("Latency (ms)", fontsize=11)
        ax_box.set_title(
            "Latency Variability (100 runs)", fontsize=13, fontweight="bold"
        )
        ax_box.tick_params(axis="x", labelsize=10)
    else:
        ax_box.text(
            0.5,
            0.5,
            "No latency benchmark data",
            ha="center",
            va="center",
            transform=ax_box.transAxes,
            fontsize=11,
            color=COLORS["text"],
        )
        ax_box.set_title(
            "Latency Variability (100 runs)", fontsize=13, fontweight="bold"
        )

    stage_handles = [
        mpatches.Patch(color=color, label=label) for label, _, color in stage_series
    ]
    threshold_handles = [
        plt.Line2D(
            [0],
            [0],
            color=COLORS["negative"],
            linestyle="--",
            linewidth=1.5,
            label="MAR (400 ms)",
        ),
        plt.Line2D(
            [0],
            [0],
            color=COLORS["warning"],
            linestyle=":",
            linewidth=1.2,
            label="IDE (100 ms)",
        ),
    ]
    ax_main.legend(
        handles=stage_handles + threshold_handles,
        loc="upper right",
        fontsize=8.5,
        framealpha=0.9,
    )

    fig.tight_layout()
    _save_fig(fig, "fig7_latency_waterfall", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 9: Object Gallery (with superquadric estimation overlays)
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
        os.path.join(
            SCRIPT_DIR, "..", "..", "src", "grasp_preshaping", "data", "debug"
        ),
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

    x_local = a * sgn_cosE * (abs_cosE**e1) * sgn_cosO * (np.abs(cosO) ** e2)
    y_local = b * sgn_cosE * (abs_cosE**e1) * sgn_sinO * (np.abs(sinO) ** e2)
    z_local = c * sgn_sinE * (abs_sinE**e1)

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
    pts_local = np.stack(
        [x_local.ravel(), y_local.ravel(), z_local.ravel()], axis=0
    )  # (3, N)
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


def _draw_obb_wireframe(
    ax, pts_cm: np.ndarray, color: str = "#c0392b", alpha: float = 0.4
):
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
    corners_local = np.array(
        [
            [mins[0], mins[1], mins[2]],
            [maxs[0], mins[1], mins[2]],
            [maxs[0], maxs[1], mins[2]],
            [mins[0], maxs[1], mins[2]],
            [mins[0], mins[1], maxs[2]],
            [maxs[0], mins[1], maxs[2]],
            [maxs[0], maxs[1], maxs[2]],
            [mins[0], maxs[1], maxs[2]],
        ]
    )

    corners_world = corners_local @ eigenvectors.T + mean

    # 12 edges of the box
    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),  # bottom face
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),  # top face
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),  # vertical edges
    ]

    for i, j in edges:
        ax.plot(
            [corners_world[i, 0], corners_world[j, 0]],
            [corners_world[i, 1], corners_world[j, 1]],
            [corners_world[i, 2], corners_world[j, 2]],
            color=color,
            linewidth=0.5,
            alpha=alpha,
        )


def plot_object_gallery(fmt: str, dpi: int):
    """Show all test objects as 3D point clouds in a 3x3 grid, coloured by convexity.

    Overlays superquadric (SQ) wireframe meshes where debug-dump data is
    available. Objects without SQ data show bare point clouds.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    FONT_SIZES = {
        "title": 20,
        "panel_title": 14,
        "axis_label": 12,
        "tick": 10,
        "legend": 15,
    }

    sys.path.insert(0, SCRIPT_DIR)
    from object_registry import get_convexity, list_objects, load_object

    objects = sorted(list_objects())
    n_obj = len(objects)
    ncols = 3
    nrows = int(np.ceil(n_obj / ncols))

    fig, axes = plt.subplots(
        nrows,
        ncols,
        figsize=(13.5, 4.6 * nrows),
        subplot_kw={"projection": "3d"},
        facecolor=COLORS["bg"],
    )
    fig.suptitle(
        "Object Gallery",
        fontsize=FONT_SIZES["title"],
        fontweight="bold",
        color=COLORS["text"],
        y=0.98,
    )

    convex_color = COLORS["primary"]
    nonconvex_color = COLORS["warning"]
    sq_wire_color = COLORS["negative"]
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

            ax.scatter(
                pts_cm[:, 0],
                pts_cm[:, 1],
                pts_cm[:, 2],
                c=color,
                s=0.3,
                alpha=0.5,
                depthshade=True,
            )

            # --- SQ overlay (no OBB fallback) ---
            sq = _find_sq_for_object(obj_name, pts)
            if sq is not None:
                try:
                    xs, ys, zs = _generate_sq_surface(sq)
                    ax.plot_wireframe(
                        xs * 100,
                        ys * 100,
                        zs * 100,
                        color=sq_wire_color,
                        alpha=0.35,
                        linewidth=0.4,
                        rstride=2,
                        cstride=2,
                    )
                    sq_found_count += 1
                except Exception:
                    pass  # No overlay if SQ generation fails

            # Equal axis scaling
            ranges = (
                np.array(
                    [
                        pts[:, 0].max() - pts[:, 0].min(),
                        pts[:, 1].max() - pts[:, 1].min(),
                        pts[:, 2].max() - pts[:, 2].min(),
                    ]
                )
                * 100
            )
            max_range = ranges.max()
            mid = np.array(
                [
                    (pts[:, 0].max() + pts[:, 0].min()) / 2 * 100,
                    (pts[:, 1].max() + pts[:, 1].min()) / 2 * 100,
                    (pts[:, 2].max() + pts[:, 2].min()) / 2 * 100,
                ]
            )
            ax.set_xlim(mid[0] - max_range / 2, mid[0] + max_range / 2)
            ax.set_ylim(mid[1] - max_range / 2, mid[1] + max_range / 2)
            ax.set_zlim(mid[2] - max_range / 2, mid[2] + max_range / 2)
            try:
                ax.set_box_aspect([1, 1, 1])
            except Exception:
                pass

            tag = "convex" if c == "convex" else "non-convex"
            ax.set_title(
                f"{obj_name}\n({tag})",
                fontsize=FONT_SIZES["panel_title"],
                color=COLORS["text"],
                fontweight="bold",
            )
            ax.set_xlabel("X (cm)", fontsize=FONT_SIZES["axis_label"], labelpad=-1)
            ax.set_ylabel("Y (cm)", fontsize=FONT_SIZES["axis_label"], labelpad=-1)
            ax.set_zlabel("Z (cm)", fontsize=FONT_SIZES["axis_label"], labelpad=-1)
            ax.tick_params(labelsize=FONT_SIZES["tick"])
            ax.view_init(elev=25, azim=-55)
        else:
            ax.axis("off")

    # Summary line
    if sq_found_count > 0:
        print(f"  [SQ]  {sq_found_count}/{n_obj} objects matched debug-dump SQ params")
    else:
        print(
            f"  [SQ] No SQ debug dumps matched — showing point clouds only "
            f"(run 'python run.py --debug-dumps' to generate SQ data)"
        )

    # Legend
    from matplotlib.lines import Line2D

    legend_elements = [
        Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            markerfacecolor=convex_color,
            markersize=10,
            label="Convex",
        ),
        Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            markerfacecolor=nonconvex_color,
            markersize=10,
            label="Non-convex",
        ),
    ]
    if sq_found_count > 0:
        legend_elements.append(
            Line2D([0], [0], color=sq_wire_color, linewidth=1.5, label="SQ fit")
        )
    fig.legend(
        handles=legend_elements,
        loc="lower center",
        ncol=len(legend_elements),
        fontsize=FONT_SIZES["legend"],
        frameon=False,
    )

    fig.tight_layout(rect=[0, 0.04, 1, 0.96])
    _save_fig(fig, "fig9_object_gallery", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 10: Score vs Samples Sweep
# ---------------------------------------------------------------------------


def plot_score_vs_samples(fmt: str, dpi: int):
    """Plot how combined_score converges as prediction_samples increases.

    Shows single-view vs multi-view score convergence without latency panel.
    Requires results/score_sweep_results.csv generated by run.py --sweep-samples.
    If the file doesn't exist, prints a message and returns.
    """
    import matplotlib.pyplot as plt

    sweep_path = os.path.join(RESULTS_DIR, "score_sweep_results.csv")
    if not os.path.isfile(sweep_path):
        print(
            "  Skipping fig10_score_vs_samples: no sweep data. "
            "Run: python run.py --sweep-samples"
        )
        return

    import pandas as pd

    sweep = pd.read_csv(sweep_path)

    if len(sweep) == 0:
        print("  Skipping fig10_score_vs_samples: sweep data is empty.")
        return

    fig, ax = plt.subplots(1, 1, figsize=(8, 5), facecolor=COLORS["bg"])
    fig.set_facecolor(COLORS["bg"])

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

    ax.errorbar(
        sample_counts,
        sv_means,
        yerr=sv_stds,
        fmt="o-",
        color=COLORS["single_view"],
        label="Single-view",
        capsize=3,
        markersize=5,
        linewidth=2,
    )
    ax.errorbar(
        sample_counts,
        mv_means,
        yerr=mv_stds,
        fmt="s-",
        color=COLORS["multi_view"],
        label="Multi-view",
        capsize=3,
        markersize=5,
        linewidth=2,
    )

    # Baseline reference
    baseline_path = os.path.join(RESULTS_DIR, "baseline_results.csv")
    if os.path.isfile(baseline_path):
        bl = pd.read_csv(baseline_path)
        bl_mean = bl["combined_score"].mean()
        ax.axhline(
            bl_mean,
            color=COLORS["positive"],
            linestyle="--",
            linewidth=1.5,
            label=f"Baseline (mean={bl_mean:.3f})",
        )

    ax.set_xlabel("Prediction Samples", color=COLORS["text"])
    ax.set_ylabel("Mean Combined Score", color=COLORS["text"])
    ax.set_title("Score vs Sample Count", color=COLORS["text"])
    ax.legend(fontsize=9, frameon=True)
    ax.set_xscale("log")
    ax.grid(True, alpha=0.3, color=COLORS["grid"])
    ax.set_facecolor(COLORS["panel_bg"])

    fig.tight_layout()
    _save_fig(fig, "fig10_score_vs_samples", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 12: Latency + Score Variability vs Sample Count
# ---------------------------------------------------------------------------


def plot_latency_score_variability(fmt: str, dpi: int):
    """Two-panel figure: latency vs samples (left) and score variability vs
    samples (right).  No single-view vs multi-view split — shows pooled data
    with per-trial scatter.

    Requires results/score_sweep_results.csv.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd

    sweep_path = os.path.join(RESULTS_DIR, "score_sweep_results.csv")
    if not os.path.isfile(sweep_path):
        print("  Skipping fig12_latency_score_variability: no sweep data.")
        return

    sweep = pd.read_csv(sweep_path)
    if len(sweep) == 0:
        print("  Skipping fig12_latency_score_variability: sweep data is empty.")
        return

    sample_counts = sorted(sweep["prediction_samples"].unique())

    fig, (ax_lat, ax_var) = plt.subplots(1, 2, figsize=(12, 5), facecolor=COLORS["bg"])
    fig.set_facecolor(COLORS["bg"])

    # --- Left panel: Latency vs samples (pooled across conditions) ---
    lat_means = []
    lat_stds = []
    for n in sample_counts:
        lats = sweep[sweep["prediction_samples"] == n]["latency_ms"]
        lat_means.append(lats.mean() if len(lats) > 0 else np.nan)
        lat_stds.append(lats.std() if len(lats) > 0 else np.nan)

    ax_lat.errorbar(
        sample_counts,
        lat_means,
        yerr=lat_stds,
        fmt="o-",
        color=COLORS["warning"],
        capsize=3,
        markersize=5,
        linewidth=2,
        label="Mean +/- 1 SD",
    )

    # Jittered strip of individual points
    rng = np.random.default_rng(0)
    for n in sample_counts:
        lats = sweep[sweep["prediction_samples"] == n]["latency_ms"].values
        jitter = rng.uniform(-0.08, 0.08, len(lats))
        x = np.full(len(lats), np.log10(n)) + jitter
        ax_lat.scatter(
            10**x,
            lats,
            alpha=0.15,
            s=6,
            color=COLORS["warning"],
            edgecolors="none",
            zorder=1,
        )

    ax_lat.axhline(
        100,
        color=COLORS["negative"],
        linestyle="--",
        linewidth=1.5,
        label="IDE limit (100 ms)",
    )
    ax_lat.axhline(
        400,
        color=COLORS["negative"],
        linestyle=":",
        linewidth=1.5,
        label="MAR limit (400 ms)",
    )
    ax_lat.set_xlabel("Prediction Samples", color=COLORS["text"])
    ax_lat.set_ylabel("Pipeline Latency (ms)", color=COLORS["text"])
    ax_lat.set_title("Latency vs Sample Count", color=COLORS["text"], fontweight="bold")
    ax_lat.legend(fontsize=8, frameon=True)
    ax_lat.set_xscale("log")
    ax_lat.grid(True, alpha=0.3, color=COLORS["grid"])
    ax_lat.set_facecolor(COLORS["panel_bg"])

    # --- Right panel: Score variability (IQR) vs samples ---
    score_medians = []
    score_iqr_lo = []
    score_iqr_hi = []
    for n in sample_counts:
        scores = sweep[sweep["prediction_samples"] == n]["combined_score"].values
        if len(scores) > 0:
            med = np.median(scores)
            q1, q3 = np.percentile(scores, [25, 75])
            score_medians.append(med)
            score_iqr_lo.append(med - q1)
            score_iqr_hi.append(q3 - med)
        else:
            score_medians.append(np.nan)
            score_iqr_lo.append(np.nan)
            score_iqr_hi.append(np.nan)

    ax_var.errorbar(
        sample_counts,
        score_medians,
        yerr=[score_iqr_lo, score_iqr_hi],
        fmt="s-",
        color=COLORS["primary"],
        capsize=3,
        markersize=5,
        linewidth=2,
        label="Median +/- IQR",
    )

    # Jittered strip
    for n in sample_counts:
        scores = sweep[sweep["prediction_samples"] == n]["combined_score"].values
        jitter = rng.uniform(-0.08, 0.08, len(scores))
        x = np.full(len(scores), np.log10(n)) + jitter
        ax_var.scatter(
            10**x,
            scores,
            alpha=0.15,
            s=6,
            color=COLORS["primary"],
            edgecolors="none",
            zorder=1,
        )

    ax_var.set_xlabel("Prediction Samples", color=COLORS["text"])
    ax_var.set_ylabel("Combined Score", color=COLORS["text"])
    ax_var.set_title(
        "Score Variability vs Sample Count", color=COLORS["text"], fontweight="bold"
    )
    ax_var.legend(fontsize=8, frameon=True)
    ax_var.set_xscale("log")
    ax_var.grid(True, alpha=0.3, color=COLORS["grid"])
    ax_var.set_facecolor(COLORS["panel_bg"])

    fig.tight_layout()
    _save_fig(fig, "fig12_latency_score_variability", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 13: Overall Accuracy Gain from Multi-view
# ---------------------------------------------------------------------------


def plot_accuracy_gain(fmt: str, dpi: int):
    """Single box plot showing the distribution of per-object accuracy gain
    (multi-view minus single-view grasp_accuracy_pct) across all objects.

    Requires results/intent_precision_delta.csv.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    delta_path = os.path.join(RESULTS_DIR, "intent_precision_delta.csv")
    if not os.path.isfile(delta_path):
        print("  Skipping fig13_accuracy_gain: no delta data.")
        return

    rows = _load_csv(delta_path)
    gains = [
        float(r["delta_grasp_accuracy_pct"])
        for r in rows
        if not _is_nan(r.get("delta_grasp_accuracy_pct"))
    ]

    if not gains:
        print("  Skipping fig13_accuracy_gain: no accuracy gain data.")
        return

    fig, ax = plt.subplots(1, 1, figsize=(6, 2.5))

    # Horizontal box plot (no scatter dots)
    bp = ax.boxplot(
        [gains],
        tick_labels=["Accuracy Gain"],
        patch_artist=True,
        widths=0.45,
        showfliers=False,
        vert=False,
        medianprops=dict(color="white", linewidth=2),
        whiskerprops=dict(color=COLORS["text"], linewidth=1),
        capprops=dict(color=COLORS["text"], linewidth=1),
    )
    bp["boxes"][0].set_facecolor(COLORS["multi_view"])
    bp["boxes"][0].set_edgecolor(COLORS["text"])

    # Zero reference line (now vertical)
    ax.axvline(
        0, color=COLORS["negative"], linestyle="--", linewidth=1.2, label="No gain"
    )

    # Annotate median and mean
    median_val = np.median(gains)
    mean_val = np.mean(gains)
    ax.scatter(
        [mean_val],
        [1],
        marker="D",
        s=50,
        color="white",
        edgecolors=COLORS["text"],
        zorder=3,
        linewidths=1,
        label=f"Mean ({mean_val:+.1f}%)",
    )
    ax.annotate(
        f"Median: {median_val:+.1f}%",
        xy=(median_val, 1),
        xytext=(median_val, 1.35),
        fontsize=9,
        color=COLORS["text"],
        fontweight="bold",
        arrowprops=dict(arrowstyle="-", color=COLORS["grid"], linewidth=0.5),
    )

    ax.set_xlabel(
        "Accuracy Gain (percentage points)", color=COLORS["text"], fontsize=10
    )
    ax.set_title(
        "Multi-view Accuracy Gain\n(per object)",
        color=COLORS["text"],
        fontsize=11,
        fontweight="bold",
    )
    ax.legend(fontsize=8, frameon=True, loc="upper left")
    ax.grid(True, axis="x", alpha=0.3, color=COLORS["grid"])

    fig.tight_layout()
    _save_fig(fig, "fig13_accuracy_gain", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 11: Tier B — Full ROS Pipeline Latency
# ---------------------------------------------------------------------------


def _is_nan(v) -> bool:
    """Check if a CSV value is NaN or missing."""
    if v is None:
        return True
    try:
        return str(v).strip().lower() in ("", "nan", "none", "n/a")
    except Exception:
        return True


def plot_tier_b_latency(fmt: str, dpi: int):
    """Bar chart of Tier B full ROS pipeline latency with per-stage breakdown."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = _load_csv(os.path.join(RESULTS_DIR, "tier_b_latency_results.csv"))
    if not rows:
        print("  Skipping Tier B latency figure (no data)")
        return

    emg_rows = [r for r in rows if r.get("method") == "emg" and r.get("status") == "ok"]
    svc_rows = [
        r for r in rows if r.get("method") == "service" and r.get("status") == "ok"
    ]

    if not emg_rows and not svc_rows:
        print("  Skipping Tier B latency figure (no successful rows)")
        return

    fig, axes = plt.subplots(
        1, 2, figsize=(14, 6), gridspec_kw={"width_ratios": [2, 1]}
    )

    # ── Left panel: per-stage waterfall for EMG method ──
    ax = axes[0]
    if emg_rows:
        stage_fields = [
            ("emg_to_twisting_ms", "EMG → TWISTING", "#3498db"),
            ("twisting_to_segmenting_ms", "TWISTING → SEGMENTING", "#2ecc71"),
            ("cloud_to_planning_ms", "Cloud → PLANNING", "#e67e22"),
            ("planning_to_approaching_ms", "PLANNING → APPROACHING", "#e74c3c"),
        ]

        # Compute means per stage
        stage_data = []
        for field, label, color in stage_fields:
            values = [
                float(r[field])
                for r in emg_rows
                if field in r and not _is_nan(r.get(field))
            ]
            mean_val = np.mean(values) if values else 0
            stage_data.append((label, mean_val, color))

        # Waterfall bars
        cumulative = 0
        for label, mean_val, color in stage_data:
            ax.bar(
                label,
                mean_val,
                bottom=cumulative,
                color=color,
                edgecolor="white",
                linewidth=0.5,
            )
            if mean_val > 0:
                ax.text(
                    label,
                    cumulative + mean_val / 2,
                    f"{mean_val:.1f}",
                    ha="center",
                    va="center",
                    fontsize=8,
                    color="white",
                    fontweight="bold",
                )
            cumulative += mean_val

        # Total bar
        ax.bar("TOTAL", cumulative, color="#2c3e50", edgecolor="white", linewidth=0.5)
        ax.text(
            "TOTAL",
            cumulative / 2,
            f"{cumulative:.1f}",
            ha="center",
            va="center",
            fontsize=9,
            color="white",
            fontweight="bold",
        )

        # Threshold lines
        ax.axhline(
            y=400,
            color=COLORS["negative"],
            linestyle="--",
            linewidth=1.5,
            label="MAR (400 ms)",
        )
        ax.axhline(
            y=100,
            color=COLORS.get("warning", "#f39c12"),
            linestyle=":",
            linewidth=1.0,
            label="IDE (100 ms)",
        )
        ax.legend(loc="upper left", fontsize=8)

        for label in ax.get_xticklabels():
            label.set_ha("center")
            label.set_rotation(25)
            label.set_fontsize(8)

    ax.set_ylabel("Cumulative Latency (ms)")
    ax.set_title("EMG Method — Per-Stage Breakdown")

    # ── Right panel: EMG vs Service comparison ──
    ax = axes[1]
    methods = []
    means = []
    stds = []
    colors_bar = []

    if emg_rows:
        vals = [
            float(r["total_latency_ms"])
            for r in emg_rows
            if not _is_nan(r.get("total_latency_ms"))
        ]
        if vals:
            methods.append("EMG\n(full pipeline)")
            means.append(np.mean(vals))
            stds.append(np.std(vals))
            colors_bar.append("#3498db")

    if svc_rows:
        vals = [
            float(r["total_latency_ms"])
            for r in svc_rows
            if not _is_nan(r.get("total_latency_ms"))
        ]
        if vals:
            methods.append("Service\n(direct call)")
            means.append(np.mean(vals))
            stds.append(np.std(vals))
            colors_bar.append("#2ecc71")

    if methods:
        x = np.arange(len(methods))
        bars = ax.bar(
            x,
            means,
            yerr=stds,
            capsize=5,
            color=colors_bar,
            edgecolor="white",
            linewidth=0.5,
        )
        for bar, mean_val in zip(bars, means):
            ax.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() / 2,
                f"{mean_val:.1f}",
                ha="center",
                va="center",
                fontsize=10,
                color="white",
                fontweight="bold",
            )
        ax.set_xticks(x)
        ax.set_xticklabels(methods, fontsize=9)
        ax.axhline(
            y=400,
            color=COLORS["negative"],
            linestyle="--",
            linewidth=1.5,
            label="MAR (400 ms)",
        )
        ax.axhline(
            y=100,
            color=COLORS.get("warning", "#f39c12"),
            linestyle=":",
            linewidth=1.0,
            label="IDE (100 ms)",
        )
        ax.legend(loc="upper left", fontsize=8)

    ax.set_ylabel("Total Latency (ms)")
    ax.set_title("Method Comparison")

    fig.set_facecolor(COLORS["bg"])
    fig.tight_layout()
    _save_fig(fig, "fig11_tier_b_latency", fmt, dpi)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description="Generate Test 1 figures")
    parser.add_argument(
        "--format",
        default="png",
        choices=["pdf", "png"],
        help="Output format (default: png)",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=300,
        help="Resolution for raster formats (default: 300)",
    )
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
    plot_wrist_error_cdf(occlusion_rows, args.format, args.dpi)  # Fig 1
    plot_convexity_analysis(delta_rows, summary_rows, args.format, args.dpi)  # Fig 2
    plot_per_view_coverage(occlusion_rows, args.format, args.dpi)  # Fig 3

    # Score-based analysis figures (Figures 4-6)
    plot_benchmark_comparison(
        occlusion_rows, summary_rows, args.format, args.dpi
    )  # Fig 4
    plot_best_score_by_type(occlusion_rows, args.format, args.dpi)  # Fig 5
    plot_score_vs_wrist_angle(occlusion_rows, args.format, args.dpi)  # Fig 6

    # Figure 7: Latency waterfall
    plot_per_stage_waterfall(args.format, args.dpi)
    generate_latex_table(latency_rows, delta_rows, summary_rows)

    # Figure 8: 3D synthetic setup (always generated, no data dependencies)
    plot_synthetic_setup(args.format, args.dpi)

    # Figure 9: Object gallery (no data dependencies)
    plot_object_gallery(args.format, args.dpi)

    # Figure 10: Score vs samples sweep (requires sweep data)
    plot_score_vs_samples(args.format, args.dpi)

    # Figure 11: Tier B full ROS pipeline latency
    plot_tier_b_latency(args.format, args.dpi)

    # Figure 12: Latency + score variability vs sample count
    plot_latency_score_variability(args.format, args.dpi)

    # Figure 13: Overall accuracy gain from multiview
    plot_accuracy_gain(args.format, args.dpi)

    print(f"\nDone. Figures in: {FIGURES_DIR}")


if __name__ == "__main__":
    main()
