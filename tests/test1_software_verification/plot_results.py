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
# Figure 1: Latency box plot per object
# ---------------------------------------------------------------------------

def plot_latency_boxplot(latency_rows: list[dict], fmt: str, dpi: int):
    """Per-object latency box plot with threshold lines."""
    import matplotlib.pyplot as plt

    if not latency_rows:
        print("  SKIP: No latency data")
        return

    # Group by object
    by_object = defaultdict(list)
    for row in latency_rows:
        by_object[row["object"]].append(float(row["pipeline_time_ms"]))

    objects = sorted(by_object.keys())
    data = [by_object[o] for o in objects]

    fig, ax = plt.subplots(figsize=(max(8, len(objects) * 1.2), 5))
    bp = ax.boxplot(data, tick_labels=objects, patch_artist=True, widths=0.6)

    for patch in bp["boxes"]:
        patch.set_facecolor(COLORS["primary"])
        patch.set_alpha(0.6)
    for median in bp["medians"]:
        median.set_color(COLORS["text"])
        median.set_linewidth(1.5)

    ax.axhline(y=400, color=COLORS["negative"], linestyle="--", linewidth=1.5, label="MAR (400 ms)")
    ax.axhline(y=100, color=COLORS["positive"], linestyle="--", linewidth=1.5, label="IDE (100 ms)")

    ax.set_ylabel("Pipeline Latency (ms)")
    ax.set_title("Test 1a: Pipeline Latency per Object", fontweight="bold")
    ax.legend(loc="upper right")
    ax.tick_params(axis="x", rotation=45)
    ax.grid(axis="y", alpha=0.3, color=COLORS["grid"])

    # Add mean annotations
    for i, (obj, times) in enumerate(zip(objects, data)):
        mean_t = np.mean(times)
        ax.annotate(f"{mean_t:.0f}", xy=(i + 1, mean_t),
                    fontsize=7, ha="center", va="bottom")

    fig.tight_layout()
    _save_fig(fig, "fig1_latency_boxplot", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 2: Latency summary bar chart
# ---------------------------------------------------------------------------

def plot_latency_summary(latency_rows: list[dict], fmt: str, dpi: int):
    """Mean +/- std latency bar chart with P95/P99 annotations."""
    import matplotlib.pyplot as plt

    if not latency_rows:
        print("  SKIP: No latency data")
        return

    by_object = defaultdict(list)
    for row in latency_rows:
        by_object[row["object"]].append(float(row["pipeline_time_ms"]))

    objects = sorted(by_object.keys())
    means = [np.mean(by_object[o]) for o in objects]
    stds = [np.std(by_object[o]) for o in objects]
    p95s = [np.percentile(by_object[o], 95) for o in objects]
    p99s = [np.percentile(by_object[o], 99) for o in objects]

    fig, ax = plt.subplots(figsize=(max(8, len(objects) * 1.2), 5))
    x = np.arange(len(objects))
    bars = ax.bar(x, means, yerr=stds, capsize=4, color=COLORS["primary"],
                  edgecolor=COLORS["text"], linewidth=0.5, alpha=0.7)

    # P95/P99 markers
    ax.scatter(x, p95s, marker="_", color=COLORS["negative"], s=100, zorder=5, label="P95")
    ax.scatter(x, p99s, marker="_", color=COLORS["warning"], s=100, zorder=5, label="P99")

    ax.axhline(y=400, color=COLORS["negative"], linestyle="--", linewidth=1, alpha=0.7, label="MAR (400 ms)")
    ax.axhline(y=100, color=COLORS["positive"], linestyle="--", linewidth=1, alpha=0.7, label="IDE (100 ms)")

    ax.set_xticks(x)
    ax.set_xticklabels(objects, rotation=45, ha="right")
    ax.set_ylabel("Pipeline Latency (ms)")
    ax.set_title("Test 1a: Latency Summary (mean +/- std, P95/P99)", fontweight="bold")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(axis="y", alpha=0.3, color=COLORS["grid"])

    # Annotate P95 values
    for i, p95 in enumerate(p95s):
        ax.annotate(f"{p95:.0f}", xy=(i, p95), fontsize=7,
                    ha="center", va="bottom", color=COLORS["negative"])

    fig.tight_layout()
    _save_fig(fig, "fig2_latency_summary", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 3: Intent precision bar chart (updated: grasp type + wrist error)
# ---------------------------------------------------------------------------

def plot_intent_precision(summary_rows: list[dict], delta_rows: list[dict],
                          fmt: str, dpi: int):
    """Grouped bar chart: single-view vs. multi-view per object.

    Panel 1: Grasp type accuracy (%) — primary metric
    Panel 2: Mean wrist rotation error (degrees)
    Panel 3: Mean position error (mm) — diagnostic
    """
    import matplotlib.pyplot as plt

    if not summary_rows:
        print("  SKIP: No intent precision data")
        return

    # Group by object
    by_object = defaultdict(dict)
    for row in summary_rows:
        by_object[row["object"]][row["condition"]] = row

    objects = sorted(by_object.keys())
    objects_with_both = [o for o in objects
                         if "single_view" in by_object[o] and "multi_view" in by_object[o]]

    if not objects_with_both:
        print("  SKIP: No objects with both single-view and multi-view data")
        return

    fig, axes = plt.subplots(1, 3, figsize=(max(12, len(objects_with_both) * 1.5), 5))

    metrics = [
        ("grasp_accuracy_pct", "Grasp Type Accuracy vs Baseline (%)"),
        ("mean_orientation_error_deg", "Wrist Rotation Error (deg)"),
        ("mean_position_error_mm", "Position Error (mm)"),
    ]

    x = np.arange(len(objects_with_both))
    width = 0.35

    for ax, (metric_key, metric_label) in zip(axes, metrics):
        sv_vals = [float(by_object[o]["single_view"].get(metric_key, 0))
                   for o in objects_with_both]
        mv_vals = [float(by_object[o]["multi_view"].get(metric_key, 0))
                   for o in objects_with_both]

        ax.bar(x - width / 2, sv_vals, width, label="Single-view",
               color=COLORS["single_view"], edgecolor=COLORS["text"], linewidth=0.3)
        ax.bar(x + width / 2, mv_vals, width, label="Multi-view",
               color=COLORS["multi_view"], edgecolor=COLORS["text"], linewidth=0.3)

        ax.set_xticks(x)
        ax.set_xticklabels([o.replace("_", "\n") for o in objects_with_both],
                           fontsize=7, rotation=45, ha="right")
        ax.set_ylabel(metric_label)
        ax.legend(fontsize=7)
        ax.set_title(metric_label)
        ax.grid(axis="y", alpha=0.3, color=COLORS["grid"])

    fig.suptitle("Test 1b: Intent Precision -- Single-view vs. Multi-view",
                 fontsize=12, fontweight="bold", color=COLORS["text"])
    fig.tight_layout()
    _save_fig(fig, "fig3_intent_precision", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 4: Intent precision delta (grasp-type-only)
# ---------------------------------------------------------------------------

def plot_intent_delta(delta_rows: list[dict], fmt: str, dpi: int):
    """Bar chart of delta (multi - single) grasp correctness per object."""
    import matplotlib.pyplot as plt

    if not delta_rows:
        print("  SKIP: No delta data")
        return

    objects = [d["object"] for d in delta_rows]
    # Use the primary metric: grasp accuracy vs baseline
    deltas = [float(d.get("delta_grasp_accuracy_pct",
                           d.get("delta_fully_correct_pct", 0))) for d in delta_rows]

    fig, ax = plt.subplots(figsize=(max(8, len(objects) * 1.2), 5))
    x = np.arange(len(objects))
    colors = [COLORS["positive"] if d > 0 else COLORS["negative"] for d in deltas]
    bars = ax.bar(x, deltas, color=colors, edgecolor=COLORS["text"], linewidth=0.5)

    ax.axhline(y=0, color=COLORS["text"], linewidth=0.5)
    ax.axhline(y=10, color=COLORS["positive"], linestyle="--", linewidth=1.5, label="IDE ($\\geq$10%)")

    ax.set_xticks(x)
    ax.set_xticklabels(objects, rotation=45, ha="right")
    ax.set_ylabel("$\\Delta$ Grasp Correct (%)")
    ax.set_title("Test 1b: Intent Precision $\\Delta$ (Multi-view $-$ Single-view)",
                 fontweight="bold")
    ax.legend()
    ax.grid(axis="y", alpha=0.3, color=COLORS["grid"])

    # Annotate values
    for i, d in enumerate(deltas):
        ax.annotate(f"{d:+.1f}%", xy=(i, d), fontsize=8,
                    ha="center", va="bottom" if d >= 0 else "top")

    fig.tight_layout()
    _save_fig(fig, "fig4_intent_delta", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 5: Cumulative wrist error CDF + grasp correctness
# ---------------------------------------------------------------------------

def plot_wrist_error_cdf(occlusion_rows: list[dict], summary_rows: list[dict],
                         fmt: str, dpi: int):
    """Two-panel figure:
      Left:  Cumulative distribution of wrist rotation errors (CDF)
             for single-view vs. multi-view.
      Right: Grasp type correctness rate per object (grouped bar).
    """
    import matplotlib.pyplot as plt

    if not occlusion_rows:
        print("  SKIP: No occlusion data for CDF plot")
        return

    # --- Left panel: CDF of wrist errors ---
    fig, (ax_cdf, ax_bar) = plt.subplots(1, 2, figsize=(14, 5),
                                          gridspec_kw={"width_ratios": [1, 1.2]})

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
        ax_cdf.plot(errors, cdf, color=color, linewidth=2, label=label)

        # Annotate median and P90
        median = np.median(errors)
        p90 = np.percentile(errors, 90)
        ax_cdf.axvline(median, color=color, linestyle=":", alpha=0.5, linewidth=1)
        ax_cdf.annotate(f"median={median:.0f}°", xy=(median, 0.5),
                        fontsize=7, color=color, ha="left", va="bottom",
                        xytext=(5, 0), textcoords="offset points")

    ax_cdf.set_xlabel("Wrist Rotation Error (deg)")
    ax_cdf.set_ylabel("Cumulative Fraction")
    ax_cdf.set_title("Wrist Rotation Error CDF", fontweight="bold")
    ax_cdf.legend(loc="lower right")
    ax_cdf.set_xlim(0, None)
    ax_cdf.set_ylim(0, 1.05)
    ax_cdf.grid(True, alpha=0.3, color=COLORS["grid"])

    # --- Right panel: Grasp correctness per object ---
    if summary_rows:
        by_object = defaultdict(dict)
        for row in summary_rows:
            by_object[row["object"]][row["condition"]] = row

        objects = sorted(by_object.keys())
        objects_with_both = [o for o in objects
                             if "single_view" in by_object[o]
                             and "multi_view" in by_object[o]]

        if objects_with_both:
            x = np.arange(len(objects_with_both))
            width = 0.35

            sv_vals = [float(by_object[o]["single_view"].get("grasp_accuracy_pct",
                        by_object[o]["single_view"].get("grasp_correct_pct", 0)))
                       for o in objects_with_both]
            mv_vals = [float(by_object[o]["multi_view"].get("grasp_accuracy_pct",
                        by_object[o]["multi_view"].get("grasp_correct_pct", 0)))
                       for o in objects_with_both]

            ax_bar.bar(x - width / 2, sv_vals, width, label="Single-view",
                       color=COLORS["single_view"], edgecolor=COLORS["text"], linewidth=0.3)
            ax_bar.bar(x + width / 2, mv_vals, width, label="Multi-view",
                       color=COLORS["multi_view"], edgecolor=COLORS["text"], linewidth=0.3)

            ax_bar.set_xticks(x)
            ax_bar.set_xticklabels([o.replace("_", "\n") for o in objects_with_both],
                                   fontsize=6, rotation=45, ha="right")
            ax_bar.set_ylabel("Grasp Type Correct (%)")
            ax_bar.set_title("Grasp Type Correctness", fontweight="bold")
            ax_bar.legend(fontsize=7)
            ax_bar.set_ylim(0, 105)
            ax_bar.grid(axis="y", alpha=0.3, color=COLORS["grid"])

            # Annotate values
            for i, (sv, mv) in enumerate(zip(sv_vals, mv_vals)):
                ax_bar.annotate(f"{sv:.0f}", xy=(i - width / 2, sv), fontsize=6,
                                ha="center", va="bottom")
                ax_bar.annotate(f"{mv:.0f}", xy=(i + width / 2, mv), fontsize=6,
                                ha="center", va="bottom")

    fig.suptitle("Test 1b: Wrist Error Distribution & Grasp Correctness",
                 fontsize=12, fontweight="bold", color=COLORS["text"])
    fig.tight_layout()
    _save_fig(fig, "fig5_wrist_cdf_grasp_correct", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 6: Pose error scatter
# ---------------------------------------------------------------------------

def plot_pose_error_scatter(occlusion_rows: list[dict], fmt: str, dpi: int):
    """Scatter plot: position error vs. orientation error, colored by condition."""
    import matplotlib.pyplot as plt

    if not occlusion_rows:
        print("  SKIP: No occlusion data")
        return

    fig, ax = plt.subplots(figsize=(7, 6))

    for cond, color, marker in [("single_view", COLORS["single_view"], "o"),
                                 ("multi_view", COLORS["multi_view"], "s")]:
        rows = [r for r in occlusion_rows
                if r.get("condition") == cond
                and "position_error_mm" in r
                and "orientation_error_deg" in r]
        if not rows:
            continue
        pos_err = [float(r["position_error_mm"]) for r in rows]
        orient_err = [float(r["orientation_error_deg"]) for r in rows]
        ax.scatter(pos_err, orient_err, c=color, marker=marker,
                   alpha=0.6, s=30, label=cond.replace("_", "-"))

    ax.axvline(x=10, color=COLORS["grid"], linestyle=":", alpha=0.5, label="10 mm threshold")
    ax.axhline(y=15, color=COLORS["grid"], linestyle="--", alpha=0.5, label="15 deg threshold")

    ax.set_xlabel("Position Error (mm)")
    ax.set_ylabel("Orientation Error (deg)")
    ax.set_title("Test 1b: Pose Error -- Single-view vs. Multi-view", fontweight="bold")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, color=COLORS["grid"])

    fig.tight_layout()
    _save_fig(fig, "fig6_pose_error_scatter", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 7: Point cloud coverage comparison
# ---------------------------------------------------------------------------

def plot_cloud_coverage(occlusion_rows: list[dict], fmt: str, dpi: int):
    """Bar chart showing % of full cloud visible in each condition."""
    import matplotlib.pyplot as plt

    if not occlusion_rows:
        print("  SKIP: No occlusion data")
        return

    # Get per-object, per-condition cloud sizes
    by_obj_cond = defaultdict(lambda: {"n_cloud": [], "n_full": []})
    for row in occlusion_rows:
        key = (row["object"], row["condition"])
        by_obj_cond[key]["n_cloud"].append(int(row.get("n_cloud_points", 0)))
        by_obj_cond[key]["n_full"].append(int(row.get("n_full_points", 1)))

    objects = sorted(set(k[0] for k in by_obj_cond))
    objects_with_both = [o for o in objects
                         if (o, "single_view") in by_obj_cond
                         and (o, "multi_view") in by_obj_cond]

    if not objects_with_both:
        print("  SKIP: No objects with both conditions")
        return

    sv_pct = []
    mv_pct = []
    for o in objects_with_both:
        sv_data = by_obj_cond[(o, "single_view")]
        mv_data = by_obj_cond[(o, "multi_view")]
        sv_pct.append(np.mean(sv_data["n_cloud"]) / max(np.mean(sv_data["n_full"]), 1) * 100)
        mv_pct.append(np.mean(mv_data["n_cloud"]) / max(np.mean(mv_data["n_full"]), 1) * 100)

    fig, ax = plt.subplots(figsize=(max(8, len(objects_with_both) * 1.2), 5))
    x = np.arange(len(objects_with_both))
    width = 0.35

    ax.bar(x - width / 2, sv_pct, width, label="Single-view", color=COLORS["single_view"])
    ax.bar(x + width / 2, mv_pct, width, label="Multi-view", color=COLORS["multi_view"])

    ax.set_xticks(x)
    ax.set_xticklabels(objects_with_both, rotation=45, ha="right")
    ax.set_ylabel("Cloud Coverage (%)")
    ax.set_title("Test 1b: Point Cloud Coverage by Condition", fontweight="bold")
    ax.legend()
    ax.grid(axis="y", alpha=0.3, color=COLORS["grid"])

    for i, (sv, mv) in enumerate(zip(sv_pct, mv_pct)):
        ax.annotate(f"{sv:.0f}%", xy=(i - width / 2, sv), fontsize=7,
                    ha="center", va="bottom")
        ax.annotate(f"{mv:.0f}%", xy=(i + width / 2, mv), fontsize=7,
                    ha="center", va="bottom")

    fig.tight_layout()
    _save_fig(fig, "fig7_cloud_coverage", fmt, dpi)


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

    fig.suptitle("Test 1b: Multi-view Advantage by Object Geometry",
                 fontsize=12, fontweight="bold", color=COLORS["text"])
    fig.tight_layout()
    _save_fig(fig, "fig7b_convexity_analysis", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 8: Tier A vs Tier B latency comparison
# ---------------------------------------------------------------------------

def plot_tier_ab_latency(tier_a_rows: list[dict], tier_b_rows: list[dict],
                         fmt: str, dpi: int):
    """Grouped bar chart comparing Tier A (grasp planning) and Tier B (full pipeline) latency.

    Tier B rows may contain multiple methods (emg, service). If both are present,
    only the EMG method is used for the primary comparison. Otherwise the available
    method is used.
    """
    import matplotlib.pyplot as plt

    if not tier_a_rows and not tier_b_rows:
        print("  SKIP: No latency data for Tier A/B comparison")
        return

    # Collect Tier A means per object
    by_object_a = defaultdict(list)
    for row in tier_a_rows:
        by_object_a[row["object"]].append(float(row["pipeline_time_ms"]))

    # Collect Tier B means per object (prefer EMG method, fall back to service)
    by_object_b = defaultdict(list)
    for row in tier_b_rows:
        t = row.get("total_latency_ms", "")
        status = row.get("status", "")
        if not t or t == "nan" or status in ("not_implemented", "timeout", "no_service"):
            continue
        # If method column exists, prefer emg; otherwise include all
        method = row.get("method", "")
        if method == "emg":
            by_object_b[row["object"]].append(("emg", float(t)))
        elif method == "service":
            by_object_b[row["object"]].append(("service", float(t)))
        else:
            by_object_b[row["object"]].append(("unknown", float(t)))

    # For each object, pick the best method (prefer emg)
    by_object_b_means = {}
    for obj, entries in by_object_b.items():
        emg_vals = [v for m, v in entries if m == "emg"]
        svc_vals = [v for m, v in entries if m == "service"]
        if emg_vals:
            by_object_b_means[obj] = np.mean(emg_vals)
        elif svc_vals:
            by_object_b_means[obj] = np.mean(svc_vals)
        else:
            by_object_b_means[obj] = np.mean([v for _, v in entries])

    # Use union of objects
    all_objects = sorted(set(by_object_a.keys()) | set(by_object_b_means.keys()))
    if not all_objects:
        return

    tier_a_means = [np.mean(by_object_a[o]) if o in by_object_a else 0 for o in all_objects]
    tier_b_means = [by_object_b_means.get(o, 0) for o in all_objects]
    has_a = [o in by_object_a for o in all_objects]
    has_b = [o in by_object_b_means for o in all_objects]

    fig, ax = plt.subplots(figsize=(max(8, len(all_objects) * 1.2), 5))
    x = np.arange(len(all_objects))
    width = 0.35

    bars_a = ax.bar(x - width / 2, tier_a_means, width, label="Tier A (Grasp Planning)",
                    color=COLORS["primary"], edgecolor=COLORS["text"], linewidth=0.3)
    bars_b = ax.bar(x + width / 2, tier_b_means, width, label="Tier B (Full Pipeline)",
                    color=COLORS["warning"], edgecolor=COLORS["text"], linewidth=0.3)

    # Dim bars where data is missing
    for i, (bar, has) in enumerate(zip(bars_a, has_a)):
        if not has:
            bar.set_alpha(0.2)
    for i, (bar, has) in enumerate(zip(bars_b, has_b)):
        if not has:
            bar.set_alpha(0.2)

    ax.axhline(y=400, color=COLORS["negative"], linestyle="--", linewidth=1.5, label="MAR (400 ms)")
    ax.axhline(y=100, color=COLORS["positive"], linestyle="--", linewidth=1.5, label="IDE (100 ms)")

    ax.set_xticks(x)
    ax.set_xticklabels(all_objects, rotation=45, ha="right")
    ax.set_ylabel("Latency (ms)")
    ax.set_title("Test 1: Tier A vs. Tier B Latency Comparison", fontweight="bold")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.3, color=COLORS["grid"])

    # Annotate overhead where both tiers have data
    for i, o in enumerate(all_objects):
        if has_a[i] and has_b[i]:
            overhead = tier_b_means[i] - tier_a_means[i]
            ax.annotate(f"+{overhead:.0f}", xy=(i, tier_b_means[i]),
                        fontsize=7, ha="center", va="bottom", color=COLORS["warning"])

    fig.tight_layout()
    _save_fig(fig, "fig8_tier_ab_latency", fmt, dpi)


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
        ax.set_xlabel("X (cm)", fontsize=7, labelpad=2)
        ax.set_ylabel("Y (cm)", fontsize=7, labelpad=2)
        ax.set_zlabel("Z (cm)", fontsize=7, labelpad=2)
        ax.tick_params(labelsize=6)

    # --- 4-panel figure ---
    fig = plt.figure(figsize=(16, 14), facecolor=BG)

    viewpoints = [
        (25, -55, "Perspective Overview"),
        (85, -90, "Top-Down View"),
        (10, 0, "Side View (from +Y)"),
        (15, -160, "Rear View (from -X)"),
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

    fig.suptitle("Test 1: Synthetic Multi-View Setup Geometry",
                 fontsize=14, fontweight="bold", y=0.98)
    fig.tight_layout(rect=[0, 0.06, 1, 0.95])
    _save_fig(fig, "fig9_synthetic_setup_3d", fmt, dpi)

# ---------------------------------------------------------------------------
# Figure 10: Per-stage latency stacked bar chart
# ---------------------------------------------------------------------------

def plot_per_stage_latency(fmt: str = "pdf", dpi: int = 150):
    """Stacked bar chart showing latency breakdown by pipeline stage."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.ticker as mticker

    rows = _load_csv(os.path.join(RESULTS_DIR, "latency_per_stage_results.csv"))
    if not rows:
        print("  Skipping per-stage latency figure (no data)")
        return

    stage_fields = [
        ("pipeline_manager_ms", "Pipeline Manager", "#3498db"),
        ("twist_propagation_ms", "Twist Propagation", "#2ecc71"),
        ("segmentation_ms", "Segmentation", "#e67e22"),
        ("pm_cloud_handling_ms", "PM Cloud Handling", "#9b59b6"),
        ("preshaping_ms", "Grasp Preshaping", "#e74c3c"),
    ]

    # Compute mean per object per stage
    from collections import defaultdict
    import numpy as np

    groups = defaultdict(list)
    for r in rows:
        if r.get("status") == "ok":
            groups[r["object"]].append(r)

    objects = sorted(groups.keys())
    if not objects:
        return

    stage_means = {}
    for field, label, _ in stage_fields:
        stage_means[label] = []
        for obj in objects:
            values = [float(r[field]) for r in groups[obj]
                      if r.get(field) and r[field] != "nan"
                      and not _is_nan(r[field])]
            stage_means[label].append(np.mean(values) if values else 0)

    # Plot stacked bars
    fig, ax = plt.subplots(figsize=(max(8, len(objects) * 1.2), 5))
    x = np.arange(len(objects))
    width = 0.6
    bottom = np.zeros(len(objects))

    for field, label, color in stage_fields:
        values = stage_means[label]
        ax.bar(x, values, width, bottom=bottom, label=label, color=color)
        bottom += np.array(values)

    # MAR threshold line
    ax.axhline(y=400, color=COLORS["negative"], linestyle="--", linewidth=1.5,
               label="MAR (400 ms)")
    ax.axhline(y=100, color=COLORS["warning"], linestyle=":", linewidth=1.0,
               label="IDE (100 ms)")

    ax.set_xlabel("Object")
    ax.set_ylabel("Latency (ms)")
    ax.set_title("Per-Stage Pipeline Latency Breakdown")
    ax.set_xticks(x)
    ax.set_xticklabels(objects, rotation=45, ha="right", fontsize=8)
    ax.legend(loc="upper right", fontsize=7)
    ax.yaxis.set_major_formatter(mticker.FormatStrFormatter("%.0f"))

    _save_fig(fig, "fig10_per_stage_latency", fmt, dpi)


def _is_nan(val) -> bool:
    """Check if a CSV value represents NaN."""
    if val is None:
        return True
    try:
        return str(val).lower() == "nan" or float(val) != float(val)
    except (ValueError, TypeError):
        return True


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

    stage_fields = [
        ("pipeline_manager_ms", "Pipeline Manager", "#3498db"),
        ("twist_propagation_ms", "Twist Propagation", "#2ecc71"),
        ("segmentation_ms", "Segmentation", "#e67e22"),
        ("pm_cloud_handling_ms", "PM Cloud Handling", "#9b59b6"),
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

    _save_fig(fig, "fig11_latency_waterfall", fmt, dpi)


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
    tier_b_rows = _load_csv(os.path.join(RESULTS_DIR, "tier_b_latency_results.csv"))
    per_stage_rows = _load_csv(os.path.join(RESULTS_DIR, "latency_per_stage_results.csv"))

    print(f"  Latency rows: {len(latency_rows)}")
    print(f"  Occlusion rows: {len(occlusion_rows)}")
    print(f"  Summary rows: {len(summary_rows)}")
    print(f"  Delta rows: {len(delta_rows)}")
    print(f"  Tier B rows: {len(tier_b_rows)}")
    print(f"  Per-stage rows: {len(per_stage_rows)}")

    print("\nGenerating figures...")
    plot_latency_boxplot(latency_rows, args.format, args.dpi)
    plot_latency_summary(latency_rows, args.format, args.dpi)
    plot_intent_precision(summary_rows, delta_rows, args.format, args.dpi)
    plot_intent_delta(delta_rows, args.format, args.dpi)
    plot_wrist_error_cdf(occlusion_rows, summary_rows, args.format, args.dpi)
    plot_pose_error_scatter(occlusion_rows, args.format, args.dpi)
    plot_cloud_coverage(occlusion_rows, args.format, args.dpi)
    plot_convexity_analysis(delta_rows, summary_rows, args.format, args.dpi)
    plot_tier_ab_latency(latency_rows, tier_b_rows, args.format, args.dpi)
    plot_per_stage_latency(args.format, args.dpi)
    plot_per_stage_waterfall(args.format, args.dpi)
    generate_latex_table(latency_rows, delta_rows, summary_rows)

    # Figure 9: 3D synthetic setup (always generated, no data dependencies)
    plot_synthetic_setup(args.format, args.dpi)

    print(f"\nDone. Figures in: {FIGURES_DIR}")


if __name__ == "__main__":
    main()
