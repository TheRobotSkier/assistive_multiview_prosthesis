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


def _save_fig(fig, name: str, fmt: str = "pdf", dpi: int = 150):
    """Save a matplotlib figure."""
    import matplotlib
    matplotlib.use("Agg")
    path = os.path.join(FIGURES_DIR, f"{name}.{fmt}")
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
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

    colors = plt.cm.Set2(np.linspace(0, 1, len(objects)))
    for patch, color in zip(bp["boxes"], colors):
        patch.set_facecolor(color)

    ax.axhline(y=400, color="red", linestyle="--", linewidth=1.5, label="MAR (400 ms)")
    ax.axhline(y=100, color="green", linestyle="--", linewidth=1.5, label="IDE (100 ms)")

    ax.set_ylabel("Pipeline Latency (ms)")
    ax.set_title("Test 1a: Pipeline Latency per Object")
    ax.legend(loc="upper right")
    ax.tick_params(axis="x", rotation=45)

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
    bars = ax.bar(x, means, yerr=stds, capsize=4, color=plt.cm.Set2(0.3),
                  edgecolor="black", linewidth=0.5)

    # P95/P99 markers
    ax.scatter(x, p95s, marker="_", color="red", s=100, zorder=5, label="P95")
    ax.scatter(x, p99s, marker="_", color="darkred", s=100, zorder=5, label="P99")

    ax.axhline(y=400, color="red", linestyle="--", linewidth=1, alpha=0.7, label="MAR (400 ms)")
    ax.axhline(y=100, color="green", linestyle="--", linewidth=1, alpha=0.7, label="IDE (100 ms)")

    ax.set_xticks(x)
    ax.set_xticklabels(objects, rotation=45, ha="right")
    ax.set_ylabel("Pipeline Latency (ms)")
    ax.set_title("Test 1a: Latency Summary (mean +/- std, P95/P99)")
    ax.legend(loc="upper right", fontsize=8)

    # Annotate P95 values
    for i, p95 in enumerate(p95s):
        ax.annotate(f"{p95:.0f}", xy=(i, p95), fontsize=7,
                    ha="center", va="bottom", color="red")

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
        ("grasp_correct_pct", "Grasp Type Correct (%)"),
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

        ax.bar(x - width / 2, sv_vals, width, label="Single-view", color="#4c72b0")
        ax.bar(x + width / 2, mv_vals, width, label="Multi-view", color="#55a868")

        ax.set_xticks(x)
        ax.set_xticklabels([o.replace("_", "\n") for o in objects_with_both],
                           fontsize=7, rotation=45, ha="right")
        ax.set_ylabel(metric_label)
        ax.legend(fontsize=7)
        ax.set_title(metric_label)

    fig.suptitle("Test 1b: Intent Precision -- Single-view vs. Multi-view", fontsize=12)
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
    # Use the new grasp_correct delta, fall back to legacy field
    deltas = [float(d.get("delta_grasp_correct_pct",
                           d.get("delta_fully_correct_pct", 0))) for d in delta_rows]

    fig, ax = plt.subplots(figsize=(max(8, len(objects) * 1.2), 5))
    x = np.arange(len(objects))
    colors = ["#55a868" if d > 0 else "#c44e52" for d in deltas]
    bars = ax.bar(x, deltas, color=colors, edgecolor="black", linewidth=0.5)

    ax.axhline(y=0, color="black", linewidth=0.5)
    ax.axhline(y=10, color="green", linestyle="--", linewidth=1.5, label="IDE ($\\geq$10%)")

    ax.set_xticks(x)
    ax.set_xticklabels(objects, rotation=45, ha="right")
    ax.set_ylabel("$\\Delta$ Grasp Correct (%)")
    ax.set_title("Test 1b: Intent Precision $\\Delta$ (Multi-view $-$ Single-view)")
    ax.legend()

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

    for cond, color, label in [("single_view", "#4c72b0", "Single-view"),
                                ("multi_view", "#55a868", "Multi-view")]:
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
    ax_cdf.set_title("Wrist Rotation Error CDF")
    ax_cdf.legend(loc="lower right")
    ax_cdf.set_xlim(0, None)
    ax_cdf.set_ylim(0, 1.05)
    ax_cdf.grid(True, alpha=0.3)

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

            sv_vals = [float(by_object[o]["single_view"].get("grasp_correct_pct",
                        by_object[o]["single_view"].get("grasp_accuracy_pct", 0)))
                       for o in objects_with_both]
            mv_vals = [float(by_object[o]["multi_view"].get("grasp_correct_pct",
                        by_object[o]["multi_view"].get("grasp_accuracy_pct", 0)))
                       for o in objects_with_both]

            ax_bar.bar(x - width / 2, sv_vals, width, label="Single-view",
                       color="#4c72b0", edgecolor="black", linewidth=0.3)
            ax_bar.bar(x + width / 2, mv_vals, width, label="Multi-view",
                       color="#55a868", edgecolor="black", linewidth=0.3)

            ax_bar.set_xticks(x)
            ax_bar.set_xticklabels([o.replace("_", "\n") for o in objects_with_both],
                                   fontsize=6, rotation=45, ha="right")
            ax_bar.set_ylabel("Grasp Type Correct (%)")
            ax_bar.set_title("Grasp Type Correctness")
            ax_bar.legend(fontsize=7)
            ax_bar.set_ylim(0, 105)

            # Annotate values
            for i, (sv, mv) in enumerate(zip(sv_vals, mv_vals)):
                ax_bar.annotate(f"{sv:.0f}", xy=(i - width / 2, sv), fontsize=6,
                                ha="center", va="bottom")
                ax_bar.annotate(f"{mv:.0f}", xy=(i + width / 2, mv), fontsize=6,
                                ha="center", va="bottom")

    fig.suptitle("Test 1b: Wrist Error Distribution & Grasp Correctness", fontsize=12)
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

    for cond, color, marker in [("single_view", "#4c72b0", "o"),
                                 ("multi_view", "#55a868", "s")]:
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

    ax.axvline(x=10, color="gray", linestyle=":", alpha=0.5, label="10 mm threshold")
    ax.axhline(y=15, color="gray", linestyle="--", alpha=0.5, label="15 deg threshold")

    ax.set_xlabel("Position Error (mm)")
    ax.set_ylabel("Orientation Error (deg)")
    ax.set_title("Test 1b: Pose Error -- Single-view vs. Multi-view")
    ax.legend(fontsize=8)

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

    ax.bar(x - width / 2, sv_pct, width, label="Single-view", color="#4c72b0")
    ax.bar(x + width / 2, mv_pct, width, label="Multi-view", color="#55a868")

    ax.set_xticks(x)
    ax.set_xticklabels(objects_with_both, rotation=45, ha="right")
    ax.set_ylabel("Cloud Coverage (%)")
    ax.set_title("Test 1b: Point Cloud Coverage by Condition")
    ax.legend()

    for i, (sv, mv) in enumerate(zip(sv_pct, mv_pct)):
        ax.annotate(f"{sv:.0f}%", xy=(i - width / 2, sv), fontsize=7,
                    ha="center", va="bottom")
        ax.annotate(f"{mv:.0f}%", xy=(i + width / 2, mv), fontsize=7,
                    ha="center", va="bottom")

    fig.tight_layout()
    _save_fig(fig, "fig7_cloud_coverage", fmt, dpi)


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
                    color="#4c72b0", edgecolor="black", linewidth=0.3)
    bars_b = ax.bar(x + width / 2, tier_b_means, width, label="Tier B (Full Pipeline)",
                    color="#c44e52", edgecolor="black", linewidth=0.3)

    # Dim bars where data is missing
    for i, (bar, has) in enumerate(zip(bars_a, has_a)):
        if not has:
            bar.set_alpha(0.2)
    for i, (bar, has) in enumerate(zip(bars_b, has_b)):
        if not has:
            bar.set_alpha(0.2)

    ax.axhline(y=400, color="red", linestyle="--", linewidth=1.5, label="MAR (400 ms)")
    ax.axhline(y=100, color="green", linestyle="--", linewidth=1.5, label="IDE (100 ms)")

    ax.set_xticks(x)
    ax.set_xticklabels(all_objects, rotation=45, ha="right")
    ax.set_ylabel("Latency (ms)")
    ax.set_title("Test 1: Tier A vs. Tier B Latency Comparison")
    ax.legend(fontsize=8)

    # Annotate overhead where both tiers have data
    for i, o in enumerate(all_objects):
        if has_a[i] and has_b[i]:
            overhead = tier_b_means[i] - tier_a_means[i]
            ax.annotate(f"+{overhead:.0f}", xy=(i, tier_b_means[i]),
                        fontsize=7, ha="center", va="bottom", color="#c44e52")

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
            sv_pct = float(d.get("single_view_grasp_correct_pct",
                                  d.get("single_view_fully_correct_pct", 0)))
            mv_pct = float(d.get("multi_view_grasp_correct_pct",
                                  d.get("multi_view_fully_correct_pct", 0)))
            delta_val = float(d.get("delta_grasp_correct_pct",
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
        mean_delta = np.mean([float(d.get("delta_grasp_correct_pct",
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

    print(f"  Latency rows: {len(latency_rows)}")
    print(f"  Occlusion rows: {len(occlusion_rows)}")
    print(f"  Summary rows: {len(summary_rows)}")
    print(f"  Delta rows: {len(delta_rows)}")
    print(f"  Tier B rows: {len(tier_b_rows)}")

    print("\nGenerating figures...")
    plot_latency_boxplot(latency_rows, args.format, args.dpi)
    plot_latency_summary(latency_rows, args.format, args.dpi)
    plot_intent_precision(summary_rows, delta_rows, args.format, args.dpi)
    plot_intent_delta(delta_rows, args.format, args.dpi)
    plot_wrist_error_cdf(occlusion_rows, summary_rows, args.format, args.dpi)
    plot_pose_error_scatter(occlusion_rows, args.format, args.dpi)
    plot_cloud_coverage(occlusion_rows, args.format, args.dpi)
    plot_tier_ab_latency(latency_rows, tier_b_rows, args.format, args.dpi)
    generate_latex_table(latency_rows, delta_rows, summary_rows)

    print(f"\nDone. Figures in: {FIGURES_DIR}")


if __name__ == "__main__":
    main()
