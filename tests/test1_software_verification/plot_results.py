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
    """Mean ± std latency bar chart with P95/P99 annotations."""
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
    ax.set_title("Test 1a: Latency Summary (mean ± std, P95/P99)")
    ax.legend(loc="upper right", fontsize=8)

    # Annotate P95 values
    for i, p95 in enumerate(p95s):
        ax.annotate(f"{p95:.0f}", xy=(i, p95), fontsize=7,
                    ha="center", va="bottom", color="red")

    fig.tight_layout()
    _save_fig(fig, "fig2_latency_summary", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 3: Intent precision bar chart
# ---------------------------------------------------------------------------

def plot_intent_precision(summary_rows: list[dict], delta_rows: list[dict],
                          fmt: str, dpi: int):
    """Grouped bar chart: single-view vs. multi-view accuracy per object."""
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
        ("fully_correct_pct", "Fully Correct (%)"),
        ("grasp_accuracy_pct", "Grasp Type Match (%)"),
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

    fig.suptitle("Test 1b: Intent Precision — Single-view vs. Multi-view", fontsize=12)
    fig.tight_layout()
    _save_fig(fig, "fig3_intent_precision", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 4: Intent precision delta
# ---------------------------------------------------------------------------

def plot_intent_delta(delta_rows: list[dict], fmt: str, dpi: int):
    """Bar chart of Δ (multi - single) per object with threshold lines."""
    import matplotlib.pyplot as plt

    if not delta_rows:
        print("  SKIP: No delta data")
        return

    objects = [d["object"] for d in delta_rows]
    deltas = [float(d["delta_fully_correct_pct"]) for d in delta_rows]

    fig, ax = plt.subplots(figsize=(max(8, len(objects) * 1.2), 5))
    x = np.arange(len(objects))
    colors = ["#55a868" if d > 0 else "#c44e52" for d in deltas]
    bars = ax.bar(x, deltas, color=colors, edgecolor="black", linewidth=0.5)

    ax.axhline(y=0, color="black", linewidth=0.5)
    ax.axhline(y=10, color="green", linestyle="--", linewidth=1.5, label="IDE (≥10%)")

    ax.set_xticks(x)
    ax.set_xticklabels(objects, rotation=45, ha="right")
    ax.set_ylabel("Δ Fully Correct (%)")
    ax.set_title("Test 1b: Intent Precision Δ (Multi-view − Single-view)")
    ax.legend()

    # Annotate values
    for i, d in enumerate(deltas):
        ax.annotate(f"{d:+.1f}%", xy=(i, d), fontsize=8,
                    ha="center", va="bottom" if d >= 0 else "top")

    fig.tight_layout()
    _save_fig(fig, "fig4_intent_delta", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 5: Pose error scatter
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
    ax.axhline(y=15, color="gray", linestyle="--", alpha=0.5, label="15° threshold")

    ax.set_xlabel("Position Error (mm)")
    ax.set_ylabel("Orientation Error (deg)")
    ax.set_title("Test 1b: Pose Error — Single-view vs. Multi-view")
    ax.legend(fontsize=8)

    fig.tight_layout()
    _save_fig(fig, "fig5_pose_error_scatter", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 6: Point cloud coverage comparison (text-based summary)
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
    _save_fig(fig, "fig6_cloud_coverage", fmt, dpi)


# ---------------------------------------------------------------------------
# LaTeX table fragment
# ---------------------------------------------------------------------------

def generate_latex_table(latency_rows: list[dict], delta_rows: list[dict]):
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
            r"\begin{tabular}{l r r r}",
            r"\toprule",
            r"\textbf{Object} & \textbf{SV (\%)} & \textbf{MV (\%)} & \textbf{$\Delta$ (\%)} \\",
            r"\midrule",
        ])
        for d in delta_rows:
            lines.append(
                f"{d['object'].replace('_', '\\_')} & "
                f"{float(d['single_view_fully_correct_pct']):.1f} & "
                f"{float(d['multi_view_fully_correct_pct']):.1f} & "
                f"{float(d['delta_fully_correct_pct']):+.1f} \\\\"
            )
        mean_delta = np.mean([float(d["delta_fully_correct_pct"]) for d in delta_rows])
        lines.extend([
            r"\midrule",
            f"\\textbf{{Mean}} & & & {mean_delta:+.1f} \\\\",
            r"\bottomrule",
            r"\end{tabular}",
            r"\caption{Test 1b: Intent precision $\Delta$ (multi-view $-$ single-view). MAR $> 0$\,\%, IDE $\geq 10$\,\%.}",
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

    print(f"  Latency rows: {len(latency_rows)}")
    print(f"  Occlusion rows: {len(occlusion_rows)}")
    print(f"  Summary rows: {len(summary_rows)}")
    print(f"  Delta rows: {len(delta_rows)}")

    print("\nGenerating figures...")
    plot_latency_boxplot(latency_rows, args.format, args.dpi)
    plot_latency_summary(latency_rows, args.format, args.dpi)
    plot_intent_precision(summary_rows, delta_rows, args.format, args.dpi)
    plot_intent_delta(delta_rows, args.format, args.dpi)
    plot_pose_error_scatter(occlusion_rows, args.format, args.dpi)
    plot_cloud_coverage(occlusion_rows, args.format, args.dpi)
    generate_latex_table(latency_rows, delta_rows)

    print(f"\nDone. Figures in: {FIGURES_DIR}")


if __name__ == "__main__":
    main()
