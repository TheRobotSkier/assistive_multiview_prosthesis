"""Plot 2 — Samples vs Output Variability.

Strip plot overlaid on a boxplot, with number of samples on the x-axis
(discrete sample counts) and output variability (combined grasp score) on
the y-axis.  Based on figure 12 data.

Data sourced from::

    tests/test1_software_verification/results/score_sweep_results.csv

Columns used: ``prediction_samples`` (1K–100K), ``combined_score``.
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .palette import TEAL, BLUE_GRAY, WHITE, apply_presentation_style

# --- Paths ---------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(
    SCRIPT_DIR, "..", "..", "tests", "test1_software_verification", "results"
)
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")


def _format_sample_label(n):
    """Format a sample count as a human-readable label (1K, 20K, ...)."""
    if n >= 1000:
        k = n // 1000
        if n % 1000 == 0:
            return f"{k}K"
        return f"{k}K"
    return str(n)


def plot_variability(fmt="png", dpi=300):
    """Generate the strip-on-boxplot figure and save it."""
    sweep_path = os.path.join(RESULTS_DIR, "score_sweep_results.csv")
    if not os.path.isfile(sweep_path):
        print("  [plot2] No sweep data — skipping.")
        return

    sweep = pd.read_csv(sweep_path)
    if len(sweep) == 0:
        print("  [plot2] Sweep data empty — skipping.")
        return

    sample_counts = sorted(sweep["prediction_samples"].unique())
    positions = np.arange(len(sample_counts))
    labels = [_format_sample_label(n) for n in sample_counts]

    # Collect score arrays per sample count
    groups = [
        sweep.loc[sweep["prediction_samples"] == n, "combined_score"].values
        for n in sample_counts
    ]

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_facecolor("none")

    # --- Boxplot layer (behind) ---
    bp = ax.boxplot(
        groups,
        positions=positions,
        widths=0.55,
        patch_artist=True,
        showfliers=False,
        zorder=2,
        medianprops=dict(color="#333333", linewidth=1.2),
        whiskerprops=dict(color="#666666", linewidth=1),
        capprops=dict(color="#666666", linewidth=1),
    )
    for patch in bp["boxes"]:
        patch.set_facecolor(BLUE_GRAY)
        patch.set_alpha(0.45)
        patch.set_edgecolor("#666666")
        patch.set_linewidth(0.8)

    # --- Strip / scatter layer (in front) ---
    rng = np.random.default_rng(0)
    for pos, scores in zip(positions, groups):
        if len(scores) == 0:
            continue
        jitter = rng.uniform(-0.18, 0.18, len(scores))
        ax.scatter(
            np.full(len(scores), pos) + jitter,
            scores,
            alpha=0.15,
            s=7,
            color=TEAL,
            edgecolors="none",
            zorder=3,
        )

    ax.set_xticks(positions)
    ax.set_xticklabels(labels, fontsize=10)
    ax.set_xlabel("Prediction Samples", fontsize=11)
    ax.set_ylabel("Combined Score", fontsize=11)
    apply_presentation_style(ax)

    fig.tight_layout()
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out_path = os.path.join(OUTPUT_DIR, f"plot2_variability.{fmt}")
    fig.savefig(
        out_path, dpi=dpi, facecolor="none", transparent=True,
        bbox_inches="tight",
    )
    plt.close(fig)
    print(f"  [plot2] saved {out_path}")
