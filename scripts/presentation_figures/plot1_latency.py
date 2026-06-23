"""Plot 1 — CUDA vs Non-CUDA Latency.

Horizontal boxplot comparing CUDA and non-CUDA segmentation latency.
A vertical line marks the 228 ms target threshold.

Data sourced from the same files used to generate figure 7 in
``tests/test1_software_verification/figures``::

    segmentation_cpu_trials.csv   (100 trials, non-CUDA)
    segmentation_cuda_trials.csv  (100 trials, CUDA)
"""
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .palette import GRAY, BLUE, BLACK, WHITE, apply_presentation_style

# --- Paths ---------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(
    SCRIPT_DIR, "..", "..", "tests", "test1_software_verification", "results"
)
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")

TARGET_MS = 228.0  # target threshold to display


def _load_latencies(csv_path):
    """Load latency_ms values (status == 'ok') from a segmentation CSV."""
    latencies = []
    if not os.path.isfile(csv_path):
        return latencies
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            if row.get("status") == "ok":
                try:
                    latencies.append(float(row["latency_ms"]))
                except (ValueError, KeyError):
                    pass
    return latencies


def plot_latency(fmt="png", dpi=300):
    """Generate the horizontal boxplot and save it."""
    cpu_lat = _load_latencies(
        os.path.join(RESULTS_DIR, "segmentation_cpu_trials.csv")
    )
    cuda_lat = _load_latencies(
        os.path.join(RESULTS_DIR, "segmentation_cuda_trials.csv")
    )

    if not cpu_lat or not cuda_lat:
        print("  [plot1] Missing latency data — skipping.")
        return

    fig, ax = plt.subplots(figsize=(8, 3))
    ax.set_facecolor("none")

    # Horizontal boxplot: non-CUDA on top, CUDA on bottom
    bp = ax.boxplot(
        [cpu_lat, cuda_lat],
        vert=False,
        positions=[2, 1],
        widths=0.5,
        patch_artist=True,
        showfliers=True,
        medianprops=dict(color=BLACK, linewidth=1.5),
        whiskerprops=dict(color="#555555", linewidth=1),
        capprops=dict(color="#555555", linewidth=1),
        flierprops=dict(
            marker="o",
            markerfacecolor="#888888",
            markeredgecolor="none",
            markersize=3,
            alpha=0.5,
        ),
    )

    # Colour the two boxes
    colors = [GRAY, BLUE]  # non-CUDA, CUDA
    for patch, color in zip(bp["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.75)
        patch.set_edgecolor("#444444")
        patch.set_linewidth(0.8)

    # Vertical threshold line at 228 ms
    ax.axvline(
        TARGET_MS,
        color=BLACK,
        linestyle="--",
        linewidth=1.4,
        zorder=5,
    )
    # Annotation for the threshold
    y_top = ax.get_ylim()[1]
    ax.annotate(
        f"{int(TARGET_MS)} ms target",
        xy=(TARGET_MS, y_top),
        xytext=(8, -10),
        textcoords="offset points",
        fontsize=9,
        color=BLACK,
        va="top",
        ha="left",
    )

    ax.set_yticks([2, 1])
    ax.set_yticklabels(["Non-CUDA", "CUDA"], fontsize=11)
    ax.set_xlabel("Latency (ms)", fontsize=11)
    ax.set_xlim(left=0)
    apply_presentation_style(ax)

    fig.tight_layout()
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out_path = os.path.join(OUTPUT_DIR, f"plot1_latency.{fmt}")
    fig.savefig(
        out_path, dpi=dpi, facecolor="none", transparent=True,
        bbox_inches="tight",
    )
    plt.close(fig)
    print(f"  [plot1] saved {out_path}")
