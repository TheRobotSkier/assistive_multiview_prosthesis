"""Plot 4 — VIO Tracking.

Dual-panel figure:

* **Main panel** — x/y trajectory scatter plot on asinh-scaled axes, one
  colour for head and one for arm, pooled across all v6 bags in
  ``data/bags``.
* **Side panel** — vertical boxplot of Euclidean distance
  √(x² + y²) from origin, one box per camera (head, arm), showing the
  spread of divergence.

The arm odom is known to diverge massively in some bags (up to ~1000 m),
so the asinh (inverse hyperbolic sine) scale is essential to visualise
both small and large excursions on the same axes.

Data sourced from the ``nav_msgs/msg/Odometry`` topics::

    /jetson/head/odom
    /jetson/arm/odom
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .palette import (
    BLUE,
    TEAL,
    BLUE_GRAY,
    SAGE,
    WHITE,
    BLACK,
    apply_presentation_style,
)
from .bag_loader import load_odom_from_bags

# --- Paths ---------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")

# Subsample factor for the scatter (trajectory messages are dense ~10 Hz).
SCATTER_SUBSAMPLE = 10


def _asinh_formatter(value, pos=None):
    """Format asinh tick values back into linear metres for readability."""
    import math

    linear = math.sinh(value)
    if abs(linear) >= 1000:
        return f"{linear / 1000:.0f}k"
    if abs(linear) >= 10:
        return f"{linear:.0f}"
    if abs(linear) >= 1:
        return f"{linear:.1f}"
    return f"{linear:.2f}"


def plot_vio_tracking(fmt="png", dpi=300, bag_data=None):
    """Generate the dual-panel VIO tracking figure and save it.

    Parameters
    ----------
    bag_data : dict or None
        Pre-loaded bag data from :func:`load_odom_from_bags`.  If ``None``
        the bags are read from disk now.
    """
    if bag_data is None:
        bag_data = load_odom_from_bags()

    if not bag_data:
        print("  [plot4] No bag data — skipping.")
        return

    # --- Pool trajectories across all bags ---------------------------------
    head_x, head_y = [], []
    arm_x, arm_y = [], []
    for d in bag_data.values():
        head_x.extend(d["head"]["x"])
        head_y.extend(d["head"]["y"])
        arm_x.extend(d["arm"]["x"])
        arm_y.extend(d["arm"]["y"])

    head_x = np.asarray(head_x, dtype=float)
    head_y = np.asarray(head_y, dtype=float)
    arm_x = np.asarray(arm_x, dtype=float)
    arm_y = np.asarray(arm_y, dtype=float)

    # Euclidean distance from origin (x,y plane)
    head_dist = np.hypot(head_x, head_y)
    arm_dist = np.hypot(arm_x, arm_y)

    # --- Figure layout: trajectory (wide) + side distance boxplot (narrow) -
    fig = plt.figure(figsize=(11, 5.5))
    gs = fig.add_gridspec(
        1, 2, width_ratios=[4.0, 1.0], wspace=0.18
    )
    ax_traj = fig.add_subplot(gs[0])
    ax_box = fig.add_subplot(gs[1])

    # =================== Main panel: trajectory scatter ===================
    ax_traj.set_facecolor("none")

    # Subsample for performance / clarity
    def _subsample(arr):
        if len(arr) > SCATTER_SUBSAMPLE:
            return arr[::SCATTER_SUBSAMPLE]
        return arr

    ax_traj.scatter(
        _subsample(np.arcsinh(head_x)),
        _subsample(np.arcsinh(head_y)),
        s=2.5,
        color=BLUE,
        alpha=0.25,
        edgecolors="none",
        label="Head",
        rasterized=True,
    )
    ax_traj.scatter(
        _subsample(np.arcsinh(arm_x)),
        _subsample(np.arcsinh(arm_y)),
        s=2.5,
        color=TEAL,
        alpha=0.25,
        edgecolors="none",
        label="Arm",
        rasterized=True,
    )

    # Origin marker
    ax_traj.scatter(
        [0], [0], marker="+", color=BLACK, s=60, linewidths=1.5, zorder=5
    )

    ax_traj.set_xlabel("x position (m, asinh)", fontsize=11)
    ax_traj.set_ylabel("y position (m, asinh)", fontsize=11)
    ax_traj.legend(
        loc="upper right",
        framealpha=0.9,
        fontsize=10,
        markerscale=3,
        handletextpad=0.3,
    )

    # asinh tick formatting -> show linear metres
    from matplotlib.ticker import FuncFormatter

    ax_traj.xaxis.set_major_formatter(FuncFormatter(_asinh_formatter))
    ax_traj.yaxis.set_major_formatter(FuncFormatter(_asinh_formatter))
    apply_presentation_style(ax_traj)

    # =================== Side panel: vertical distance boxplot ===========
    ax_box.set_facecolor("none")

    bp = ax_box.boxplot(
        [head_dist, arm_dist],
        vert=True,
        positions=[1, 2],
        widths=0.55,
        patch_artist=True,
        showfliers=False,  # hide outliers for a cleaner summary
        medianprops=dict(color=BLACK, linewidth=1.4),
        whiskerprops=dict(color="#555555", linewidth=1),
        capprops=dict(color="#555555", linewidth=1),
    )
    box_colors = [BLUE_GRAY, SAGE]  # head, arm
    for patch, color in zip(bp["boxes"], box_colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.8)
        patch.set_edgecolor("#444444")
        patch.set_linewidth(0.8)

    ax_box.set_xticks([1, 2])
    ax_box.set_xticklabels(["Head", "Arm"], fontsize=11)
    ax_box.set_ylabel(
        "Euclidean distance from origin √(x²+y²) (m)", fontsize=11
    )
    apply_presentation_style(ax_box)

    # --- Summary stats to stdout ------------------------------------------
    print(
        f"  [plot4] head: n={len(head_x)} dist "
        f"med={np.median(head_dist):.2f} max={np.max(head_dist):.2f} m"
    )
    print(
        f"  [plot4] arm:  n={len(arm_x)} dist "
        f"med={np.median(arm_dist):.2f} max={np.max(arm_dist):.2f} m"
    )

    fig.subplots_adjust(left=0.08, right=0.97, top=0.95, bottom=0.12, wspace=0.20)
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out_path = os.path.join(OUTPUT_DIR, f"plot4_vio_tracking.{fmt}")
    fig.savefig(
        out_path, dpi=dpi, facecolor="none", transparent=True,
        bbox_inches="tight",
    )
    plt.close(fig)
    print(f"  [plot4] saved {out_path}")
