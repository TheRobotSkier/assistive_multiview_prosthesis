#!/usr/bin/env python3
"""Generate figures for Test 3: Functional User Trial analysis.

Reads data.json from the same directory and produces PNG figures in figures/.

Usage:
    python plot_test3.py
    python plot_test3.py --format pdf
    python plot_test3.py --dpi 600
"""

import argparse
import json
import os
from collections import defaultdict

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_PATH = os.path.join(SCRIPT_DIR, "data.json")
FIGURES_DIR = os.path.join(SCRIPT_DIR, "figures")

# ---------------------------------------------------------------------------
# Project color palette (from test1 plot_results.py)
# ---------------------------------------------------------------------------
COLORS = {
    "primary": "#5099e9",
    "bg": "#ffffff",
    "panel_bg": "#ffffff",
    "single_view": "#50999e",
    "multi_view": "#5099e9",
    "positive": "#27ae60",
    "negative": "#c0392b",
    "warning": "#e67e22",
    "text": "#2c3e50",
    "grid": "#bdc3c7",
}

# Failure mode colour palette — one distinct colour per mode
FAILURE_COLORS = {
    "t": "#e74c3c",  # Trigger — red
    "v": "#e67e22",  # Vision — orange
    "i": "#f1c40f",  # Intent — yellow
    "s": "#2ecc71",  # Segmentation — green
    "d": "#3498db",  # Deactivation — blue
    "p": "#9b59b6",  # Proximity — purple
    "l": "#1abc9c",  # Localisation — teal
}


def _load_data() -> dict:
    """Load the trial data JSON."""
    with open(DATA_PATH) as f:
        return json.load(f)


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
# Figure 1: Failure Mode State-Space Mapping
# ---------------------------------------------------------------------------

def plot_failure_mode_catplot(data: dict, fmt: str, dpi: int):
    """Categorical scatterplot tracking system failures across trial configurations.

    X-axis: unique experimental trial configurations (ordered).
    Y-axis: 7 categorical failure modes ordered 1–7.
    Each character in Failure strings is unpacked as a distinct scatter point.
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import seaborn as sns

    failures_map = data["Failures"]
    # Build ordered list of (number, name, char)
    failure_modes = []
    for key in sorted(failures_map.keys()):
        num = int(key.split(".")[0])
        name = key.split(". ")[1]
        char = failures_map[key]
        failure_modes.append((num, name, char))

    # Build ordered trial labels: P1 Dual, P1 Single, P2 Single
    raw_trials = [trial["Trial"] for trial in data["data"]]
    p1_dual = sorted([t for t in raw_trials if "Participant 1" in t and "Dual" in t])
    p1_single = sorted([t for t in raw_trials if "Participant 1" in t and "Single" in t])
    p2_single = sorted([t for t in raw_trials if "Participant 2" in t and "Single" in t])
    trial_labels = p1_dual + p1_single + p2_single

    # Unpack every character in every Failure string
    rows = []
    for trial in data["data"]:
        trial_name = trial["Trial"]
        for fail_str in trial["Failure"]:
            for ch in fail_str:
                # Find which failure mode this character belongs to
                for num, name, char in failure_modes:
                    if ch == char:
                        rows.append({
                            "Trial": trial_name,
                            "Failure Mode": f"{num}. {name}",
                            "Mode Number": num,
                            "Failure Char": ch,
                        })
                        break

    if not rows:
        print("  SKIP fig1: No failure data found")
        return

    import pandas as pd
    df = pd.DataFrame(rows)

    # Ensure categorical ordering
    mode_order = [f"{num}. {name}" for num, name, _ in failure_modes]
    df["Failure Mode"] = pd.Categorical(df["Failure Mode"], categories=mode_order, ordered=True)
    df["Trial"] = pd.Categorical(df["Trial"], categories=trial_labels, ordered=True)

    # Build colour list for each failure mode
    mode_color_map = {}
    for num, name, char in failure_modes:
        mode_color_map[f"{num}. {name}"] = FAILURE_COLORS.get(char, COLORS["primary"])

    g = sns.catplot(
        data=df,
        x="Trial",
        y="Failure Mode",
        hue="Failure Mode",
        order=trial_labels,
        hue_order=mode_order,
        palette=mode_color_map,
        kind="swarm",
        size=8,
        alpha=0.75,
        edgecolor=COLORS["text"],
        linewidth=0.4,
        legend=False,
        height=6,
        aspect=1.8,
    )

    fig = g.fig
    ax = g.ax

    ax.set_xlabel("Experimental Trial Configuration", fontsize=11, fontweight="bold")
    ax.set_ylabel("Failure Mode", fontsize=11, fontweight="bold")
    ax.set_title("Failure Mode State-Space Mapping", fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.3, color=COLORS["grid"], axis="both")

    # Rotate x labels for readability
    for label in ax.get_xticklabels():
        label.set_rotation(30)
        label.set_ha("right")
        label.set_fontsize(8)

    for label in ax.get_yticklabels():
        label.set_fontsize(9)

    fig.tight_layout()
    _save_fig(fig, "fig1_failure_mode_state_space", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 2: Temporal Performance & Task Completion Efficiency Matrix
# ---------------------------------------------------------------------------

def plot_temporal_performance(data: dict, fmt: str, dpi: int):
    """Two-panel figure: boxplot of normalised completion time + stacked bar chart."""
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import pandas as pd

    fps = data["FPS"]
    reference_data = data["reference"]
    trial_data = data["data"]

    # Compute per-participant biological baseline (mean frames across both hands)
    participant_baselines = defaultdict(list)
    for ref in reference_data:
        # Extract participant number from trial name
        parts = ref["Trial"].split()
        participant = f"Participant {parts[1]}"
        for f in ref["Frames"]:
            participant_baselines[participant].append(f)

    participant_mean_baseline = {}
    for p, frames in participant_baselines.items():
        participant_mean_baseline[p] = np.mean(frames)

    # Gather successful attempts (numeric Frames values)
    success_records = []
    outcome_records = []

    for trial in trial_data:
        trial_name = trial["Trial"]
        parts = trial_name.split()
        participant = f"Participant {parts[1]}"
        view = parts[2]  # "Single" or "Dual"
        group_label = f"{participant} {view} View"

        for i, (frame_val, fail_str) in enumerate(zip(trial["Frames"], trial["Failure"])):
            if frame_val is not None:
                # Successful attempt
                duration_sec = frame_val / fps
                baseline_sec = participant_mean_baseline[participant] / fps
                normalised_pct = (duration_sec / baseline_sec) * 100.0
                success_records.append({
                    "Group": group_label,
                    "Participant": participant,
                    "Trial": trial_name,
                    "Attempt": i + 1,
                    "Frames": frame_val,
                    "Duration (s)": duration_sec,
                    "Normalised (%)": normalised_pct,
                })
                outcome_records.append({
                    "Group": group_label,
                    "Outcome": "Valid Attempt",
                })
            else:
                # Failed attempt (null)
                outcome_records.append({
                    "Group": group_label,
                    "Outcome": "Failed / Out of Time",
                })

    if not success_records and not outcome_records:
        print("  SKIP fig2: No trial data")
        return

    df_success = pd.DataFrame(success_records) if success_records else pd.DataFrame()
    df_outcome = pd.DataFrame(outcome_records)

    # Ordered groups: P1 Dual, P1 Single, P2 Single
    groups = [
        "Participant 1 Dual View",
        "Participant 1 Single View",
        "Participant 2 Single View",
    ]

    fig, (ax_a, ax_b) = plt.subplots(
        1, 2, figsize=(14, 6),
        gridspec_kw={"width_ratios": [1.2, 1], "wspace": 0.35},
    )

    # --- Subplot A: Boxplot ---
    if not df_success.empty:
        box_data = [df_success[df_success["Group"] == g]["Normalised (%)"].values
                     for g in groups]

        bp = ax_a.boxplot(
            box_data,
            tick_labels=groups,
            patch_artist=True,
            widths=0.5,
            showfliers=True,
            flierprops=dict(marker="o", markersize=5, alpha=0.5,
                            markerfacecolor=COLORS["primary"],
                            markeredgecolor=COLORS["text"], markeredgewidth=0.4),
            medianprops=dict(color="white", linewidth=2),
            whiskerprops=dict(color=COLORS["text"], linewidth=1),
            capprops=dict(color=COLORS["text"], linewidth=1),
            boxprops=dict(linewidth=0.8),
        )

        for patch in bp["boxes"]:
            patch.set_facecolor(COLORS["primary"])
            patch.set_alpha(0.7)
            patch.set_edgecolor(COLORS["text"])

        # Jittered scatter overlay
        rng = np.random.default_rng(42)
        for i, g in enumerate(groups):
            vals = df_success[df_success["Group"] == g]["Normalised (%)"].values
            jitter = rng.uniform(-0.15, 0.15, len(vals))
            ax_a.scatter(
                np.full(len(vals), i + 1) + jitter,
                vals,
                alpha=0.5,
                s=25,
                color=COLORS["multi_view"],
                edgecolors="white",
                linewidth=0.4,
                zorder=3,
            )

    # 100% reference line
    ax_a.axhline(
        y=100,
        color=COLORS["negative"],
        linestyle="--",
        linewidth=1.5,
        label="Biological Baseline (100%)",
    )

    ax_a.set_xlabel("Configuration", fontsize=11, fontweight="bold")
    ax_a.set_ylabel("Normalised Task Completion Time (%)", fontsize=11, fontweight="bold")
    ax_a.set_title("(A) Task Completion Efficiency", fontsize=13, fontweight="bold")
    ax_a.legend(fontsize=9, loc="upper right", framealpha=0.9)
    ax_a.grid(axis="y", alpha=0.3, color=COLORS["grid"])

    # Annotate mean for each participant
    if not df_success.empty:
        for i, g in enumerate(groups):
            vals = df_success[df_success["Group"] == g]["Normalised (%)"].values
            mean_val = np.mean(vals)
            ax_a.annotate(
                f"{mean_val:.0f}%",
                xy=(i + 1, mean_val),
                xytext=(0, 12),
                textcoords="offset points",
                ha="center",
                fontsize=8,
                fontweight="bold",
                color=COLORS["text"],
            )

    # --- Subplot B: Stacked Bar Chart ---
    valid_counts = []
    failed_counts = []
    for g in groups:
        sub = df_outcome[df_outcome["Group"] == g]
        valid_counts.append((sub["Outcome"] == "Valid Attempt").sum())
        failed_counts.append((sub["Outcome"] == "Failed / Out of Time").sum())

    x = np.arange(len(groups))
    width = 0.5

    bars_valid = ax_b.bar(
        x, valid_counts, width,
        color=COLORS["positive"],
        edgecolor="white",
        linewidth=0.8,
        label="Valid Attempt",
        alpha=0.85,
    )
    bars_failed = ax_b.bar(
        x, failed_counts, width,
        bottom=valid_counts,
        color=COLORS["negative"],
        edgecolor="white",
        linewidth=0.8,
        label="Failed / Out of Time",
        alpha=0.85,
    )

    # Annotate counts on bars
    for i, (v, f) in enumerate(zip(valid_counts, failed_counts)):
        total = v + f
        if v > 0:
            ax_b.text(i, v / 2, str(v), ha="center", va="center",
                      fontsize=10, fontweight="bold", color="white")
        if f > 0:
            ax_b.text(i, v + f / 2, str(f), ha="center", va="center",
                      fontsize=10, fontweight="bold", color="white")
        ax_b.text(i, total + 0.3, f"n={total}", ha="center", va="bottom",
                  fontsize=8, color=COLORS["text"])

    ax_b.set_xticks(x)
    ax_b.set_xticklabels(groups, fontsize=9, rotation=15, ha="right")
    ax_b.set_xlabel("Configuration", fontsize=11, fontweight="bold")
    ax_b.set_ylabel("Attempt Count", fontsize=11, fontweight="bold")
    ax_b.set_title("(B) Attempt Outcome Ratio", fontsize=13, fontweight="bold")
    ax_b.legend(fontsize=9, loc="upper right", framealpha=0.9)
    ax_b.grid(axis="y", alpha=0.3, color=COLORS["grid"])
    ax_b.set_ylim(0, max(v + f for v, f in zip(valid_counts, failed_counts)) + 2)

    fig.subplots_adjust(left=0.08, right=0.97, top=0.93, bottom=0.12, wspace=0.35)
    _save_fig(fig, "fig2_temporal_performance", fmt, dpi)


# ---------------------------------------------------------------------------
# Figure 3: Trial Execution Reliability & Resilience Profile
# ---------------------------------------------------------------------------

def plot_reliability_profile(data: dict, fmt: str, dpi: int):
    """Horizontal 100% stacked bar chart showing proportional reliability distribution.

    Three tiers:
      - Successful (No Errors): numeric Frames AND empty Failure string
      - Successful with Error: numeric Frames AND non-empty Failure string
      - Failed: null Frames
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import pandas as pd

    trial_data = data["data"]

    # Categorise each attempt, grouping by participant + view type
    records = []
    for trial in trial_data:
        trial_name = trial["Trial"]
        # Extract participant and view type from trial name
        parts = trial_name.split()
        participant = f"Participant {parts[1]}"
        view = parts[2]  # "Single" or "Dual"
        group_label = f"{participant} {view} View"

        for i, (frame_val, fail_str) in enumerate(zip(trial["Frames"], trial["Failure"])):
            if frame_val is not None and fail_str == "":
                tier = "Successful (No Errors)"
            elif frame_val is not None and fail_str != "":
                tier = "Successful with Error"
            else:
                tier = "Failed"
            records.append({
                "Group": group_label,
                "Tier": tier,
            })

    if not records:
        print("  SKIP fig3: No trial data")
        return

    df = pd.DataFrame(records)

    # Define group order: P1 Single, P1 Dual, P2 Single
    group_order = [
        "Participant 1 Dual View",
        "Participant 1 Single View",
        "Participant 2 Single View",
    ]
    df["Group"] = pd.Categorical(df["Group"], categories=group_order, ordered=True)

    # Tier ordering and colours
    tier_order = ["Successful (No Errors)", "Successful with Error", "Failed"]
    tier_colors = {
        "Successful (No Errors)": COLORS["positive"],
        "Successful with Error": COLORS["warning"],
        "Failed": COLORS["negative"],
    }

    # Compute counts per group
    counts = df.groupby(["Group", "Tier"], observed=False).size().unstack(fill_value=0)
    # Ensure all tiers present
    for tier in tier_order:
        if tier not in counts.columns:
            counts[tier] = 0
    counts = counts[tier_order]  # reorder columns
    counts = counts.reindex(group_order)

    # Convert to percentages
    totals = counts.sum(axis=1)
    pct = counts.div(totals, axis=0) * 100

    fig, ax = plt.subplots(figsize=(12, max(4, len(group_order) * 1.0)))

    # Horizontal stacked bar
    left = np.zeros(len(group_order))
    for tier in tier_order:
        bars = ax.barh(
            range(len(group_order)),
            pct[tier].values,
            left=left,
            color=tier_colors[tier],
            edgecolor="white",
            linewidth=0.8,
            alpha=0.85,
            height=0.6,
        )
        # Annotate percentage inside bars where there's enough room
        for j, (val, l) in enumerate(zip(pct[tier].values, left)):
            if val >= 8:  # only annotate if segment is wide enough
                ax.text(
                    l + val / 2, j, f"{val:.0f}%",
                    ha="center", va="center",
                    fontsize=9, fontweight="bold", color="white",
                )
        left += pct[tier].values

    # Y-axis labels
    ax.set_yticks(range(len(group_order)))
    ax.set_yticklabels(group_order, fontsize=10)

    # Count annotations on the right
    for j, (total, group_name) in enumerate(zip(totals.values, group_order)):
        ax.text(101, j, f"n={total}", va="center", ha="left",
                fontsize=8, color=COLORS["text"])

    ax.set_xlabel("Cumulative Percentage (%)", fontsize=11, fontweight="bold")
    ax.set_xlim(0, 100)
    ax.set_title("Trial Execution Reliability & Resilience Profile",
                 fontsize=14, fontweight="bold")
    ax.grid(axis="x", alpha=0.3, color=COLORS["grid"])

    # Legend
    legend_patches = [
        mpatches.Patch(color=tier_colors[t], label=t, alpha=0.85)
        for t in tier_order
    ]
    ax.legend(
        handles=legend_patches,
        loc="upper right",
        fontsize=9,
        framealpha=0.9,
        edgecolor=COLORS["grid"],
        title="Outcome Tier",
        title_fontsize=10,
    )

    # Invert so first trial is at top
    ax.invert_yaxis()

    fig.tight_layout()
    _save_fig(fig, "fig3_reliability_profile", fmt, dpi)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Generate Test 3 figures")
    parser.add_argument(
        "--format", default="png", choices=["pdf", "png"],
        help="Output format (default: png)",
    )
    parser.add_argument(
        "--dpi", type=int, default=300,
        help="Resolution for raster formats (default: 300)",
    )
    args = parser.parse_args()

    os.makedirs(FIGURES_DIR, exist_ok=True)

    print("Loading trial data...")
    data = _load_data()
    print(f"  FPS: {data['FPS']}")
    print(f"  Failure modes: {len(data['Failures'])}")
    print(f"  Reference entries: {len(data['reference'])}")
    print(f"  Trial configurations: {len(data['data'])}")

    print("\nGenerating figures...")
    plot_failure_mode_catplot(data, args.format, args.dpi)
    plot_temporal_performance(data, args.format, args.dpi)
    plot_reliability_profile(data, args.format, args.dpi)

    print(f"\nDone. Figures in: {FIGURES_DIR}")


if __name__ == "__main__":
    main()
