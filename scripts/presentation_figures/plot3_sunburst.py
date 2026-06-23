"""Plot 3 — Error Type Sunburst.

Two-ring sunburst chart of functional user-trial failure types.

Inner ring shows two aggregated categories:
    EMG       -> t (Trigger), d (Deactivation)
    Tracking  -> l (Localisation), v (Vision), i (Intent), p (Proximity)

Segmentation (s) appears **only** in the outer ring with no parent segment in
the inner ring, leaving a visual gap in the inner ring at that angular
position.  This is achieved by giving the inner "Segmentation" node the
background colour (white) so it reads as empty space, while the outer "s"
wedge still renders with its own colour.

Data sourced from::

    tests/test1_funtional_user_trial/data.json

The ``Failure`` entries are strings of concatenated single-character codes;
each character is counted as one occurrence of that error type.
"""
import json
import os

import plotly.graph_objects as go

from .palette import (
    TEAL,
    CYAN,
    SAGE,
    GRAY,
    BLUE_GRAY,
    BLUE,
    WHITE,
    BLACK,
)


def _opaque(hex_color):
    """Convert a #rrggbb hex color to a fully-opaque rgba(r,g,b,1) string.

    Plotly's sunburst marker applies partial opacity to hex colors by
    default; forcing explicit alpha=1 keeps the wedges solid while still
    allowing a transparent background.
    """
    h = hex_color.lstrip("#")
    r = int(h[0:2], 16)
    g = int(h[2:4], 16)
    b = int(h[4:6], 16)
    return f"rgba({r},{g},{b},1)"


def _force_opaque_wedges(png_path):
    """Post-process a PNG so non-background pixels are fully opaque.

    kaleido 0.2.1 renders Plotly sunburst wedges at a fixed partial alpha
    (~0.7) regardless of the trace ``opacity`` setting.  This reads the
    exported PNG, sets every pixel with alpha > 0 to alpha=255, and writes
    it back — yielding solid wedge colours while preserving the transparent
    background.
    """
    import numpy as np
    from PIL import Image

    img = Image.open(png_path).convert("RGBA")
    arr = np.array(img)
    # Any pixel that is not fully transparent becomes fully opaque
    mask = arr[:, :, 3] > 0
    arr[mask, 3] = 255
    Image.fromarray(arr, mode="RGBA").save(png_path)

# --- Paths ---------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_PATH = os.path.join(
    SCRIPT_DIR, "..", "..", "tests", "test1_funtional_user_trial", "data.json"
)
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")

# --- Category definitions ------------------------------------------------
# Map each error code to its aggregated parent category.
# Segmentation ("s") is its own standalone outer node with a *hidden* inner
# parent (rendered as a gap).
EMG_CODES = ["t", "d"]
TRACKING_CODES = ["l", "v", "i", "p"]
SEGMENTATION_CODES = ["s"]

# Human-readable labels for each code.
CODE_LABELS = {
    "t": "Trigger",
    "l": "Localisation",
    "v": "Vision",
    "i": "Intent",
    "s": "Segmentation",
    "d": "Deactivation",
    "p": "Proximity",
}


def _load_error_counts():
    """Count occurrences of each error code across all trial failures."""
    with open(DATA_PATH) as f:
        data = json.load(f)

    counts = {c: 0 for c in "tlvisdp"}
    for trial in data.get("data", []):
        for failure_str in trial.get("Failure", []):
            for ch in failure_str:
                if ch in counts:
                    counts[ch] += 1
    return counts


def _build_sunburst_data(counts):
    """Build the labels / parents / values / colours arrays for Plotly.

    Structure (inside -> outside):

        (root, hidden)
          |-- EMG            [inner, BLUE]
          |     |-- Trigger        [outer]
          |     |-- Deactivation   [outer]
          |-- Tracking       [inner, TEAL]
          |     |-- Localisation  [outer]
          |     |-- Vision        [outer]
          |     |-- Intent        [outer]
          |     |-- Proximity     [outer]
          |-- (gap)          [inner, WHITE = no label]
                |-- Segmentation  [outer, SAGE]
    """
    labels = []
    parents = []
    values = []
    colors = []
    texts = []  # displayed text (label + percentage)

    total_all = sum(counts.values())

    # Inner-ring categories.  Each inner node's value is the sum of its
    # children so the angular span matches the outer wedges beneath it.
    emg_total = sum(counts[c] for c in EMG_CODES)
    tracking_total = sum(counts[c] for c in TRACKING_CODES)
    seg_total = sum(counts[c] for c in SEGMENTATION_CODES)

    def _pct(n):
        return f"{n / total_all * 100:.1f}%"

    # --- EMG (inner) ---
    labels.append("EMG")
    parents.append("")
    values.append(emg_total)
    colors.append(_opaque(BLUE))
    texts.append(f"EMG<br>{_pct(emg_total)}")

    # --- Tracking (inner) ---
    labels.append("Tracking")
    parents.append("")
    values.append(tracking_total)
    colors.append(_opaque(TEAL))
    texts.append(f"Tracking<br>{_pct(tracking_total)}")

    # --- Segmentation inner node (rendered as a GAP) ---
    # It still occupies angular space (value = seg_total) so the outer
    # "Segmentation" wedge can attach to it, but it is coloured white
    # and has NO visible label so it reads as empty space.
    #
    # The node MUST have a label so the outer "Segmentation" child can
    # reference it as its parent (an empty parent string would make the
    # child a root/inner node itself).  We use an internal sentinel label
    # that is never displayed.
    GAP_LABEL = "__gap__"
    labels.append(GAP_LABEL)
    parents.append("")
    values.append(seg_total)
    colors.append("rgba(0,0,0,0)")  # fully transparent = invisible gap
    texts.append("")

    # --- Outer nodes (full names only, no letter abbreviations) ---
    outer_specs = [
        # (code, parent_label, color)
        ("t", "EMG", _opaque(BLUE)),
        ("d", "EMG", _opaque(BLUE_GRAY)),
        ("l", "Tracking", _opaque(TEAL)),
        ("v", "Tracking", _opaque(CYAN)),
        ("i", "Tracking", _opaque(SAGE)),
        ("p", "Tracking", _opaque(GRAY)),
        ("s", GAP_LABEL, _opaque(SAGE)),
    ]

    for code, parent, color in outer_specs:
        n = counts[code]
        full = CODE_LABELS[code]
        labels.append(full)
        parents.append(parent)
        values.append(n)
        colors.append(color)
        texts.append(f"{full}<br>{_pct(n)}")

    return labels, parents, values, colors, texts


def plot_sunburst(fmt="png", width=900, height=900, scale=2):
    """Generate the sunburst figure and save it."""
    counts = _load_error_counts()
    labels, parents, values, colors, texts = _build_sunburst_data(counts)

    fig = go.Figure(
        go.Sunburst(
            labels=labels,
            parents=parents,
            values=values,
            marker=dict(colors=colors, line=dict(color=WHITE, width=2)),
            opacity=1,  # force fully-opaque wedges (transparent bg only)
            branchvalues="total",
            text=texts,
            textinfo="text",
            hovertemplate="%{customdata}<extra></extra>",
            customdata=texts,
            insidetextorientation="radial",
            sort=False,
        )
    )

    fig.update_layout(
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        margin=dict(l=10, r=10, t=10, b=10),
        font=dict(family="DejaVu Sans, Arial", size=22, color=BLACK),
        showlegend=False,
    )

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out_path = os.path.join(OUTPUT_DIR, f"plot3_sunburst.{fmt}")
    # kaleido is used for static image export
    fig.write_image(out_path, width=width, height=height, scale=scale)

    # kaleido 0.2.1 renders sunburst wedges at a fixed alpha (~0.7) even
    # with opacity=1 set on the trace.  Post-process the PNG so every
    # non-background pixel (alpha > 0) becomes fully opaque (alpha=255),
    # giving solid wedge colours while keeping the transparent background.
    _force_opaque_wedges(out_path)

    print(f"  [plot3] saved {out_path}")

    # Report counts for verification
    total = sum(counts.values())
    print(
        f"  [plot3] error counts: "
        f"t={counts['t']} l={counts['l']} v={counts['v']} "
        f"i={counts['i']} s={counts['s']} d={counts['d']} p={counts['p']} "
        f"(total={total})"
    )
