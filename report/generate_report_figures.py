#!/usr/bin/env python3
"""
Generate report figures (plots, tables, equations, code) as transparent SVG
(or PNG when PNG is smaller), one file per figure number.

Usage:
    python report/generate_report_figures.py            # generate all
    python report/generate_report_figures.py 9 12 20    # generate a subset

Outputs land in report/figures/<number>.{svg,png}.

Content is sourced from the project's report LaTeX (report/*.txt) and verified
against the real implementation under src/grasp_preshaping/.
"""

import argparse
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, Rectangle
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3d projection)

try:
    import cairosvg
except ImportError:  # optional: only needed for PNG rasterisation
    cairosvg = None
from pygments import highlight
from pygments.formatters import SvgFormatter
from pygments.lexers import BashLexer, PythonLexer, RustLexer

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
ROOT = os.path.dirname(os.path.abspath(__file__))
FIG_DIR = os.path.join(ROOT, "figures")

# Keep SVG text as references to DejaVu (matplotlib default) for compact files.
plt.rcParams["svg.fonttype"] = "none"
plt.rcParams["font.family"] = "DejaVu Sans"
plt.rcParams["mathtext.fontset"] = "dejavusans"
plt.rcParams["axes.edgecolor"] = "#111827"
plt.rcParams["axes.labelcolor"] = "#111827"
plt.rcParams["xtick.color"] = "#111827"
plt.rcParams["ytick.color"] = "#111827"
plt.rcParams["text.color"] = "#111827"

# Cohesive black / white / red / blue palette (slideshow-friendly).
PAL = {
    "primary": "#2563eb",   # blue   — positive / goal / stability
    "accent": "#dc2626",    # red    — negative / risk / action force
    "blue": "#0284c7",      # sky    — secondary blue
    "dark": "#111827",      # near-black — text & outlines
    "amber": "#6b7280",     # grey   — process / transition / neutral
    "morton_z": "#DBEAFE",  # light blue tint
    "morton_y": "#F3F4F6",  # light grey tint
    "morton_x": "#FEE2E2",  # light red tint
    "morton_m": "#E5E7EB",  # light neutral tint
    "header": "#111827",    # near-black header
    "row_alt": "#F9FAFB",   # near-white alt row
    "highlight": "#E0E7FF", # light indigo highlight
}


# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------
def save_fig(fig, number, hint=""):
    """Save a matplotlib figure as SVG and PNG (transparent), keep the smaller."""
    os.makedirs(FIG_DIR, exist_ok=True)
    svg_path = os.path.join(FIG_DIR, f"{number}.svg")
    png_path = os.path.join(FIG_DIR, f"{number}.png")
    fig.savefig(svg_path, format="svg", transparent=True,
                bbox_inches="tight", pad_inches=0.15)
    fig.savefig(png_path, format="png", transparent=True,
                bbox_inches="tight", pad_inches=0.15, dpi=160)
    plt.close(fig)
    return _keep_smaller(number, svg_path, png_path, hint)


def save_code(code, language, number, hint="", font_size=13, title=None):
    """Render a code string to SVG via Pygments (transparent), compare to PNG.

    Note: Pygments' SvgFormatter ignores the requested font_size and always
    renders at 14px with a 19px line step. We therefore parse the *actual*
    metrics out of the rendered SVG so the dimensions match reality.
    """
    os.makedirs(FIG_DIR, exist_ok=True)
    lexer = {"bash": BashLexer, "python": PythonLexer, "rust": RustLexer}[language]()
    formatter = SvgFormatter(
        style="default",
        nobackground=True,        # transparent background
        font_size=font_size,
        line_numbers=False,
        nowrap=False,
    )
    body = highlight(code, lexer, formatter)

    # Parse the ACTUAL rendering metrics from Pygments' output (it ignores
    # font_size and always uses 14px / 19px line step).
    fs_match = re.search(r'<g[^>]*font-size="(\d+)px"', body)
    real_fs = int(fs_match.group(1)) if fs_match else 14
    ys = [int(m) for m in re.findall(r'<text[^>]*\sy="(\d+)"', body)]
    if len(ys) >= 2:
        line_step = ys[1] - ys[0]
        first_baseline = ys[0]
    else:
        line_step = int(round(real_fs * 1.4))
        first_baseline = real_fs
    # Monospace char width ~0.6em; measure the longest line for true width.
    lines = code.split("\n")
    # Account for tab expansion (Pygments renders tabs as ~8 spaces).
    expanded = [ln.expandtabs(4) for ln in lines]
    max_chars = max((len(ln) for ln in expanded), default=1)
    char_w = real_fs * 0.62

    title_h = (real_fs + 16) if title else 0
    top_pad = 8
    bottom_pad = 10
    width = max(320, int(max_chars * char_w) + 28)
    height = int(title_h + top_pad + len(lines) * line_step + bottom_pad)
    body = _add_svg_dimensions(body, width, height)

    if title:
        title_svg = (
            f'<text x="14" y="{real_fs + 6}" '
            f'font-family="DejaVu Sans" font-size="{real_fs + 3}" '
            f'font-weight="bold" fill="#111827">{title}</text>'
        )
        body = _inject_svg_title(body, title_svg, title_h + top_pad)

    svg_path = os.path.join(FIG_DIR, f"{number}.svg")
    with open(svg_path, "w", encoding="utf-8") as fh:
        fh.write(body)

    png_path = os.path.join(FIG_DIR, f"{number}.png")
    if cairosvg is None:
        if os.path.exists(png_path):
            os.remove(png_path)
        print(f"  [{number}] {hint}: {number}.svg (cairosvg not installed)")
        return f"{number}.svg"
    try:
        cairosvg.svg2png(bytestring=body.encode("utf-8"), write_to=png_path,
                         background_color=None, scale=2.0)
    except Exception as exc:  # conversion failed -> keep SVG
        if os.path.exists(png_path):
            os.remove(png_path)
        print(f"  [{number}] {hint}: {number}.svg (png conv failed: {exc})")
        return f"{number}.svg"
    return _keep_smaller(number, svg_path, png_path, hint)


def _add_svg_dimensions(svg, width, height):
    """Inject width/height/viewBox into the Pygments root <svg> tag."""
    marker = '<svg xmlns="http://www.w3.org/2000/svg">'
    replacement = (f'<svg xmlns="http://www.w3.org/2000/svg" '
                   f'width="{width}" height="{height}" '
                   f'viewBox="0 0 {width} {height}">')
    return svg.replace(marker, replacement, 1)


def _keep_smaller(number, svg_path, png_path, hint):
    s_svg = os.path.getsize(svg_path)
    s_png = os.path.getsize(png_path)
    if s_png < s_svg:
        os.remove(svg_path)
        chosen = f"{number}.png"
    else:
        os.remove(png_path)
        chosen = f"{number}.svg"
    print(f"  [{number}] {hint}: {chosen} "
          f"(svg={s_svg // 1024}KB, png={s_png // 1024}KB)")
    return chosen


def _inject_svg_title(svg, title_svg, shift_y):
    """Shift the Pygments content down by shift_y and insert a title line.

    Pygments wraps its content in a <g> tag that carries font attributes, e.g.
    '<g font-family="monospace" font-size="14px">'. We must translate THAT
    group (not a plain '<g>'), so we match the first <g...> opening tag.
    """
    m = re.search(r'<g(?=[\s>])[^>]*>', svg)
    if not m:
        return svg  # unexpected structure; leave unchanged rather than corrupt
    open_tag = m.group(0)
    # Insert a translate transform, preserving any existing attributes.
    if "transform=" in open_tag:
        new_tag = open_tag.replace(
            "transform=", f'transform="translate(0 {shift_y})" ', 1)
        # Handle the case where transform had no quotes already is rare; the
        # Pygments tag has no transform, so this branch is just defensive.
    else:
        new_tag = open_tag[:-1] + f' transform="translate(0 {shift_y})">'
    svg = svg[:m.start()] + new_tag + svg[m.end():]
    # Insert the title text just before the (now translated) <g>.
    return svg[:m.start()] + title_svg + "\n" + svg[m.start():]


def draw_table(ax, headers, rows, cell_colors=None, header_color=None,
               fontsize=11, row_height=1.6, title=None, col_align=None):
    """Draw a styled matplotlib table."""
    header_color = header_color or PAL["header"]
    cell_colors = cell_colors or {}
    ax.axis("off")
    if title:
        ax.set_title(title, fontsize=fontsize + 3, fontweight="bold",
                     color=PAL["dark"], pad=14)
    table = ax.table(cellText=rows, colLabels=headers, loc="center",
                     cellLoc="center", colLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(fontsize)
    table.scale(1, row_height)
    n_cols = len(headers)
    # Header row.
    for j in range(n_cols):
        c = table[0, j]
        c.set_facecolor(header_color)
        c.set_edgecolor("white")
        c.set_linewidth(1.5)
        c.set_text_props(color="white", fontweight="bold")
    # Body rows.
    for i in range(len(rows)):
        for j in range(n_cols):
            cell = table[i + 1, j]
            base = PAL["row_alt"] if i % 2 else "white"
            cell.set_facecolor(cell_colors.get((i, j), base))
            cell.set_edgecolor("#d9e2e9")
            cell.set_linewidth(0.8)
            if col_align and j < len(col_align) and col_align[j] == "left":
                cell.get_text().set_ha("left")
                cell.PAD = 0.04
    return table


def new_fig(figsize=(7, 4)):
    fig = plt.figure(figsize=figsize)
    return fig


# ---------------------------------------------------------------------------
# 9 — Offset / forward-velocity parameter sweep (exits)
# ---------------------------------------------------------------------------
def fig_09():
    headers = ["Offset (m)", "Twist $l_x$ (m/s)", "Success Rate", "Avg. Score"]
    rows = [
        ["-0.15", "0.00", "222 / 240", "0.272"],
        ["-0.20", "0.00", "218 / 240", "0.175"],
        ["-0.25", "0.00", "219 / 240", "0.180"],
        ["-0.25", "0.05", "240 / 240", "0.413"],
        ["-0.25", "0.10", "240 / 240", "0.313"],
        ["-0.30", "0.10", "240 / 240", "0.377"],
        ["-0.35", "0.10", "240 / 240", "0.409"],
    ]
    # Highlight the chosen operating point (best avg score, full success).
    hl = {(3, j): PAL["highlight"] for j in range(4)}
    fig = new_fig((7.2, 4.0))
    ax = fig.add_subplot(111)
    draw_table(ax, headers, rows, cell_colors=hl, fontsize=11,
               title="Grasp prediction performance vs. hand offset & forward twist")
    fig.text(0.5, 0.03,
             "Static offset and 1-D hand-frame twist from the hit point. "
             "Highlighted row is the chosen operating point.",
             ha="center", fontsize=8.5, color="#555", style="italic")
    return save_fig(fig, 9, "offset/twist sweep table")


# ---------------------------------------------------------------------------
# 12 — Morton code bit-interleaving table
# ---------------------------------------------------------------------------
def fig_12():
    fig = new_fig((8.6, 4.2))
    ax = fig.add_subplot(111)
    ax.axis("off")
    ax.set_title("3D Morton code bit interleaving  (X=1, Y=2, Z=4)",
                 fontsize=13, fontweight="bold", color=PAL["dark"], pad=12)

    # Section 1: input coordinates (decimal -> binary).
    inputs = [
        ("Z input", "$4_{10} = \\mathbf{100}_2$"),
        ("Y input", "$2_{10} = \\mathbf{010}_2$"),
        ("X input", "$1_{10} = \\mathbf{001}_2$"),
    ]
    # Section 2: interleaving grid. Bits read MSB->LSB: Z1 Y1 X1 Z0 Y1 X0 ...
    # Z=100, Y=010, X=001 -> interleaved = 1 0 0 0 1 0 0 0 1 = 273
    z_bits = ["1", ".", ".", "0", ".", ".", "0", ".", "."]
    y_bits = [".", "0", ".", ".", "1", ".", ".", "0", "."]
    x_bits = [".", ".", "0", ".", ".", "0", ".", ".", "1"]
    m_bits = ["1", "0", "0", "0", "1", "0", "0", "0", "1"]

    col_colors = [PAL["morton_z"], PAL["morton_y"], PAL["morton_x"]]

    y0 = 0.90
    # Input rows.
    for i, (lab, val) in enumerate(inputs):
        y = y0 - i * 0.07
        ax.text(0.06, y, lab, fontsize=11, fontweight="bold", color=PAL["dark"])
        ax.text(0.30, y, val, fontsize=11, color=col_colors[i],
                bbox=dict(facecolor=col_colors[i], edgecolor="none", pad=3))

    # Interleaving header.
    yh = y0 - 3 * 0.07 - 0.02
    ax.text(0.06, yh, "Bit interleaving process",
            fontsize=11, fontweight="bold", color=PAL["dark"])

    grid_top = yh - 0.05
    row_labels = ["Z bits", "Y bits", "X bits", "Morton"]
    bit_rows = [z_bits, y_bits, x_bits, m_bits]
    row_colors = [PAL["morton_z"], PAL["morton_y"], PAL["morton_x"], PAL["morton_m"]]
    x0, dx = 0.30, 0.055
    for ri, (lab, bits, rc) in enumerate(zip(row_labels, bit_rows, row_colors)):
        y = grid_top - ri * 0.08
        ax.text(0.06, y, lab, fontsize=10.5, fontweight="bold", color=PAL["dark"])
        for ci, b in enumerate(bits):
            if b == ".":
                ax.text(x0 + ci * dx, y, "·", fontsize=11, color="#bbb", ha="center")
            else:
                ax.text(x0 + ci * dx, y, b, fontsize=12, fontweight="bold",
                        ha="center", va="center",
                        bbox=dict(facecolor=rc, edgecolor="#cfd8dc", pad=2.5))

    # Result.
    yr = grid_top - 4 * 0.08 - 0.01
    ax.text(0.06, yr, "Result", fontsize=11, fontweight="bold", color=PAL["dark"])
    ax.text(0.30, yr,
            "$100010001_2 = 273_{10}$",
            fontsize=12, fontweight="bold", color=PAL["primary"])
    ax.text(0.5, 0.04,
            "Bits of X/Y/Z are interleaved (Z Y X order) into one 1-D key that "
            "preserves spatial locality for cache-friendly TSDF traversal.",
            ha="center", fontsize=8.5, color="#555", style="italic")
    return save_fig(fig, 12, "morton interleaving table")


# ---------------------------------------------------------------------------
# 13 — Sorting Morton IDs (schematic)
# ---------------------------------------------------------------------------
def fig_13():
    fig, axes = plt.subplots(1, 3, figsize=(11.5, 3.8))
    fig.suptitle("Sorting points by Morton code -> run-length offset index",
                 fontsize=13, fontweight="bold", color=PAL["dark"], y=1.02)

    # Stage 1: unsorted points with voxel ids.
    ax = axes[0]
    ax.set_title("1. Voxelize (compute Morton id)", fontsize=10.5, color=PAL["primary"])
    pts = np.array([[0, 0], [2, 1], [1, 3], [3, 0], [0, 2], [1, 1], [3, 2], [2, 3]])
    ids = []
    for (x, y) in pts:
        # 2-bit morton for illustration.
        def split2(v):
            return (v & 1) | ((v & 2) << 1)
        ids.append(split2(x) | (split2(y) << 1))
    ax.scatter(pts[:, 0], pts[:, 1], s=260, c=PAL["blue"], alpha=0.85,
               edgecolor=PAL["dark"], zorder=3)
    for (x, y), m in zip(pts, ids):
        ax.text(x, y, str(m), ha="center", va="center", color="white",
                fontsize=10, fontweight="bold", zorder=4)
    ax.set_xlim(-0.6, 3.6); ax.set_ylim(-0.6, 3.6)
    ax.set_xticks(range(4)); ax.set_yticks(range(4))
    ax.set_aspect("equal"); ax.grid(True, color="#e0e6ea")

    # Stage 2: sort by id.
    ax = axes[1]
    ax.set_title("2. Sort by Morton id", fontsize=10.5, color=PAL["primary"])
    order = np.argsort(ids)
    sorted_ids = [ids[i] for i in order]
    for i, m in enumerate(sorted_ids):
        ax.add_patch(Rectangle((i, 0), 0.9, 1, facecolor=PAL["amber"],
                               edgecolor=PAL["dark"]))
        ax.text(i + 0.45, 0.5, str(m), ha="center", va="center",
                fontsize=11, fontweight="bold")
    ax.set_xlim(-0.2, len(sorted_ids) + 0.2); ax.set_ylim(-0.6, 1.6)
    ax.axis("off")
    ax.annotate("par_sort_by_key", (len(sorted_ids) / 2, -0.35),
                ha="center", fontsize=9, color=PAL["accent"], style="italic")

    # Stage 3: run-length offsets.
    ax = axes[2]
    ax.set_title("3. Build run offsets (voxel -> points)", fontsize=10.5,
                 color=PAL["primary"])
    unique_ids = []
    offsets = [0]
    for m in sorted_ids:
        if not unique_ids or unique_ids[-1] != m:
            unique_ids.append(m)
            offsets.append(offsets[-1])
        offsets[-1] += 1
    # draw grouped bars
    x = 0.0
    cmap = [PAL["morton_z"], PAL["morton_y"], PAL["morton_x"], PAL["morton_m"]]
    for ui, uid in enumerate(unique_ids):
        cnt = sorted_ids.count(uid)
        ax.add_patch(Rectangle((x, 0), cnt * 0.9, 1, facecolor=cmap[ui % len(cmap)],
                               edgecolor=PAL["dark"]))
        ax.text(x + cnt * 0.45, 0.5, f"id {uid}\n×{cnt}", ha="center",
                va="center", fontsize=9, fontweight="bold")
        ax.annotate(f"offset\n{offsets[ui]}", (x + cnt * 0.45, 1.15),
                    ha="center", fontsize=8, color=PAL["accent"])
        x += cnt * 0.9 + 0.3
    ax.set_xlim(-0.2, x + 0.2); ax.set_ylim(-0.6, 1.9)
    ax.axis("off")
    return save_fig(fig, 13, "morton sort schematic")


# ---------------------------------------------------------------------------
# 14 — TSDF benefits for grasping (text card)
# ---------------------------------------------------------------------------
def fig_14():
    fig = new_fig((7.5, 4.6))
    ax = fig.add_subplot(111)
    ax.axis("off")
    ax.set_title("Why a TSDF for grasp preshaping?", fontsize=14,
                 fontweight="bold", color=PAL["dark"], pad=10)

    benefits = [
        ("O(1) proximity lookup",
         "Distance to the surface is a direct voxel query, turning expensive "
         "collision checks into array reads along the finger-closure path."),
        ("Narrow-band truncation",
         "Only voxels within a few cells of the surface are stored, bounding "
         "memory and compute to exactly the band fingers interact with."),
        ("Free signed inside/outside",
         "The sign separates object interior from free space, so the optimiser "
         "knows whether a contact penetrated the surface."),
        ("Analytic surface normals",
         "The field gradient yields per-voxel normals used directly for the "
         "alignment and wrench-proxy grasp scores."),
        ("Backside completion",
         "Superquadric geometry fills the unobserved backside, giving a stable "
         "closure target even from a single viewpoint."),
    ]
    y = 0.88
    for i, (head, body) in enumerate(benefits):
        c = [PAL["primary"], PAL["accent"], PAL["blue"], PAL["amber"],
             PAL["primary"]][i]
        ax.add_patch(Rectangle((0.02, y - 0.035), 0.03, 0.05, facecolor=c,
                               edgecolor="none"))
        ax.text(0.08, y, head, fontsize=11.5, fontweight="bold", color=c)
        ax.text(0.08, y - 0.055, body, fontsize=9.2, color="#333", va="top",
                wrap=True)
        y -= 0.175
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    return save_fig(fig, 14, "tsdf benefits card")


# ---------------------------------------------------------------------------
# 15 — BFS wavefront: random seed cells, neighbours coloured differently
# ---------------------------------------------------------------------------
def fig_15():
    rng = np.random.default_rng(11)
    n = 14
    # Pick a few random seed cells and give each a distinct base colour.
    n_seeds = 4
    seeds = []
    while len(seeds) < n_seeds:
        c = (rng.integers(0, n), rng.integers(0, n))
        if c not in seeds:
            seeds.append(c)

    # Distinct hues for each seed's wavefront.
    seed_hues = [0.55, 0.02, 0.13, 0.33, 0.78, 0.45]
    from matplotlib.colors import hsv_to_rgb

    # Multi-source BFS: each cell inherits its nearest seed (by Manhattan BFS
    # distance); the ring index = distance from that seed. We colour by
    # (seed_hue, lightness=ring) so neighbours sit on a different shade than
    # the cell they surround, exactly like a BFS wavefront.
    dist = np.full((n, n), -1)
    seed_id = np.full((n, n), -1)
    from collections import deque
    dq = deque()
    for i, (x, y) in enumerate(seeds):
        dist[x, y] = 0
        seed_id[x, y] = i
        dq.append((x, y, i, 0))
    while dq:
        x, y, sid, d = dq.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < n and 0 <= ny < n and dist[nx, ny] == -1:
                dist[nx, ny] = d + 1
                seed_id[nx, ny] = sid
                dq.append((nx, ny, sid, d + 1))

    max_d = dist.max()
    # Build an RGBA grid.
    grid = np.ones((n, n, 4))
    for x in range(n):
        for y in range(n):
            sid = seed_id[x, y]
            d = dist[x, y]
            hue = seed_hues[sid % len(seed_hues)]
            # Lightness decreases with ring distance; neighbours (d differs by
            # 1) always get a visibly different shade.
            light = 0.92 - 0.10 * d
            rgb = hsv_to_rgb([hue, 0.55, max(0.25, light)])
            grid[x, y, :3] = rgb
            grid[x, y, 3] = 1.0

    fig, ax = plt.subplots(figsize=(6.6, 6.6))
    ax.imshow(grid, origin="lower", interpolation="nearest")
    # Seed cell outlines.
    for (x, y) in seeds:
        ax.add_patch(Rectangle((y - 0.5, x - 0.5), 1, 1, fill=False,
                               edgecolor=PAL["dark"], lw=2.5))
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_title("BFS wavefront expansion from random seed cells\n"
                 "each ring is a different shade of its seed's colour",
                 fontsize=12, fontweight="bold", color=PAL["dark"], pad=12)
    # Legend.
    from matplotlib.patches import Patch
    handles = [Patch(facecolor=hsv_to_rgb([seed_hues[i % len(seed_hues)],
                                          0.55, 0.92]),
                     edgecolor=PAL["dark"], label=f"seed {i + 1} (ring 0)")
               for i in range(n_seeds)]
    handles.append(Patch(facecolor="#dddddd", edgecolor="#999",
                         label="outer ring (max distance)"))
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.02, 1.0),
              fontsize=8.5, framealpha=0.9, title="wavefront origin")
    return save_fig(fig, 15, "BFS wavefront grid")


# ---------------------------------------------------------------------------
# 19 — Superquadric shapes & parameters
# ---------------------------------------------------------------------------
def _spow(base, exp):
    """Signed power: sign(b)*|b|^e, the superquadric power function."""
    return np.sign(base) * np.abs(base) ** exp


def fig_19():
    shapes = [
        ("Sphere", 1.0, 1.0, PAL["blue"]),
        ("Box", 0.1, 0.1, PAL["accent"]),
        ("Cylinder", 0.1, 1.0, PAL["primary"]),
    ]
    fig = plt.figure(figsize=(11, 4.2))
    fig.suptitle("Superquadric templates: a single inside-outside function "
                 "$F(x,y,z)$ covers all three primitives",
                 fontsize=12.5, fontweight="bold", color=PAL["dark"], y=1.0)
    for i, (name, e1, e2, color) in enumerate(shapes):
        ax = fig.add_subplot(1, 3, i + 1, projection="3d")
        a = b = c = 1.0
        eta = np.linspace(-np.pi / 2, np.pi / 2, 50)
        omega = np.linspace(-np.pi, np.pi, 50)
        eta, omega = np.meshgrid(eta, omega)
        ce = _spow(np.cos(eta), e1); se = _spow(np.sin(eta), e1)
        co = _spow(np.cos(omega), e2); so = _spow(np.sin(omega), e2)
        x = a * ce * co; y = b * ce * so; z = c * se
        ax.plot_surface(x, y, z, color=color, alpha=0.75, edgecolor="none",
                        shade=True)
        ax.set_title(f"{name}\n$\\epsilon_1={e1},\\ \\epsilon_2={e2}$",
                     fontsize=11, color=PAL["dark"])
        ax.set_box_aspect((1, 1, 1))
        lim = 1.3
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
        ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.pane.set_edgecolor((0, 0, 0, 0.1))
    fig.text(0.5, 0.01,
             "$a,b,c$ = half-extents along each axis.  "
             "Box/cylinder use $\\epsilon=0.1$ (not 0) for a non-singular gradient.",
             ha="center", fontsize=9, color="#555", style="italic")
    return save_fig(fig, 19, "superquadric shapes")


# ---------------------------------------------------------------------------
# 20 — Superquadric equation
# ---------------------------------------------------------------------------
def fig_20():
    fig = new_fig((7.5, 2.3))
    ax = fig.add_subplot(111)
    ax.axis("off")
    ax.set_title("Superquadric inside-outside function",
                 fontsize=13, fontweight="bold", color=PAL["dark"], pad=12)
    eq = (r"$F(x,y,z)=\left(\left(\left|\frac{x}{a}\right|^{2/\epsilon_1}"
          r"+\left|\frac{y}{b}\right|^{2/\epsilon_1}\right)^{\epsilon_1/\epsilon_2}"
          r"+\left|\frac{z}{c}\right|^{2/\epsilon_2}\right)^{\epsilon_2}-1$")
    ax.text(0.5, 0.62, eq, ha="center", va="center", fontsize=18,
            color=PAL["dark"])
    ax.text(0.5, 0.20,
            r"$F=0$ on surface,  $F<0$ inside,  $F>0$ outside.   "
            r"$(\epsilon_1,\epsilon_2)=(1,1)\!\to\!$sphere, "
            r"$(0.1,0.1)\!\to\!$box, $(0.1,1.0)\!\to\!$cylinder",
            ha="center", fontsize=10, color="#444")
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    return save_fig(fig, 20, "superquadric equation")


# ---------------------------------------------------------------------------
# 21 — Point cloud -> oriented bounding box via PCA
# ---------------------------------------------------------------------------
def fig_21():
    rng = np.random.default_rng(7)
    # An elongated, rotated box-like cluster.
    n = 600
    raw = np.column_stack([
        rng.uniform(-2.0, 2.0, n),
        rng.uniform(-0.3, 0.3, n),
        rng.uniform(-0.5, 0.5, n),
    ])
    R = np.array([[0.74, -0.66, 0.10],
                  [0.66, 0.75, -0.05],
                  [-0.07, 0.09, 0.99]])
    pts = (R @ raw.T).T

    # PCA.
    mean = pts.mean(axis=0)
    cov = np.cov((pts - mean).T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    eigvals = eigvals[order]
    eigvecs = eigvecs[:, order]

    # OBB extents in principal frame.
    proj = (pts - mean) @ eigvecs
    half = (proj.max(axis=0) - proj.min(axis=0)) / 2

    # OBB corners in world frame.
    signs = np.array([[sx, sy, sz] for sx in (-1, 1)
                      for sy in (-1, 1) for sz in (-1, 1)])
    corners = mean + (signs * half) @ eigvecs.T

    fig = plt.figure(figsize=(7.6, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=6, c=PAL["blue"],
               alpha=0.35, label="point cloud")
    # OBB edges.
    edges = [(0, 1), (0, 2), (1, 3), (2, 3), (4, 5), (4, 6), (5, 7), (6, 7),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    for a, b in edges:
        ax.plot(*zip(corners[a], corners[b]), color=PAL["accent"], lw=2)
    # PCA axes.
    colors = [PAL["primary"], PAL["amber"], PAL["accent"]]
    labels = ["PC1 (longest)", "PC2", "PC3"]
    for i in range(3):
        v = eigvecs[:, i] * (half[i] * 1.4)
        ax.quiver(*mean, *v, color=colors[i], lw=2.5, arrow_length_ratio=0.12)
    ax.scatter(*mean, color=PAL["dark"], s=30, zorder=5)
    ax.set_title("Oriented bounding box from PCA of the point cloud",
                 fontsize=12, fontweight="bold", color=PAL["dark"], pad=10)
    ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
    ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_edgecolor((0, 0, 0, 0.08))
    # Legend proxies.
    from matplotlib.lines import Line2D
    handles = [Line2D([0], [0], color=PAL["accent"], lw=2, label="OBB edges")]
    for c, lab in zip(colors, labels):
        handles.append(Line2D([0], [0], color=c, lw=2.5, label=lab))
    ax.legend(handles=handles, loc="upper left", fontsize=8.5, framealpha=0.9)
    return save_fig(fig, 21, "PCA oriented bounding box")


# ---------------------------------------------------------------------------
# 28 — Camera & backside AGREE on TSDF sign (blending table)
# ---------------------------------------------------------------------------
def fig_28():
    headers = ["Region (cells from surface)", "Camera sign",
               "What the pipeline does"]
    rows = [
        ["0 .. trunc−Δ  (near shell)", "agree (kept)",
         "Use the camera distance unchanged — camera is authoritative at the "
         "visible surface."],
        ["trunc−Δ .. trunc  (outer band)", "agree (blend)",
         "Smoothstep-interpolate from camera distance into the superquadric "
         "Taubin distance."],
        ["> trunc  (beyond band)", "agree (SQ only)",
         "Superquadric distance defines the field exclusively (camera has no "
         "data here)."],
    ]
    fig = new_fig((8.6, 3.6))
    ax = fig.add_subplot(111)
    draw_table(ax, headers, rows, fontsize=10, row_height=2.0,
               col_align=["center", "center", "left"],
               title="Congruent camera & superquadric signs  "
                     "(Δ = SQ_BLEND_DELTA_CELLS = 3, trunc = 4)")
    fig.text(0.5, 0.03,
             "blend_start = trunc − Δ,  blend_end = trunc.  "
             "Distances are blended only in the outer band for a continuous gradient.",
             ha="center", fontsize=8.5, color="#555", style="italic")
    return save_fig(fig, 28, "congruent-sign blending table")


# ---------------------------------------------------------------------------
# 32 — Python: load URDF with pinocchio + get a transform
# ---------------------------------------------------------------------------
def fig_32():
    code = '''import numpy as np
import pinocchio as pin

# Load the simplified Mia-hand URDF used for LUT generation.
# (6 actuated revolute joints: index/little/mrl/ring/thumb_opp/thumb_fle)
model = pin.buildModelFromUrdf("urdf/mia_hand_flat.urdf")
data = model.createData()

# Configuration vector q (nq=6), values in [0, 1]:
#   [index_fle, little_fle, mrl_fle, ring_fle, thumb_opp, thumb_fle]
q = pin.neutral(model)
q[0] = 0.5   # index half-closed
q[2] = 0.5   # mrl half-closed
q[4] = 1.0   # thumb abducted
q[5] = 0.5   # thumb half-closed

# Forward kinematics -> world transform of a named frame (index sensor/tip).
pin.forwardKinematics(model, data, q)
fid = model.getFrameId("mia_index_sensor")
pin.updateFramePlacement(model, data, fid)
T = data.oMf[fid]

print("translation:", T.translation)   # 3-vector
print("rotation:   ", T.rotation)      # 3x3 matrix
M = T.homogeneous                      # 4x4 SE(3) transform
'''
    return save_code(code, "python", 32,
                     "pinocchio URDF load + transform", font_size=12,
                     title="Loading the Mia-hand URDF with Pinocchio")


# ---------------------------------------------------------------------------
# 33 — Fingertip sweep across the LUT in 3D (real data)
# ---------------------------------------------------------------------------
def _dq_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def _dq_to_se3(dq):
    """Decode a dual quaternion [real_wxyz, dual_wxyz] -> 4x4 SE(3).

    Mirrors src/grasp_preshaping/src/lut_helper.rs DualQuaternion::to_se3().
    """
    qr = np.asarray(dq[:4], dtype=float)
    qd = np.asarray(dq[4:8], dtype=float)
    n = np.linalg.norm(qr)
    qr = qr / n if n > 0 else np.array([1.0, 0, 0, 0])
    # translation = 2 * qd * conj(qr), take the vector part.
    conj = np.array([qr[0], -qr[1], -qr[2], -qr[3]])
    t = _dq_mul(_dq_mul(qd, conj), np.array([1.0, 0, 0, 0]))[1:] * 2.0
    w, x, y, z = qr
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def _set_equal_aspect_3d(ax, points):
    """Force an equal aspect ratio on a 3D axes around the given points.

    matplotlib's 3D axes don't honour ``set_aspect('equal')``; this computes a
    cubic bounding box from the points and sets matching per-axis limits so the
    hand geometry isn't stretched along any axis.
    """
    pts = np.asarray(points)
    lo = pts.min(axis=0)
    hi = pts.max(axis=0)
    center = (lo + hi) / 2
    span = (hi - lo).max() / 2 + 1e-6
    ax.set_xlim(center[0] - span, center[0] + span)
    ax.set_ylim(center[1] - span, center[1] + span)
    ax.set_zlim(center[2] - span, center[2] + span)


def fig_33():
    lut_path = os.path.join(ROOT, "..", "src", "grasp_preshaping", "data",
                            "finger_contact_lut.npz")
    data = np.load(lut_path)
    n_steps = data["index_table"].shape[0]

    # Joint chains (indices into each table): base -> ... -> tip.
    index_chain = [0, 4, 2, 6]              # IndexMcp -> Pip -> Dip -> Tip
    thumb_chain = [0, 1, 2]                 # ThumbAbd(Mcp) -> Pip -> Tip
    mrl_chains = [                          # Middle/Ring/Little: Mcp -> Pip -> Dip -> Tip
        [0, 2, 1, 3],                       # Middle
        [6, 5, 4],                          # Ring (no Mcp stored)
        [8, 7, 6],                          # Little (no Mcp stored)
    ]

    def chain_at(table_key, chain, step):
        table = data[table_key]
        return np.array([_dq_to_se3(table[step, ci, :])[:3, 3] for ci in chain])

    def sweep_at(table_key, cidx):
        table = data[table_key]
        return np.array([_dq_to_se3(table[s, cidx, :])[:3, 3]
                         for s in range(n_steps)])

    fig = plt.figure(figsize=(8.6, 7.0))
    ax = fig.add_subplot(111, projection="3d")

    # --- Palm base (faint) at the open step, to ground the hand. -----------
    palm = data["palm_table"]               # (4, 8): ProxUlna, ProxRadi, DistUlna, DistRadi
    palm_pts = np.array([_dq_to_se3(palm[i, :])[:3, 3] for i in range(4)])
    # Order into a sensible quad: prox-ulna, dist-ulna, dist-radi, prox-radi.
    quad = palm_pts[[0, 2, 3, 1]]
    from matplotlib.patches import Polygon
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    poly = Poly3DCollection([quad], alpha=0.10, facecolor=PAL["amber"],
                            edgecolor=PAL["amber"], linewidth=0.8)
    ax.add_collection3d(poly)

    # --- Other fingers (middle/ring/little), faint, for full-hand context.-
    for ci in mrl_chains:
        for s, alpha in zip([0, n_steps - 1], [0.18, 0.30]):
            chain = chain_at("mrl_table", ci, s)
            ax.plot(chain[:, 0], chain[:, 1], chain[:, 2], color=PAL["amber"],
                    lw=2.0, alpha=alpha)

    # --- Index finger: skeleton chain at 3 closure steps ------------------
    for s, alpha in zip([0, n_steps // 2, n_steps - 1], [0.35, 0.6, 0.95]):
        chain = chain_at("index_table", index_chain, s)
        ax.plot(chain[:, 0], chain[:, 1], chain[:, 2], color=PAL["primary"],
                lw=3.2, alpha=alpha)
        ax.scatter(chain[:, 0], chain[:, 1], chain[:, 2], color=PAL["primary"],
                   s=34, alpha=alpha, edgecolors="white", linewidths=0.5,
                   zorder=5)
    # Index tip sweep path (faded) + direction arrow.
    idx_sweep = sweep_at("index_table", 6)
    ax.plot(idx_sweep[:, 0], idx_sweep[:, 1], idx_sweep[:, 2],
            color=PAL["primary"], lw=1.4, alpha=0.35, linestyle="--")
    ax.quiver(*idx_sweep[0], *(idx_sweep[-1] - idx_sweep[0]),
              color=PAL["primary"], lw=1.6, arrow_length_ratio=0.14, alpha=0.7)

    # --- Thumb: skeleton chain at 3 closure steps -------------------------
    for s, alpha in zip([0, n_steps // 2, n_steps - 1], [0.35, 0.6, 0.95]):
        chain = chain_at("thumb_opp_mode1_table", thumb_chain, s)
        ax.plot(chain[:, 0], chain[:, 1], chain[:, 2], color=PAL["accent"],
                lw=3.2, alpha=alpha)
        ax.scatter(chain[:, 0], chain[:, 1], chain[:, 2], color=PAL["accent"],
                   s=34, alpha=alpha, edgecolors="white", linewidths=0.5,
                   zorder=5)
    # Thumb tip sweep path (faded) + direction arrow.
    th_sweep = sweep_at("thumb_opp_mode1_table", 2)
    ax.plot(th_sweep[:, 0], th_sweep[:, 1], th_sweep[:, 2],
            color=PAL["accent"], lw=1.4, alpha=0.35, linestyle="--")
    ax.quiver(*th_sweep[0], *(th_sweep[-1] - th_sweep[0]),
              color=PAL["accent"], lw=1.6, arrow_length_ratio=0.14, alpha=0.7)

    # --- Viewing angle chosen to separate thumb (opposition) from index ---
    # The thumb moves mostly in the x-z plane (opposition sweep), while the
    # index curls in y-z. An elevation of ~22° with azimuth ~-65° opens up
    # the angle between the two fingers so both chains read clearly.
    ax.view_init(elev=22, azim=-65)

    ax.set_title("Fingertip sweep across the precomputed LUT\n"
                 "(index & thumb joint chains over 11 closure steps)",
                 fontsize=12, fontweight="bold", color=PAL["dark"], pad=10)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
    ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_edgecolor((0, 0, 0, 0.08))

    # Equal aspect so the hand isn't stretched.
    all_pts = np.vstack([
        sweep_at("index_table", 6), sweep_at("thumb_opp_mode1_table", 2),
        chain_at("index_table", index_chain, 0), palm_pts,
    ])
    _set_equal_aspect_3d(ax, all_pts)

    # Custom legend (proxy artists).
    from matplotlib.lines import Line2D
    legend_handles = [
        Line2D([0], [0], color=PAL["primary"], lw=3, label="Index chain"),
        Line2D([0], [0], color=PAL["accent"], lw=3, label="Thumb chain"),
        Line2D([0], [0], color=PAL["primary"], lw=1.4, ls="--",
               alpha=0.6, label="Tip sweep path"),
        Line2D([0], [0], color=PAL["amber"], lw=2, alpha=0.4,
               label="Other fingers / palm"),
    ]
    ax.legend(handles=legend_handles, loc="upper left", fontsize=8.5,
              framealpha=0.9)
    fig.text(0.5, 0.01,
             "Solid links = joint chains at open / mid / closed.  "
             "Dashed = tip sweep path.  Arrows = open→closed direction.",
             ha="center", fontsize=8.5, color="#555", style="italic")
    return save_fig(fig, 33, "fingertip LUT sweep 3D")


# ---------------------------------------------------------------------------
# 35 — Normal & force vectors stored for fingertip contacts (3D)
# ---------------------------------------------------------------------------
def fig_35():
    fig = plt.figure(figsize=(7.8, 6))
    ax = fig.add_subplot(111, projection="3d")

    # Two opposing fingertip contacts (thumb tip vs index tip) to show the
    # stored surface normal n and closure force direction f.
    contacts = [
        {"pos": np.array([-0.6, 0.0, 0.0]), "normal": np.array([-1.0, 0.0, 0.0]),
         "force": np.array([1.0, 0.0, 0.0]), "color": PAL["primary"],
         "label": "thumb tip"},
        {"pos": np.array([0.6, 0.0, 0.0]), "normal": np.array([1.0, 0.0, 0.0]),
         "force": np.array([-1.0, 0.0, 0.0]), "color": PAL["accent"],
         "label": "index tip"},
    ]
    # Object being grasped.
    u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
    cx = 0.25 * np.cos(u) * np.sin(v)
    cy = 0.25 * np.sin(u) * np.sin(v)
    cz = 0.4 * np.cos(v)
    ax.plot_surface(cx, cy, cz, color=PAL["amber"], alpha=0.25,
                    edgecolor="none")

    for c in contacts:
        p = c["pos"]
        # Fingertip sphere.
        ax.scatter(*p, color=c["color"], s=220, edgecolor=PAL["dark"], zorder=5)
        # Surface normal (outward, from TSDF gradient).
        ax.quiver(*p, *c["normal"], color=PAL["blue"], lw=2.6,
                  arrow_length_ratio=0.18, length=0.55)
        # Force direction (closure sweep p_lo -> p_hi).
        ax.quiver(*p, *c["force"], color=PAL["accent"], lw=2.6,
                  arrow_length_ratio=0.18, length=0.55)
        ax.text(p[0], p[1], p[2] + 0.18, c["label"], fontsize=9,
                color=c["color"], fontweight="bold")

    # Object centre marker.
    ax.scatter([0], [0], [0], color=PAL["dark"], s=20)
    ax.set_title("Per-contact vectors stored for scoring:\n"
                 r"surface normal $\mathbf{n}$ (blue)  &  "
                 r"force direction $\mathbf{f}$ (orange)",
                 fontsize=11.5, fontweight="bold", color=PAL["dark"], pad=12)
    ax.set_xlim(-1.2, 1.2); ax.set_ylim(-1.0, 1.0); ax.set_zlim(-0.8, 0.8)
    ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
    ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_edgecolor((0, 0, 0, 0.08))
    from matplotlib.lines import Line2D
    ax.legend(handles=[
        Line2D([0], [0], color=PAL["blue"], lw=2.6,
               label=r"normal $\mathbf{n}$  (TSDF gradient)"),
        Line2D([0], [0], color=PAL["accent"], lw=2.6,
               label=r"force $\mathbf{f}$  (closure dir)"),
    ], loc="lower left", fontsize=8.5, framealpha=0.9)
    return save_fig(fig, 35, "fingertip normal & force vectors")


# ---------------------------------------------------------------------------
# 36 — Rust: load the LUT npz and use it
# ---------------------------------------------------------------------------
def fig_36():
    code = '''use grasp_preshaping::lut_helper::{Contact, FingerLUT};

// Load the precomputed finger-closure LUT (.npz of dual quaternions).
// Each contact stores an SE(3) transform per closure step, so runtime
// collision detection is just array lookups against the TSDF.
let lut = FingerLUT::load("data/finger_contact_lut.npz");

// --- How the grasp type is chosen -------------------------------------
// Every search particle carries a grasp_type in {0,1,2}:
//   0 = Cylindrical, 1 = Pinch, 2 = Lateral.
// Iteration 0 assigns it uniformly at random; later iterations either
// inherit the parent elite's type (90% of the time) or mutate (10%)
// to a score-weighted distribution. After the last iteration the single
// highest-scoring particle wins, and its grasp_type is the chosen grasp.
//
// The chosen type selects a different contact set, thumb mode, and a
// per-type maximum closure (the LUT stores the self-collision limit):
let max_closure = lut.get_max_closure(chosen_grasp_type); // 0, 1, or 2

// Control value in [0, max_closure] along the closure path.
let control = 0.6 * max_closure;

// World transform of the index fingertip at this closure step.
let se3 = lut.get_se_transform(Contact::IndexTip, control);
let location = lut.get_location(Contact::IndexTip, control);

println!("IndexTip @ {:.2}: pos = [{:.3}, {:.3}, {:.3}]",
         control, location.x, location.y, location.z);
'''
    return save_code(code, "rust", 36, "rust LUT load + grasp type", font_size=12,
                     title="Loading the LUT and choosing the grasp type (Rust)")


# ---------------------------------------------------------------------------
# 39 — 1M samples from a 2D Gaussian
# ---------------------------------------------------------------------------
def fig_39():
    rng = np.random.default_rng(42)
    n = 1_000_000
    mean = [0.3, -0.2]
    cov = [[1.0, 0.6], [0.6, 1.4]]
    samples = rng.multivariate_normal(mean, cov, size=n)

    fig, ax = plt.subplots(figsize=(6.4, 5.6))
    hb = ax.hexbin(samples[:, 0], samples[:, 1], gridsize=90, bins="log",
                   cmap="viridis", mincnt=1)
    cb = fig.colorbar(hb, ax=ax, label="sample count (log)")
    cb.outline.set_visible(False)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(f"{n:,} samples from a 2-D Gaussian\n"
                 r"$\mu=(0.3,\,-0.2),\ \Sigma_{xx}=1.0,\ \Sigma_{yy}=1.4,\ "
                 r"\Sigma_{xy}=0.6$",
                 fontsize=12, fontweight="bold", color=PAL["dark"])
    ax.set_aspect("equal")
    ax.grid(True, color="#e0e6ea", linewidth=0.5)
    return save_fig(fig, 39, "1M 2D gaussian samples")


# ---------------------------------------------------------------------------
# 40 — 3 iterations of ES convergence (20k samples)
# ---------------------------------------------------------------------------
def fig_40():
    rng = np.random.default_rng(3)
    target = np.array([1.8, -1.2])          # random target point
    n = 20_000
    sigma0 = 1.6
    decay = 0.7
    elite_ratio = 0.05

    # Iteration 0: broad population from the motion-model prior.
    samples = rng.normal([0, 0], sigma0, size=(n, 2))

    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    states = [samples.copy()]
    cur = samples
    for it in range(2):
        # Random per-sample probability weight (score-weighted selection).
        weights = rng.uniform(0.0, 1.0, size=n)
        # Score = closeness to target * weight (multi-modal hypothesis).
        dist = np.linalg.norm(cur - target, axis=1)
        score = (1.0 / (1.0 + dist)) * weights
        # Truncation selection: top elite_ratio.
        k = max(1, int(n * elite_ratio))
        elite_idx = np.argpartition(score, -k)[-k:]
        elites = cur[elite_idx]
        # Score-weighted parent selection.
        p = score[elite_idx]
        p = p / p.sum()
        parents = elites[rng.choice(len(elites), size=n, p=p)]
        sigma = sigma0 * (decay ** (it + 1))
        cur = parents + rng.normal(0, sigma, size=(n, 2))
        states.append(cur.copy())

    titles = ["Iteration 0  (broad prior, $\\sigma_0$)",
              "Iteration 1  ($\\sigma=0.7\\sigma_0$)",
              "Iteration 2  ($\\sigma=0.7^2\\sigma_0$)"]
    for ax, st, t in zip(axes, states, titles):
        ax.hexbin(st[:, 0], st[:, 1], gridsize=55, cmap="magma",
                  mincnt=1, alpha=0.9)
        ax.scatter(*target, marker="*", s=220, c=PAL["amber"],
                   edgecolors="white", linewidths=1.5, zorder=5,
                   label="random target")
        ax.set_title(t, fontsize=10.5, color=PAL["primary"], fontweight="bold")
        ax.set_xlim(-5, 5); ax.set_ylim(-5, 5)
        ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
    axes[0].legend(loc="upper left", fontsize=8, framealpha=0.9)
    fig.suptitle("Evolution strategy: 20k particles, top-5% elites, "
                 "decaying proposal variance",
                 fontsize=12.5, fontweight="bold", color=PAL["dark"], y=1.02)
    fig.text(0.5, -0.02,
             "Score-weighted parent selection + random per-sample weights focus "
             "the population around the target over 3 iterations.",
             ha="center", fontsize=9, color="#555", style="italic")
    return save_fig(fig, 40, "ES convergence 3 iterations")


# ---------------------------------------------------------------------------
# 43 — Code: alignment & wrench proxy
# ---------------------------------------------------------------------------
def fig_43():
    code = '''// Grasp scoring: geometric alignment and wrench (force-closure) proxy.
// Source: src/grasp_preshaping/src/planner.rs

// Alignment: how well each finger's closure force opposes the surface normal.
// reward = clamp( mean( max(0, -n . f) ), 0, 1 )
fn compute_alignment(contacts: &[ActiveContact]) -> f64 {
    let vals: Vec<f64> = contacts.iter()
        .filter(|c| c.force_direction.norm() >= 0.5)
        .map(|c| (-dot(&c.surface_normal, &c.force_direction)).max(0.0))
        .collect();
    vals.is_empty() ? 0.0 : clamp(mean(vals), 0.0, 1.0)
}

// Wrench proxy: spatial balance of contact normals (no mass estimate needed).
// force_closure = clamp( 1 - ||mean(n)||, 0, 1 )
fn compute_force_closure(contacts: &[ActiveContact]) -> f64 {
    let mean_n = contacts.iter()
        .map(|c| c.surface_normal)
        .sum::<Vec3>() / contacts.len() as f64;
    clamp(1.0 - mean_n.norm(), 0.0, 1.0)
}
'''
    return save_code(code, "rust", 43, "alignment + wrench proxy", font_size=12,
                     title="Alignment and wrench-proxy scoring (Rust)")


# ---------------------------------------------------------------------------
# 44 — Contact tiers table (verified against planner.rs)
# ---------------------------------------------------------------------------
def fig_44():
    headers = ["Tier", "Condition", "Score range"]
    rows = [
        ["Tier 4", "No contact (finger sweep clear)", "[0.00, 0.05]"],
        ["Tier 3", "Invalid start (palm inside object)", "[0.01, 0.09]"],
        ["Tier 2", "Partial (too few finger groups)", "[0.00, 0.50]"],
        ["Tier 1", "Valid (thumb + index + extras)", "[0.80, 1.00]"],
    ]
    # Tier 0 (reject) = red tint, Tier 1 (best) = blue tint, greys in between.
    tier_colors = {0: "#FEE2E2", 1: "#DBEAFE", 2: "#F3F4F6", 3: "#E5E7EB"}
    cell_colors = {(i, 0): tier_colors[i] for i in range(4)}
    fig = new_fig((9.6, 3.6))
    ax = fig.add_subplot(111)
    draw_table(ax, headers, rows, cell_colors=cell_colors, fontsize=11,
               row_height=1.9, col_align=["center", "left", "center"],
               title="Tiered grasp scoring hierarchy  (verified vs. planner.rs)")
    fig.text(0.5, 0.03,
             "Tier 1: 0.8 + 0.2·contact_fraction.   "
             "Tier 2: 0.25·contact_fraction + 0.25·proximity_penalty.",
             ha="center", fontsize=8.5, color="#555", style="italic")
    return save_fig(fig, 44, "contact tiers table")


# ---------------------------------------------------------------------------
# 45 — Combined score equation
# ---------------------------------------------------------------------------
def fig_45():
    fig = new_fig((7.6, 2.5))
    ax = fig.add_subplot(111)
    ax.axis("off")
    ax.set_title("Combined grasp score", fontsize=13, fontweight="bold",
                 color=PAL["dark"], pad=12)
    eq = (r"$S=\dfrac{w_p\,P \;+\; w_a\,A \;+\; w_f\,F \;+\; w_c\,C}"
          r"{w_p+w_a+w_f+w_c}$")
    ax.text(0.5, 0.60, eq, ha="center", va="center", fontsize=22,
            color=PAL["dark"])
    ax.text(0.5, 0.18,
            r"$P$=sample prob., $A$=alignment, $F$=force-closure proxy, "
            r"$C$=tiered contact score"
            "\n" + r"weights: $w_c=3.0$ (dominant),  $w_p=w_a=w_f=1.0$",
            ha="center", fontsize=10, color="#444")
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    return save_fig(fig, 45, "combined score equation")


# ---------------------------------------------------------------------------
# 47 — Timing breakdown table
# ---------------------------------------------------------------------------
def fig_47():
    headers = ["Pipeline phase", "Mean time (ms)"]
    rows = [
        ["ROI filtering & Morton sort", "8.02"],
        ["Superquadric backside estimation", "4.37"],
        ["TSDF volumetric construction", "2.99"],
        ["Single iteration scoring", "5.82"],
        ["Evolutionary optimization loop", "53.34"],
        ["Total pipeline latency", "70.62"],
    ]
    cell_colors = {(5, 0): PAL["highlight"], (5, 1): PAL["highlight"]}
    fig = new_fig((7.2, 4.0))
    ax = fig.add_subplot(111)
    draw_table(ax, headers, rows, cell_colors=cell_colors, fontsize=11,
               col_align=["left", "center"],
               title="Grasp-evaluation latency  (20k particles, 5 iters, "
                     "10k pts, i5-13600k)")
    fig.text(0.5, 0.03,
             "Stages measured independently; total is a separate end-to-end "
             "measurement (not a sum).",
             ha="center", fontsize=8.5, color="#555", style="italic")
    return save_fig(fig, 47, "timing breakdown table")


# ---------------------------------------------------------------------------
# 53 — Run commands
# ---------------------------------------------------------------------------
def fig_53():
    code = """# ── On the Jetson (robotlab) ──────────────────────────────────────────
make run                  # build + launch the full pipeline on the Jetson
                          # (EMG + wrist + hand + perception), hardware enabled

# ── On the host (developer laptop) ───────────────────────────────────
make up                   # start the prosthesis + mobile_sam containers (detached)
make shell                # open an interactive shell inside the container
make run                  # same pipeline launch, but from the host checkout
                          # (detects USB ports, sets up the hand inside the container)
"""
    return save_code(code, "bash", 53, "run commands", font_size=12,
                     title="Running the system")


# ---------------------------------------------------------------------------
# 55 — Debug commands
# ---------------------------------------------------------------------------
def fig_55():
    code = """# ── Jetson: full pipeline with hardware + rich diagnostics ───────────
make run-log-debug        # run with pipeline_diagnostics node on (TF jumps,
                          # topic rates, pose divergence, clock offset), logs teed
                          # to logs/host-log-<ts>.txt

# ── Host: camera-only pipeline with diagnostics ─────────────────────
make run-camera-log-debug # camera pipeline (no hand/wrist) + diagnostics node,
                          # for isolating perception issues

make record-debug         # record a rosbag AND capture system telemetry
                          # (host CPU/GPU/RAM/drift/NIC + Jetson) side-by-side

# ── Analysis (after a run) ───────────────────────────────────────────
make analyze-bag          # summarize the latest bag + sysmon (plots)
make analyze-log          # summarize the latest host-log (plots)
"""
    return save_code(code, "bash", 55, "debug commands", font_size=12,
                     title="Debugging commands")


# ---------------------------------------------------------------------------
# 57 — UDP network tuning fix
# ---------------------------------------------------------------------------
def fig_57():
    code = """# make network-tune-all  ->  tunes BOTH host and Jetson.
# Fixes CycloneDDS UDP fragment drops from two RealSense PointCloud2 streams
# (each 5-8 MB, fragmented into ~120 RTPS messages of 64 KB).
#
# scripts/tune_network_host.sh writes a persistent /etc/sysctl.d/ config:

# Max socket receive buffer: 2 GB ceiling for setsockopt(SO_RCVBUF).
# Default ~208 KB drops a single 8 MB cloud.
net.core.rmem_max = 2147483647

# Max socket send buffer (symmetrical ceiling for published fused clouds).
net.core.wmem_max = 2147483647

# Kernel IP-fragment reassembly memory: 128 MB.
# Default ~4 MB cannot hold 150+ fragments from two concurrent clouds.
net.ipv4.ipfrag_high_thresh = 134217728

# Seconds to hold an incomplete fragment queue: 3 s.
# Default 30 s wastes memory on stale fragments; 3 s >> pointcloud cadence.
net.ipv4.ipfrag_time = 3

# Applied immediately via `sysctl -w` and persisted across reboots.
"""
    return save_code(code, "bash", 57, "udp network tune", font_size=12,
                     title="UDP buffer fix  (make network-tune-all)")


# ---------------------------------------------------------------------------
# Dispatch
# ---------------------------------------------------------------------------
FIGURES = {
    9: fig_09, 12: fig_12, 13: fig_13, 14: fig_14, 15: fig_15, 19: fig_19,
    20: fig_20, 21: fig_21, 28: fig_28, 32: fig_32, 33: fig_33, 35: fig_35,
    36: fig_36, 39: fig_39, 40: fig_40, 43: fig_43, 44: fig_44, 45: fig_45,
    47: fig_47, 53: fig_53, 55: fig_55, 57: fig_57,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("numbers", type=int, nargs="*",
                        help="figure numbers to generate (default: all)")
    args = parser.parse_args()
    targets = sorted(args.numbers) if args.numbers else sorted(FIGURES)
    print(f"Generating {len(targets)} figures into {FIG_DIR}/")
    for n in targets:
        if n not in FIGURES:
            print(f"  [{n}] UNKNOWN figure number, skipping")
            continue
        try:
            FIGURES[n]()
        except Exception as exc:
            print(f"  [{n}] FAILED: {exc}")
            raise
    print("Done.")


if __name__ == "__main__":
    main()
