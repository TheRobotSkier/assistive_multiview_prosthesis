"""Plot 5 — ArUco Pose Ambiguity.

Dual-panel figure illustrating that a planar marker (e.g. an ArUco tag)
viewed from an angle admits **multiple 3D pose solutions** that produce
nearly-identical 2D image projections.

* **Left panel** — side view (depth Z vs height Y): the camera sits at the
  origin, the image plane is the short vertical line, and two marker
  orientations are shown edge-on — tilted "back" (blue) and tilted
  "forward" (teal).  Projection rays from the camera through the marker
  corners nearly overlap, converging on almost the same image points.
* **Right panel** — image plane (what the camera sees): the projected
  quadrilaterals for both solutions are overlaid, showing how two very
  different 3D poses yield almost the same 2D outline — the ambiguity.

No titles; minimal text.
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .palette import BLUE, TEAL, GRAY, BLACK, apply_presentation_style

# --- Paths ---------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")

# --- Geometry parameters -------------------------------------------------
FOCAL = 1.2          # focal length (image plane at Z = FOCAL)
DEPTH = 5.0          # marker centre distance along Z
HALF = 1.0           # half-edge length of the square marker
THETA = np.radians(35.0)  # tilt magnitude around the X axis

# Square marker corners in its own local frame (counter-clockwise).
MARKER_CORNERS = np.array(
    [
        [-HALF, -HALF, 0.0],
        [HALF, -HALF, 0.0],
        [HALF, HALF, 0.0],
        [-HALF, HALF, 0.0],
    ]
)


def _rot_x(theta):
    """Rotation matrix for a rotation about the X axis."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ]
    )


def _pose_corners(theta):
    """Return the 4 marker corners in world space for a tilt ``theta``."""
    rot = _rot_x(theta)
    t = np.array([0.0, 0.0, DEPTH])
    return (rot @ MARKER_CORNERS.T).T + t


def _project(corners, f=FOCAL):
    """Pinhole perspective projection -> 2D image-plane coordinates."""
    x = f * corners[:, 0] / corners[:, 2]
    y = f * corners[:, 1] / corners[:, 2]
    return np.column_stack([x, y])


def _draw_camera(ax, z=0.0):
    """Draw a small camera glyph pointing toward +Z."""
    size = 0.13
    tri = plt.Polygon(
        [
            (z, -size),
            (z, size),
            (z + size * 1.6, 0.0),
        ],
        closed=True,
        facecolor=BLACK,
        edgecolor="none",
        zorder=6,
    )
    ax.add_patch(tri)


def _close_ring(pts2d):
    """Close a 2D polygon by repeating the first point at the end."""
    return np.vstack([pts2d, pts2d[:1]])


def plot_ambiguity(fmt="png", dpi=300):
    """Generate the dual-panel pose-ambiguity figure and save it."""
    # Two candidate poses: tilted back (+theta) and forward (-theta).
    back_3d = _pose_corners(+THETA)
    fwd_3d = _pose_corners(-THETA)
    back_2d = _project(back_3d)
    fwd_2d = _project(fwd_3d)

    # --- Figure layout: side view (left) + image view (right) -------------
    fig, (ax_side, ax_img) = plt.subplots(
        1, 2, figsize=(10.5, 7.0)
    )
    for ax in (ax_side, ax_img):
        ax.set_facecolor("none")

    # =================== Left panel: side view (Z vs Y) ===================
    # Depth (Z) on the x-axis, height (Y) on the y-axis.
    # Image plane
    ax_side.plot(
        [FOCAL, FOCAL], [-0.35, 0.35],
        color=GRAY, linewidth=2.5, zorder=3,
    )
    # Camera
    _draw_camera(ax_side, z=0.0)

    # Projection rays (origin -> marker corners) for top & bottom edges.
    # In side view the 4 corners collapse to the min-Y and max-Y points.
    for corners_3d, color in ((back_3d, BLUE), (fwd_3d, TEAL)):
        ys = corners_3d[:, 1]
        idx_top = int(np.argmax(ys))
        idx_bot = int(np.argmin(ys))
        for idx in (idx_top, idx_bot):
            cz, cy = corners_3d[idx, 2], corners_3d[idx, 1]
            ax_side.plot(
                [0.0, cz], [0.0, cy],
                color=color, linewidth=0.9, alpha=0.45,
                linestyle="--", zorder=2,
            )

    # Marker edge-on segments (back / forward) with corner dots.
    for corners_3d, color in ((back_3d, BLUE), (fwd_3d, TEAL)):
        ys = corners_3d[:, 1]
        idx_top = int(np.argmax(ys))
        idx_bot = int(np.argmin(ys))
        seg = corners_3d[[idx_bot, idx_top], 1:]  # [Y, Z]
        ax_side.plot(
            seg[:, 1], seg[:, 0],      # Z on x, Y on y
            color=color, linewidth=3.2, solid_capstyle="round", zorder=5,
        )
        ax_side.scatter(
            seg[:, 1], seg[:, 0],
            color=color, s=28, zorder=6, edgecolors="none",
        )

    ax_side.set_xlim(-0.2, DEPTH + 1.2)
    ax_side.set_ylim(-1.5, 1.5)
    ax_side.set_xlabel("Depth  Z", fontsize=11)
    ax_side.set_ylabel("Height  Y", fontsize=11)
    ax_side.set_aspect("auto")
    apply_presentation_style(ax_side)

    # =================== Right panel: image plane ========================
    lim = 0.34
    for pts2d, color, ls in (
        (_close_ring(back_2d), BLUE, "-"),
        (_close_ring(fwd_2d), TEAL, "--"),
    ):
        ax_img.plot(
            pts2d[:, 0], pts2d[:, 1],
            color=color, linewidth=2.4, linestyle=ls, zorder=4,
        )
        ax_img.fill(
            pts2d[:, 0], pts2d[:, 1],
            color=color, alpha=0.16, zorder=3,
        )
        ax_img.scatter(
            pts2d[:-1, 0], pts2d[:-1, 1],
            color=color, s=30, zorder=5, edgecolors="none",
        )

    ax_img.axhline(0, color=GRAY, linewidth=0.6, alpha=0.4, zorder=1)
    ax_img.axvline(0, color=GRAY, linewidth=0.6, alpha=0.4, zorder=1)
    ax_img.set_xlim(-lim, lim)
    ax_img.set_ylim(-lim, lim)
    ax_img.set_xlabel("u", fontsize=11)
    ax_img.set_ylabel("v", fontsize=11)
    ax_img.set_aspect("equal")
    apply_presentation_style(ax_img)

    fig.tight_layout()
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out_path = os.path.join(OUTPUT_DIR, f"plot5_ambiguity.{fmt}")
    fig.savefig(
        out_path, dpi=dpi, facecolor="none", transparent=True,
        bbox_inches="tight",
    )
    plt.close(fig)
    print(f"  [plot5] saved {out_path}")
