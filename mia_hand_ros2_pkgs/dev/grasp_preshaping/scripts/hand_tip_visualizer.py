import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import os
import sys

try:
    from mia_hand_ros2_pkgs.dev.grasp_preshaping.scripts.model import (
        COLLISION_GEOMETRIES,
        CONTACT_DEFINITIONS,
        get_sampled_contact_transforms,
        get_q_full,
        low,
        model as pin_model,
        data as pin_data,
        high,
        pin,
    )
except ModuleNotFoundError:
    # Allow direct execution via `python path/to/hand_tip_visualizer.py`.
    repo_root = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)
    from mia_hand_ros2_pkgs.dev.grasp_preshaping.scripts.model import (
        COLLISION_GEOMETRIES,
        CONTACT_DEFINITIONS,
        get_sampled_contact_transforms,
        get_q_full,
        low,
        model as pin_model,
        data as pin_data,
        high,
        pin,
    )


def _plot_sphere_wire(ax, center, radius, color="#888888", alpha=0.2):
    u = np.linspace(0.0, 2.0 * np.pi, 14)
    v = np.linspace(0.0, np.pi, 10)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    return ax.plot_wireframe(x, y, z, color=color, alpha=alpha, linewidth=0.6)


def _plot_cylinder_wire(ax, T, radius, length, color="#888888", alpha=0.25):
    t = np.linspace(0.0, 2.0 * np.pi, 18)
    z_vals = np.array([-0.5 * length, 0.5 * length], dtype=float)
    artists = []

    for z in z_vals:
        local = np.stack(
            [radius * np.cos(t), radius * np.sin(t), np.full_like(t, z), np.ones_like(t)],
            axis=0,
        )
        world = T @ local
        line = ax.plot(world[0], world[1], world[2], color=color, alpha=alpha, linewidth=0.8)[0]
        artists.append(line)

    for angle in np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False):
        local = np.array(
            [
                [radius * np.cos(angle), radius * np.cos(angle)],
                [radius * np.sin(angle), radius * np.sin(angle)],
                [-0.5 * length, 0.5 * length],
                [1.0, 1.0],
            ],
            dtype=float,
        )
        world = T @ local
        line = ax.plot(world[0], world[1], world[2], color=color, alpha=alpha, linewidth=0.6)[0]
        artists.append(line)

    return artists


def _plot_box_wire(ax, T, half_extents, color="#888888", alpha=0.25):
    hx, hy, hz = half_extents
    corners = np.array(
        [
            [-hx, -hy, -hz, 1.0],
            [hx, -hy, -hz, 1.0],
            [hx, hy, -hz, 1.0],
            [-hx, hy, -hz, 1.0],
            [-hx, -hy, hz, 1.0],
            [hx, -hy, hz, 1.0],
            [hx, hy, hz, 1.0],
            [-hx, hy, hz, 1.0],
        ],
        dtype=float,
    ).T
    wc = (T @ corners)[:3].T

    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ]

    artists = []
    for i, j in edges:
        line = ax.plot(
            [wc[i, 0], wc[j, 0]],
            [wc[i, 1], wc[j, 1]],
            [wc[i, 2], wc[j, 2]],
            color=color,
            alpha=alpha,
            linewidth=0.7,
        )[0]
        artists.append(line)
    return artists


def estimate_workspace_bounds(samples=250):
    """Estimate fixed axis limits from all sampled contact positions."""
    all_pts = []
    for _ in range(samples):
        q = np.random.uniform(low, high)
        transforms = get_sampled_contact_transforms(q)
        all_pts.extend([t[:3, 3] for t in transforms.values()])

    pts = np.vstack(all_pts)
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    center = 0.5 * (mins + maxs)

    span = float(np.max(maxs - mins))
    half = max(0.08, 0.55 * span)
    return center, half


def main():
    group_colors = {
        "thumb": "#f4a261",
        "index": "#e76f51",
        "middle": "#2a9d8f",
        "ring": "#457b9d",
        "little": "#8d99ae",
        "palm": "#6a994e",
    }

    contact_group = {c["name"]: c["group"] for c in CONTACT_DEFINITIONS}
    sorted_contacts = sorted(contact_group.keys())

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(left=0.08, right=0.98, top=0.96, bottom=0.26)

    center, half = estimate_workspace_bounds()
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)
    ax.set_box_aspect((1.0, 1.0, 1.0))
    
    ax.set_title("MIA Hand Contact Point Visualizer")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")

    # Draw a faint origin marker for reference.
    ax.scatter([0.0], [0.0], [0.0], c="black", s=20, alpha=0.35)

    initial_q = np.array([0.0, 0.0, 0.0], dtype=float)
    point_artist = None
    text_artist = None
    geom_artists = []

    def redraw(q_active):
        nonlocal point_artist, text_artist, geom_artists

        transforms = get_sampled_contact_transforms(q_active)
        q_f = get_q_full(q_active)
        pin.forwardKinematics(pin_model, pin_data, q_f)

        xyz = np.array([transforms[name][:3, 3] for name in sorted_contacts], dtype=float)
        colors = [group_colors.get(contact_group[name], "#4c78a8") for name in sorted_contacts]

        if point_artist is not None:
            point_artist.remove()
        point_artist = ax.scatter(
            xyz[:, 0],
            xyz[:, 1],
            xyz[:, 2],
            c=colors,
            s=36,
            alpha=0.9,
            depthshade=True,
        )

        for artist in geom_artists:
            if isinstance(artist, list):
                for sub in artist:
                    sub.remove()
            else:
                artist.remove()
        geom_artists = []

        for geom_name, geom_info in COLLISION_GEOMETRIES.items():
            m_joint = pin_data.oMi[geom_info["joint_id"]]
            m_geom = m_joint * geom_info["placement"]
            T = m_geom.homogeneous
            gtype = geom_info["type"]
            params = geom_info["params"]

            if gtype == "sphere":
                artist = _plot_sphere_wire(
                    ax,
                    center=T[:3, 3],
                    radius=params["radius"],
                    color="#888888",
                    alpha=0.22,
                )
                geom_artists.append(artist)
            elif gtype == "cylinder":
                artists = _plot_cylinder_wire(
                    ax,
                    T=T,
                    radius=params["radius"],
                    length=params["length"],
                    color="#888888",
                    alpha=0.25,
                )
                geom_artists.append(artists)
            elif gtype == "box":
                artists = _plot_box_wire(
                    ax,
                    T=T,
                    half_extents=params["half_extents"],
                    color="#888888",
                    alpha=0.30,
                )
                geom_artists.append(artists)

        if text_artist is not None:
            text_artist.remove()
        text_artist = ax.text2D(
            0.02,
            0.02,
            (
                f"Thumb={q_active[0]:.3f}  TISIT={q_active[1]:.3f}  MRL={q_active[2]:.3f}\\n"
                f"Showing {len(sorted_contacts)} sampled contact points"
            ),
            transform=ax.transAxes,
            fontsize=10,
        )

        fig.canvas.draw_idle()

    # Slider controls.
    ax_thumb = fig.add_axes([0.16, 0.16, 0.68, 0.03])
    ax_tisit = fig.add_axes([0.16, 0.11, 0.68, 0.03])
    ax_mrl = fig.add_axes([0.16, 0.06, 0.68, 0.03])

    s_thumb = Slider(ax_thumb, "Thumb Flex", float(low[0]), float(high[0]), valinit=initial_q[0])
    s_tisit = Slider(ax_tisit, "TISIT Motor", float(low[1]), float(high[1]), valinit=initial_q[1])
    s_mrl = Slider(ax_mrl, "MRL Flex", float(low[2]), float(high[2]), valinit=initial_q[2])

    def on_slider_change(_):
        q = np.array([s_thumb.val, s_tisit.val, s_mrl.val], dtype=float)
        redraw(q)

    s_thumb.on_changed(on_slider_change)
    s_tisit.on_changed(on_slider_change)
    s_mrl.on_changed(on_slider_change)

    ax_reset = fig.add_axes([0.86, 0.015, 0.1, 0.04])
    b_reset = Button(ax_reset, "Reset")

    def on_reset(_):
        s_thumb.reset()
        s_tisit.reset()
        s_mrl.reset()

    b_reset.on_clicked(on_reset)

    redraw(initial_q)
    plt.show()


if __name__ == "__main__":
    main()
