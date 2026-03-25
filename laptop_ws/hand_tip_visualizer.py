import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from model import get_all_finger_positions, tip_data, low, high


def sphere_mesh(center, radius, n_u=20, n_v=14):
    """Generate a sphere surface mesh centered at `center`."""
    u = np.linspace(0.0, 2.0 * np.pi, n_u)
    v = np.linspace(0.0, np.pi, n_v)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    return x, y, z


def estimate_workspace_bounds(samples=250):
    """Estimate fixed axis limits by random motor sampling."""
    all_pts = []
    for _ in range(samples):
        q = np.random.uniform(low, high)
        pos = get_all_finger_positions(q)
        all_pts.extend(pos.values())

    pts = np.vstack(all_pts)
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    center = 0.5 * (mins + maxs)

    span = float(np.max(maxs - mins))
    half = max(0.08, 0.55 * span)
    return center, half


def main():
    finger_colors = {
        "thumb": "#f4a261",
        "index": "#e76f51",
        "middle": "#2a9d8f",
        "ring": "#457b9d",
        "little": "#8d99ae",
    }

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(left=0.08, right=0.98, top=0.96, bottom=0.26)

    center, half = estimate_workspace_bounds()
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)
    ax.set_box_aspect((1.0, 1.0, 1.0))
    
    ax.set_title("MIA Hand Fingertip Visualizer")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")

    # Draw a faint origin marker for reference.
    ax.scatter([0.0], [0.0], [0.0], c="black", s=20, alpha=0.35)

    initial_q = np.array([0, 0, 0], dtype=float)
    sphere_artists = {}
    line_artists = {}

    def redraw(q_active):
        positions = get_all_finger_positions(q_active)

        for artist in sphere_artists.values():
            artist.remove()
        sphere_artists.clear()

        for artist in line_artists.values():
            artist.remove()
        line_artists.clear()

        for finger, pos in positions.items():
            line = ax.plot(
                [0.0, float(pos[0])],
                [0.0, float(pos[1])],
                [0.0, float(pos[2])],
                color=finger_colors.get(finger, "#4c78a8"),
                alpha=0.8,
                linewidth=1.8,
            )[0]
            line_artists[finger] = line

            radius = max(float(tip_data[finger]["radius"]), 0.005)
            x, y, z = sphere_mesh(pos, radius)
            artist = ax.plot_surface(
                x,
                y,
                z,
                color=finger_colors.get(finger, "#4c78a8"),
                alpha=0.72,
                linewidth=0,
                shade=True,
            )
            sphere_artists[finger] = artist

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
