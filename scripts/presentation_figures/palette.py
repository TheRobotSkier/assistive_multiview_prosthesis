"""Shared color palette and styling for presentation figures.

Six-color palette plus black/white.  All presentation figure modules import
their colors from here to guarantee visual consistency.
"""

# --- The six-color palette (plus black/white) ---
TEAL = "#3494ba"
CYAN = "#58b6c0"
SAGE = "#75bda7"
GRAY = "#7a8c8e"
BLUE_GRAY = "#84acb6"
BLUE = "#2683c6"

BLACK = "#000000"
WHITE = "#ffffff"


def apply_presentation_style(ax):
    """Apply consistent styling to a matplotlib Axes.

    Transparent background, no top/right spines, light grid, consistent
    fonts.  Does NOT set a title (presentation figures have no titles).
    """
    ax.set_facecolor("none")
    # Remove top and right spines for a cleaner look
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    # Lighten remaining spines
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color("#cccccc")
        ax.spines[spine].set_linewidth(0.8)
    # Light grid behind data
    ax.grid(True, alpha=0.25, color="#dddddd", linewidth=0.6, zorder=0)
    ax.tick_params(colors="#444444", labelsize=9)
    ax.xaxis.label.set_color("#222222")
    ax.yaxis.label.set_color("#222222")
