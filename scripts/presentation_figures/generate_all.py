"""Generate all presentation figures.

Entry point that produces the five presentation figures:

    1. CUDA vs Non-CUDA Latency           (plot1_latency)
    2. Samples vs Output Variability      (plot2_variability)
    3. Error Type Sunburst                (plot3_sunburst)
    4. VIO Tracking                       (plot4_vio_tracking)
    5. ArUco Pose Ambiguity               (plot5_ambiguity)

Outputs are written to ``scripts/presentation_figures/output/``.

Usage::

    python3 -m scripts.presentation_figures.generate_all
    python3 -m scripts.presentation_figures.generate_all --fmt pdf
    python3 -m scripts.presentation_figures.generate_all --plots 1 3
    python3 -m scripts.presentation_figures.generate_all --rebuild-cache

Plot 4 reads MCAP bags which is slow.  Decoded odom data is cached as an
``.npz`` file (``output/odom_cache.npz``); subsequent runs load it in
milliseconds.  Use ``--rebuild-cache`` to force a fresh decode.
"""
import argparse
import sys

from . import (
    plot1_latency,
    plot2_variability,
    plot3_sunburst,
    plot4_vio_tracking,
    plot5_ambiguity,
)
from .bag_loader import load_odom_cached

# Registry mapping plot number -> (name, callable)
PLOT_REGISTRY = {
    1: ("plot1_latency", plot1_latency.plot_latency),
    2: ("plot2_variability", plot2_variability.plot_variability),
    3: ("plot3_sunburst", plot3_sunburst.plot_sunburst),
    4: ("plot4_vio_tracking", plot4_vio_tracking.plot_vio_tracking),
    5: ("plot5_ambiguity", plot5_ambiguity.plot_ambiguity),
}


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Generate presentation figures."
    )
    parser.add_argument(
        "--fmt",
        default="png",
        choices=["png", "pdf", "svg"],
        help="Output image format (default: png).",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=300,
        help="Raster DPI for png output (default: 300).",
    )
    parser.add_argument(
        "--plots",
        nargs="+",
        type=int,
        choices=sorted(PLOT_REGISTRY.keys()),
        default=None,
        help="Subset of plot numbers to generate (default: all).",
    )
    parser.add_argument(
        "--rebuild-cache",
        action="store_true",
        help="Force re-decoding of MCAP bags for Plot 4, ignoring the "
        "odom cache.  The cache is rewritten afterwards.",
    )
    args = parser.parse_args(argv)

    which = args.plots or sorted(PLOT_REGISTRY.keys())

    # Pre-load bag data once so Plot 4 doesn't re-read all bags.
    # Uses the on-disk cache unless --rebuild-cache is given.
    bag_data = None
    if 4 in which:
        print("Loading odom data for Plot 4 ...")
        bag_data = load_odom_cached(force_rebuild=args.rebuild_cache)

    print(f"Generating {len(which)} figure(s) as {args.fmt} ...")
    for num in which:
        name, func = PLOT_REGISTRY[num]
        print(f"[{num}] {name} ...")
        if num == 4:
            func(fmt=args.fmt, dpi=args.dpi, bag_data=bag_data)
        elif num == 3:
            # Plotly uses width/height/scale, not dpi
            func(fmt=args.fmt)
        else:
            func(fmt=args.fmt, dpi=args.dpi)

    print("Done.")


if __name__ == "__main__":
    sys.exit(main())
