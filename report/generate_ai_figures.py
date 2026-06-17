#!/usr/bin/env python3
"""generate_ai_figures.py — render presentation-slide figures with an AI model.

Each figure is a single conceptual slide built from simple geometric shapes in a
cohesive black / white / red / blue palette.  A shared "Keynote-style" visual
prompt is combined with a per-figure description that is grounded in the actual
project code (grasp search, superquadric fitting, TSDF fusion, GTSAM FGO, ...).

The script asks for an OpenAI API key in the terminal on first run (or reads it
from the OPENAI_API_KEY environment variable).  Images are saved as
``<number>.png`` in ``report/figures/`` (the numbers never overlap with the
code-generated figures).

Usage
-----
    python report/generate_ai_figures.py                 # all 17 figures
    python report/generate_ai_figures.py 4 7 61          # only these numbers
    python report/generate_ai_figures.py --show-prompts  # print prompts, no API
    OPENAI_API_KEY=sk-... python report/generate_ai_figures.py   # non-interactive

Environment overrides
---------------------
    OPENAI_API_KEY            API key (skips the interactive prompt)
    OPENAI_IMAGE_MODEL        model name (default: gpt-image-1)
    OPENAI_IMAGE_SIZE         image size  (default: 1536x512, 3:1 ultra-wide)
    OPENAI_IMAGE_QUALITY      quality     (default: medium)
"""

from __future__ import annotations

import base64
import getpass
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
FIG_DIR = os.path.join(ROOT, "figures")

# ---------------------------------------------------------------------------
# API configuration (overridable via environment)
# ---------------------------------------------------------------------------
MODEL = os.environ.get("OPENAI_IMAGE_MODEL", "gpt-image-2")
SIZE = os.environ.get("OPENAI_IMAGE_SIZE", "1536x512")  # landscape, 3:1
QUALITY = os.environ.get("OPENAI_IMAGE_QUALITY", "low")


# ---------------------------------------------------------------------------
# Shared visual style — clean flat 2D, black / white / red / blue
# ---------------------------------------------------------------------------
# Structure: canvas/palette definition → [VISUAL] → bracketed negative prompt.
# The negative prompt is deliberately formatted as a bracketed instruction block
# so gpt-image-2 treats it as a rule, not something to draw.
# Per-figure visuals that need a split layout start with their own layout prime.
STYLE = (
    "A clean, flat 2D presentation slide. "
    "Solid white background (#FFFFFF). "
    "All shapes use precise thin near-black (#111827) outlines. "
    "Shape fills use only three accent colors: "
    "blue (#2563EB) for primary concepts, goals, and positive states; "
    "red (#DC2626) for risks, disruptions, and negative states; "
    "grey (#6B7280) for processes, transitions, and secondary elements. "
    "[VISUAL] "
    "The composition is crisp, minimal, and centered with generous whitespace. "
    "[Style instructions: Clean flat 2D vector graphic. Do not include: "
    "3D rendering, perspective depth, drop shadows, gradients, glow effects, "
    "blurry edges, organic brush strokes, sketch hatching, isometric views, "
    "text, characters, people, scenery, or photorealistic textures.]"
)


# ---------------------------------------------------------------------------
# Per-figure descriptions (grounded in the project source)
#
# Each entry:
#   title  : short human label (used in console output only)
#   visual : concrete geometric scene description — no meta-language, no
#            abstract concepts, no exact counts.  Multi-panel figures start
#            with an explicit layout prime.
#   phrase : (unused in prompt — kept for reference)
# ---------------------------------------------------------------------------
FIGURES = {
    4: {
        "title": "Grasp prediction is hard due to high dimensionality",
        "visual": (
            "A single full-bleed canvas. A dense, intricate lattice of thin "
            "grey lines crossing at many different angles, filling the entire "
            "frame. Somewhere in the middle of the lattice, one tiny blue dot "
            "is nearly invisible against the dense lines. A thin red circle is "
            "drawn around the area where the blue dot hides, but the dot is "
            "still hard to see."
        ),
        "phrase": "HIGH-DIMENSIONAL GRASP SEARCH",
    },
    7: {
        "title": "A large pose search space vs. a bounded region",
        "visual": (
            "A two-panel, side-by-side split screen. A crisp vertical grey "
            "line perfectly divides the left and right halves. "
            "Left half: a vast empty grid of thin grey lines with a single "
            "tiny blue dot placed somewhere in the grid, extremely hard to "
            "find. "
            "Right half: the same grey grid, but a solid blue rectangle is "
            "drawn around a small area, and inside that rectangle sits the blue "
            "dot, now easy to spot."
        ),
        "phrase": "BOUND THE SEARCH, FIND THE POSE",
    },
    10: {
        "title": "A known hit point makes the bounding box redundant",
        "visual": (
            "A single centered composition. A large blue dashed rectangle sits "
            "in the middle. A thick red diagonal line crosses through the "
            "rectangle from corner to corner, striking it out. Next to the "
            "crossed-out rectangle, a single solid blue dot with a thin blue "
            "ring around it replaces the rectangle."
        ),
        "phrase": "ONE POINT REPLACES THE BOX",
    },
    16: {
        "title": "Initial TSDF sign decided only by the cameras",
        "visual": (
            "A single centered composition. A thick horizontal grey line runs "
            "across the middle, representing a surface. On the left, a small "
            "blue square with a triangle notch sits above the line, "
            "representing a camera. A dashed grey line extends from the blue "
            "square diagonally to the surface line. Small blue squares fill "
            "the space between the camera and the surface line. Small red "
            "squares fill the space behind the surface line."
        ),
        "phrase": "CAMERAS DECIDE THE SIGN",
    },
    22: {
        "title": "Gauss-Newton descent with Levenberg-Marquardt damping",
        "visual": (
            "A single centered composition. A grey U-shaped curve drawn as a "
            "thick line. Three blue dots step down the left slope of the curve "
            "toward the bottom, connected by short blue arrows. From the first "
            "blue dot, a long red dashed arrow shoots past the bottom of the "
            "curve to the right slope, overshooting. Next to it, a shorter blue "
            "solid arrow lands near the bottom safely."
        ),
        "phrase": "DAMPED GAUSS-NEWTON",
    },
    23: {
        "title": "Implicit proximity to the superquadric as a fitting cost",
        "visual": (
            "A single centered composition. A large blue rounded shape, like a "
            "smooth pebble outline, drawn in the center. Several small grey "
            "dots are scattered around the outside of the blue shape at varying "
            "distances. Thin red arrows point from each grey dot "
            "perpendicularly toward the nearest point on the blue outline."
        ),
        "phrase": "IMPLICIT PROXIMITY COST",
    },
    24: {
        "title": "Reject superquadric fits whose residual exceeds a threshold",
        "visual": (
            "A two-panel, side-by-side split screen. A crisp vertical grey "
            "line perfectly divides the left and right halves. "
            "Left half: a blue rounded shape with grey dots hugging tightly "
            "along its outline. A blue checkmark sits above it. "
            "Right half: a red rounded shape with grey dots scattered far from "
            "its outline on all sides. A red cross sits above it."
        ),
        "phrase": "REJECT POOR FITS",
    },
    25: {
        "title": "Taubin distance: algebraic distance divided by the gradient",
        "visual": (
            "A single centered composition. A thick blue curved line runs "
            "across the frame like a hill. A grey dot floats above the curve on "
            "the steep left slope. A short red dashed arrow drops vertically "
            "from the grey dot to the curve, stopping short. A longer blue "
            "solid arrow extends from the grey dot diagonally to the actual "
            "closest point on the curve further down the slope."
        ),
        "phrase": "TAUBIN DISTANCE",
    },
    26: {
        "title": "Newton-Raphson refines the Taubin distance",
        "visual": (
            "A single centered composition. A thick blue curved line runs "
            "across the frame like a hill. A grey dot floats above the curve. "
            "Three blue dots descend in sequence from the grey dot toward the "
            "curve, each one closer to the surface, connected by short blue "
            "arrows along a tangent line. The final blue dot sits directly on "
            "the blue curve."
        ),
        "phrase": "NEWTON-RAPHSON REFINEMENT",
    },
    27: {
        "title": "Domains of authority compute the TSDF distance",
        "visual": (
            "A single centered composition. A grid of small squares, like a "
            "checkerboard, divided into four large colored regions. Each region "
            "is a different shade: light blue, dark blue, light grey, dark "
            "grey. A solid blue dot sits at the center of each region. Dashed "
            "grey lines mark the boundaries between the four regions."
        ),
        "phrase": "DOMAINS OF AUTHORITY",
    },
    34: {
        "title": "Dual quaternions encode rotation and translation together",
        "visual": (
            "A single centered composition. On the left, a grey L-shaped "
            "corner representing an origin frame. On the right, a blue "
            "L-shaped corner that is both rotated and shifted to a new position "
            "from the grey one. A grey arrow connects the grey corner to the "
            "blue corner. Between them, two linked circles: a blue circle and "
            "a red circle, joined side by side like a chain link."
        ),
        "phrase": "DUAL QUATERNIONS",
    },
    38: {
        "title": "Sequential Monte Carlo vs. Evolution Strategies",
        "visual": (
            "A two-panel, side-by-side split screen. A crisp vertical grey "
            "line perfectly divides the left and right halves. "
            "Left half: many grey dots of different sizes scattered widely and "
            "randomly across the area. "
            "Right half: a blue funnel shape, wide at the top and narrow at the "
            "bottom. Several grey dots enter the wide top, a few blue dots pass "
            "through the narrow neck, and new blue dots cluster tightly below "
            "the neck."
        ),
        "phrase": "SMC vs EVOLUTION STRATEGIES",
    },
    41: {
        "title": "ESO flow: Score -> Select -> Resample -> Decay",
        "visual": (
            "A single centered horizontal composition. Three blue rectangles "
            "in a row, connected by grey arrows pointing right. "
            "Left rectangle: several grey dots inside, some with short vertical "
            "bars beneath them of varying heights. "
            "Middle rectangle: only a few blue dots inside, the rest removed. "
            "Right rectangle: a tight cluster of blue dots inside a small blue "
            "circle."
        ),
        "phrase": "SCORE - SELECT - RESAMPLE - DECAY",
    },
    42: {
        "title": "ESO capability vs. actual use (wrist rotation only)",
        "visual": (
            "A two-panel, side-by-side split screen. A crisp vertical grey "
            "line perfectly divides the left and right halves. "
            "Left half: a blue Y-shaped object with many grey arrows pointing "
            "outward in all directions around it. "
            "Right half: the same blue Y-shaped object with only a single blue "
            "curved arrow circling around its base."
        ),
        "phrase": "USED FOR WRIST ROTATION",
    },
    56: {
        "title": "The timesync command aligns host and Jetson clocks",
        "visual": (
            "A two-panel, side-by-side split screen. A crisp vertical grey "
            "line perfectly divides the left and right halves. "
            "Left half: two grey circles, each with two short clock-hand lines "
            "at mismatched angles, connected by a grey double-headed arrow with "
            "a small blue circular arrow symbol. "
            "Right half: two blue circles, each with clock-hand lines at the "
            "same matching angle, connected by a blue double-headed arrow."
        ),
        "phrase": "SYNC THE CLOCKS",
    },
    61: {
        "title": "New TSDF + MobileSAM fusion approach",
        "visual": (
            "A single centered horizontal composition. Three shapes in a row, "
            "connected by grey arrows pointing right. "
            "Left shape: a grey rectangle with a blue dot on it. "
            "Middle shape: the same grey rectangle but with a red outline drawn "
            "around the area where the blue dot sits. "
            "Right shape: a blue rounded shape representing a fused 3D cloud."
        ),
        "phrase": "SAM-MASKED TSDF FUSION",
    },
    62: {
        "title": "Factor-graph optimisation (GTSAM) vs. a Kalman filter",
        "visual": (
            "A two-panel, side-by-side split screen. A crisp vertical grey "
            "line perfectly divides the left and right halves. "
            "Left half: a horizontal chain of grey circles connected by single "
            "grey arrows, one after another in a straight line. "
            "Right half: blue circles arranged in a web pattern, connected by "
            "multiple blue lines of different lengths crossing between them, "
            "forming an interconnected graph."
        ),
        "phrase": "FACTOR GRAPH vs KALMAN FILTER",
    },
}


# ---------------------------------------------------------------------------
# Prompt construction
# ---------------------------------------------------------------------------
def build_prompt(fig: dict) -> str:
    """Combine the shared style with a figure's visual description."""
    return STYLE.replace("[VISUAL]", fig["visual"] + " ")


# ---------------------------------------------------------------------------
# Generation
# ---------------------------------------------------------------------------
def get_api_key() -> str:
    """Read the key from the environment, or prompt interactively (hidden)."""
    key = os.environ.get("OPENAI_API_KEY", "").strip()
    if key:
        return key
    print("An OpenAI API key is required to generate images.", file=sys.stderr)
    key = getpass.getpass("OpenAI API key (input hidden): ").strip()
    if not key:
        sys.exit("No API key provided. Aborting.")
    return key


def generate_one(client, number: int, fig: dict) -> str:
    """Generate a single image and write it to disk. Returns the filename."""
    from openai import OpenAI  # noqa: F401  (type hint only)

    prompt = build_prompt(fig)
    response = client.images.generate(
        model=MODEL,
        prompt=prompt,
        quality=QUALITY,
        size=SIZE,
        n=1,
    )
    image_bytes = base64.b64decode(response.data[0].b64_json)
    os.makedirs(FIG_DIR, exist_ok=True)
    out_path = os.path.join(FIG_DIR, f"{number}.png")
    with open(out_path, "wb") as fh:
        fh.write(image_bytes)
    return out_path


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _parse_numbers(argv: list[str]) -> list[int]:
    """Pull integer figure numbers out of argv (ignoring flags)."""
    nums = []
    for tok in argv:
        if tok.startswith("-"):
            continue
        try:
            nums.append(int(tok))
        except ValueError:
            pass
    return nums


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    show_prompts = "--show-prompts" in argv

    requested = _parse_numbers(argv)
    numbers = sorted(requested) if requested else sorted(FIGURES)

    unknown = [n for n in numbers if n not in FIGURES]
    if unknown:
        sys.exit(f"Unknown figure number(s): {unknown}. "
                 f"Available: {sorted(FIGURES)}")

    if show_prompts:
        print(f"--- Prompts for {len(numbers)} figure(s) "
              f"(model={MODEL}, size={SIZE}, quality={QUALITY}) ---\n")
        for n in numbers:
            fig = FIGURES[n]
            print(f"=== Figure {n}: {fig['title']} ===")
            print(build_prompt(fig))
            print("\n" + "-" * 78 + "\n")
        return 0

    # Live generation needs the OpenAI package.
    try:
        from openai import OpenAI
    except ImportError:
        sys.exit("The 'openai' package is not installed. "
                 "Install it with:  pip install openai")

    key = get_api_key()
    client = OpenAI(api_key=key)

    print(f"Generating {len(numbers)} AI figure(s) -> {FIG_DIR}")
    print(f"  model={MODEL}  size={SIZE}  quality={QUALITY}\n")

    ok, fail = 0, 0
    for n in numbers:
        fig = FIGURES[n]
        title = fig["title"]
        try:
            out = generate_one(client, n, fig)
            size_kb = os.path.getsize(out) // 1024
            print(f"  [{n}] {title}: {os.path.basename(out)} ({size_kb} KB)")
            ok += 1
        except Exception as exc:  # keep going on per-figure errors
            print(f"  [{n}] {title}: FAILED ({exc})", file=sys.stderr)
            fail += 1

    print(f"\nDone. {ok} succeeded, {fail} failed.")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
