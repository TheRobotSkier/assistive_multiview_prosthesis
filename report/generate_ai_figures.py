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
            "A single full-bleed canvas split into two stacked layers that show "
            "why grasp search is hard. "
            "Top layer: a hand outline drawn in blue on the far left, and a small "
            "target object drawn in red on the far right, far apart. Between them "
            "floats a dense, intricate lattice of thin grey lines crossing at "
            "many different angles, representing the huge continuous space of "
            "possible hand positions, orientations, wrist rotations and grasp "
            "types. The lattice completely fills the space between the hand and "
            "the object. "
            "Bottom layer: the same dense grey lattice is shown alone, filling the "
            "entire frame, and somewhere in the middle one tiny blue dot is nearly "
            "invisible against the dense lines, representing the single correct "
            "grasp configuration. A thin red circle is drawn around the area where "
            "the blue dot hides, but the dot is still hard to find."
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
            "A single centered composition that explains how cameras decide "
            "whether a voxel is inside or outside the object. "
            "A thick horizontal grey line runs across the middle, representing "
            "the visible object surface. On the left, a small blue square with a "
            "triangle notch sits above the line, representing a camera. A dashed "
            "grey line extends from the blue square diagonally through the surface "
            "line and continues behind it, representing a viewing ray. "
            "In the region between the camera and the surface line, several small "
            "blue squares are placed, each marked with a small plus symbol, "
            "representing voxels the camera sees as OUTSIDE (positive sign). "
            "In the region behind the surface line, along the same dashed ray, "
            "several small red squares are placed, each marked with a small minus "
            "symbol, representing voxels the camera votes as INSIDE (negative sign) "
            "because they are hidden behind the surface."
        ),
        "phrase": "CAMERAS DECIDE THE SIGN",
    },
    22: {
        "title": "Gauss-Newton descent with Levenberg-Marquardt damping",
        "visual": (
            "A single centered composition that explains why Gauss-Newton can "
            "overshoot and why a little damping helps. "
            "A grey U-shaped curve drawn as a thick line, representing the "
            "fitting cost surface with its minimum at the bottom centre. "
            "On the left slope, three blue dots step down toward the bottom, "
            "connected by short blue arrows, representing safe damped steps. "
            "From the first blue dot near the top of the left slope, a long red "
            "dashed arrow shoots far past the bottom of the curve and lands high "
            "up on the right slope, overshooting badly; this represents the raw "
            "undamped Gauss-Newton step that is too aggressive on a steep, "
            "curved surface. Next to it, a shorter blue solid arrow leaves the "
            "same starting dot and lands safely near the bottom, representing the "
            "damped step that stays controlled."
        ),
        "phrase": "DAMPED GAUSS-NEWTON",
    },
    23: {
        "title": "Implicit proximity to the superquadric as a fitting cost",
        "visual": (
            "A single centered composition that explains how the superquadric "
            "inside-outside value F is used as a fitting cost without computing "
            "true distances. "
            "A large blue rounded shape, like a smooth pebble outline, drawn in "
            "the centre, representing the superquadric surface where F equals "
            "zero. The interior of the shape is shaded a very light blue, "
            "representing the region where F is negative (inside). "
            "Several small grey dots are scattered around the outside of the blue "
            "shape at varying distances, representing observed point cloud "
            "points. Each grey dot is connected to the blue outline by a thin red "
            "arrow pointing perpendicularly from the dot toward the nearest point "
            "on the blue outline; the length of each red arrow represents how far "
            "that point's F value is from zero. Points close to the outline have "
            "short red arrows; points far away have long red arrows. The fitting "
            "goal is to shrink all the red arrows toward zero by moving and "
            "scaling the blue shape."
        ),
        "phrase": "IMPLICIT PROXIMITY COST",
    },
    24: {
        "title": "Reject superquadric fits whose residual exceeds a threshold",
        "allow_text": True,
        "visual": (
            "A two-panel, side-by-side split screen that explains that a "
            "superquadric fit is only accepted if its mean squared residual is "
            "below a fixed threshold. A crisp vertical grey line perfectly "
            "divides the left and right halves. "
            "Left half: a blue rounded shape with many small grey dots hugging "
            "tightly along its outline, each dot sitting almost exactly on the "
            "shape boundary. Above the shape, a large blue checkmark is drawn, "
            "and next to it a short label reading 'residual 0.02' followed by a "
            "short label reading '< 0.15', indicating the residual is below the "
            "threshold so the fit is accepted. "
            "Right half: a red rounded shape with many small grey dots scattered "
            "far from its outline on all sides, none of them close to the "
            "boundary. Above the shape, a large red cross is drawn, and next to "
            "it a short label reading 'residual 0.40' followed by a short label "
            "reading '> 0.15', indicating the residual exceeds the threshold so "
            "the fit is rejected and backside estimation is skipped."
        ),
        "phrase": "REJECT POOR FITS",
    },
    25: {
        "title": "Taubin distance: algebraic distance divided by the gradient",
        "visual": (
            "A single centered composition that explains why the raw "
            "superquadric value F is not a true distance and how dividing by the "
            "gradient magnitude fixes it. "
            "A thick blue curved line runs across the frame like a hill, "
            "representing the superquadric surface where F equals zero. "
            "A grey dot floats above the curve on the steep left slope, "
            "representing an observed point. "
            "Two arrows leave the grey dot. "
            "First, a short red dashed arrow drops straight down from the grey "
            "dot to a point on the curve directly below it; this short arrow is "
            "labelled with a small red minus symbol and represents the naive "
            "vertical distance, which is misleading on a steep slope because it "
            "is much shorter than the true distance to the surface. "
            "Second, a longer blue solid arrow extends from the grey dot "
            "diagonally, perpendicular to the curve, down to the actual closest "
            "point on the curve further down the slope; this arrow is labelled "
            "with a small blue plus symbol and represents the Taubin distance, "
            "which approximates the true perpendicular distance by dividing F by "
            "the gradient magnitude. The blue arrow is clearly longer and more "
            "accurate than the red arrow."
        ),
        "phrase": "TAUBIN DISTANCE",
    },
    26: {
        "title": "Newton-Raphson refines the Taubin distance",
        "visual": (
            "A single centered composition that explains how one Newton-Raphson "
            "step corrects the first-order Taubin approximation by re-evaluating "
            "at the projected point. "
            "A thick blue curved line runs across the frame like a hill, "
            "representing the true superquadric surface where F equals zero. "
            "A grey dot floats above the curve on the steep slope, representing "
            "the observed point. "
            "A first blue dot sits partway down toward the curve, connected to "
            "the grey dot by a short blue arrow along the gradient direction; "
            "this first blue dot is the estimated surface point from the "
            "first-order Taubin step, but it still sits slightly above the true "
            "curve. "
            "From this first blue dot, a second short blue arrow continues down "
            "along the local gradient to a second blue dot that sits directly on "
            "the blue curve; this second step is the Newton-Raphson correction, "
            "which re-evaluates F at the projected point and corrects the "
            "residual, landing on the true surface. A third blue dot also sits on "
            "the curve to show the sequence converging onto the surface."
        ),
        "phrase": "NEWTON-RAPHSON REFINEMENT",
    },
    27: {
        "title": "Domains of authority compute the TSDF distance",
        "visual": (
            "A single centered composition that explains how the TSDF volume is "
            "divided into regions where either the camera or the superquadric is "
            "the trusted source of distance. "
            "A horizontal blue line runs across the lower-middle of the frame, "
            "representing the visible object surface. Above the blue line, a small "
            "blue camera symbol sits on the left, representing the observing "
            "camera. "
            "The space is divided into three labelled horizontal bands stacked "
            "from top to bottom. "
            "The top band is shaded light grey and sits far above the surface; "
            "this is the deep unobserved region where only the superquadric "
            "defines the distance. "
            "The middle band is shaded a medium grey and sits just above and "
            "around the surface; this is the transition zone where camera and "
            "superquadric distances are smoothly blended. "
            "The bottom band is shaded light blue and sits right at and below the "
            "visible surface; this is the near-surface region where the camera "
            "distance is trusted as authoritative. "
            "Dashed grey horizontal lines mark the boundaries between the three "
            "bands. A single blue dot sits inside each band to mark its region."
        ),
        "phrase": "DOMAINS OF AUTHORITY",
    },
    30: {
        "title": "A LUT precomputes finger positions so runtime is just lookups",
        "visual": (
            "A single centered horizontal composition that explains intuitively "
            "what a lookup table (LUT) is and how it is used here. "
            "On the far left, a blue outline of a human hand drawn with its "
            "fingers open, representing the real prosthesis with many joints. "
            "Next to it, a large grey rectangle drawn like a grid or spreadsheet "
            "with many small cells arranged in rows and columns, representing the "
            "precomputed lookup table. A short grey arrow points from the open "
            "hand into the grid, showing that the hand's full closing motion was "
            "simulated offline and every finger position along the way was saved "
            "into the table. "
            "On the far right, the same blue hand outline but now drawn with its "
            "fingers partially closed around a small red dot representing the "
            "target object. A short blue arrow points from the grey grid to this "
            "closed hand, showing that at runtime the system simply looks up the "
            "saved finger position for the desired closure amount instead of "
            "doing any expensive calculation. "
            "The overall reading is: the hard kinematics work is done once "
            "offline and stored in the grid, so the running system only reads "
            "from the grid."
        ),
        "phrase": "LOOKUP TABLE",
    },
    34: {
        "title": "Dual quaternions encode rotation and translation together",
        "visual": (
            "A single centered composition that explains how a dual quaternion "
            "stores a full rigid-body transform, both rotation and translation, "
            "in a single compact unit that avoids gimbal lock and interpolates "
            "smoothly. "
            "On the left, a grey L-shaped corner representing an origin "
            "reference frame, with two short perpendicular grey arrows marking "
            "its axes. "
            "On the right, a blue L-shaped corner that is both rotated by a "
            "clear angle and shifted to a new position away from the grey one, "
            "with two short perpendicular blue arrows marking its axes; this "
            "represents the target frame after a rigid transform. "
            "A grey arrow connects the grey corner to the blue corner, "
            "representing the translation part. A small curved blue arrow near "
            "the blue corner represents the rotation part. "
            "Between the two corners, drawn prominently in the centre, two "
            "linked circles sit side by side like a chain link: a blue circle "
            "on the left representing the real quaternion that encodes the "
            "rotation, and a red circle on the right linked to it representing "
            "the dual quaternion part that encodes the translation. Together "
            "the two linked circles represent the single dual quaternion that "
            "holds the whole transform."
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
            "A single centered horizontal composition that explains the four "
            "stages of one evolution-strategy iteration, read left to right. "
            "Four blue rectangles in a row, connected by grey arrows pointing "
            "right, each rectangle representing one stage of the loop. "
            "First rectangle on the left: several grey dots inside, each grey "
            "dot with a short vertical bar of varying height beneath it; the "
            "bars represent the score assigned to each particle, some tall and "
            "some short. This is the SCORE stage. "
            "Second rectangle: only a few blue dots inside, the lowest-scoring "
            "grey dots removed, leaving only the top five percent of particles "
            "by score; this is the SELECT stage that keeps only the elite "
            "fraction. "
            "Third rectangle: many blue dots inside again, regenerated from the "
            "elite parents with small Gaussian perturbations around them, the "
            "new dots clustered near the parent positions; this is the RESAMPLE "
            "stage where score-weighted parent selection refills the "
            "population. "
            "Fourth rectangle on the right: the same many blue dots but now "
            "drawn inside a small tight blue circle, noticeably more tightly "
            "clustered than before; this is the DECAY stage that shrinks the "
            "mutation variance each iteration so the population concentrates "
            "around the best grasp."
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


# A relaxed variant for the few figures that need a short numeric value or a
# few labels to convey the concept (e.g. a threshold).  It still forbids prose,
# sentences, or paragraphs — only short labels and numbers are permitted.
STYLE_WITH_TEXT = (
    "A clean, flat 2D presentation slide. "
    "Solid white background (#FFFFFF). "
    "All shapes use precise thin near-black (#111827) outlines. "
    "Shape fills use only three accent colors: "
    "blue (#2563EB) for primary concepts, goals, and positive states; "
    "red (#DC2626) for risks, disruptions, and negative states; "
    "grey (#6B7280) for processes, transitions, and secondary elements. "
    "Any text must be short labels or single numbers only, rendered in a clean "
    "sans-serif font, near-black (#111827) or accent colour, large and crisp. "
    "[VISUAL] "
    "The composition is crisp, minimal, and centered with generous whitespace. "
    "[Style instructions: Clean flat 2D vector graphic. Do not include: "
    "3D rendering, perspective depth, drop shadows, gradients, glow effects, "
    "blurry edges, organic brush strokes, sketch hatching, isometric views, "
    "paragraphs of text, sentences, characters, people, scenery, or "
    "photorealistic textures. Short numeric values and short labels ARE allowed.]"
)


# ---------------------------------------------------------------------------
# Prompt construction
# ---------------------------------------------------------------------------
def build_prompt(fig: dict) -> str:
    """Combine the shared style with a figure's visual description."""
    style = STYLE_WITH_TEXT if fig.get("allow_text") else STYLE
    return style.replace("[VISUAL]", fig["visual"] + " ")


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
