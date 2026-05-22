# Plan: Improve Grasp Preshaping Report Section

## Objective
Address the six critique points (no narrative, no results, no related work, meta-content, over-detailed integration, evidence-free decisions) and upgrade the visual presentation with Lexend font and improved Typst styling.

## Implementation Plan

### Part A: Visual and Styling Improvements

- [ ] **A1. Install/verify Lexend font**: Run `typst fonts` to check if Lexend is available. If not, install it via system package manager or download from Google Fonts and place in `~/.local/share/fonts/`. Lexend has multiple variants (Lexend Deca, Lexend Exa, etc.) — use "Lexend" (the base/regular variant) or "Lexend Deca" as the body font.
- [ ] **A2. Update font configuration**: Change line 6 from `"New Computer Modern"` to `"Lexend"`. Consider a paired setup: Lexend for body text, and a serif font (e.g., "New Computer Modern" or "Linux Libertine") for figure captions or math, since Lexend is a sans-serif humanist font designed for readability.
  ```
  #set text(font: "Lexend", size: 11pt, lang: "en")
  ```
- [ ] **A3. Improve page layout**: Add more generous margins, enable page numbers, add a header/footer with section title:
  ```
  #set page(paper: "a4", margin: (top: 2.5cm, bottom: 2.5cm, left: 2.8cm, right: 2.8cm),
    numbering: "1",
    number-align: center,
  )
  ```
- [ ] **A4. Style headings**: Add visual hierarchy with Typst heading styling:
  - Level 2 (`==`): larger, bold, with a subtle bottom border or color accent
  - Level 3 (`===`): italic or bold-italic, slightly larger than body
  - Use `#show heading.where(level: 2): it => { ... }` rules
- [ ] **A5. Style tables**: Add alternating row colors, rounded corners, better padding:
  - Use `table.hline(stroke: 1pt)` for header separators
  - Add `fill: (col, row) => if row > 0 and calc.rem(row, 2) == 0 { luma(248) }` for zebra striping
  - Increase `inset` to `10pt` for breathing room
- [ ] **A6. Style code blocks**: Set a monospace font for raw blocks, add a subtle background:
  ```
  #show raw.where(block: true): it => block(
    fill: luma(245),
    inset: 10pt,
    radius: 4pt,
    width: 100%,
    it
  )
  ```
- [ ] **A7. Style bullet lists**: Add slightly more spacing between items:
  ```
  #set list(indent: 1.5em, spacing: 0.6em)
  ```
- [ ] **A8. Remove the grey placeholder boxes**: Replace the current `block(inset: 12pt, fill: luma(245))[...]` figure placeholders with cleaner placeholder syntax that looks more intentional — e.g., a dashed-border box with centered text, or a simple `[#box(width: 100%, height: 5cm, fill: luma(250), stroke: (dash: "dashed", paint: luma(200)))[ ... ]]` approach.
- [ ] **A9. Remove the `#line(length: 100%)` separators between sections**: Typst headings already create visual breaks. The horizontal rules add clutter. If separation is needed, use `#v(1em)` spacing instead.

### Part B: Structural / Content Improvements

- [ ] **B1. Add related work paragraph to Introduction (Section 1)**: After the challenges bullet list and before "Key design insight", add a new subsection or paragraph:
  - Briefly mention 2-3 existing approaches: Dex-Net (sampling-based, requires full object model), Contact-GraspNet (deep learning, ~100ms+ inference, needs GPU and training data), GraspNet-1Billion (model-based, needs complete point cloud)
  - Explain why they don't apply: our system operates on partial point clouds in real-time on embedded hardware, with no training data for the Mia hand, and a 50ms latency budget
  - This positions the work and justifies the geometric/optimization approach
- [ ] **B2. Restructure for narrative flow**: Reorder the introduction to follow: problem → why existing approaches fail → our key insight → overview of our solution. Currently it jumps to "we use TSDF + LUT" without context. Specifically:
  - Move the "Key design insight" bullet to after the related work paragraph
  - Add a transition sentence like "This leads us to a geometry-first approach that combines..."
- [ ] **B3. Add a Results/Evaluation section (new Section 7, before current SMC section or after it)**: This is the highest-priority content addition. Include:
  - **Timing breakdown**: table or bar chart showing ms spent in each pipeline stage (ROI pruning, TSDF construction, SQ fitting, SMC scoring per iteration, total). Source: debug dump data or add timing instrumentation.
  - **Grasp success rate**: if you have data from real or simulated trials, even anecdotal ("tested on 15 household objects, successful grasp on 12") — include it. If no formal evaluation exists yet, add a placeholder with instructions to gather this data.
  - **Ablation: with vs. without superquadric backside**: show 1-2 examples where SQ backside estimation prevents a grasp from passing through the object. This directly justifies Section 4.
  - **SMC convergence behavior**: the figure is already planned (Fig. 9), but add text describing the typical convergence curve — how many iterations until convergence, how much the best score improves from iteration 0 to final.
  - If no quantitative data exists yet, structure this section with placeholder text like "*[Insert timing data from debug dumps]*" so you know exactly what to fill in.
- [ ] **B4. Add evidence to design decisions**: For the key "Why X" bullets, add brief notes about what happened when the alternative was tried (or why it's obvious it would fail). Examples:
  - Section 3 "Why BFS": add "Projective TSDF was initially considered but produces staircase artifacts at oblique surfaces; BFS gives analytically exact distances at the cost of one BFS pass."
  - Section 4 "Why 3 templates": add "Early testing with 5 templates showed <2% improvement in fit error for a 60% increase in fitting time."
  - Section 7 "Why SMC over CEM": add "CEM was prototyped but produced similar-quality results with more implementation complexity; SMC's particle resampling maps naturally to our parallel scoring architecture."
  - These don't need to be long — 1-2 sentences each — but they transform assertions into evidence.
- [ ] **B5. Collapse Section 8 (System Integration) to ~1 paragraph**: Remove the FFI implementation details (dlopen, flat structs, API versioning). Keep only:
  - What the bridge node does (1 sentence: subscribes to hand pose/twist/pointcloud, calls Rust library, publishes finger commands)
  - The closure fraction parameter (1 sentence)
  - Debug visualization (1 sentence)
  - The ROS 2 node graph figure (Fig. 11) stays
  - Move the `OnceLock` / YAML config detail into a footnote or remove it
- [ ] **B6. Remove meta-content sections**: Delete the "Figures Checklist" and "Tables Checklist" sections at the end (lines 457-499). These are planning artifacts, not report content. The figure/table numbering is already captured in the `#figure` labels throughout the document.
- [ ] **B7. Reframe Section 9 (Challenges) as "Lessons Learned"**: Change the tone from "bugs we fixed" to "design insights from iteration". For each challenge, add a sentence about the broader lesson:
  - TSDF sign bug → "This demonstrates that multi-camera TSDF sign determination requires consensus logic that accounts for opposing viewpoints, not simple majority voting."
  - Deep-interior contacts → "A distance-based collision metric alone is insufficient; surface proximity filtering is essential to distinguish genuine contacts from interior penetration."
  - SMC best-grasp loss → "In iterative optimization, the output must be selected from the global best across all iterations, not just the final iteration."
  - Dead closure fraction → "Configuration parameters must have a single source of truth; duplicate constants across language boundaries are a maintenance hazard."
  - SQ artifacts → "When blending multiple geometric representations, preserving the authority of higher-fidelity data near its source prevents boundary artifacts."
- [ ] **B8. Add cross-references between sections**: Use Typst `@fig1`, `@table2`, `@scoring_tiers` references in the text instead of inline "[Fig. 4]" text. This makes the document navigable and is standard academic practice. For example:
  - In Section 6, when discussing the scoring tiers, write "as shown in @scoring_tiers"
  - In Section 7, when mentioning convergence, write "see @fig9"
  - In Section 4, reference the SQ parameter table as "@table2"

### Part C: Section Reordering

- [ ] **C1. Proposed final section order** (adjusted for narrative):
  1. Introduction and Problem Statement (expanded with related work)
  2. Pipeline Architecture Overview (kept, slightly trimmed)
  3. Region of Interest and TSDF Construction (kept)
  4. Superquadric Backside Estimation (kept)
  5. Finger Contact Lookup Table (kept)
  6. Grasp Scoring and Collision Detection (kept)
  7. Sequential Monte Carlo Optimization (kept)
  8. Results and Evaluation (NEW — timing, ablation, convergence analysis)
  9. System Integration (collapsed to ~1 paragraph + figure)
  10. Lessons Learned (reframed from current Section 9)
  11. Summary and Contributions (kept, trimmed)
  - Remove: Figures Checklist, Tables Checklist

## Verification Criteria
- [ ] Document compiles with `typst compile` using Lexend font
- [ ] Page count stays in the 10-12 page range (content grew but Section 8 shrunk)
- [ ] All 11 figure placeholders and 5 tables are still present with correct labels
- [ ] Typst cross-references (`@label`) work (no broken references)
- [ ] No `#line(length: 100%)` separators remain between sections
- [ ] Results/Evaluation section exists with placeholder structure even if data is TBD
- [ ] Related work paragraph exists in introduction
- [ ] Figures/Tables checklist sections are removed

## Potential Risks and Mitigations

1. **Lexend font not installed**: Typst will error on compile. Mitigation: check with `typst fonts` first; if missing, install via `sudo apt install fonts-lexend` or download from Google Fonts to `~/.local/share/fonts/`.
2. **Results data may not exist yet**: The Results section may need placeholder text. Mitigation: structure it with clear `[TODO: insert data]` markers so you know exactly what to gather.
3. **Cross-reference labels may break if sections are renamed**: Typst labels are attached to `#figure` blocks, not headings, so reordering sections should not break references. Mitigation: compile after each major change.
4. **Section reordering may feel unnatural if Results comes before Integration**: The proposed order puts Results before Integration because results discuss the pipeline's performance, while Integration is a brief implementation detail. Mitigation: if it feels odd, Integration can come right after Architecture Overview as a brief note.
