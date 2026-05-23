# Resolution Plan: Grasp Section Bracketed Items

## Objective
Resolve all 14 bracketed items `[...]` in `report/grasp_section.txt` by verifying truth against the codebase, correcting inaccuracies, filling gaps, and improving prose.

---

## Implementation Plan

### Category A: Factual Verification (Items where you expressed doubt — verified against code)

- [ ] **Item 1** (Line 2 — BFS initialization): **Keep text as-is.** The BFS is correctly described — it seeds from unique occupied voxels identified via distinct Morton codes. Your doubt is unfounded; the code confirms the report is accurate.
  - *Verification*: `pointcloud_helper.rs:367-378` iterates `offsets` (unique Morton IDs), sets `distance=0` for each, and pushes to the queue.

- [ ] **Item 10** (Line 45 — Incongruent signs): **Fix the ambiguous sentence.** The close-to-surface and far-from-surface descriptions are accurate. But the phrase "carve out free space from occluded regions but not overwrite the free space established by the cameras" is misleading. The code gives SQ sign full authority in all incongruent cases — it flips camera signs in BOTH directions. Rewrite to:
  > "When signs disagree, the pipeline trusts the superquadric sign for the inside/outside decision while preserving the camera distance magnitude near the visible surface. Farther from the observed shell, the system switches entirely to the superquadric distance metric, with smoothstep interpolation at the boundary."
  - *Verification*: `pointcloud_helper.rs:634-654` — SQ sign wins unconditionally in `!signs_agree` branch.

### Category B: Content Gaps (Items needing new material or figures)

- [ ] **Item 4** (Line 13 — TSDF image): **Generate a visualization** of the unsigned TSDF after BFS (before sign assignment). Use the debug export pipeline (`debug_export.rs`) to dump an `.npz` file and plot a 2D cross-section with the distance field color-mapped and the zero-crossing highlighted.

- [ ] **Item 5** (Line 28 — Superquadric intro): **Add 2-3 sentences** introducing superquadrics plus the inside-outside function:
  > `F(x,y,z) = ((|x/a|^(2/ε₁) + |y/b|^(2/ε₁))^(ε₁/ε₂) + |z/c|^(2/ε₂))^ε₂ - 1`
  > with F=0 at the surface, F<0 inside, F>0 outside. Note that (ε₁, ε₂) control the shape: (1,1)=sphere, (0.1,0.1)≈box, (0.1,1.0)≈cylinder.
  - *Source*: `superquadric.rs:72-77`, `superquadric.rs:82-114`.

- [ ] **Item 11** (Line 49 — Finished TSDF figure): **Generate a visualization** of the final signed TSDF after superquadric backside integration. Show a cross-section with the original observed surface, the completed backside, and the sign boundary. Use the debug pipeline.

- [ ] **Item 12** (Line 56 — Contact points figure): **Run the existing Python script** at `src/grasp_preshaping/scripts/visualize_grasp_debug.py` to generate a figure showing the pre-computed contact points from the finger LUT (`data/finger_contact_lut.npz`). Verify the script works and capture the output.

- [ ] **Item 14** (Line 135 — TODO timing): **Measure and insert** the actual pipeline timing. Run the pipeline and record the value from `pipeline_time_ms` (`c_api.rs:585`). Insert as e.g. "~120 ms" (or whatever the measured value is).

### Category C: Prose Improvements (Items needing revision or additional explanation)

- [ ] **Item 2** (Line 6 — Sign assignment explanation): **Optionally strengthen** the justification by incorporating the architectural separation-of-concerns argument currently in the comment block (lines 8-11): the pipeline was designed to accept a fused point cloud rather than raw depth maps, making the BFS+ray-vote approach a natural consequence of the interface contract.

- [ ] **Item 3** (Line 8 — Commented design decisions): **Lift the comment block into the main text** after revision. The architectural argument (camera-agnostic interface) is valid and strengthens the paper. Prune any defensive language and state it as a deliberate design choice.

- [ ] **Item 7** (Line 34 — Sharp corners): **Improve the explanation** to be more precise:
  > "The primitives use ε=0.1 rather than true zero for box and cylinder templates to maintain numerical stability. Extremely sharp corners (ε → 0) produce near-singular gradients in the implicit function, degrading the Taubin distance approximation which divides by the gradient norm."
  - *Rationale*: `superquadric.rs:247-280` shows the gradient norm is central to both the Taubin approximation and Newton-Raphson refinement.

- [ ] **Item 8** (Line 36 — Fitting residual): **Add a brief explanation** of the fitting residual:
  > "The fitting residual is the mean squared error of the implicit function evaluated at each observed point (F(p) ≈ 0 on the surface). A threshold of 0.15 was chosen as a sensible default — values above this indicate the point cloud does not conform well to any primitive, making backside estimation unreliable."
  - *Source*: `superquadric.rs:35-36`, `superquadric.rs:650`, `runtime_config.rs:74`.

- [ ] **Item 9** (Line 39 — Taubin section): **Restructure** to first define the superquadric function (from item 5), then explain the Taubin approximation more concisely: D = F(x) / ||∇F(x)|| with one Newton-Raphson refinement step. The current level of technical detail is acceptable for a thesis/report, but reordering would improve readability.

### Category D: Factual Corrections (Items where the report is wrong or misleading)

- [ ] **Item 6** (Line 32 — PCA axis ambiguity): **Correct the claim.** The code does NOT perform axis swaps. Rewrite to:
  > "PCA establishes the orientation, locking the rotation for all template fits. For the cylinder template (ε₁=0.1, ε₂=1.0), the Z PCA axis is implicitly treated as the symmetry axis. This is acceptable because [your justification here — e.g., target objects do not exhibit severe axial ambiguity, or the approach direction from the motion model constrains orientation]."
  - *Verification*: `superquadric.rs:612-655` — single PCA, no axis permutations. OBB provides scale along each axis independently.

- [ ] **Item 13** (Line 84 — Tier-based scoring): **Revise the section** to match the current code (`planner.rs:330-508`):
  - **Tier 4**: Score range is 0.00-0.05 (TSDF proximity at mid-closure, not "fully closed"). Add: superquadric fallback gives 0.00-0.04 when TSDF is out of range.
  - **Tier 3**: Score range is 0.01-0.09 (not fixed 0.10). It's depth-graded: deep penetration → 0.01, barely inside surface → 0.09. Without SQ, flat 0.04-0.05.
  - **Tier 2**: Update formula: `contact_count_score × 0.25 + penalty_multiplier × 0.25`. Explain that `penalty_multiplier` accounts for distance-to-surface of missing thumb/index fingers (not a generic proximity term).
  - **Tier 1**: Score range 0.80-1.00 is correct. Formula `0.8 + 0.2 × contact_fraction` matches the code.

---

## Verification Criteria

- [All 14 bracketed items are addressed with a clear action: keep, fix, add, or revise]
- [Item 10 (incongruent signs) has the misleading sentence replaced with accurate description]
- [Item 6 (PCA axis ambiguity) is corrected to reflect actual code behavior]
- [Item 13 (tier scoring) matches the Tier 3, 4, and 2 formulas in `planner.rs`]
- [Items 4, 11, 12 have generated figures referenced in the report]
- [Item 14 has a real timing value instead of TODO]
- [Items 5, 7, 8 have improved explanatory text inserted]

---

## Potential Risks and Mitigations

1. **Risk: Measurement noise in pipeline timing (Item 14)**
   *Mitigation*: Run multiple trials and report a representative value (mean/median). Add a qualifier like "~120 ms on an Intel i7-12700" to indicate hardware dependence.

2. **Risk: Python visualization script may be outdated or broken (Items 4, 11, 12)**
   *Mitigation*: Test the script first. If it fails, write a minimal matplotlib visualization directly from the `.npz` output.

3. **Risk: Tier 3 score range correction may affect other parts of the report**
   *Mitigation*: Search for any other references to "0.10" or "fixed" regarding Tier 3 and update them consistently.

---

## Alternative Approaches

1. **For Item 6 (axis ambiguity)**: Instead of correcting the text, you could add a code change to try all 3 axis permutations for the cylinder template. But this triples cylinder fitting cost. The simpler fix (correcting the claim) is recommended since the PCA-based approach works for your target objects.

2. **For Item 13 (tier scoring)**: Rather than revising the existing text, you could extract the scoring details into a separate table listing each tier's condition, score range, and formula explicitly — making future updates easier. This is recommended if the scoring system is still evolving.
