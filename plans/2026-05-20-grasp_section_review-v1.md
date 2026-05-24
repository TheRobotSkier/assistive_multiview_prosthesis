
# Grasp Section Versions — Review and Improvement Plan

## Objective

Review the four versions of the grasp section (`grasp_section.tex` = v1, `grasp_section_v2.tex` = v2, `grasp_section_v3.tex` = v3, `grasp_preshaping_section.typ` = Typst draft) for:
1. Factual and terminological consistency with the actual codebase
2. Appropriate emphasis of the right things
3. V3 readiness for manual review
4. Sound reasoning in the evolution from v1 → v2 → v3

---

## Evolution Summary

### V1 → V2
V2 is essentially v1 with inline reviewer notes in `[brackets]` flagging unclear claims, missing explanations, factual concerns, and structural suggestions. No content was actually rewritten — it is an annotated review copy.

Key annotations in v2:
- Related Work section needs sources; some claims are questioned (latency budget, geometric baseline statement)
- Architecture pipeline flow should be a flowchart, not inline text
- Morton ordering benefits described incompletely (should mention explicit voxel-to-array mapping, not just cache locality)
- BFS-based TSDF design decision about "no camera intrinsics" is questioned — the project actually has access to camera data
- Sign determination description possibly wrong about backside case
- Design Decisions section feels redundant — should merge into body text
- LUT: "flatten kinematic chain" reason may be wrong (should be Pinocchio compatibility, not open-loop approximation)
- SMC: should mention the evolution from 1M random samples to 20k with SMC iterations
- Integration section lost the API version check detail

### V2 → V3
V3 addresses most v2 annotations by:
- Filling in Related Work subsections with actual citations and descriptions
- Renaming "SMC" to "Evolution Strategy" / "Evolution Strategy Optimization"
- Converting the tier scoring from itemize to a proper LaTeX table
- Adding the 1M → 20k history in the optimization section
- Removing the separate "Design Decisions" subsections and integrating reasoning into body text
- Rewriting in more formal academic language
- Adding placeholder evaluation tables

### Typst Draft
A standalone Typst document that is more detailed and structured than any LaTeX version. Includes parameter tables, module tables, placeholder figures, a "Lessons Learned" section, and a "Summary and Contributions" section. This appears to be a parallel/complementary draft, not a direct v4.

---

## Issues Found

### Category A: Factual Inconsistencies with the Codebase

#### A1. Pipeline Order: SQ before TSDF vs. TSDF before SQ

**Location:** All versions describe the pipeline as: `... → Morton Sort → Superquadric Backside Estimation → TSDF Construction → ...`

**Code reality:** In `c_api.rs:396-414`, the actual order is:
1. Morton sort (line 396)
2. Superquadric fitting (lines 399-404)
3. TSDF construction (lines 406-414)

So SQ fitting does happen before TSDF construction — this is **correct** in all versions. However, the Typst draft (line 115-117) lists: `... → TSDF Construction → Superquadric Backside Estimation → ...` which is **wrong** — it reverses the order.

**Severity:** High — the Typst draft has the order backwards.

---

#### A2. Terminology: "SMC" vs. "Evolution Strategy"

**Location:** V3 renames all "SMC" references to "Evolution Strategy" / "ES"

**Code reality:** The codebase consistently uses SMC terminology:
- `src/grasp_preshaping/src/config.rs:36-42` — `ITERATIONS`, `DECAY_RATE`, `ELITE_RATIO`, `SMC_CONVERGENCE_TOL`, `SMC_MIN_ITERATIONS`
- `src/grasp_preshaping/src/predictor.rs:206` — `SmcParticle` struct
- Function names: `sample_initial_particles`, `resample_around_elites`, `select_elite_indices`

V3's rename to "Evolution Strategy" is a **judgment call**. The algorithm as implemented (truncation selection + Gaussian perturbation + elite injection) is technically closer to a $(\mu,\lambda)$-Evolution Strategy than classical Sequential Monte Carlo. However:
- The code and all internal documentation use "SMC"
- The v2 reviewer note at line 139 explicitly asks "I am not sure if this is still called SMC"
- Renaming in the paper but not the code creates a terminology disconnect

**Recommendation:** Either keep "SMC" for consistency with the codebase, or if renaming to ES, acknowledge the mapping explicitly: "The optimization uses an Evolution Strategy (ES), referred to as SMC in the implementation."

---

#### A3. Search Space Dimensionality

**Location:**
- V1/V2: "8-dimensional: 6D pose + 1D grasp type + 1D wrist rotation"
- V3: "eight continuous and categorical dimensions: 6-DOF propagation pose + 1D continuous wrist rotation + 1D categorical grasp selection"
- Typst: "approximately 9 effective degrees of freedom (6D pose + grasp type + wrist rotation + closure)"

**Code reality:** The `SmcParticle` struct (`predictor.rs:211-220`) has: `pose` (6D via dual quaternion), `grasp_type` (categorical, 3 values), `wrist_rotation` (continuous). Closure amount is NOT a search dimension — it is determined deterministically by the sweep-for-collision. So:
- V1/V2/V3 are correct at 8 effective dimensions
- The Typst draft is **wrong** to count closure as a 9th dimension

**Severity:** Medium — the Typst draft adds a dimension that doesn't exist in the search space.

---

#### A4. "50 ms" Latency Budget

**Location:** V1, V2, V3 all state approximately 50ms latency budget

**Code reality:** The `pipeline_time_ms` field in the response (`c_api.rs:96`) measures actual execution time. The config has `prediction_samples: 20000` and `iterations: 5`. The actual timing depends on hardware and object complexity. The 50ms claim appears to be a design target, not a measured result.

V2 annotation at line 34 notes: "[Not sure that is the case, our requirements say something else]"

**Recommendation:** V3 should clarify whether 50ms is a requirement or a measured result. The TODO placeholders in the evaluation section suggest this hasn't been measured yet.

---

#### A5. Noise Scaling: $\sqrt{t}$ vs. $i\sqrt{t}$

**Location:**
- V1: "scaled by $\sqrt{t}$"
- V2: "scaled by $i\sqrt{t}$" with annotation "[Why do we scale it with this]"
- V3: "scales it by $i\sqrt{t}$ to model a stochastic Wiener diffusion process"
- Typst: "Add Gaussian noise scaled by `sqrt(t)` and covariance"

**Code reality:** In `predictor.rs:113`: `let sqrt_t = t.sqrt().max(0.0);` — the noise is scaled by $\sqrt{t}$, NOT $i\sqrt{t}$.

V2 introduced an erroneous $i$ (imaginary unit?) and V3 compounded it by adding a "Wiener diffusion process" justification. The Typst draft has it correct.

**Severity:** High — V2 and V3 contain a mathematically nonsensical $i\sqrt{t}$ factor.

---

#### A6. Tier 2 Score Formula

**Location:**
- V1/V2: "Score in [0.0--0.5]"
- V3 table: "[0.00, 0.50]"
- Typst: "`0.25 × contact_fraction + 0.25 × proximity_penalty`"

**Code reality:** In `planner.rs:431`: `contact_score: (contact_count_score * 0.25) + (penalty_multiplier * 0.25)`. This gives a maximum of 0.5 (when both fractions are 1.0), so the range [0.0, 0.5] is correct.

However, V1/V2/V3 describe Tier 2 as just a range without explaining the two-component formula. The Typst draft is more accurate. V3's table just says "[0.00, 0.50]" without the formula.

**Severity:** Low — the range is correct but the mechanism is underspecified in v3.

---

#### A7. Tier 1 Score Formula

**Location:**
- V1/V2: "Score in [0.8--1.0]"
- V3: "[0.80, 1.00]"
- Typst: "`0.8 + 0.2 × contact_fraction`"

**Code reality:** `planner.rs:444`: `contact_score: 0.8 + 0.2 * contact_count_score`. The Typst is exactly correct.

**Severity:** Low — V3's range is correct but lacks the formula.

---

#### A8. Cylindrical Grasp `min_fingers`

**Location:** Typst table says cylindrical requires `min_fingers: 2` (thumb + index)

**Code reality:** `planner.rs:188`: `min_fingers: 2`. This is correct.

However, V1/V2/V3 describe the Tier 1 condition as "sufficient finger diversity, thumb and index engaged" without specifying the numeric threshold. The code checks `finger_count < spec.min_fingers || !has_thumb || !has_index`, which for cylindrical means at least 2 distinct finger groups AND thumb AND index must all be present.

---

#### A9. Cylindrical Grasp `score_contacts` Count

**Location:** Typst table says cylindrical has 17 score contacts

**Code reality:** `planner.rs:163-185` lists 21 contacts in `score_contacts` for cylindrical (including 4 palm contacts). Counting them: ThumbAbdPip, ThumbAbdDip, ThumbAbdTip, IndexMcp, IndexDip, IndexPip, IndexTip, MiddleMcp, MiddlePip, MiddleDip, MiddleTip, RingDip, RingPip, RingTip, LittleDip, LittlePip, LittleTip, PalmProxUlna, PalmProxRadi, PalmDistUlna, PalmDistRadi = 21.

**Severity:** Low — Typst says 17, code has 21. The discrepancy may be because palm contacts were added later.

---

#### A10. V3 "BFS-based TSDF" Design Decision Justification

**Location:** V3 line 70: "Because multi-camera streaming architectures introduce synchronization overhead and projection artifacts along occluded borders, computing distances directly on the fused point cloud via a BFS is significantly more efficient."

**V2 annotation:** "So in the context that this work is in, we do actually have access to all this, so it is not really a valid [argument]. Projective TSDF is fully valid, but would need to be done with two cameras."

**Code reality:** The code receives a fused point cloud (`/segmentation/object_cloud`), not individual depth maps. The C API struct `GraspComputeRequestFFI` takes camera positions but not camera intrinsics or individual depth images. So V1's original justification ("we do not have access to camera intrinsics or individual depth maps") is actually correct from the pipeline's perspective — the segmentation bridge consumes the raw camera data and outputs a fused cloud.

V3's rewritten justification about "synchronization overhead and projection artifacts" is a weaker argument. The real reason is simpler: the input to this module is already a fused point cloud, not raw depth images.

**Severity:** Medium — V3 weakens a valid argument.

---

#### A11. V3 Removed "API Version Check" from Integration

**Location:** V1 line 166 mentions "An API version check prevents ABI mismatches." V3 does not mention this.

**Code reality:** `c_api.rs:642-644`: `grasp_preshaping_api_version() -> u32 { 5 }` — the version check exists.

**Severity:** Low — minor omission, but the version check is a real engineering detail worth mentioning.

---

#### A12. Prediction Horizon

**Location:**
- V1/V2: "prediction horizon" (no specific value)
- V3: "predefined prediction horizon"
- Typst: "5 s prediction horizon"

**Code reality:** `config/grasp_preshaping.yaml:17`: `prediction_horizon_s: 5.0`

The Typst is correct. V3 could benefit from stating the actual value.

---

### Category B: Emphasis and Structural Issues

#### B1. V3 Overly Formal/Acrobatic Language

V3 uses extremely formal passive voice throughout ("The proposed framework deliberately avoids...", "Evaluating the proposed framework against representative data-driven and geometric grasp planning methods contextualizes the underlying design decisions"). This makes it harder to read than v1/v2 without adding precision. Several passages are unnecessarily verbose.

Examples:
- "Operating without a backside estimation mechanism restricts the TSDF to the visible sensor shell" vs. v1's clearer "Without backside estimation, the TSDF only contains information from the visible surface"
- "Spatial observations yield only partial point clouds, even when utilizing multi-view camera configurations" vs. v1's "Only partial point cloud observations, even with multi-view cameras"

**Recommendation:** Tone down the formality while keeping the improved structure.

---

#### B2. V3 Missing Key Design Insights from Typst

The Typst draft includes several important elements that v3 lacks:
- **Lessons Learned section** with specific challenges and resolutions (TSDF sign voting, deep-interior contacts, SMC best-loss, closure fraction dead code, SQ boundary artifacts)
- **Module table** mapping Rust files to responsibilities
- **Parameter tables** with actual default values
- **Grasp type specifications table** (score_contacts, min_contacts, min_fingers, thumb mode)
- **Explicit fallback behavior** ("if SQ fit fails → camera-only TSDF")

These are the most interesting and unique aspects of the implementation and should be emphasized.

---

#### B3. V3 Evaluation Section Still Has TODOs

The evaluation section in v3 (`grasp_section_v3.tex:172-198`) has proper table structure but still contains `TODO` placeholders. This is the main blocker for v3 being "review-ready."

---

#### B4. Related Work Citations in V3

V3 adds citations (`\cite{mahler2017dex}`, `\cite{sundermeyer2021contact}`, `\cite{fang2020graspnet}`, `\cite{mohammadi2020open}`) but these need to be verified against the actual bibliography. The citation for the "coupled finger mechanics" claim (`\cite{mohammadi2020open}`) should be checked — it may refer to the OpenHand/Mia Hand paper.

---

### Category C: V2 Annotation Resolution Tracking

| V2 Annotation | Resolved in V3? | Correctly? | Notes |
|---|---|---|---|
| Related Work needs sources | Yes | Partially | Citations added but need verification |
| "50ms" latency claim questioned | No | — | Still stated as fact without measurement |
| Architecture should be flowchart | Partially | — | V3 references Figure but still uses inline text |
| Morton ordering benefits incomplete | Yes | Yes | V3 explains voxel-to-array mapping well |
| BFS design decision invalid | Yes | Poorly | V3 changed the argument to a weaker one |
| Sign determination backside case confusion | Yes | Partially | V3 added thin-surface caveat |
| SQ PCA axis-swapping for cylinders | Yes | Yes | V3 mentions this |
| Design Decisions section redundant | Yes | Yes | Merged into body text |
| LUT flatten reason wrong | Yes | Yes | V3 says "simplified to match single-DOF constraints" |
| Taubin distance needs explanation | Yes | Yes | V3 explains variables |
| 15% → 5% improvement claim | Yes | Yes | V3 includes this |
| Smoothstep needs explanation | Yes | Yes | V3 says "cubic Hermite interpolation function" |
| Backside sign description possibly wrong | Yes | Partially | V3 clarifies but could be clearer |
| SMC → 1M random samples history | Yes | Yes | V3 includes this |
| Is it still called SMC? | Yes | Debatable | V3 renamed to ES — may cause confusion with code |
| Tier scoring should be a table | Yes | Yes | V3 uses proper table |
| Evaluation still TODO | Partially | — | Structure added, data still TODO |

---

## Implementation Plan

### Phase 1: Fix Factual Errors in V3 (Priority: High)

- [ ] **A5 Fix:** Remove the erroneous $i\sqrt{t}$ in v3 line 55. Replace with $\sqrt{t}$. The code (`predictor.rs:113`) uses `t.sqrt()`. The "Wiener diffusion process" framing can stay but the math must be $\sqrt{t}$, not $i\sqrt{t}$.
- [ ] **A3 Fix:** Verify the Typst draft separately — its "9 DOF" claim is wrong (closure is deterministic, not a search dimension). If the Typst is being maintained in parallel, fix this.
- [ ] **A10 Fix:** Restore the original v1 justification for BFS-based TSDF: the module receives a fused point cloud, not individual depth maps. V3's current "synchronization overhead" argument is weaker. A combined justification works best: "The input to this module is already a fused, segmented point cloud; individual camera depth maps and intrinsics are not available at this pipeline stage. BFS gives exact nearest-surface distances without projection artifacts."
- [ ] **A11 Fix:** Add the API version check back to v3's Integration section. It is a one-sentence detail that demonstrates engineering rigor.
- [ ] **A12 Fix:** Add the actual prediction horizon value (5.0 s) in v3's ROI section, matching the config.

### Phase 2: Resolve Terminology (Priority: High)

- [ ] **A2 Decision:** Decide whether to keep "SMC" or "Evolution Strategy" in v3. Options:
  - (a) Keep "SMC" — consistent with all code, configs, and internal docs
  - (b) Use "ES" but add a note: "The implementation refers to this as SMC (Sequential Monte Carlo)"
  - (c) Use "SMC" in the paper title/headers but describe it as an ES variant in the text
  - **Recommendation:** Option (a) — keep SMC. The algorithm has enough SMC characteristics (population-based sampling, weighted resampling, convergence monitoring) that the name is defensible, and consistency with the codebase reduces confusion.

### Phase 3: Improve V3 Readability (Priority: Medium)

- [ ] **B1 Fix:** Review v3 for overly verbose passive constructions and simplify. Target v1's clarity with v3's improved structure. Key passages to simplify:
  - Opening challenges list (v1 is clearer)
  - "Why superquadrics" paragraph (v1 is more direct)
  - Superquadric challenge paragraph (v1 is more readable)
- [ ] **A4 Clarification:** Change the "50ms" claim to clearly distinguish between design target and measured result. E.g., "The design targets a total pipeline latency of approximately 50 ms" rather than stating it as an achieved fact.

### Phase 4: Fill V3 Evaluation Section (Priority: Medium)

- [ ] **B3 Fix:** Run the test suite (`tests/test1_software_verification/`) to get actual timing data. The results table in `tests/test1_software_verification/figures/test1_results_table.tex` already has latency data per object. Extract the mean pipeline time and per-stage breakdowns.
- [ ] Fill in the `TODO` placeholders in the evaluation timing table with actual data from test runs.
- [ ] Add at least qualitative ablation results for the SQ backside estimation (even without formal experiments, the debug visualization provides evidence).

### Phase 5: Incorporate Typst Strengths into V3 (Priority: Low — Future Work)

- [ ] **B2 Fix:** Consider adding a condensed "Lessons Learned" or "Design Insights" subsection to v3, drawing from the Typst's table of challenges/resolutions. This is one of the most valuable parts of the Typst draft.
- [ ] Consider adding the grasp type specifications table (from Typst) showing score_contacts, min_contacts, min_fingers per type.
- [ ] Consider adding key parameter values inline (TSDF resolution = 5mm, truncation = 4 cells, 20k particles, etc.) rather than leaving them abstract.

### Phase 6: Fix Typst Draft (Priority: Low — If Maintaining in Parallel)

- [ ] **A1 Fix:** Correct the pipeline order in the Typst draft — SQ fitting comes BEFORE TSDF construction, not after.
- [ ] **A3 Fix:** Remove closure amount from the DOF count — it is deterministic (9 → 8).
- [ ] **A9 Fix:** Update cylindrical score_contacts count from 17 to 21 (or recount to verify).

---

## Verification Criteria

- [ ] All mathematical notation ($\sqrt{t}$, score formulas, dimensionality counts) matches the Rust source code
- [ ] Pipeline stage order matches `c_api.rs:361-414`
- [ ] Terminology (SMC/ES, grasp type names, parameter names) is consistent between paper and code
- [ ] V3 evaluation section has no remaining TODO placeholders
- [ ] All v2 reviewer annotations have been addressed or explicitly deferred
- [ ] The Related Work citations resolve to actual bibliography entries
- [ ] Tier scoring ranges and formulas match `planner.rs`

## Potential Risks and Mitigations

1. **Timing data unavailable:** The evaluation TODOs may require running the full test suite on actual hardware. Mitigation: Use the existing `test1_results_table.tex` data which already has per-object latency measurements.

2. **Citation verification:** The `\cite{mohammadi2020open}` reference needs to be confirmed as the correct source for the "coupled finger mechanics" claim. Mitigation: Cross-reference with the Mia Hand / OpenHand literature.

3. **SMC vs ES terminology debate:** Reviewers may challenge either name. Mitigation: Be prepared to defend either choice with references to the optimization literature. The algorithm shares characteristics with both SMC particle filters and $(\mu,\lambda)$-ES.

4. **Typst and LaTeX divergence:** If both are maintained in parallel, they will drift. Mitigation: Decide on one canonical version and treat the other as reference material.
