# Grasp Section V3 — Final Review and Revision Plan

## Objective

Review `rapport/grasp_section_v3.tex` against the actual codebase, V1/V2/Typst drafts, and user feedback. Produce a detailed action plan to fix factual errors, improve terminology precision, strengthen arguments, and ensure V3 is ready for manual review.

**Source files analyzed:**
- `rapport/grasp_section.tex` (V1)
- `rapport/grasp_section_v2.tex` (V2)
- `rapport/grasp_section_v3.tex` (V3 — primary target)
- `rapport/grasp_preshaping_section.typ` (Typst parallel draft)
- `src/grasp_preshaping/src/planner.rs` — grasp specs, scoring, finger groups
- `src/grasp_preshaping/src/predictor.rs` — noise scaling, SMC particles, resampling
- `src/grasp_preshaping/src/config.rs` — all config constants
- `src/grasp_preshaping/src/lut_helper.rs` — Contact enum, FingerGroup enum
- `config/grasp_preshaping.yaml` — runtime parameter defaults

---

## Implementation Plan

### A. Factual Corrections

- [ ] **A1. Fix `$i\sqrt{t}$` → `$\sqrt{t}$`** at `grasp_section_v3.tex:55`. The code at `predictor.rs:113` is `t.sqrt().max(0.0)` — plain real-valued sqrt, no imaginary unit. The `$i$` is a typo (likely vim input mode artifact as user suspects). Add a brief parenthetical explaining why sqrt(t): *"The $\sqrt{t}$ scaling follows from the Wiener process variance growing linearly with time — the standard deviation therefore grows as $\sqrt{t}$, ensuring the noise amplitude remains consistent with the diffusion model's statistical properties."* Remove the phrase "stochastic Wiener diffusion process" or simplify to just "diffusion process" since calling it a Wiener process with $\sqrt{t}$ noise scaling is the explanation itself.

- [ ] **A2. Refine ES terminology for precision** throughout V3. The user wants the most precise term possible for expert readers, independent of code naming. The algorithm at `predictor.rs:288-422` implements: truncation selection (`elite_ratio: 0.05`), Gaussian perturbation with geometrically decaying variance (`decay_rate: 0.7`), elite preservation (`elite_preserve_ratio: 0.01`), and score-weighted parent selection. This is a $(\mu, \lambda)$-Evolution Strategy with adaptive step-size control. Recommended terminology: **"$(\mu, \lambda)$-Evolution Strategy with elitist preservation and geometrically decaying mutation variance"** for the first formal introduction, then "ES" or "evolution strategy" thereafter. The subsection title at line 151 should become something like "Evolution Strategy Optimization" (already close). The key is to use the ES term with the proper qualifiers so an expert immediately understands the algorithm class. Keep "SMC" out of the report entirely (it's only in the code).

- [ ] **A3. Fix the 50ms latency claim** at `grasp_section_v3.tex:33`. V2's annotation correctly flagged that the actual system requirement is different. The user's requirement spec states: total pipeline latency ≤400ms (target ≤100ms), covering the full EMG-to-motor-command path. The grasp preshaping module is only one part of this pipeline. Replace "50ms" with the correct framing: the grasp preshaping module must complete within a fraction of the total pipeline budget. Something like: *"the embedded execution constraint dictates a CPU-only pipeline completing within a strict latency window that leaves sufficient time for downstream motor command processing within the overall system budget of ≤400 ms."* The exact grasp module timing will be filled from benchmarks (TODO in evaluation section).

- [ ] **A4. Verify and clarify the thumb+index requirement** at `grasp_section_v3.tex:131` (Tier 1). The code at `planner.rs:384-387` explicitly checks `has_thumb` and `has_index` as hard requirements for Tier 1:
  ```rust
  if finger_count < spec.min_fingers || !has_thumb || !has_index {
      // Tier 2: Soft rejection
  }
  ```
  This applies to ALL three grasp types (cylindrical, pinch, lateral). The Tier 1 description in V3 says "Thumb, index, and required fingers engaged" — this is correct but could be more explicit. It is specifically **thumb and index finger** that are mandatory, not just "any 2 fingers." This is a deliberate design choice because the MIA hand's coupled mechanics mean the thumb provides opposition and the index provides the primary contact surface. Update the Tier 1 description to be explicit: *"Valid Contact (Thumb and index finger engaged, plus required additional fingers)"* and add a brief note explaining this is a domain-specific constraint from the prosthesis kinematics.

- [ ] **A5. Fix BFS-based TSDF justification** at `grasp_section_v3.tex:69-71`. The current V3 argument is weak: "multi-camera streaming architectures introduce synchronization overhead." The user clarifies that the cameras *could* stream depth maps — it's a system design decision, not a technical limitation. The real argument should be framed as an **architectural design choice**: the system was designed around a fused, segmented point cloud as the interface between perception and planning. This is a cleaner separation of concerns — the perception pipeline handles camera synchronization, registration, and segmentation upstream, delivering a single coherent point cloud. The grasp planner then operates on this abstract representation without needing camera-specific knowledge (intrinsics, extrinsics, depth map format). This architectural decision simplifies the planner, makes it camera-agnostic, and avoids duplicating TSDF fusion logic that already exists upstream. Additionally, the BFS approach provides analytically exact nearest-surface distances without the staircase artifacts inherent to projective TSDF at oblique surfaces. Rewrite this paragraph to frame it as a deliberate design decision with these advantages.

### B. Tone and Language

- [ ] **B1. Reduce overly formal language throughout V3.** The user agrees V3 went overboard when trying to remove "we" and add formality. Specific instances to tone down (non-exhaustive list — apply judgment throughout):
  - Line 14: "yield only partial point clouds" → "provide only partial point clouds"
  - Line 81: "Operating without a backside estimation mechanism restricts the TSDF to the visible sensor shell" → "Without backside estimation, the TSDF only contains information from the visible surface" (V1's version was clearer)
  - Line 84: "Superquadrics provide a highly unified alternative" → "Superquadrics provide a unified alternative"
  - Line 98: "utilizes a domain-based blending scheme operating under two distinct structural regimes" → "uses a domain-based blending scheme with two regimes"
  - Line 106: "To eliminate the need for computationally demanding inverse kinematics" → "To avoid running inverse kinematics"
  - Line 138: "The pipeline addresses this by utilizing a wrench proxy" → "We address this with a wrench proxy"
  - Throughout: "the authors" → "we" where natural (the report is a thesis, first-person plural is standard)
  - Line 49: "maximizing performance through zero-cost abstractions and safe concurrency" — simplify
  - Line 84: "mapping a single implicit function" → "using a single implicit function"
  
  The goal is to restore the clarity of V1's voice while keeping V3's structural improvements. Not every sentence needs changing — just the ones where the formality obscures meaning.

### C. Content Additions from Typst Draft

- [ ] **C1. Add a "Lessons Learned" subsection** before the Evaluation section. The Typst draft (`grasp_preshaping_section.typ:510-550`) has an excellent table of 5 engineering challenges with root causes and resolutions. Adapt this into V3 as a formal subsection (not a log/diary). Write it as concise engineering observations:
  1. Multi-camera sign voting bug (majority vote fails with opposing cameras)
  2. Deep-interior contact filtering (negative TSDF regions scored as valid contacts)
  3. Best-grasp tracking across iterations (global best vs. last-iteration best)
  4. Closure fraction dead code (Rust constant was unused, actual control in C++ ROS param)
  5. SQ boundary artifacts (one-directional sign override + minimum cell distance)
  
  Each should be 2-3 sentences: what happened, why, and the resolution. Frame as engineering insights applicable beyond this specific system.

- [ ] **C2. Add parameter tables** with actual default values from `config/grasp_preshaping.yaml`. The Typst draft includes these as figures. Add two compact tables to V3:
  - Table 1: Superquadric parameters (SQ_MAX_GN_ITERATIONS: 4, SQ_GN_DAMPING: 0.1, etc.)
  - Table 2: ES/optimization parameters (iterations: 5, decay_rate: 0.7, elite_ratio: 0.05, etc.)
  
  These give the reader concrete numbers and demonstrate engineering rigor.

- [ ] **C3. Add grasp type specification table** from Typst draft (`grasp_preshaping_section.typ:340-352`). The code confirms:
  - Cylindrical: 17 score_contacts, min_contacts=3, min_fingers=2 (thumb+index), thumb=abduction
  - Pinch: 2 score_contacts (ThumbAbdTip, IndexTip), min_contacts=2, min_fingers=2, thumb=abduction
  - Lateral: 5 score_contacts (ThumbAddTip + 4 Index side contacts), min_contacts=2, min_fingers=2, thumb=adduction
  
  This directly supports the Tier 1 description and makes the three grasp types concrete.

- [ ] **C4. Add graceful degradation mention.** The Typst draft notes: "if SQ fit fails → camera-only TSDF." The code confirms `sq_enable_backside: true` with `sq_min_fit_points: 20` and `sq_fit_error_threshold: 0.15` as guards. Add a sentence in the Superquadric section noting that if insufficient points are available or the fit error exceeds the threshold, backside estimation is skipped and the pipeline proceeds with camera-only data.

### D. Citation and Reference Issues

- [ ] **D1. Convert all Related Work citations to comments.** The user confirmed that Dex-Net, Contact-GraspNet, and GraspNet-1Billion are NOT in the bibliography yet. Wrap the entire `\cite{...}` references in LaTeX comments for now. Specifically:
  - Line 24: `\cite{mahler2017dex}` → `%\cite{mahler2017dex}` (keep the text, comment out the cite command)
  - Line 27: `\cite{sundermeyer2021contact}` → `%\cite{sundermeyer2021contact}`
  - Line 30: `\cite{fang2020graspnet}` → `%\cite{fang2020graspnet}`
  - Line 33: `\cite{mohammadi2020open}` → `%\cite{mohammadi2020open}`
  
  This preserves the text for later bibliography integration while preventing LaTeX compilation errors.

### E. Minor Improvements

- [ ] **E1. Add brief $\sqrt{t}$ explanation** alongside fix A1. After correcting the formula, add 1-2 sentences explaining: the Wiener process has variance $\sigma^2 t$, so the standard deviation scales as $\sigma\sqrt{t}$. Using $\sqrt{t}$ rather than $t$ ensures the noise grows at the correct rate for a diffusion process — linear scaling would over-disperse samples at long horizons.

- [ ] **E2. Improve V3 opening challenges** at lines 12-18. V1's version is clearer and more direct. Consider restoring V1's wording:
  - V1: "Only partial point cloud observations, even with multi-view cameras"
  - V3: "Spatial observations yield only partial point clouds, even when utilizing multi-view camera configurations"
  
  Apply similar simplification to the other three bullet points.

- [ ] **E3. Ensure the "Proposed Approach" subsection** at line 32 correctly reflects the latency budget per A3. Also verify the claim about "no existing grasp dataset covers our specific hand prosthesis" — this is correct per the code (MIA hand with coupled mechanics, no standard dataset covers this). Keep this argument.

- [ ] **E4. Add a note on the TSDF construction order.** V1 at line 33 has an important clarifying sentence absent from V3: "Note that the TSDF is constructed after the superquadric backside estimation, since the superquadric parameters are needed during TSDF construction to fill in the backside." Add this to V3's Architecture subsection (around line 47) since it explains the non-obvious pipeline ordering.

- [ ] **E5. Verify the Typst pipeline order issue is NOT in V3.** The Typst draft has the wrong order (TSDF → SQ), but V3 has the correct order (SQ → TSDF) at lines 43-44. No change needed — just confirm V3 is correct. ✓ Confirmed correct.

---

## Verification Criteria

- [ ] V3 compiles without LaTeX errors (especially after citation commenting)
- [ ] No `$i\sqrt{t}$` anywhere — only `$\sqrt{t}$`
- [ ] Algorithm terminology uses ES with proper qualifiers, no "SMC" in the report text
- [ ] The 50ms claim is replaced with correct system-level latency context
- [ ] Tier 1 description explicitly states "thumb and index finger" requirement
- [ ] BFS justification frames it as a design decision, not a limitation
- [ ] Overly formal language reduced to match V1's clarity
- [ ] Lessons Learned subsection present with 5 entries
- [ ] Parameter tables with actual values from config included
- [ ] Grasp type specification table included
- [ ] Graceful degradation mentioned for SQ fitting
- [ ] All `\cite{}` commands either have bibliography entries or are commented out
- [ ] Pipeline ordering note (SQ before TSDF) is present

## Potential Risks and Mitigations

1. **Scope creep from Typst merge** — Adding too much Typst content could bloat V3
   Mitigation: Keep additions concise — parameter tables are compact, Lessons Learned should be 5 short paragraphs max

2. **ES terminology precision vs. accessibility** — Using $(\mu, \lambda)$-ES notation may confuse non-expert readers
   Mitigation: Define the term clearly on first use, then use "ES" shorthand; the notation is standard in evolutionary computation literature

3. **BFS justification rewrite may overclaim** — Need to be honest that projective TSDF was a viable alternative
   Mitigation: Frame as a trade-off decision, not as projective TSDF being inferior; emphasize architectural cleanliness and exact distances as the deciding factors

## Evolution Summary: V1 → V2 → V3

The progression is sound:
- **V1**: Clean, clear draft with good technical content but informal and with TODOs
- **V2**: V1 + inline reviewer annotations flagging real issues (50ms claim, BFS justification, SMC naming, etc.)
- **V3**: Addresses most V2 concerns with structural improvements (formal Related Work, proper tables, tiered scoring table), but introduced new issues ($i\sqrt{t}$ typo, over-formalization, weakened BFS argument)

The plan above brings V3 to a state that combines V1's clarity with V3's structure, adds the best elements from the Typst draft, and ensures all factual claims match the codebase.
