# Grasp Section V3 — Remaining Issues Plan

## Objective

Cross-check both the first plan (`2026-05-20-grasp_section_review-v1.md`) and second plan (`2026-05-20-grasp_section_v3_final_review-1.md`) against the current state of `rapport/grasp_section_v3.tex` to identify everything still needing attention.

---

## Items Already Completed (from previous implementation)

- [x] A1: Fixed `$i\sqrt{t}$` → `$\sqrt{t}$` with diffusion explanation
- [x] A2: Renamed to `$(\mu, \lambda)$`-ES with proper qualifiers
- [x] A3: Replaced 50ms claim with ≤400ms system budget
- [x] A4: Tier 1 now says "Thumb and index finger engaged"
- [x] A5: BFS justification rewritten as architectural design decision
- [x] C1: Lessons Learned subsection added (5 entries)
- [x] C2: Parameter tables added (SQ + ES params)
- [x] C3: Grasp type specification table added
- [x] C4: Graceful degradation note added in Template Selection
- [x] D1: All `\cite{}` commands commented out
- [x] E4: Pipeline ordering note added (SQ before TSDF)

---

## Remaining Issues

### Factual Corrections

- [ ] **R1. Cylindrical score_contacts count is wrong** — `grasp_section_v3.tex:197` says "17" but the code at `planner.rs:163-185` has **21** score_contacts (17 finger contacts + 4 palm contacts). The first plan flagged this as A9. The Typst draft also had 17, which was already wrong. Fix the table to say 21, and update the explanatory paragraph at line 203 to mention that cylindrical grasps include palm contacts in addition to finger contacts.

- [ ] **R2. Pinch grasp min_contacts** — `grasp_section_v3.tex:198` says min_contacts=2, which matches `planner.rs:231`. This is correct. No change needed.

- [ ] **R3. Cylindrical grasp description in paragraph** — `grasp_section_v3.tex:203` says "all fingers except the middle, ring, and little fingertips participate in scoring." This is incorrect — the code at `planner.rs:163-185` includes ALL finger tips (MiddleTip, RingTip, LittleTip) AND palm contacts. The correct description: "All finger segments (including fingertips) and four palm contacts participate in scoring."

### B1: Overly Formal Language (Remaining Instances)

The previous B1 pass addressed the most egregious cases, but several instances of overly formal language remain:

- [ ] **Line 29**: "While highly versatile" — acceptable in context (describing Contact-GraspNet's generality), keep as-is.
- [ ] **Line 37**: "highly incomplete, partial point clouds" — acceptable emphasis, keep as-is.
- [ ] **Line 46**: "highly structured 1D array" — acceptable (Morton ordering does produce structured memory), keep as-is.
- [ ] **Line 53**: "maximizing performance through zero-cost abstractions and safe concurrency" — this is standard Rust marketing language. Simplify to: "leveraging zero-cost abstractions and safe concurrency" or "using zero-cost abstractions and safe concurrency via the \texttt{rayon} crate."
- [ ] **Line 66**: "highly deterministic, cache-friendly memory operations" — "highly deterministic" is odd here. Simplify to "deterministic, cache-friendly memory operations."
- [ ] **Line 92**: "mitigates this structural variation" — acceptable in context, keep as-is.
- [ ] **Line 99**: "This structural refinement reduces" — "structural" is used incorrectly here. The refinement is numerical/algorithmic, not structural. Change to "This refinement reduces."
- [ ] **Line 104**: "the highly accurate camera-derived distances" — "highly" is unnecessary. Change to "the accurate camera-derived distances" or just "camera-derived distances."
- [ ] **Line 115**: "targets a specific structural failure mode" — "structural" is vague here. Change to "targets a specific failure mode."
- [ ] **Line 120**: "structural LUT layout" — "structural" is used oddly. Change to "LUT layout" or "finger LUT layout."
- [ ] **Line 167**: "highly productive regions" — acceptable in optimization context, keep as-is.
- [ ] **Line 169**: "This structural decay ensures" — "structural" again. Change to "This decay ensures."

### First Plan Items Not Yet Addressed

- [ ] **A6/A7 (Tier score formulas)**: The first plan noted that V3's tier table only shows score ranges, not the actual formulas. The Typst draft provides: Tier 2 = `0.25 × contact_fraction + 0.25 × proximity_penalty`, Tier 1 = `0.8 + 0.2 × contact_fraction`. The code confirms these at `planner.rs:431,444`. Consider adding these formulas to the Tier table or the explanatory paragraph below it for completeness.

- [ ] **A12 (Prediction horizon value)**: The first plan noted that the actual prediction horizon is 5.0s (from `config/grasp_preshaping.yaml:17`). V3 line 59 says "predefined prediction horizon" without stating the value. Add "5\,s" to make it concrete.

- [ ] **A11 (API version check)**: The user explicitly said this was removed intentionally ("not an important detail... more of a nice dx choice"). This is correctly excluded. No action needed.

### Minor Consistency Issues

- [ ] **R4. "camera voting network" terminology** — `grasp_section_v3.tex:71,104` uses "camera voting network" and "camera voting scheme" but there is no neural network involved — it's a geometric ray-casting voting algorithm. Change to "per-camera ray voting scheme" (line 71 already partially says this but line 104 says "camera voting network").

- [ ] **R5. ES parameter table uses `smc_` prefixed names** — `grasp_section_v3.tex:237-238` shows `smc_convergence_tol` and `smc_min_iterations`. These are the actual YAML config key names from `config/grasp_preshaping.yaml:34-35`. Since the table documents real parameter names, this is correct. However, consider adding a footnote or parenthetical: "(retained from earlier implementation naming)" to explain the SMC prefix in an ES-titled table.

---

## Verification Criteria

- [ ] Cylindrical score_contacts count = 21 (matching `planner.rs:163-185`)
- [ ] No remaining "structural" used as a vague modifier where "numerical", "algorithmic", or nothing would be more precise
- [ ] No "highly" used where it adds no meaning
- [ ] "camera voting network" replaced with accurate terminology
- [ ] Prediction horizon value (5 s) stated explicitly
- [ ] Tier score formulas present in the table or explanatory text
- [ ] All changes maintain the formal tone without being overwrought

## Potential Risks and Mitigations

1. **Score_contacts count discrepancy** — The code clearly has 21, but earlier drafts and the Typst all said 17. This may have been an intentional simplification (excluding palm contacts from the count). Mitigation: Count the actual contacts in the code and report the true number; if palm contacts were excluded for a reason, document that reason.

2. **Tier formula addition may clutter the table** — Adding formulas to the existing table could make it too wide. Mitigation: Put formulas in the explanatory paragraph below the table rather than in the table itself.
