# Dense Reward Strategy: Grasp Scoring Refactor

## Objective

Transform the grasp scoring pipeline from a **sparse** reward model (where many failure modes return `None` or `NEG_INFINITY`, giving the optimizer zero gradient) to a **dense** reward landscape where every candidate pose produces a meaningful `GraspScoreResult` with tiered contact scores. This gives the optimizer a continuous "warmer/colder" signal across the entire search space.

A key design decision (see discussion below) is that the new `contact_score` **replaces** `contact_count_score` in the weighted sum, since `contact_score` subsumes everything `contact_count_score` did while extending meaningfully into failure tiers.

---

## Current State Analysis

### Sparse Failure Modes (Current Code)

The existing code has **three distinct paths that discard information**:

1. **No collision at all** (`sweep_for_collision` returns `None`): Hand sweeps fully closed, hits nothing. Currently returns a `GraspScoreResult` with all-zero scores but `combined = NEG_INFINITY` in consumers (`planner.rs:292-300`).

2. **Start-position collision** (`sample == 0`): Palm or open-hand fingers already inside the object. Currently returns `None` (`planner.rs:303`), which cascades to `NEG_INFINITY` in consumers (`c_api.rs:205-219`).

3. **Insufficient finger diversity** (`finger_set.len() < spec.min_fingers`): Contacts found but concentrated on too few fingers. Currently returns `None` (`planner.rs:334-336`), also cascading to `NEG_INFINITY`.

### Why `contact_count_score` Is Replaced (Not Kept Alongside)

The existing `contact_count_score` (`planner.rs:339-343`) is computed as:

```
contact_count_score = active_contacts / min_contacts   (clamped to [0, 1])
```

This is only ever non-zero for Tier 1 (valid grasps that pass the `min_fingers` gate). For all other tiers it is dead-zero because the code returns `None` before reaching the computation.

The new `contact_score` for Tier 1 is:

```
contact_score = 0.8 + 0.2 * (active_contacts / min_contacts)   (range [0.8, 1.0])
```

Both encode the same underlying signal — a contact fraction — but `contact_score` extends meaningfully into Tier 2/3/4. Keeping both in `combined_score` would **double-count** the contact quantity signal for Tier 1 grasps. The cleanest approach is to retire `contact_count_score` from the weighted sum and let `contact_score` serve as the sole contact-based term.

The `contact_count_score` field remains on `GraspScoreResult` as a **diagnostic** (useful for logging/debug export), but it no longer participates in `combined_score`. The `active_contact_count` raw integer is also preserved for diagnostics.

### Affected Files

| File | Role | Impact |
|---|---|---|
| `src/planner.rs` | Core scoring logic | **Primary** — return type, scoring tiers, struct/method changes |
| `src/c_api.rs` | FFI consumer, `score_all_samples`, `select_best_grasp` | **High** — removes `Option` handling, uses dense `combined_score` |
| `src/config.rs` | Weight constants | **Medium** — replace `GRASP_WEIGHT_CONTACT_COUNT` with `GRASP_WEIGHT_CONTACT_SCORE` |
| `examples/debug_dump.rs` | Debug example consuming scores | **Medium** — remove `Option` match branches, add `contact_score` to export |
| `benches/pipeline.rs` | Benchmark code | **Low** — update function pointer type signatures |
| `src/debug_export.rs` | Debug dump serialization | **Medium** — add `contact_score` column to npz export |
| `scripts/visualize_grasp_debug.py` | Visualization consumer | **Low** — update `_find_best_grasp`, add tier distribution stats |

### Files That Do NOT Need Changes

- `include/grasp_preshaping/ffi_types.hpp` — The FFI response struct `GraspComputeResponseFFI` already has a `combined_score` field. It will simply carry a dense value instead of `NEG_INFINITY`. No ABI breakage.

### Key Data Flow

```
score_cylindrical/pinch/lateral (planner.rs)
  → score_grasp (planner.rs)           ← currently returns Option<GraspScoreResult>
    → score_all_samples (c_api.rs)     ← matches on Option, assigns NEG_INFINITY
      → select_best_grasp (c_api.rs)   ← picks max by combined_score
        → compute_from_request          ← errors if !found_collision on best
```

---

## Implementation Plan

### Phase 1: Struct and Return-Type Changes in `planner.rs`

- [ ] **1.1. Add `contact_score` field to `GraspScoreResult`** (`src/planner.rs:8-16`)
  - Add `pub contact_score: f64` field. This is the new unified tiered metric (0.0–1.0).
  - Rationale: Provides the dense "gradient" signal. Without it, the tiered scoring has no home in the struct.

- [ ] **1.2. Replace `w_contact_count` with `w_contact_score` in `combined_score`** (`src/planner.rs:18-33`)
  - Replace `weights.w_contact_count` / `self.contact_count_score` with `weights.w_contact_score` / `self.contact_score` in the weighted sum.
  - The `contact_count_score` field stays on the struct as a diagnostic but no longer feeds into `combined_score`.
  - Rationale: `contact_score` subsumes `contact_count_score`. Using both would double-count the contact fraction for Tier 1 grasps.

- [ ] **1.3. Replace `w_contact_count` with `w_contact_score` in `GraspWeights`** (`src/planner.rs:35-52`)
  - Rename the field from `w_contact_count` to `w_contact_score`.
  - Set default from a new config constant `GRASP_WEIGHT_CONTACT_SCORE`.
  - Remove the `GRASP_WEIGHT_CONTACT_COUNT` reference.
  - Rationale: The weight struct must match the new `combined_score` computation.

- [ ] **1.4. Change `score_grasp` return type from `Option<GraspScoreResult>` to `GraspScoreResult`** (`src/planner.rs:284-354`)
  - This is the core change. Every code path must now produce a `GraspScoreResult`.
  - The `None` returns at lines 303 and 335-336 become `GraspScoreResult` with tiered `contact_score` values.
  - Rationale: Eliminates the `Option` wrapper that currently causes information loss.

- [ ] **1.5. Implement Tier 4: No Collision** — `sweep_for_collision` returns `None` branch (`src/planner.rs:292-300`)
  - Return `GraspScoreResult { contact_score: 0.0, contact_count_score: 0.0, closure_amount: 0.0, alignment_score: 0.0, force_closure_score: 0.0, active_contact_count: 0, found_collision: false }`.
  - Rationale: Score 0.0 means "empty space, no physical data." The optimizer must translate toward the object.

- [ ] **1.6. Implement Tier 3: Start Collision** — `Some(0)` branch (`src/planner.rs:303`)
  - Replace `None` with `GraspScoreResult { contact_score: 0.1, contact_count_score: 0.0, closure_amount: 0.0, alignment_score: 0.0, force_closure_score: 0.0, active_contact_count: 0, found_collision: false }`.
  - Rationale: Score 0.1 provides a strong, non-negligible signal — "you found the object but you're too deep, back up!" Using 0.1 instead of epsilon ensures the optimizer can distinguish this from "no object at all."

- [ ] **1.7. Implement Tier 2: Soft Rejection** — insufficient fingers branch (`src/planner.rs:334-336`)
  - Instead of returning `None`, compute:
    - `base_contact_fraction = (active.len() as f64 / spec.min_contacts as f64).min(1.0)`
    - `contact_score = base_contact_fraction * 0.5` (penalty multiplier, placing this in the 0.0–0.5 range; with at least some contacts this will typically land in 0.3–0.5)
  - Compute `alignment_score` and `force_closure_score` from the active contacts (they exist, just on too few fingers).
  - Set `found_collision: true` since a collision *was* found during the sweep.
  - Rationale: The hand made contact but didn't engage enough fingers. Minor translation/rotation could fix it. This is the "warmer" zone.

- [ ] **1.8. Implement Tier 1: Valid Grasp** — existing success branch (`src/planner.rs:338-352`)
  - Compute `contact_score = 0.8 + 0.2 * (active.len() as f64 / spec.min_contacts as f64).min(1.0)`.
  - Keep existing `alignment_score`, `force_closure_score`, `contact_count_score` computations unchanged.
  - Rationale: Valid grasps occupy the top tier. The 0.8–1.0 range leaves headroom for the optimizer to "polish" from good to great.

- [ ] **1.9. Update public function signatures** (`src/planner.rs:81-119`)
  - Change `score_cylindrical`, `score_pinch`, `score_lateral` return types from `Option<GraspScoreResult>` to `GraspScoreResult`.
  - Rationale: These are the public API surface. Removing `Option` propagates the dense guarantee to all callers.

### Phase 2: Config Updates in `config.rs`

- [ ] **2.1. Replace `GRASP_WEIGHT_CONTACT_COUNT` with `GRASP_WEIGHT_CONTACT_SCORE`** (`src/config.rs:30-34`)
  - Remove `pub const GRASP_WEIGHT_CONTACT_COUNT: f64 = 1.5;`.
  - Add `pub const GRASP_WEIGHT_CONTACT_SCORE: f64 = 3.0;` (high weight, per task spec).
  - Rationale: Contact score is the primary directional signal and must dominate the combined score during search.

- [ ] **2.2. Review and adjust weight balance** (`src/config.rs:31-34`)
  - The task specifies: Contact Score weight = **High**, Alignment/Force Closure = **Moderate**.
  - Recommended final weights: `w_contact_score = 3.0`, `w_probability = 0.5`, `w_alignment = 1.0`, `w_force_closure = 1.0`.
  - Current `GRASP_WEIGHT_PROBABILITY = 1.0` → reduce to `0.5`.
  - Current `GRASP_WEIGHT_ALIGNMENT = 1.0` → keep at `1.0`.
  - Current `GRASP_WEIGHT_FORCE_CLOSURE = 1.0` → keep at `1.0`.
  - Rationale: The current equal-ish weights (1.0/1.0/1.0/1.5) don't differentiate the directional vs. diagnostic roles. The new `w_contact_score` at 3.0 dominates, ensuring the planner prioritizes "finding and touching the object" before optimizing force-closure physics.

### Phase 3: Consumer Updates in `c_api.rs`

- [ ] **3.1. Update `ScorerFn` type alias** (`src/c_api.rs:135-140`)
  - Change from `fn(...) -> Option<GraspScoreResult>` to `fn(...) -> GraspScoreResult`.
  - Rationale: The function pointer type must match the new non-optional signatures.

- [ ] **3.2. Rewrite `score_all_samples`** (`src/c_api.rs:169-223`)
  - Remove the `match scorer(...)` / `None` branch entirely.
  - Directly call `scorer(...)` and get a `GraspScoreResult`.
  - Compute `combined` using `result.combined_score(&weights, sp.sample_probability)` for **all** results — not just `found_collision`.
  - Remove the `NEG_INFINITY` fallback. Every sample now gets a real combined score.
  - Rationale: The dense reward means every sample contributes to the gradient. No more "blind" samples.

- [ ] **3.3. Verify `select_best_grasp`** (`src/c_api.rs:225-231`)
  - No structural change needed — it already picks the max by `combined`. With dense scores, it will now always find a "best" (even if the best is a Tier 3/4 result).
  - Rationale: Verification that the existing logic works correctly with the new scoring.

- [ ] **3.4. Update `compute_from_request` error handling** (`src/c_api.rs:443-450`)
  - Remove or relax the `!best.result.found_collision` error check. With dense scoring, the "best" grasp might be Tier 3 or Tier 4.
  - Instead, use `contact_score` thresholds: if `best.result.contact_score < 0.1`, report a warning but still return a result. If `contact_score == 0.0`, return an error (no object in reach at all).
  - Rationale: The FFI layer should no longer treat non-collision as a hard failure. Tier 3 results are actionable.

### Phase 4: Example and Benchmark Updates

- [ ] **4.1. Update `debug_dump.rs`** (`examples/debug_dump.rs:128-187`)
  - Change the scorer function pointer type from `Option<GraspScoreResult>` to `GraspScoreResult`.
  - Remove the `match scorer(...)` / `None` branch.
  - Always construct a `ScoredGraspExport` with the returned result.
  - Compute `combined` via `result.combined_score(...)` unconditionally.
  - Add `contact_score: result.contact_score` to the `ScoredGraspExport` construction.
  - Rationale: Example code must compile and reflect the new API.

- [ ] **4.2. Update `pipeline.rs` benchmarks** (`benches/pipeline.rs:173-178`)
  - Update the scorer function pointer type in `bench_full_pipeline` from `Option<GraspScoreResult>` to `GraspScoreResult`.
  - Update `bench_scoring_functions` similarly if needed (it calls the public score functions directly).
  - Rationale: Benchmarks must compile against the new API.

### Phase 5: Debug Export Schema Update

- [ ] **5.1. Add `contact_score` column to `ScoredGraspExport`** (`src/debug_export.rs:16-35`)
  - Add `pub contact_score: f64` field.
  - Update the npz column count from 27 to 28 and document the new column.
  - Update `export_npz` to write the new column. Insert `contact_score` at position [10] (after `wrist_rotation`), shifting `pose_se3` right. Alternatively, append as column [11] to minimize disruption.
  - Rationale: Debug dumps must capture the new metric for offline analysis.

- [ ] **5.2. Update `c_api.rs` debug export construction** (`src/c_api.rs:382-409`)
  - Add `contact_score: sg.result.contact_score` to the `ScoredGraspExport` construction inside the debug visualization block.
  - Rationale: The export must include the new field.

- [ ] **5.3. Update `visualize_grasp_debug.py` npz loading** (`scripts/visualize_grasp_debug.py:118-194`)
  - Add 28-column format handling alongside existing 27/26/25/24 backward compatibility.
  - Parse `contact_score` from the new column position.
  - Rationale: The Python viewer must read the new format.

### Phase 6: Visualization Script Updates

- [ ] **6.1. Update `_find_best_grasp`** (`scripts/visualize_grasp_debug.py:216-226`)
  - Remove the `grasps["found_collision"][i]` filter condition, or make it optional.
  - With dense scoring, low-tier results (Tier 3/4) have real combined scores and can be the "best" when no Tier 1/2 results exist.
  - Keep `found_collision` as a visual indicator (color/opacity) rather than a hard filter.
  - Rationale: The visualization should show all tiers so developers can see the gradient landscape.

- [ ] **6.2. Update `print_summary`** (`scripts/visualize_grasp_debug.py:263-328`)
  - Add tier distribution statistics: count how many results fall into Tier 1 (contact_score >= 0.8), Tier 2 (0.3–0.7), Tier 3 (0.1), Tier 4 (0.0).
  - Report `contact_score` statistics alongside existing score reporting.
  - Rationale: Makes the dense reward landscape visible in debug output.

- [ ] **6.3. Update grasp rendering in `get_grasp_indices`** (`scripts/visualize_grasp_debug.py:627-654`)
  - Use `contact_score` for opacity/size scaling instead of filtering on `found_collision`.
  - Tier 4 (score 0.0) could be rendered as tiny, transparent dots. Tier 3 (0.1) as small markers. Tier 1/2 as full markers.
  - Rationale: Visualizes the full gradient landscape.

---

## Weight Summary (Before → After)

| Weight | Before | After | Role |
|---|---|---|---|
| `w_probability` | 1.0 | 0.5 | Sample probability (reduced, less important than contact) |
| `w_alignment` | 1.0 | 1.0 | Surface normal alignment (moderate, diagnostic) |
| `w_force_closure` | 1.0 | 1.0 | Force closure balance (moderate, diagnostic) |
| `w_contact_count` | 1.5 | **removed** | Subsumed by `w_contact_score` |
| `w_contact_score` | — | **3.0** | Primary directional signal (high weight) |

## Tier Summary

| Tier | Condition | `contact_score` | `found_collision` | Meaning |
|---|---|---|---|---|
| Tier 4 | No collision (empty space) | 0.0 | false | Object not found — major translation needed |
| Tier 3 | Start collision (sample==0) | 0.1 | false | Inside object — pull back |
| Tier 2 | Too few fingers | `fraction * 0.5` | true | Almost there — minor adjustment |
| Tier 1 | Valid grasp | `0.8 + 0.2 * fraction` | true | Success — polish from good to great |

---

## Verification Criteria

- [ ] `cargo build` succeeds with zero errors after all changes.
- [ ] `cargo test` passes (if tests exist).
- [ ] `cargo run --example debug_dump` produces a valid `.npz` file.
- [ ] Every call to `score_cylindrical`/`score_pinch`/`score_lateral` returns a `GraspScoreResult` (no `None` possible).
- [ ] Tier boundaries are correct: Tier 4 = 0.0, Tier 3 = 0.1, Tier 2 = 0.0–0.5, Tier 1 = 0.8–1.0.
- [ ] `combined_score` produces finite values for all tiers (no `NEG_INFINITY` anywhere).
- [ ] The `GraspComputeResponseFFI` struct layout is **unchanged** (no FFI breakage) — `combined_score` field simply carries a dense value.
- [ ] The C++ header `ffi_types.hpp` requires **no changes** — the FFI struct is ABI-compatible.
- [ ] `contact_count_score` no longer appears in the `combined_score` weighted sum.
- [ ] `w_contact_count` / `GRASP_WEIGHT_CONTACT_COUNT` are fully removed.

## Potential Risks and Mitigations

1. **ABI breakage from `GraspComputeResponseFFI` changes**
   - Mitigation: The FFI response struct does **not** include `contact_score` as a field. The `combined_score` field already exists and will simply carry a dense value. No C++ header changes are needed.

2. **`select_best_grasp` picks a Tier 3/4 result when no valid grasp exists**
   - Mitigation: The `compute_from_request` function should check `contact_score` thresholds and return an appropriate error message if the best result is Tier 4 (no object in reach). Tier 3 results are valid signals that can be acted upon.

3. **Weight rebalancing changes existing grasp quality**
   - Mitigation: The weight changes are in `config.rs` constants. They can be tuned empirically. The refactor preserves the *structure* of scoring; only the weights and tier mapping change.

4. **Debug dump format change (27 → 28 columns) breaks existing analysis scripts**
   - Mitigation: The Python loader already handles backward-compatible column counts (24/25/26/27). Adding 28-column handling follows the established pattern.

5. **Performance regression from always computing `combined_score`**
   - Mitigation: The `combined_score` computation is trivial (3 multiplies + 1 divide). Previously, `NEG_INFINITY` was assigned for ~80% of samples; now a real score is computed. The overhead is negligible.

6. **Removing `w_contact_count` breaks any external consumer that constructs `GraspWeights` manually**
   - Mitigation: `GraspWeights` is only constructed via `Default::default()` (which reads from `config.rs` constants) in the current codebase. The `debug_dump.rs` example and benchmarks use `GraspWeights::default()`. No external code constructs it manually.

## Alternative Approaches

1. **Keep `contact_count_score` alongside `contact_score` in the weighted sum**: Would double-count the contact fraction for Tier 1 grasps, adding complexity without information gain. The new `contact_score` subsumes the old metric entirely.

2. **Use an enum `GraspTier` instead of a float `contact_score`**: Encode tiers as an enum {NoCollision, StartCollision, SoftRejection, Valid} with associated scores. Trade-off: More type-safe but harder to use in weighted sums and gradient-based optimization. The float approach is more natural for the optimizer.

3. **Keep `Option` for the FFI boundary only**: Return `GraspScoreResult` internally but wrap in `Option` at the FFI layer for backward compatibility. Trade-off: Adds an unnecessary conversion layer and doesn't fully achieve the "no Option" goal. The FFI response struct already has a `success` flag that serves this purpose.
