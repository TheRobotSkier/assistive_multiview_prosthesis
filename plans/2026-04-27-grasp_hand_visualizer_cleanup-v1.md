# Grasp Hand Model Cleanup and Rendering Speedup

## Objective

Simplify and correct the hand representation in the grasp visualizer so the displayed hand is anatomically cleaner, includes the missing ring/little fingertip representation, removes the unnecessary base-to-first-joint line, and renders faster when many grasps are visible.

## Initial Assessment

### Project structure summary

- `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py` owns the interactive PyVista viewer, including the current hand skeleton topology, the mode switch between `best` and `all`, and the actor rebuild logic. Source: `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:48-75`, `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:699-835`, `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:920-931`.
- `docker_ws/dev/grasp_preshaping/scripts/model.py` owns the hand contact model and already exposes contact transforms for all sampled fingertip/contact points. Source: `docker_ws/dev/grasp_preshaping/scripts/model.py:24-52`, `docker_ws/dev/grasp_preshaping/scripts/model.py:79-122`, `docker_ws/dev/grasp_preshaping/scripts/model.py:456-474`.
- `docker_ws/dev/grasp_preshaping/urdf/mia_hand_flat.urdf` defines the physical hand link/joint chain. Source: `docker_ws/dev/grasp_preshaping/urdf/mia_hand_flat.urdf:61-99`, `docker_ws/dev/grasp_preshaping/urdf/mia_hand_flat.urdf:199-236`, `docker_ws/dev/grasp_preshaping/urdf/mia_hand_flat.urdf:291-373`.

### Relevant files examined

- `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py`
- `docker_ws/dev/grasp_preshaping/scripts/model.py`
- `docker_ws/dev/grasp_preshaping/urdf/mia_hand_flat.urdf`
- `docker_ws/dev/grasp_preshaping/scripts/hand_tip_visualizer.py` for an example of reusing sampled contact transforms instead of hand-specific frame chains. Source: `docker_ws/dev/grasp_preshaping/scripts/hand_tip_visualizer.py:125-154`, `docker_ws/dev/grasp_preshaping/scripts/hand_tip_visualizer.py:194-260`.

### Findings and implications

1. The visualizer currently hard-codes two different hand skeleton topologies: a `simple` model with thumb/index/middle only and a `full` model with all five fingers, selected by grasp mode. Source: `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:48-75`, `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:801-835`. 
   Implication: the viewer intentionally shows different hand structures in different modes, which explains why ring/little detail is missing in some cases and why the UX feels inconsistent.

2. Ring and little fingertip information already exists in the model layer, but the current hand skeleton does not use it. Source: `docker_ws/dev/grasp_preshaping/scripts/model.py:102-110`, `docker_ws/dev/grasp_preshaping/scripts/model.py:456-474`.
   Implication: the missing ring/little tips can be added without changing the URDF by rendering from sampled contact transforms.

3. The thumb has an extra intermediate frame in the URDF chain, and the visualizer currently draws that chain directly. Source: `docker_ws/dev/grasp_preshaping/urdf/mia_hand_flat.urdf:291-373`, `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:760-799`.
   Implication: the collinear thumb point is a topology issue in the current skeleton draw list, not a data issue.

4. The current hand rendering path is likely slow because it creates many separate PyVista actors and recomputes FK per visible grasp in `all` mode. Source: `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:730-835`, `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:920-931`.
   Implication: the main performance win will come from reducing actor count and consolidating per-grasp geometry, not from micro-optimizing line drawing.

### Prioritized risks and challenges

1. **Rendering performance with many visible grasps** — highest priority because it affects the interactive experience directly and scales with grasp count. Source: `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:827-835`.
2. **Correct fingertip coverage for ring/little** — next priority because it is a correctness gap visible in the current model. Source: `docker_ws/dev/grasp_preshaping/scripts/model.py:102-110` and `docker_ws/dev/grasp_preshaping/urdf/mia_hand_flat.urdf:61-99`.
3. **Thumb redundancy / collinearity** — important for visual clarity, but lower risk than missing geometry or poor responsiveness. Source: `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:66-75`, `docker_ws/dev/grasp_preshaping/urdf/mia_hand_flat.urdf:291-373`.
4. **Mode unification UX regressions** — lower priority because it is mainly a labeling and maintainability concern, not a functional blocker. Source: `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:13-25`, `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:999-1012`.

## Implementation Plan

- [ ] **[Status: Not Started] Define a single hand visualization topology.** Replace the current `simple` versus `full` split with one canonical hand glyph that is reused in both grasp modes. Keep the same topology everywhere and vary only styling if needed for readability. Rationale: this removes structural inconsistency and simplifies future maintenance. Source basis: `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:48-75`, `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:801-835`.

- [ ] **[Status: Not Started] Switch fingertip sourcing to model contact transforms.** Use the existing sampled contact transforms for fingertip positions, especially for ring and little fingers, instead of relying only on URDF link frames. Rationale: this fills the current ring/little tip gap without changing the URDF and aligns the visualizer with the model layer that already knows the tip locations. Source basis: `docker_ws/dev/grasp_preshaping/scripts/model.py:79-122`, `docker_ws/dev/grasp_preshaping/scripts/model.py:456-474`, `docker_ws/dev/grasp_preshaping/scripts/hand_tip_visualizer.py:125-154`.

- [ ] **[Status: Not Started] Remove the base-to-first-joint line segments from the rendered hand.** Keep the proximal joint points if they help orientation, but do not connect the palm/base directly to the first finger joint. Rationale: this matches the requested visual style and reduces clutter without losing fingertip placement. Source basis: `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:760-799`.

- [ ] **[Status: Not Started] Collapse the thumb to a non-redundant chain.** Eliminate the middle thumb point that lies on a visually redundant straight segment, or replace it with a simpler root-to-tip representation if that reads better in the scene. Rationale: the thumb is the clearest case where the current chain contains an unnecessary intermediate point. Source basis: `docker_ws/dev/grasp_preshaping/urdf/mia_hand_flat.urdf:291-373`, `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:66-75`.

- [ ] **[Status: Not Started] Rebuild the hand as compact per-grasp geometry instead of many tiny actors.** Prefer a single polyline/point-based hand actor per grasp instance, or the smallest possible actor set, rather than separate line and sphere actors for every segment. Rationale: this is the main lever for improving render speed because the current implementation emits many actors per hand. Source basis: `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:771-799`, `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:827-835`.

- [ ] **[Status: Not Started] Keep the mode behavior consistent while simplifying the model.** Preserve the existing `best` and `all` grasp display modes, but make them share the same hand topology so the viewer no longer changes anatomical structure between modes. Rationale: this preserves user workflow while removing the simple/full split. Source basis: `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:333-343`, `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:920-931`.

- [ ] **[Status: Not Started] Update the hand-related text and help labels to reflect the unified model.** Remove language that implies separate simple/full hand models and describe the hand toggle in terms of visibility or density only. Rationale: the UI should match the new single-model behavior. Source basis: `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:13-25`, `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:999-1012`.

- [ ] **[Status: Not Started] Validate the refactor against representative dumps and dense grasp sets.** Confirm that ring/little tips appear, the thumb no longer shows an unnecessary middle point, the base-to-first-joint lines are gone, and the viewer remains responsive when many grasps are shown. Rationale: these are the user-visible acceptance criteria for the change set. Source basis: `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py:827-835`, `docker_ws/dev/grasp_preshaping/scripts/model.py:102-110`.

## Verification Criteria

- The same hand topology is shown in both `best` and `all` grasp modes.
- Ring and little fingers visibly include fingertip markers or segments.
- The thumb no longer contains an obvious redundant collinear intermediate point.
- No line is drawn from the palm/base directly to the first joint for any finger.
- The number of hand-rendering actors per visible grasp is materially reduced.
- Interactive mode changes no longer trigger a large hand reconstruction cost beyond the necessary geometry refresh.

## Potential Risks and Mitigations

1. **Risk: contact-transform tips do not visually align with the intended fingertip appearance.**  
   Mitigation: anchor the unified model on the existing sampled tip contacts and compare against the current URDF chain so the chosen transforms remain anatomically consistent. Source basis: `docker_ws/dev/grasp_preshaping/scripts/model.py:456-474`.

2. **Risk: a single compact actor loses per-finger coloring or readability.**  
   Mitigation: keep per-finger color arrays or split the compact model into a small number of grouped actors rather than reverting to dozens of primitives.

3. **Risk: performance remains limited when the scene contains many grasps.**  
   Mitigation: keep the unified model lightweight for `all` mode, and reserve richer styling for the single best grasp only if needed.

4. **Risk: naming changes or missing contact keys break the viewer.**  
   Mitigation: fall back to existing URDF frame positions if a sampled contact name is unavailable, but treat that as a compatibility fallback rather than the default path.

## Alternative Approaches

1. **Minimal-change approach:** keep the current code structure, but modify the full model to include ring/little tip points and remove the thumb’s redundant segment. Trade-off: lowest risk, but it preserves the simple/full split and most of the rendering overhead.

2. **Contact-driven single model:** build the hand entirely from the model’s sampled contact transforms plus a small set of root frames. Trade-off: best consistency and likely best correctness, with moderate refactor effort.

3. **Two-detail-level single topology:** use one topology everywhere, but scale detail by mode, for example showing full labels only for the best grasp and a sparse version for all grasps. Trade-off: preserves responsiveness while keeping a single conceptual model.

## Assumptions

- The goal is to improve the visualizer only; the URDF and underlying grasp computation should remain unchanged unless a fallback is needed.
- The hand should keep a single coherent topology across modes, even if styling differs by mode.
- It is acceptable to reuse existing contact transforms from `model.py` for visual fingertip placement.
- No new documentation file is needed; this plan is intended for execution tracking only.
