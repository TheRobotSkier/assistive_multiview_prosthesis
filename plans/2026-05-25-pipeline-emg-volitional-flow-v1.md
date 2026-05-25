# Pipeline EMG-Volitional Control Flow

## Objective

Redesign the pipeline state machine and control flow so that EMG gestures drive the full grasp cycle: power gesture → twist propagation → collision-triggered segmentation → preshaping → approach with wrist + hand → near-zone force closure → force-stable volitional control → open release.

## State Machine Design

```
IDLE
  │  POWER gesture (confidence ≥ threshold)
  ▼
TWISTING          ← NEW: twist propagation active, waiting for hit
  │  twist detects collision → publishes click_positive
  │  pipeline_manager subscribes to new /twist_propagation/hit_detected Bool
  ▼
SEGMENTING        ← unchanged
  │  object cloud received via /segmentation/object_cloud
  ▼
PLANNING          ← unchanged
  │  preshaping service completes → target_hand_pose + closures published
  ▼
APPROACHING       ← MODIFIED: twist deactivates at 20cm stop-distance
  │  proximity controller: wrist moves + partial finger closure
  │  within_stop_distance (≤20cm from hit) → deactivate twist
  │  near_zone entered (≤8cm) → transition to GRASPING
  ▼
GRASPING          ← unchanged
  │  force controller closes fingers fully
  │  force_stable → transition to HOLDING
  ▼
HOLDING           ← unchanged, but now followed by VOLITIONAL
  │  force_stable sustained → transition to VOLITIONAL
  ▼
VOLITIONAL        ← NEW: user-in-the-loop EMG control
  │  FLEXION → increase force target (+adjust)
  │  EXTENSION → decrease force target (-adjust)
  │  POWER    → toggle between FORCE mode and WRIST mode
  │             in WRIST mode: FLEXION→wrist+, EXTENSION→wrist−
  │             in FORCE mode: FLEXION→force+, EXTENSION→force−
  │  OPEN     → transition to RELEASING
  ▼
RELEASING         ← unchanged
  │  hand opens, monitored via joint_states
  ▼
IDLE
```

## Implementation Plan

### Phase 1: Pipeline Manager — New States & Transitions

- [ ] **Add TWISTING state** to `State` enum (value 7)
- [ ] **Add VOLITIONAL state** to `State` enum (value 8)
- [ ] **Update `_can_transition`** to include new valid transitions:
  - `IDLE → TWISTING`
  - `TWISTING → SEGMENTING`
  - `HOLDING → VOLITIONAL`
  - `VOLITIONAL → RELEASING, IDLE`
  - `VOLITIONAL → VOLITIONAL` (for mode toggle without state exit)
- [ ] **Update `_on_emg_gesture`**:
  - POWER gesture in `IDLE` → `TWISTING` (activate twist, do NOT trigger segmentation)
  - In `VOLITIONAL`: FLEXION/EXTENSION action depends on current sub-mode (force vs wrist)
  - POWER in `VOLITIONAL` toggles sub-mode (force ↔ wrist)
  - OPEN in `VOLITIONAL` → `RELEASING`
- [ ] **Add `_emg_volitional_mode`** tracking: `"force"` or `"wrist"` (default `"force"`)
- [ ] **Add volitional force adjust**: in force mode, publish `Float64` to `/force_controller/manual_adjust` with ±1.0 per gesture tick
- [ ] **Add volitional wrist relay**: in wrist mode, publish incremental position to `/wrist/set_position` (reuse existing `_emg_wrist_target_deg` tracking but now driven by FLEXION/EXTENSION directly, not via /emg_grasp/intent)
- [ ] **Update `_on_force_status`**: when `HOLDING` and `force_stable` sustained for configurable duration → `VOLITIONAL`
- [ ] **Add `TWISTING`-to-`SEGMENTING` transition**: subscribe to new `/twist_propagation/hit_detected` Bool topic. On `True` while in `TWISTING`, transition to `SEGMENTING`.
- [ ] **Add stop-distance monitoring**: subscribe to `/twist_propagation/collision_distance` Float32. When distance ≤ `twist_stop_distance_m` (parameter, default 0.20) and in `APPROACHING`, deactivate twist propagation via service call.

### Phase 2: Pipeline Manager — New Parameters & Config

- [ ] **Declare new parameters**:
  - `twist_stop_distance_m` (float, default 0.20) — distance from hit point to deactivate twist
  - `twist_hit_detected_topic` (str, default `/twist_propagation/hit_detected`)
  - `collision_distance_topic` (str, default `/twist_propagation/collision_distance`)
  - `volitional_force_adjust_step` (float, default 1.0) — force delta per FLEXION/EXTENSION tick
  - `volitional_wrist_velocity_scale` (float, default 45.0) — wrist deg/s per proportional unit
  - `volitional_entry_delay_s` (float, default 0.5) — sustained force_stable before entering VOLITIONAL
- [ ] **Add new subscriptions**:
  - `/twist_propagation/hit_detected` Bool — transitions TWISTING→SEGMENTING
  - `/twist_propagation/collision_distance` Float32 — triggers twist deactivation at 20cm
- [ ] **Update `prosthesis_config.yaml`** with all new parameters in `pipeline_manager` section

### Phase 3: Twist Propagation — Collision Distance & Hit-Detected Topics

- [ ] **Add `collision_distance_topic` parameter** (str, default `/twist_propagation/collision_distance`)
- [ ] **Add `collision_distance_pub` publisher** in node `__init__` — Float32, publishes distance from hand to predicted hit point in metres
- [ ] **Add `hit_detected_topic` parameter** (str, default `/twist_propagation/hit_detected`)
- [ ] **Add `hit_detected_pub` publisher** — Bool, TRANSIENT_LOCAL durability, published `True` on first hit detection after activation, `False` on deactivation/reset
- [ ] **Compute collision distance**: in `_run_idle_cycle`, after `_propagate_and_find_hit` returns a hit, compute Euclidean distance from current hand pose to hit point in cloud frame. Publish on `collision_distance_pub`.
- [ ] **Latch hit_detected**: On activation, set `_hit_detected = False`. When first hit is found after activation, publish `True` and set `_hit_detected = True`. On deactivation/reset, publish `False`.
- [ ] **Publish distance even without hit**: When no hit in current cycle, publish `-1.0` sentinel on collision_distance so pipeline_manager knows distance is invalid (matching existing `hit_time` sentinel pattern).

### Phase 4: Force Controller — Manual Adjust Subscription

- [ ] **Verify `/force_controller/manual_adjust` subscription** exists and properly handles:
  - +1.0 → increase target force by `force_adjust_step`
  - −1.0 → decrease target force by `force_adjust_step`
  - Clamp to `[target_force_min, target_force_max]`
- [ ] **Add `manual_adjust_topic` parameter** if not already present
- [ ] **Log manual adjustments** at info level for operator feedback

### Phase 5: EMG Grasp Controller — Wrist Toggle Intent

- [ ] **Add `MODE_TOGGLE` intent type** to `IntentType` enum in `emg_grasp_state_machine.py`
- [ ] **In `CONTROL_GRASP` mode**: when POWER gesture received and hand is already in CONTROL_GRASP → emit `MODE_TOGGLE` intent to switch to `CONTROL_WRIST`
- [ ] **In `CONTROL_WRIST` mode**: when POWER gesture received → emit `MODE_TOGGLE` intent to switch back to `CONTROL_GRASP`
- [ ] **Update `pipeline_manager._on_emg_grasp_intent`** to handle `MODE_TOGGLE`: set `_emg_volitional_mode` to either `"force"` or `"wrist"` based on current mode

### Phase 6: Config & Launch Updates

- [ ] **Add new parameters to `prosthesis_config.yaml`**:
  - `pipeline_manager` section: all new params from Phase 2
  - `twist_propagation` section: `collision_distance_topic`, `hit_detected_topic`
- [ ] **Verify `pipeline.launch.py`** passes all new config sections correctly (no further changes needed — existing `_node_params` pattern handles new config)

### Phase 7: Build & Validate

- [ ] **Build** with `colcon build --symlink-install --packages-up-to prosthesis_launch`
- [ ] **Python syntax check** all modified/new files
- [ ] **Smoke test**: launch pipeline with `pipeline.launch.py emg:=true` and verify:
  - All nodes start without errors
  - Topics `/twist_propagation/collision_distance` and `/twist_propagation/hit_detected` appear
  - Pipeline manager publishes correct state sequence
- [ ] **Integration test**: with simulator/digital twin:
  - POWER → TWISTING state appears in `/pipeline/state_name`
  - Hand moves → twist detects hit → `hit_detected=true` → SEGMENTING
  - Object cloud arrives → PLANNING → APPROACHING
  - Hand approaches → within 20cm → twist deactivates
  - Enters near zone → GRASPING → force stable → HOLDING
  - Sustained force → VOLITIONAL → FLEXION/EXTENSION adjust force
  - POWER toggles to wrist mode → FLEXION/EXTENSION move wrist
  - OPEN → RELEASING → IDLE (ready for next POWER gesture)

## Verification Criteria

- [ ] POWER gesture from IDLE activates twist propagation (no immediate segmentation)
- [ ] Twist hit detection triggers segmentation via click_positive (existing path)
- [ ] Pipeline state transitions: IDLE → TWISTING → SEGMENTING → PLANNING → APPROACHING → GRASPING → HOLDING → VOLITIONAL → RELEASING → IDLE
- [ ] Twist propagation deactivates when hand is ≤20cm from predicted collision point
- [ ] Force controller activates when hand enters near zone (≤8cm from target, existing proximity behavior)
- [ ] Force stable → HOLDING → sustained stable → VOLITIONAL (with configurable delay)
- [ ] In VOLITIONAL mode: FLEXION increases force, EXTENSION decreases force, POWER toggles force↔wrist
- [ ] OPEN gesture returns to IDLE from any state (including VOLITIONAL)
- [ ] All new parameters in `prosthesis_config.yaml` are loaded correctly
- [ ] All timing and threshold parameters are tunable via `emg_live.yaml`

## Potential Risks and Mitigations

1. **Twist-to-segmentation race**: Twist may detect hit and pipeline_manager may not yet be in TWISTING state
   Mitigation: Hit_detected Bool is latched TRANSIENT_LOCAL, so late subscribers still receive it. Pipeline manager checks state before processing.

2. **Distance topic publish timing**: Collision distance published at twist cycle rate (~50 Hz), pipeline_manager polls at 1-5 Hz
   Mitigation: Pipeline_manager uses a dedicated subscription callback (not polling), so it receives the latest value immediately.

3. **Twist and proximity controller both commanding wrist**: If both are active, wrist may receive conflicting commands
   Mitigation: Twist is deactivated at 20cm before proximity takes over. Wrist commands are only published by the active pipeline state.

4. **Force adjust and manual adjust duplication**: Both pipeline_manager (via volitional mode) and emg_grasp_controller (via intents) may publish adjust commands
   Mitigation: In VOLITIONAL mode, pipeline_manager drives force adjust directly. The emg_grasp_controller's force adjust intents are only processed in CONTROL_GRASP mode (non-VOLITIONAL states).

5. **VOLITIONAL mode vs HOLDING ambiguity**: Force stable may flicker on/off, causing rapid HOLDING↔VOLITIONAL transitions
   Mitigation: `volitional_entry_delay_s` debounces the transition. Only sustained (configurable, default 0.5s) force_stable triggers VOLITIONAL.

## Alternative Approaches

1. **No dedicated VOLITIONAL state**: Reuse HOLDING and add EMG gesture checks in all active states instead
   Trade-off: Less visible state progression, debugging harder, but fewer code changes.

2. **Pipeline manager directly subscribes to click_positive**: Instead of adding a hit_detected topic, pipeline manager can watch click_positive messages to detect hits
   Trade-off: More fragile (click_positive published for other reasons), harder to distinguish first hit from retarget hits.

3. **Twist propagation handles stop-distance internally**: Twist could auto-deactivate when within 20cm instead of publishing distance for pipeline manager
   Trade-off: Less visibility for the orchestrator, harder to debug, but fewer ROS topic hops.

