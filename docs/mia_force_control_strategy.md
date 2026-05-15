# MIA Hand Force Control Strategy

Decision record for the force-aware grasping approach used in the WT hand interface project.

---

## 1. Current Implementation — Velocity Contact-Stop

The current force-aware closure is a **velocity-based contact-stop** approach, not true force control.

**Core logic:** `src/grasp_preshaping/nodes/force_aware_closure.py`

Each finger runs a state machine with these states:

| State | Behavior |
|-------|----------|
| `OPEN_LOOP` | Close at decaying velocity toward the predicted grasp closure point |
| `CONTACT_SEEK` | Past the predicted point, close at minimum speed looking for contact |
| `CONTACTED` | Force threshold exceeded — hold velocity at zero (or a configurable hold velocity) |
| `SAFETY_STOPPED` | Terminal state after timeout or over-closure limit exceeded |
| `RELEASED` | Reset state that transitions back to `OPEN_LOOP` |

The closing speed decays as the finger approaches the predicted closure:

```
speed = min_speed + (max_speed - min_speed) * clamp(distance / decay_distance, 0, 1) ^ decay_exponent
```

Contact is detected by `check_force_contact()` using **two criteria**:

1. **Absolute threshold** — `force >= contact_force_threshold`
2. **Spike detection** — `force - baseline >= contact_force_spike_threshold`

The baseline is a rolling average of recent force readings (`compute_baseline()`).

**Orchestration:** `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`

- In the **FAR** zone, fingers use position controllers at partial closure (default 30%).
- When the hand enters the **NEAR** zone (within 8 cm of target), the node switches from position controllers to velocity controllers and enters the force-aware `FINAL_CLOSURE` phase.
- Safety guards: `final_closure_timeout_s` (default 3–4 s) and `max_extra_closure` (default 0.2–0.3 rad past predicted) prevent runaway closure.
- Force data staleness (no effort readings for > 500 ms) also triggers a safety stop.

**Configuration:** `config/prosthesis_config.yaml` defines two profiles — `soft` and `strong` — with distinct threshold, speed, and timeout values.

---

## 2. Force Data Source

Force readings come from **MIA Hand strain gauges** embedded in each finger joint. The data path is:

```
strain gauges → MIA firmware → ROS2 /joint_states.effort[]
```

These are **raw ADC values** (4-digit integers, typically ranging 0–200), **not calibrated Newtons**. There is no known linear conversion factor or per-finger calibration curve.

The `grasp_proximity_controller_node` reads effort from `/joint_states` by matching joint names (`j_thumb_fle`, `j_index_fle`, `j_mrl_fle`) and extracting the corresponding `effort[]` entry. A rolling window of recent values (deque, maxlen=50) provides the baseline for spike detection.

Thresholds in the config (e.g., `contact_force_threshold: 50.0`) are **dimensionless ADC values**, not Newtons.

---

## 3. Capabilities

What the current velocity contact-stop approach **can** do:

| Capability | How |
|------------|-----|
| Close fingers at variable speed | Decaying velocity profile with configurable exponent |
| Detect initial contact via force threshold | Absolute ADC threshold or spike above rolling baseline |
| Detect contact before predicted closure | Force check runs in `OPEN_LOOP` state too |
| Stop on contact | Velocity set to zero, state locked to `CONTACTED` |
| Hold stopped (zero velocity) after contact | Controlled via `contact_hold_velocity` parameter |
| Safety timeout | `final_closure_timeout_s` kills velocity after N seconds |
| Safety over-closure limit | Stops at `predicted_closure + max_extra_closure` |
| Resistance to sensor noise | Rolling baseline for spike detection |
| Sensor dropout protection | Stale data timeout triggers stop |
| Multiple grasp force profiles | `soft` and `strong` presets in config |
| Per-finger independent state | Each finger has its own state machine |

---

## 4. Limitations

What the current approach **cannot** do:

| Limitation | Impact |
|------------|--------|
| **Regulate force to a target level** | No PID or feedback loop around a force setpoint; only binary contact/no-contact |
| **Maintain constant contact force** | After contact, velocity = 0 (or small hold velocity). Force reading is ignored. |
| **Impedance control** | No spring-damper relationship between position and force |
| **Slip detection or compensation** | No force derivative or traction analysis |
| **True force/current regulation** | `set_motor_speed` accepts `max_cur` (current percentage) as a limit only, not a setpoint |
| **Calibrated force values** | ADC values have no Newton mapping; thresholds are hand-tuned |
| **Gradual force ramp** | Force is binary — either closing or stopped |
| **Per-finger differentiated forces** | All fingers use the same profile (though states are independent) |

---

## 5. Recommendation

**Stick with velocity contact-stop for the initial hardware deployment.**

Rationale:

- **Simplicity** — The state machine is small (~117 lines of pure Python), well-tested (9 unit tests), and deterministic. There are no PID loops to tune or stability issues to debug.
- **Safety** — Multiple independent safety stops (timeout, over-closure, stale data) ensure the hand won't close indefinitely. True force control adds risk of oscillation, force overshoot, or clamping that could damage grasped objects.
- **Demonstrability** — Contact-stop is sufficient for proof-of-concept: the system detects an object, closes until contact, holds. This demonstrates the full perception-to-grasp pipeline without the complexity of force regulation.
- **Unknown hardware capability** — The MIA firmware's `set_motor_speed` interface accepts a `max_cur` percentage parameter, but this is a current *limit*, not a setpoint. It caps the motor current during velocity moves but does not allow commanding a specific current or force directly. Whether the firmware supports direct current control needs investigation.

The contact-stop approach is not the final form, but it is the right choice for the current maturity level of both the software pipeline and the hardware understanding.

---

## 6. Future Work

If true force/effort control becomes necessary (e.g., for delicate object handling, multi-finger force distribution, or slip compensation), the following steps are required:

1. **Research MIA firmware commands** — Investigate whether `MAIA2Finger` / `CppDriver` expose commands for:
   - Direct current setpoint (not just velocity with current limit)
   - Force/torque control mode
   - Current feedback (actual motor current, not strain gauge ADC)

2. **Design an effort command interface for ros2_control** — If the hardware supports current control, add an `effort` command interface in the MIA hardware plugin (`mia_hand_ros2_control`), alongside the existing `position` and `velocity` command interfaces.

3. **Implement a proper force controller** — With a known current-to-force relationship (or at least a monotonic relationship between commanded current and strain gauge reading), build a PID or admittance controller that:
   - Commands motor current (or effort) instead of velocity
   - Closes the loop on strain gauge ADC values
   - Supports a configurable target force per finger

4. **Calibrate strain gauge to force** — Establish a mapping from ADC to Newtons (even if approximate) to allow meaningful force-level configuration. This may require a force-sensing calibration jig and per-finger calibration data stored in config.
