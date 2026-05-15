# MIA Hand Tangential Force Channels — Analysis

## 1. What the Data Is

The MIA hand reports six force channels per `get_finger_forces()` call — three normal-force and three tangential-force — one per finger (thumb, index, MRL). All six are parsed identically as signed 4-digit ASCII integers from the serial response message.

**`src/mia_hand_driver/src/mia_hand_driver/cpp_driver.cpp:467-468`** — function signature:
```cpp
bool CppDriver::get_finger_forces(
    int32_t& thumb_nfor, int32_t& index_nfor, int32_t& mrl_nfor,
    int32_t& thumb_tfor, int32_t& index_tfor, int32_t& mrl_tfor)
```

Parsing examples (all follow the same pattern: read 4 ASCII chars, apply sign bit from 2 bytes earlier):

| Channel        | Value bytes (rx_msg) | Sign byte (rx_msg) | Lines       |
|----------------|----------------------|---------------------|-------------|
| `thumb_nfor`   | [44–47]             | [42]                | 495–501     |
| `index_nfor`   | [17–20]             | [15]                | 503–509     |
| `mrl_nfor`     | [53–56]             | [51]                | 511–517     |
| `thumb_tfor`   | [35–38]             | [33]                | 519–525     |
| `index_tfor`   | [26–29]             | [24]                | 527–533     |
| `mrl_tfor`     | [8–11]              | [6]                 | 537–543     |

All six channels are fully parsed and valid after `get_finger_forces()` returns `true`. No data is discarded at the driver level.

## 2. Why They’re Currently Unused

In `MiaHandSystemInterface::read()` (`src/mia_hand_ros2_control/src/mia_hand_ros2_control/mia_hand_system_interface.cpp:319-371`), all six force values are received from the driver (lines 340–349), but only the three normal-force channels are mapped:

```cpp
// Lines 347-353 — only normal forces mapped to /joint_states.effort
if (mia_hand_->get_finger_forces(
      thumb_nfor, index_nfor, mrl_nfor,
      thumb_tfor, index_tfor, mrl_tfor))
{
    jnt_eff_state_[0] = static_cast<double>(thumb_nfor);  // thumb
    jnt_eff_state_[1] = static_cast<double>(index_nfor);  // index
    jnt_eff_state_[2] = static_cast<double>(mrl_nfor);    // MRL
}
```

The three tangential-force variables (`thumb_tfor`, `index_tfor`, `mrl_tfor`) are local to the `read()` method and are never assigned to any state interface, published on any topic, or examined by any subscriber. They are silently discarded at the end of each `read()` cycle.

No Python node in `src/` references `tfor`, `tangential`, `shear`, or `finger_forces` — confirming no downstream consumer exists for these values.

## 3. Potential Value for Grasp Monitoring

Tangential (shear) force could provide signals that normal force alone cannot:

- **Object slip** — A rapid change in shear direction or a shear drop while normal force holds steady may indicate incipient slip. Classical friction-cone slip detection requires both normal and shear components.
- **Grasp instability** — Oscillating shear forces during a static hold suggest the object is shifting within the grip.
- **Over-tightening** — Steadily climbing shear without a proportional normal increase can indicate the finger pads are shearing against the object surface due to excessive grip force.
- **Contact state estimation** — Combined normal+tangential force allows reconstruction of the full contact wrench at each fingertip, useful for grasp quality metrics.

## 4. How to Expose Them

**Option A — Custom ROS2 message per finger**

Define a custom message (e.g. `FingerForce.msg`) containing `float32 normal` and `float32 tangential`, and publish three of them (or a `FingerForces.msg` array) on a new topic such as `/mia_hand/finger_forces`. This requires:
- A new message package or adding the `.msg` to an existing one
- Patching `MiaHandSystemInterface::read()` to populate and publish
- A subscriber node that consumes the topic for slip detection

**Option B — Keep unused (immediate priority is normal-force contact detection)**

Acknowledge the data exists but do not invest engineering time until normal-force-based contact detection is deployed and found insufficient.

## 5. Recommendation

**Defer to future work.** The normal-force contact detection approach currently under development is sufficient for the initial grasping pipeline deployment. Tangential force channels exist and are fully parsed — they can be used later if:

1. Normal-force contact detection proves inadequate in practice (false positives, missed contacts).
2. Slip events become a reliability concern during grasp-and-hold operations.
3. Grasp quality metrics requiring full contact wrench become a priority.

No code changes are recommended at this stage. This document serves as the reference for future developers who need to access the existing tangential force data.
