# MIA Hand Force Threshold Calibration

## What the Raw Values Represent

The MIA hand embeds strain gauges in each finger's flexion joint. The values
published on `/joint_states.effort[]` are **raw ADC readings** from those
strain gauges — integer values typically in the range 0–400 during normal
operation. They are **not calibrated to Newtons** and there is no known linear
conversion factor.

| Finger | Joint name       | Typical idle value (no load) |
|--------|------------------|------------------------------|
| Thumb  | `j_thumb_fle`    | ~215                         |
| Index  | `j_index_fle`    | ~208                         |
| MRL    | `j_mrl_fle`      | ~260                         |

These values were recorded on one specific hand (FTDI FTAO4Z0Y). Different
hands — and the same hand over time — may drift due to temperature, wear, or
firmware updates.

## Calibration Script

### Prerequisites

- ROS 2 environment sourced
- MIA hand driver running and publishing `/joint_states`
- Hand at rest: fully open, no objects touching the fingertips

### Running

```bash
ros2 run prosthesis_launch calibrate_mia_force_thresholds.py
```

Add to your `setup.cfg` / `setup.py` entry point:

```
[console_scripts]
calibrate_mia_force_thresholds = scripts.calibrate_mia_force_thresholds:main
```

Or run directly:

```bash
python3 scripts/calibrate_mia_force_thresholds.py --duration 10
```

**Arguments:**

| Flag            | Default | Description                                  |
|-----------------|---------|-----------------------------------------------|
| `--duration` / `-d` | 5.0 s | How long to record idle force samples.        |
| `--output` / `-o`   | _(stdout)_ | Future: file path for saving the YAML snippet. Redirect stdout for now. |

### What It Does

1. Subscribes to `/joint_states` and collects every `effort[]` entry for
   `j_thumb_fle`, `j_index_fle`, `j_mrl_fle`.
2. Records for the specified duration while you keep the hand idle.
3. Computes per-finger statistics: min, max, mean, standard deviation.
4. Derives recommended thresholds:
   - **contact_force_threshold** = mean + 3 * stddev (3-sigma above idle noise)
   - **contact_force_spike_threshold** = 2 * stddev (detect sudden change above baseline)
5. Uses the maximum thresholds across all three fingers as the single value
   per profile (since the current config uses one value per profile, not per finger).
6. The `strong` profile values are 2x the `soft` profile values to require
   more force before stopping.

### Example Output

```
============================================================
  MIA Hand Force Calibration Report
============================================================

  Thumb (50 samples):
    min               :    207.0
    max               :    226.0
    mean              :    215.3
    stddev            :      4.2
    contact_threshold :    227.9
    spike_threshold   :      8.4

  Index (50 samples):
    min               :    200.0
    max               :    219.0
    mean              :    208.1
    stddev            :      3.8
    contact_threshold :    219.5
    spike_threshold   :      7.6

  Mrl (50 samples):
    min               :    250.0
    max               :    274.0
    mean              :    260.7
    stddev            :      5.1
    contact_threshold :    276.0
    spike_threshold   :     10.2

------------------------------------------------------------
  Suggested YAML snippet for prosthesis_config.yaml
------------------------------------------------------------

  profiles:
    soft:
      contact_force_threshold:       276.0
      contact_force_spike_threshold:  10.2
    strong:
      contact_force_threshold:       552.0
      contact_force_spike_threshold:  20.5

============================================================
  Raw values are ADC units — NOT Newtons.
  See docs/mia_force_calibration.md for interpretation.
============================================================
```

## Interpreting and Applying Results

1. **Paste** the YAML snippet into `config/prosthesis_config.yaml` under
   `force.profiles.soft` and `force.profiles.strong`, replacing the existing
   values.

2. **Verify** with a simple grasp test:
   - Start the pipeline in soft profile.
   - Close the hand on a compliant object (e.g., a foam ball).
   - The fingers should stop shortly after contact.
   - If fingers stop too early (idle noise triggers false contact), **increase**
     `contact_force_threshold` by 10–20%.
   - If fingers don't stop until well after contact (overshoot), **decrease**
     `contact_force_threshold`.

3. **Tune per-finger thresholds** (advanced):
   - The current `grasp_proximity_controller_node` applies the same profile
     values to all fingers. The calibration script reports the maximum across
     fingers as a safe default.
   - If one finger is consistently over- or under-sensitive, you can modify
     `force_aware_closure.py` to accept per-finger thresholds.

## Important Notes

- **Re-calibrate** after sensor drift, firmware updates, or significant
  temperature changes.
- The idle force values are hand-specific. If you swap the MIA hand hardware,
  re-run the calibration.
- The `strong` profile uses **2x** the soft thresholds by default. You may
  adjust this multiplier if a different sensitivity is desired.
- Threshold units are dimensionless ADC integers. Do not interpret them as
  Newtons or any physical force unit.
