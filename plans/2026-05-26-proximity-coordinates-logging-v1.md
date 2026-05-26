# Plan: Report Proximity as Coordinates Instead of Scalar Distance

## Objective

Change the proximity controller's log messages so that instead of only reporting a scalar distance (e.g. `dist=0.338 m`), they also report the **coordinate offset** between the current grasp contact position and the planned grasp contact position (e.g. `offset=[+0.12, -0.08, +0.30] m`). This lets you see *how far away* in each axis, making it possible to calibrate the `grasp_contact_offset` in `prosthesis_config.yaml` by observing the directional breakdown.

## Current Behavior

The log lines in `host-log-v37.txt` look like:
```
FAR mode (dist=0.338 m): partial closure + wrist
NEAR mode (dist=0.050 m): full closure
```

Only a scalar Euclidean distance is shown. There is no directional information.

The distance is computed at `grasp_proximity_controller_node.py:262-263` by calling `_compute_proximity_distance()`, which internally already computes `dx`, `dy`, `dz` at lines 311-313 but only returns `math.sqrt(dx*dx + dy*dy + dz*dz)`.

## Implementation Plan

- [ ] **1. Modify `_compute_proximity_distance` to also return the per-axis offset.** In `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:301-314`, change the method to return both the scalar distance and the `(dx, dy, dz)` tuple. The signature becomes `-> tuple[float, tuple[float, float, float]]`. The `dx`, `dy`, `dz` values are already computed at lines 311-313 — just return them alongside the distance instead of discarding them.

- [ ] **2. Update the call site in `_control_loop`.** At `grasp_proximity_controller_node.py:262-263`, unpack both values: `dist, (dx, dy, dz) = self._compute_proximity_distance(...)`. This is the only call site, so no other code needs updating.

- [ ] **3. Update the four log messages in `_control_loop` to include coordinate offsets.** All four log lines that currently show only `dist=` should also show the offset vector:
  - Line 270: `Left near zone` — add `offset=[{dx:+.3f}, {dy:+.3f}, {dz:+.3f}]`
  - Line 277: `Entered near zone` — add `offset=[{dx:+.3f}, {dy:+.3f}, {dz:+.3f}]`
  - Line 285: `NEAR mode` — add `offset=[{dx:+.3f}, {dy:+.3f}, {dz:+.3f}]`
  - Line 290: `FAR mode` — add `offset=[{dx:+.3f}, {dy:+.3f}, {dz:+.3f}]`

  Example new format: `FAR mode (dist=0.338 m, offset=[+0.120, -0.080, +0.300]): partial closure + wrist`

## Verification Criteria

- [ ] After rebuilding and running, the proximity controller log lines show both `dist=` and `offset=[x, y, z]` values.
- [ ] The `offset` values are signed (positive/negative), indicating direction in each axis.
- [ ] The scalar `dist` still matches `sqrt(dx^2 + dy^2 + dz^2)` (no regression in distance computation).
- [ ] The `grasp_contact_offset` value from `prosthesis_config.yaml:142` (`[0.1543, -0.1485, -0.1352]`) is still applied correctly (no change to offset logic, only logging).

## Potential Risks and Mitigations

1. **Log line becomes longer and harder to read at a glance**
   Mitigation: The `offset=[x, y, z]` is appended after `dist=` so the scalar distance is still immediately visible. The signed `+.3f` format keeps it compact.

2. **Breaking any log-parsing scripts**
   Mitigation: The existing `dist=` substring and format are preserved; the offset is purely additive. Any regex matching `dist=\d+\.\d+` will still work.

## Alternative Approaches

1. **Log offset on a separate line**: Could log the offset as a separate throttled message. Rejected — adds noise and makes it harder to correlate distance with direction at the same timestamp.
2. **Replace distance entirely with coordinates**: Could drop `dist=` and only show `offset=[x, y, z]`. Rejected — the scalar distance is needed for quick mental comparison against the enter/exit thresholds, which are scalar values.
