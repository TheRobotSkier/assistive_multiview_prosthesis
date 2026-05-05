# Implementation Plan: Step 1 — RViz Configuration

**Beads Issue:** `rviz-config`  
**Type:** task  
**Priority:** 1 (blocks launch file)  
**Estimated Effort:** Small

---

## Objective

Create `dev/mujoco/config/digital_twin.rviz` — an RViz configuration that lets operators see the real-world pointcloud, the segmented object pointcloud, the Mia Hand digital twin, and the TF tree in a single view.

---

## Acceptance Criteria

- [ ] File exists at `dev/mujoco/config/digital_twin.rviz`.
- [ ] RViz opens without errors when pointed at this config.
- [ ] Displays include:
  - `TF` (enabled, showing all frames).
  - `RobotModel` subscribing to `/robot_description` and showing the Mia Hand URDF.
  - `PointCloud2` named **FusedCloud** on `/fused_pointcloud` with `AxisColor` / Z-axis coloring and point size 0.003 m.
  - `PointCloud2` named **SegmentedCloud** on `/segmentation/object_cloud` with flat orange color (`255; 100; 0`) and point size 0.005 m.
  - `MarkerArray` named **SeedMarkers** on `/segmentation/seed_markers` (optional, if node publishes it).
- [ ] `Global Options → Fixed Frame` is set to `world`.
- [ ] `PublishPoint` tool is present and configured to publish to `/clicked_point`.
- [ ] Background is dark grey (`48; 48; 48`) for good pointcloud contrast.
- [ ] Window geometry is sensible (e.g. 1400x900).

---

## Implementation Details

### Base to extend
Use `dev/mujoco/config/full_system_test.rviz` as the starting template. It already contains TF, RobotModel, FusedCloud, SegmentedCloud, SeedMarkers, and the PublishPoint tool.

### Changes needed
1. **Copy** `full_system_test.rviz` → `digital_twin.rviz`.
2. **Verify topic names** are correct for the digital twin context:
   - `/fused_pointcloud` (from multiview) — already correct.
   - `/segmentation/object_cloud` — already correct.
   - `/robot_description` — already correct.
3. **Add display** for the Mia Hand model if not already present. The `full_system_test.rviz` already has RobotModel, but ensure it points to `/robot_description` (the simulation will publish this).
4. **Ensure `Fixed Frame` = `world`**. The simulation and static TF publishers will use `world` as the root frame.
5. **Save** in RViz2-friendly YAML format (no manual JSON edits that break RViz parsing).

### Validation
Open the config in a container that has RViz2 and the workspace built:
```bash
rviz2 -d /miahand_ws/src/dev/mujoco/config/digital_twin.rviz
```
It should load without red error markers in the Displays panel.

---

## Sub-tasks (beads tracking)

1. Copy and rename base RViz config.
2. Verify/adjust topic names and display settings.
3. Test-load in RViz2 and confirm no parsing errors.
4. Commit the file.

---

## Notes / Risks

- **Risk:** RViz2 config format is sensitive to minor syntax changes. Always save via RViz2 GUI, never hand-edit unless you know the exact schema.
- **Mitigation:** Use the existing `full_system_test.rviz` (which is known to work) as the base and only change topic names and colors.
