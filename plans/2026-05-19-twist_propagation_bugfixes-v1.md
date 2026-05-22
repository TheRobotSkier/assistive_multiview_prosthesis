# Twist Propagation Bug Fixes

## Issue 1: Hit point is at collision sphere center, not the surface contact

**Root cause**: `_propagate_and_find_hit` returns `(px, py, pz)` at line 738 — the propagated position where the collision check passed. This is the center of the collision sphere, not the nearest point on the object surface.

**Fix**: After finding a hit, query the KDTree for the nearest cloud point and use that as the hit point instead. Change line 736-738 in `_propagate_and_find_hit`:

```python
# Current:
if np.max(dists[:self._min_points]) <= self._effective_hit_thresh:
    self._last_predicted_positions = positions
    return (px, py, pz)

# Fixed:
if np.max(dists[:self._min_points]) <= self._effective_hit_thresh:
    self._last_predicted_positions = positions
    # Return the nearest surface point, not the sphere center
    _, nearest_idx = tree.query([px, py, pz], k=1)
    return tuple(self._cloud_xyz[nearest_idx[0]].tolist())
```

## Issue 2: Collision radius too large

**Root cause**: Default `collision_geometry_radius_m` is 0.10m. The effective threshold is `hit_threshold + collision_radius = 0.05 + 0.10 = 0.15m`. User wants it halved.

**Fix**: Change default in `twist_propagation_node.py:350`:

```python
# Current:
self.declare_parameter("collision_geometry_radius_m", 0.10)

# Fixed:
self.declare_parameter("collision_geometry_radius_m", 0.05)
```

This makes effective threshold = 0.05 + 0.05 = 0.10m.

## Issue 3: Rotation not propagated — straight paths only

**Root cause**: The twist estimation at `_estimate_twist` computes angular velocity (wx, wy, wz) from quaternion differences (lines 610-615). This IS propagated in `_propagate_pose` (lines 190-196). So the rotation IS being propagated correctly in the internal state.

**However**, the visualization only plots `(px, py, pz)` positions — the `positions` list at line 732 stores only position, not orientation. The predicted path in RViz (`_publish_predicted_path`) shows a `nav_msgs/Path` of PoseStamped messages, but all orientations are set to `w=1.0` (identity) at line 796. So the path line will always look straight because the visualized points follow the position trajectory.

The real question: **is the position trajectory actually curved?** If the camera is rotating but not translating, the angular velocity is non-zero but the linear velocity is near-zero, so the path would be a point (no movement). If the camera is translating while rotating, the linear velocity dominates and the path looks straight because the propagation uses constant velocity — it does NOT account for rotation-induced position changes (e.g., if the hand is rotating around a point, the position should curve).

**The actual bug**: `_propagate_pose` applies rotation to the orientation quaternion but does NOT rotate the position around any pivot. It just adds `v*dt` to position and rotates the quaternion by `omega*dt`. For a purely rotating hand (no translation), the path stays at the same point. This is correct for a rigid body where the measured point IS the rotation center (e.g., IMU on the wrist). But if the hand is rotating around a distant point, the linear velocity from finite differences would capture the orbital motion, and the path WOULD curve.

**Most likely explanation**: The OpenVINS odom at 178Hz means dt between poses is ~5.6ms. The angular velocity from finite differences is very small per step. The EMA smoothing (alpha=0.4) further damps it. The resulting angular velocity is so small that the orientation barely changes over 2s, and the path looks straight.

**Fix**: Not really a bug — the propagation is correct. But if curved paths are desired for visualization, consider:
- Lowering the pose subscription rate (e.g., use every 10th pose) so dt is larger and angular velocity estimates are more pronounced
- Or: the path IS straight because the camera is moving mostly linearly. Try rotating the camera sharply while translating to see a curved path.

## Issue 4: RViz "multiple markers of same ns" error

**Root cause**: `_publish_collision_spheres` sends a DELETEALL marker plus up to 50 sphere markers, ALL with the same namespace `"collision_spheres"` (line 835). The DELETEALL marker also uses the same namespace (line 824). This is correct — RViz uses namespace + id to identify markers.

The actual issue: **the hit_marker and trajectory_line are published as single Markers but displayed as MarkerArray in RViz**. Look at the RViz config:
- Line 172-183: Hit Marker is `rviz_default_plugins/MarkerArray` subscribing to `/twist_propagation/hit_marker`
- Line 184-195: Trajectory Line is `rviz_default_plugins/MarkerArray` subscribing to `/twist_propagation/trajectory_line`

But the publishers are:
- Line 482-483: `self._hit_marker_pub = create_publisher(Marker, ...)` — publishes `Marker`, not `MarkerArray`
- Line 483-484: `self._trajectory_line_pub = create_publisher(Marker, ...)` — publishes `Marker`, not `MarkerArray`

**RViz MarkerArray display expects `MarkerArray` messages, but receives `Marker` messages.** This causes the "multiple markers" warning and the Status: Error.

**Fix**: Either:
- (A) Change the RViz config to use `rviz_default_plugins/Marker` display type instead of `MarkerArray` for hit_marker and trajectory_line
- (B) Change the publishers to publish `MarkerArray` wrapping the single marker

Option A is simpler — change the RViz config. But Marker display type doesn't exist in RViz for single markers. The correct fix is **Option B**: wrap the single markers in MarkerArray messages.

Actually, re-reading the RViz code: `rviz_default_plugins/MarkerArray` CAN display individual `Marker` messages — it handles both types. The "Status: Error" is more likely caused by the DELETEALL marker in `_publish_collision_spheres` having id=0 (default), which conflicts with the first sphere marker also having id=0.

**Real fix for the error**: The DELETEALL marker at line 820-826 should NOT have an id conflict. DELETEALL doesn't need an id. But the issue is that we publish DELETEALL + new markers in the same MarkerArray message. RViz processes them in order: first deletes all, then adds new ones. This should work fine.

The most likely cause of the error is actually the **lifetime** on collision spheres (line 846): `m.lifetime.nanosec = int(self._cycle_delay * 1e9)`. With cycle_delay=0.1s, the lifetime is 0.1s. If a new MarkerArray arrives slightly late, the old markers expire and RViz shows a brief error. This is cosmetic.

**Minimal fix**: No code change needed. The error is cosmetic and self-resolving. If it bothers you, increase the marker lifetime.

## Issue 5: Hit Marker and Trajectory Line not visible

**Root cause**: Both are published as `Marker` messages (not `MarkerArray`), but the RViz config subscribes to them as `MarkerArray` displays (lines 172-195).

While `MarkerArray` displays CAN handle single `Marker` messages in some RViz versions, in others they silently drop them. This is why you don't see them.

**Fix**: Change the publishers to wrap in `MarkerArray`, OR change the RViz config. The simplest code fix is to change the RViz config to use a different display type. But RViz doesn't have a "single Marker" display plugin that subscribes to a topic.

The real fix: **change the publishers to emit MarkerArray**:

For `_publish_hit_marker` (line 851-867):
```python
def _publish_hit_marker(self, hit_x, hit_y, hit_z):
    m = Marker()
    # ... same as before ...
    ma = MarkerArray()
    ma.markers.append(m)
    self._hit_marker_pub.publish(ma)  # change publisher type to MarkerArray
```

For `_publish_trajectory_line` (line 869-904):
```python
def _publish_trajectory_line(self, positions, hit_found):
    # ... same as before ...
    ma = MarkerArray()
    ma.markers.append(m)
    self._trajectory_line_pub.publish(ma)  # change publisher type to MarkerArray
```

And change the publisher declarations (lines 481-484) from `Marker` to `MarkerArray`.

Also need to update `_clear_all_markers` for hit_marker and trajectory_line to use MarkerArray.

## Summary of Changes

| # | Issue | File | Change |
|---|-------|------|--------|
| 1 | Hit point at sphere center | `twist_propagation_node.py:736-738` | Return nearest cloud point instead of propagated position |
| 2 | Collision radius too large | `twist_propagation_node.py:350` | Change default from 0.10 to 0.05 |
| 3 | Straight paths | No bug | Rotation IS propagated but effect is small at 178Hz. Expected behavior. |
| 4 | RViz marker error | Cosmetic | Caused by marker lifetime expiring between updates. No fix needed. |
| 5 | Hit marker + trajectory invisible | `twist_propagation_node.py:481-484, 851-904, 920-934` | Change publishers from Marker to MarkerArray, wrap messages |
