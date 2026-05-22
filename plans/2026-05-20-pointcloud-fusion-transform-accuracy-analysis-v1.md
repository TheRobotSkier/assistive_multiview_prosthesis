# Pointcloud Fusion — Transform Accuracy Analysis

**Date:** 2026-05-20  
**Version:** v4  
**Status:** Root Cause Identified

---

## The Problem: Wrong Frame Used as Bridge Anchor

The fused point clouds show "high precision, low accuracy" — the overlap between views is consistently offset/rotated rather than randomly noisy. This is a **systematic calibration error**, not a timing issue.

## Root Cause

The bridge uses `head_d435i_head_color_optical_frame_body_display` as the anchor frame with `anchor_frame_mode = "link"`. This means it computes:

```
T(head_cam0 → link) = T(head_cam0 → color_optical_frame_body_display)
```

But `body_display` is associated with the **color optical frame**, not the **link frame**. The RealSense frame hierarchy is:

```
link → color_frame → color_optical_frame
```

The extrinsic between `color_optical_frame` and `link` is:
- Translation: `[0, 0.015, 0]` (color_frame is 15mm offset from link)
- Rotation: `rpy(-π/2, 0, -π/2)` (optical convention: Z forward, Y down)

This means the bridge is off by:
- **15mm translation** (small but visible)
- **~90° rotation** (this is the dominant error — it causes the systematic misalignment you see)

## Evidence

The locked extrinsic values from the log:
```
head: LOCKED extrinsic head_cam0->head_d435i_head_link: t=[0.2211, -0.0271, -0.4051] q=[-0.3652, 0.6245, -0.5301, -0.4424]
arm:  LOCKED extrinsic arm_cam0->arm_d435i_arm_link: t=[0.4514, -0.0834, -0.1710] q=[0.0416, 0.7907, -0.5056, -0.3426]
```

These are large transforms (22cm and 45cm translations, significant rotations). If this were just the mounting extrinsic between two cameras on the same device, we'd expect small values (a few cm). The large values suggest the `body_display` frame is not in the right coordinate system.

## The Fix

Change `anchor_frame_mode` from `"link"` to `"optical"` for both cameras. The `"optical"` mode composes the anchor transform with the RealSense depth-optical-to-link extrinsic:

```
T(head_cam0 → link) = T(head_cam0 → body_display) × T(depth_optical → link)
```

Wait — that's also wrong. The `"optical"` mode uses `fallback_optical_frame` (which is `depth_optical_frame`), not `color_optical_frame`. And `body_display` is the color optical frame body, not the depth optical frame.

The correct composition should be:
```
T(head_cam0 → link) = T(head_cam0 → color_optical_body_display) × T(color_optical → link)
```

Where `T(color_optical → link) = T(link → color_optical)^{-1}` is available from the RealSense static chain.

## Implementation Plan

- [ ] **Task 1**: Add a new anchor_frame_mode `"color_optical"` that composes the anchor transform with `T(color_optical_frame → link)` from the RealSense static chain. This correctly converts from the color optical frame body to the link frame.

- [ ] **Task 2**: Alternatively (simpler), add a new parameter `anchor_to_link_frame` that specifies the RealSense frame to compose with. For the head camera, this would be `head_d435i_head_color_optical_frame`. The bridge would compute:
  ```
  T(head_cam0 → link) = T(head_cam0 → body_display) × T(color_optical → link)
  ```

- [ ] **Task 3**: Simplest approach — just look up `T(body_display → link)` directly from the TF tree and compose it. Since `body_display` is `color_optical_frame_body_display` and `link` is the RealSense link, the TF chain `body_display → link` goes through the color optical → color frame → link path. But `body_display` is NOT in the RealSense tree — it's in the OpenVINS tree. So we can't look up `body_display → link` directly.

  The correct approach is to use the known RealSense extrinsics:
  ```
  T(color_optical → link) = T(color_optical → color_frame)^{-1} × T(color_frame → link)^{-1}
  ```
  These are the same nominal static transforms the bridge already publishes.

- [ ] **Task 4**: Implement the composition. In `_resolve_bridge_transform_live`, when `anchor_frame_mode == "link"`, look up `T(color_optical_frame → link)` from the TF buffer and compose:
  ```python
  # anchor_frame is head_d435i_head_color_optical_frame_body_display
  # We need to convert from color_optical to link
  color_optical = "head_d435i_head_color_optical_frame"  # derived from anchor name
  color_to_link = self._lookup_matrix(color_optical, link_frame)
  if color_to_link is not None:
      return parent_to_anchor @ color_to_link, ...
  ```

- [ ] **Task 5**: Update config and rebuild.

## Verification Criteria

- Locked extrinsic values should show small translations (a few cm) since the OpenVINS camera and RealSense are on the same physical mount
- Fused point clouds from both cameras should overlap properly in RViz
- `dual` count should be consistently high with no systematic offset between views
