# Localization Rework V6 — Master Phase Index

**Date:** 2026-06-13
**Parent plan:** `plans/2026-06-13-localization-rework-plan-v6.md`
**Status:** Planning complete. Ready for execution.

---

## Overview

The V6 localization rework is decomposed into 5 sequential phases (0–4) with internal parallelism. Each phase has its own detailed plan file. This index provides the dependency graph, parallelism strategy, and execution order.

---

## Phase Plans

| Phase | Plan File | Objective | Est. Time | Parallel Agents |
|-------|-----------|-----------|-----------|-----------------|
| **0** | `plans/2026-06-13-localization-v6-phase0-environment.md` | Add gtsam + open3d to Docker, rebuild, verify | 30–45 min | 1 (serial) |
| **1** | `plans/2026-06-13-localization-v6-phase1-foundation.md` | Pure-logic libraries + message definitions | 2–3 days | 3 (parallel) |
| **2** | `plans/2026-06-13-localization-v6-phase2-ros-nodes.md` | Keyframe buffer node + MobileSAM server | 3–4 days | 2 (parallel) |
| **3** | `plans/2026-06-13-localization-v6-phase3-tsdf-sift.md` | TSDF fusion node + SIFT feature node | 3–4 days | 2 (parallel) |
| **4** | `plans/2026-06-13-localization-v6-phase4-integration.md` | Twist integration + GTSAM node + launch + config + E2E tests | 2–3 days | 1 (serial) |

**Total estimated time:** 11–15 days (wall-clock with parallelism: ~9–12 days)

---

## Dependency Graph

```
Phase 0 (environment)
  │
  ▼
Phase 1 (foundation)  ──── 3 parallel agents
  │   Task A: sensor_fusion_msgs
  │   Task B: cloud_utils (pure numpy)
  │   Task C: se3_helpers + factor_graph + umeyama (pure logic)
  │
  ▼
Phase 2 (ROS nodes)   ──── 2 parallel agents
  │   Task A: keyframe_buffer node (needs 1B)
  │   Task B: MobileSAM server (needs nothing)
  │
  ▼
Phase 3 (TSDF + SIFT) ──── 2 parallel agents
  │   Task A: tsdf_fusion node (needs 1B, 2A, 2B)
  │   Task B: cross_camera_features node (needs 1B, 1C)
  │
  ▼
Phase 4 (integration) ──── 1 serial agent
      Task A: twist propagation TSDF integration
      Task B: gtsam_tracker ROS node
      Task C: launch file updates
      Task D: config additions
      Task E: end-to-end testing
```

---

## Critical Path

The longest dependency chain that determines minimum wall-clock time:

```
Phase 0 → Phase 1 Task B (cloud_utils) → Phase 2 Task A (keyframe_buffer)
         → Phase 3 Task A (tsdf_fusion) → Phase 4 Task E (integration testing)
```

The TSDF fusion node is the most complex component and sits at the end of the critical path. Everything before it must be solid.

---

## Parallelism Strategy

### Phase 1 (3 agents)
- **Agent 1:** Task A (`sensor_fusion_msgs`) — quick, ~1 hour, then assists elsewhere.
- **Agent 2:** Task B (`cloud_utils`) — ~1 day, pure numpy, fully testable on host.
- **Agent 3:** Task C (`se3_helpers` + `factor_graph` + `umeyama`) — ~1.5 days, needs gtsam (in-container).

### Phase 2 (2 agents)
- **Agent 1:** Task A (`keyframe_buffer` node) — ~2.5 days, needs cloud_utils from Phase 1.
- **Agent 2:** Task B (MobileSAM server) — ~1.5 days, fully independent, Docker-only.

### Phase 3 (2 agents)
- **Agent 1:** Task A (`tsdf_fusion` node) — ~2 days, needs keyframe_buffer + MobileSAM + cloud_utils.
- **Agent 2:** Task B (`cross_camera_features` SIFT node) — ~1.5 days, needs cloud_utils + se3_helpers + umeyama.

### Phase 4 (1 agent)
- Serial integration. All components exist; this phase wires them together and validates.

---

## Hardware-Free Validation Strategy

The core principle (from the existing `test_pointcloud_fusion.py` pattern): **separate pure logic from ROS plumbing, test pure logic with synthetic data.**

| Component | Validation Method | Hardware Needed? |
|-----------|-------------------|------------------|
| `cloud_utils` | Synthetic numpy arrays (projection, masking, depth rasterization) | No |
| `se3_helpers` | Known transforms, round-trip tests | No |
| `factor_graph` | Synthetic trajectories fed to GTSAM smoother | No |
| `umeyama` | Known R, t + noise, verify recovery | No |
| `keyframe_buffer` logic | Mock poses, spatial gate, ring buffer, ROI query | No |
| `tsdf_fusion` core | **Synthetic scene test** (cylinder at known position, mock SAM mask, verify recovery < 2cm) | No |
| SIFT matching | Synthetic image pair with known homography | No |
| MobileSAM server | Sample image + click → verify mask shape | No |
| GTSAM node | Mock odom publishers | No |
| Full pipeline | `mock.launch.py` + rosbag replay | No |
| **Live ArUco + head odom** | **Requires hardware session** | **Yes** |

**Key insight:** If all the pure-logic tests pass (especially the TSDF synthetic scene test), the probability of the ROS pipeline working is high. The remaining risk is live data quality (ArUco availability, head odom, actual cloud organization), which requires the fresh comprehensive bag (V6 §12).

---

## Rosbag Evidence Integration

The two existing rosbags informed the V6 plan and the phase decomposition:

| Bag Finding | Phase Impact |
|-------------|-------------|
| Clouds are `height==1` (unorganized) | Phase 1 Task B: unorganized-safe design is default. Phase 3 Task A: depth rasterization is the primary path. |
| Keyframe ~2.7 MB (not 5.5 MB) | Phase 2 Task A: memory budget relaxed to ~320 MB. |
| Images at 6 Hz (not 30 Hz) | Phase 3 Task B: SIFT processes every frame, no decimation. |
| Arm odom structure confirmed | Phase 1 Task C: between-factor noise model validated. |
| Head odom absent in bags | Phase 4 Task B: `head_pose_source: "tf"` fallback added. |
| ArUco topics absent in bags | Phase 4 Task B: graceful degradation when ArUco is absent. |

**Caveat:** The bags are from an older system with selective recording. Topic absences are NOT proof of absence on the live system. All "absent topic" findings are treated as "needs live confirmation," not "broken."

---

## Execution Checklist

Before starting execution, confirm:

- [ ] V6 plan (`plans/2026-06-13-localization-rework-plan-v6.md`) is approved.
- [ ] This master index is approved.
- [ ] Agent assignments for each phase are decided.

Execution order:

1. **Execute Phase 0** (`plans/2026-06-13-localization-v6-phase0-environment.md`) — single agent, must complete fully.
2. **Execute Phase 1** (`plans/2026-06-13-localization-v6-phase1-foundation.md`) — 3 parallel agents. Verification gate: all pure-logic tests pass.
3. **Execute Phase 2** (`plans/2026-06-13-localization-v6-phase2-ros-nodes.md`) — 2 parallel agents. Verification gate: keyframe_buffer + MobileSAM server work in mock mode.
4. **Execute Phase 3** (`plans/2026-06-13-localization-v6-phase3-tsdf-sift.md`) — 2 parallel agents. Verification gate: TSDF synthetic scene test passes, SIFT node publishes on mock data.
5. **Execute Phase 4** (`plans/2026-06-13-localization-v6-phase4-integration.md`) — 1 agent. Verification gate: full mock pipeline runs, bag replay validates keyframe_buffer + gtsam_tracker.

**After Phase 4:** Record the fresh comprehensive bag (V6 §12) on hardware, then replay it through the full pipeline as the final pre-deployment validation.

---

## Risk Summary

| Risk | Phase | Mitigation |
|------|-------|------------|
| gtsam wheel unavailable for Python 3.12 | 0 | Build from source; add libboost-all-dev |
| Service generation in ament_python | 2 | Define services in sensor_fusion_msgs (CMake) |
| SIFT fails across viewpoints | 3 | Degrade gracefully; SuperPoint is future upgrade |
| ArUco topics absent on live system | 4 | Graceful degradation; odom+visual only |
| Head odom absent on live system | 4 | TF fallback (`marker_map → head_imu`) |
| TSDF artifacts from sparse clouds | 3 | DBSCAN cleanup; multi-view fusion fills gaps |
| Stale-bag assumptions mislead | All | Fresh comprehensive bag recorded before deployment |

The single highest-impact risk is **live ArUco availability** — it's the primary drift-correction source and cannot be verified without hardware. The GTSAM tracker is designed to degrade gracefully (odom + visual factors only) if ArUco is absent, but drift performance will be worse.