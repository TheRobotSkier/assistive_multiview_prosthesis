# Current Issues — Context & Operator Answers

This document captures the project owner's answers to targeted questions about
the multiview prosthesis pipeline, plus the engineering insights that emerged
from cross-referencing those answers with the log/bag analysis. It is written
so that a developer or analyst **without full project context** can understand
the system's intended behaviour, the current failure modes, and the operator's
priorities.

---

## System Overview (for context)

The system tracks two cameras (a "head" camera and an "arm" camera) in 3D
space relative to a fixed ArUco marker (ID 0) on a table. Each camera runs
OpenVINS (visual-inertial odometry) on a Jetson, which fuses IMU + camera
data to estimate pose. A Python node (`aruco_marker_pose_node`) layers
ArUco marker corrections on top of the OpenVINS estimate to keep drift in
check. Pose data, compressed images, and compressed depth are relayed over
a Cat5e cable to a host laptop, which decompresses them and reconstructs
coloured point clouds locally.

**The core problem right now:** the arm camera's VIO estimate diverges
runaway — within seconds it can report being hundreds of metres from the
marker despite physically being under 1 metre away. The head camera is
usually more stable but not immune.

---

## Q1: What are the known-good target rates?

**Operator answer:**

> We relay at ~5 fps on depth and rgb. I can't remember if odom is also
> throttled on the jetson side. This was done since the connection was
> highly congested. I have been considering adjusting the fps to something
> like 15 fps, but it is a lower priority until things are running more
> smoothly. The relevant logic is in the relay node on the jetson side.
>
> I believe we also have some debounce logic on the marker updates, though
> that might purely be in what is sent to the host laptop, since the marker
> updates work by a different path on the jetson.
>
> Points should match the fps of the depth and rgb roughly. It is a problem
> that the points are not showing up, and I would love for that to be fixed.
> The whole idea of sending depth over the actual pointclouds was to reduce
> congestion on the connection, which seems to be one of the reasons we are
> actually getting a much better connection than we used to (with e.g. 0
> images and 0 pointclouds).

**Engineering insight:**

- The depth+rgb relay throttle (~5 fps) is a deliberate congestion-reduction
  measure, confirmed working. The bag analysis shows depth arriving at
  2.8–4.8 Hz and rgb at 4.4–4.7 Hz, which matches.
- Odom is **not** throttled — it arrives at ~38–42 Hz on both head and arm,
  which is the native OpenVINS output rate.
- The pointcloud architecture (send compressed depth+rgb, reconstruct
  points on the host) is sound and intentional. The 0-points problem is a
  host-side sync/wiring issue, not a design flaw.
- The `[DIAG-PC]` block added to `pipeline_diagnostics_node.py` will
  localise exactly where the pointcloud chain breaks (input loss vs
  decompress failure vs sync drop).

---

## Q2: What is the expected steady-state `pos_norm` range?

*(pos_norm = the magnitude of the OpenVINS position estimate, reported in
the `[DIAG]` heartbeat line. It indicates how far the estimator thinks the
camera is from its own origin.)*

**Operator answer:**

> I am actually not sure what pos_norm refers to. The general movement of
> the cameras in real life is a max of 1–2 m in most of the recorded bags.
>
> I think there is a combined issue with heavy drift from the IMU, and
> potentially not much of a way to correct it. The IMU should be calibrated
> well with Kalibr, so it is weird where the excessive drift comes from
> exactly — if it is a visual odom thing or an IMU thing.
>
> Marker updates seem to be able to keep the drift under control in some
> settings, but others not so much.

**Engineering insight:**

- `pos_norm` is **not** "distance from marker 0." It is the magnitude of
  the raw OpenVINS global-frame position vector. In a healthy run it should
  stay small (sub-metre to a few metres), reflecting how far the camera has
  moved from where OpenVINS initialized.
- The arm VIO reporting `pos_norm=19718m` is catastrophic divergence, not
  a coordinate-frame artifact.
- The drift source ambiguity (visual vs IMU) is the key open question. The
  new `[DRIFT-INCIDENT]` telemetry (with `pos_slope`) will help distinguish
  gradual IMU bias drift from sudden visual-initiated jumps.

---

## Q3: What is the expected head-arm relative distance?

**Operator answer:**

> Probably no more than 1.5 m in real test conditions. Right now a lot of
> tests are just us moving the cameras around manually, so they are not
> fixed to either head or arm. Maybe even 1 m, but at least in that kind
> of magnitude.

**Engineering insight:**

- The bag analysis showed head-arm relative distance ranging from 0.26 m to
  19,504 m — the latter is pure divergence, not physical separation.
- A healthy run should show head-arm distance in the 0.2–1.5 m range with
  low variance. The `std/mean=1.78` instability metric confirms the arm
  branch is the source of the wild swings.
- This metric is a strong pass/fail candidate for automated validation.

---

## Q4: Is `marker_map` globally stable, or can it reanchor/drift?

**Operator answer:**

> Marker ID 0 is intended to be used as an origin. The idea is that it is
> never moved during a run. It can move across runs, but often it is
> situated flat on a table, with us moving the cameras around it.
>
> It seems to be a bit hit and miss — we do see reanchoring, but sometimes
> the reanchoring just moves the frame, and then the camera immediately
> drifts off. The drift seems to accelerate in speed over time, and this
> can happen very quickly. In a few seconds the whole system can go from
> standing still to moving at completely unrealistic speeds.

**Engineering insight:**

- `marker_map` (the frame anchored to marker 0) is intended to be globally
  stable within a run. Reanchoring should snap the estimate back to truth,
  not introduce new drift.
- The "reanchor then immediate drift-off" pattern suggests the reanchor
  corrects position but the underlying velocity/bias estimate remains
  corrupted, so the system immediately diverges again.
- The drift acceleration ("standing still to unrealistic speeds in
  seconds") is characteristic of a positive-feedback loop in the estimator
  — once velocity estimate goes wrong, it feeds bad prediction into the
  next update, compounding the error.

---

## Q5: What causes large arm jumps?

**Operator answer:**

> Dedicated jumps often follow the pattern of the system drifting off and
> moving at high speeds, then reanchoring happens, moving the system from
> like 100s of metres away back to marker ID 0. I feel like that is good
> behavior though, so the real issue is that it can even move 100s of
> metres away from the marker despite in reality being under 1 m from it.
>
> As I understand this issue comes from VIO, not sure where specifically,
> which is really a big issue for us.

**Engineering insight:**

- This confirms the reanchor is **compensatory**, not the root cause. The
  big jumps visible in TF and bag analysis (marker_map -> arm_imu jumping
  0.5 m repeatedly, 21,680 total TF jumps) are the reanchor snapping back.
- The real root cause is upstream in the OpenVINS estimator allowing
  runaway divergence before the marker correction can catch it.
- The 1,462 chi2 marker rejections (mean val 960 vs gate 10.83) show the
  estimator state is so far off that marker measurements look statistically
  impossible — the gate rejects them, so no correction gets through, and
  divergence continues unopposed.

---

## Q6: Which startup failure is more "normal"?

**Operator answer:**

> The startup sequence is a bit weird. We start the system on the host
> side, then the jetson side, and then we initialize OpenVINS. One reason
> why no odom exists initially is likely just that we have not yet
> initialized. This is expected, but in the logs it reads as a crucial
> error. Could be nice to fix it in the logs, but to us it is expected.
>
> The bigger problem is that things drift off. We have also had issues with
> congestion and things not being available; there might still be bugs in
> relation to a large refactor we did on this, but it seems largely
> resolved.

**Engineering insight:**

- "No OpenVINS odom received yet" during startup is **expected** and should
  not be treated as a critical error. The analyzer now classifies it under
  a "Startup Phase Classification" section that marks it as expected during
  bring-up.
- The startup sequence (host → jetson → OpenVINS init) means there is an
  inherent window where odom is absent. The new `[STARTUP] phase=odom_ready`
  event will let the analyzer distinguish "startup expected" from
  "steady-state failure."
- The startup grace period in `aruco_marker_pose_node` (relaxed VIO health
  checks for N seconds after first marker detection) is the right mechanism
  for this.

---

## Q7: Should corrections be conservative or aggressive?

**Operator answer:**

> Very good question. I am not totally sure. We do want smooth velocity
> estimation, but aggressive position recovery. Velocity is used to
> propagate pose and find hitpoints — if it jumps around unrealistically
> that will be a problem.
>
> We want fast position recovery to avoid this really unrealistic and
> excessive drift, and it seems like fast recovery sometimes can make VIO
> restabilize. Also position needs to be precise more than smooth for
> point cloud fusion, and point cloud localization (as a start pose for
> twist propagation).

**Engineering insight:**

- This is a two-objective optimization: **aggressive position correction**
  (snap back to truth fast) but **smooth velocity** (don't corrupt the
  propagation step).
- The current reanchor logic zeros twist on large reanchors
  (`zero_twist_on_large_reanchor`), which is correct for preventing
  velocity spikes — but it does not fix the underlying corrupted velocity
  estimate that caused the divergence.
- The chi2 gate (currently rejecting 1,462 corrections) is too tight when
  the estimator is already diverged. A recovery-mode gate (wider when VIO
  is invalid) would let corrections through to pull the estimate back.

---

## Q8: Which metric do you trust most as a "truth signal"?

**Operator answer:**

> Right now it seems speed is indicative. Real speed is much lower than
> failure-mode drift. It changes whether it is the head or the arm that
> drifts off. We have not had a stable run in a long time at this point.
>
> I would imagine that overall pose from marker to current pose would be a
> better metric once the system hopefully stabilizes. Like how much would
> the system correct, or how far from predicted pose (only taking the
> marker into account) to the actual pose.

**Engineering insight:**

- Speed plausibility is currently the most reliable failure detector
  (real movement < 2 m/s; failure mode > 100 m/s). This is already encoded
  in the VIO health check (`max_linear_velocity_mps`).
- The operator's proposed metric — "distance between marker-predicted pose
  and actual pose" — is essentially the **marker innovation** (the residual
  between what the marker says the pose should be and what OpenVINS
  estimates). This is exactly what the chi2 gate measures.
- Once the system stabilizes, tracking the marker innovation distribution
  over time will be the best health metric. The `[MARKER_REJECT]` logging
  already captures when this exceeds the gate.

---

## Q9: Which warnings are usually benign/noise?

**Operator answer:**

> I am actually not sure. I have stopped looking as much at the logs after
> we made the analysis and heavy debug log setup.

**Engineering insight:**

- The operator has shifted to relying on the post-run analysis reports
  rather than reading raw logs. This means the analysis tooling is now the
  primary diagnostic surface — its accuracy and completeness are critical.
- Known benign noise:
  - "No OpenVINS odom received yet" during startup (expected).
  - `/tf_static` transport-latency warnings (latched topic artifact, now
    exempted via `LATCHED_TOPICS`).
  - The 21,718 WARN lines from `pipeline_diagnostics_node` are mostly TF
    jump warnings — noisy but informative when aggregated.

---

## Q10: What does a "pass" run look like?

**Operator answer:**

> - Camera-to-camera pose based on marker is super low (mm precision).
> - mm-precision localization to get highly accurate pointcloud fusion.
> - Smooth / realistic velocity/twist for human movement that is also
>   accurate.
> - Actually getting pointclouds and fused pointclouds as output would be
>   a win.

**Engineering insight:**

These four goals define the pass criteria. Translating to measurable gates:

| Goal | Measurable gate |
|------|----------------|
| mm-precision camera-to-camera pose | Head-arm relative distance std < 5 mm; marker innovation < 5 mm |
| Accurate pointcloud fusion | `/jetson/*/points` non-zero at target fps; fusion node producing `/fused_pointcloud` |
| Smooth realistic velocity | Max speed < 2 m/s; no velocity warnings; twist propagation stable |
| Pointclouds as output | Both `/jetson/head/points` and `/jetson/arm/points` > 0 Hz; `/fused_pointcloud` > 0 Hz |

---

## Key Takeaways

1. **The #1 problem is VIO runaway divergence**, not reanchoring, not
   pointcloud sync, not network congestion. Reanchoring is the system
   trying to compensate. The divergence originates in the OpenVINS
   estimator (visual or IMU path — still unknown).

2. **The pointcloud absence is a separate, fixable issue.** The depth+rgb
   data arrives on the host; the break is in the host-side sync/reconstruction
   chain. The new `[DIAG-PC]` instrumentation will pinpoint the exact stage.

3. **Startup "no odom" warnings are expected** and should not be treated as
   critical. The analyzer now separates startup-phase from steady-state.

4. **The analysis tooling is the primary diagnostic surface** — the operator
   relies on reports, not raw logs. Report accuracy and completeness are
   high priority.

5. **Correction policy should be dual-mode**: aggressive position recovery
   when VIO is invalid (wide gate, fast reanchor), conservative when valid
   (tight gate, protect smoothness).
