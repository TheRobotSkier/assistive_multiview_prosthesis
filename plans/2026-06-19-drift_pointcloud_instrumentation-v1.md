# Drift Root-Cause and Pointcloud Recovery — Step-by-Step Implementation Plan

## Objective

Create a simple, execution-ready plan to instrument the system so we can pinpoint the **first cause of drift** and isolate why **pointcloud output is missing** even when image/depth streams arrive.

## Implementation Plan

- [ ] **Step 1. Establish baseline run package (logs + bag + report outputs).**
  - Collect one known-bad run and preserve all artifacts (host log, jetson log, bag, analysis outputs).
  - Rationale: all new instrumentation needs one reference run to compare before/after.

- [ ] **Step 2. Add startup-phase boundary events in Jetson marker node.**
  - Emit structured events for: odom first-seen, VIO initialized, correction enabled.
  - Rationale: startup “no odom yet” is expected; this boundary prevents false escalation in analysis.

- [ ] **Step 3. Add side-specific VIO transition counters (head/arm).**
  - Track and emit: valid→invalid count, invalid→valid count, total invalid dwell time, max continuous invalid duration.
  - Rationale: transition timing reveals instability onset much better than end-of-run snapshots.

- [ ] **Step 4. Add innovation trend telemetry before reject/accept decisions.**
  - Record rolling summary windows for chi2, translation innovation norm, rotation innovation norm.
  - Rationale: identifies whether drift starts as gradual innovation growth or abrupt outlier bursts.

- [ ] **Step 5. Add reject-burst detection and structured incident events.**
  - Emit burst metrics for 1s and 5s windows by side and reject type.
  - Rationale: correlates marker rejection storms with immediate VIO degradation.

- [ ] **Step 6. Add reanchor effectiveness metrics.**
  - Log before/after residual magnitude and time-to-stabilize (until valid + bounded velocity).
  - Rationale: distinguishes healthy recovery from temporary snaps that immediately re-diverge.

- [ ] **Step 7. Add runaway detector telemetry.**
  - Emit structured events when implausible velocity and rapid pos_norm slope are detected.
  - Rationale: provides explicit first-alert trigger tied to your observed failure mode.

- [ ] **Step 8. Extend host analyzer with phase-aware severity policy.**
  - Split report into Startup vs Steady-State sections; downgrade expected pre-init warnings.
  - Rationale: prevents known expected startup conditions from obscuring real faults.

- [ ] **Step 9. Extend analyzer with drift-causality timeline section.**
  - Correlate in time: reject bursts → VIO transitions → runaway alerts → reanchors.
  - Rationale: creates actionable root-cause chain instead of disconnected symptom lists.

- [ ] **Step 10. Instrument pointcloud pipeline stage counters (per side).**
  - Add counters and rates for: depth_in, rgb_in, sync_match, points_out.
  - Rationale: isolates exactly where the pipeline breaks when points are absent.

- [ ] **Step 11. Add pointcloud sync-failure reason codes.**
  - Track reason counts: timestamp mismatch, queue overflow, stale frame, missing pair.
  - Rationale: verifies whether missing points are from synchronization vs lifecycle/QoS issues.

- [ ] **Step 12. Add pointcloud lifecycle/readiness markers.**
  - Emit node state events: started, subscribed, receiving input, producing output.
  - Rationale: quickly separates “node not active” from “active but dropping pairs.”

- [ ] **Step 13. Add analyzer section: POINTCLOUD CHAIN HEALTH.**
  - Report per-side stage rates, drop percentages, and dominant failure reason.
  - Rationale: makes pointcloud failure immediately visible and localizable in one section.

- [ ] **Step 14. Add operational pass/fail gates aligned to project goals.**
  - Include gates for: runaway velocity ceiling, invalid dwell threshold, points output non-zero, relative distance envelope.
  - Rationale: provides deterministic acceptance criteria for each run.

- [ ] **Step 15. Run A/B validation on at least one known-bad scenario.**
  - Compare pre-instrumentation and post-instrumentation outputs for clarity and root-cause resolution.
  - Rationale: ensures the added telemetry materially improves diagnosis quality.

- [ ] **Step 16. Optional follow-up: add known-good baseline profile.**
  - Encode expected ranges from stable runs once available.
  - Rationale: enables automatic anomaly scoring against healthy behavior.

## Verification Criteria

- [ ] Reports separate startup expected conditions from steady-state faults.
- [ ] Drift timeline includes first-trigger evidence (reject burst and transition context).
- [ ] Head and arm both show VIO transition metrics and invalid dwell durations.
- [ ] Pointcloud section identifies failing stage for each side when output is zero.
- [ ] At least one run yields a clear causal chain from onset to failure.
- [ ] Pass/fail gates produce a deterministic run verdict.

## Potential Risks and Mitigations

1. **Risk: Instrumentation adds excessive log volume.**  
   Mitigation: Use compact structured summaries and event throttling windows.

2. **Risk: New metrics still lack causal ordering across components.**  
   Mitigation: Standardize structured event timestamps and side labels, then correlate in one analyzer timeline.

3. **Risk: Pointcloud issue appears simple but is multi-factor.**  
   Mitigation: enforce stage-by-stage counters and reason coding to guarantee localization.

4. **Risk: Thresholds are tuned before enough evidence is collected.**  
   Mitigation: collect evidence first; tune only after stable causality signatures are visible.

## Alternative Approaches

1. **Drift-first only:** instrument only VIO/marker path first; defer pointcloud instrumentation.  
   Trade-off: faster drift diagnosis, slower pointcloud restoration.

2. **Pointcloud-first only:** restore points path first, then deepen drift telemetry.  
   Trade-off: immediate output recovery, but drift root cause remains uncertain.

3. **Balanced parallel (recommended):** lightweight drift and pointcloud instrumentation together.  
   Trade-off: moderate scope, highest end-to-end diagnostic value.