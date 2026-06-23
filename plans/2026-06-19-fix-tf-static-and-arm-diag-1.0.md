# Fix Two Analysis-Side Issues: /tf_static False-Positive & Hidden Arm DIAG Data

## Objective

Fix two verified analysis-engine bugs that produce misleading output:
1. **Issue A:** `analyze_bag.py` reports a false 14776ms "clock delta" on `/tf_static` because latched (TRANSIENT_LOCAL) topics are not exempted from the transport-latency computation.
2. **Issue B:** `analyze_log.py` hides the arm VIO row entirely when `markers_detected == 0`, making it appear as if the arm `[DIAG]` lines were never parsed — even though they ARE matched by the regex and correctly routed to `arm_markers`/`arm_corrections`. The rich diagnostic fields (`vio=INVALID`, `pos_norm=7.77m`) are captured by the regex but discarded.

**Verified root causes:**
- `analyze_bag.py:1240-1247` — iterates ALL topics for clock offset with zero latched-topic exemption (confirmed: no `LATCHED_TOPICS`, `TRANSIENT`, or `/tf_static` string exists anywhere in `analyze_bag.py`).
- `analyze_log.py:687-689` — `if markers == 0: print("—"); continue` hides the arm row. The regex `RE_VIO_DIAG` (line 104-107) captures 7 groups but the parser (line 362-377) only uses groups 1, 2, 7 — discarding `vio` (group 4), `pos_norm` (group 5), and `odom` (group 6).

**Related gap (same root cause as Issue A):** `analyze_log.py` has `LATCHED_TOPICS = {"/tf_static"}` (line 119) but only applies it in `print_timeline` (line 531-532). The plot path (line 783) and metrics path (line 876) do NOT filter latched topics, so they inherit the same false-positive.

---

## Implementation Plan

### Issue A: /tf_static Latency False-Positive (analyze_bag.py)

- [x] **A1.** Add a `LATCHED_TOPICS` constant near the top of `analyze_bag.py` (after the imports / constants section), matching the one in `analyze_log.py:119`. Value: `LATCHED_TOPICS = {"/tf_static"}`. Include a comment explaining that TRANSIENT_LOCAL durability topics publish once at boot and their frozen timestamps produce meaningless transport deltas.
  - **Rationale:** Centralizes the exemption set so all clock-offset code paths can reference it consistently.

- [x] **A2.** In the clock-offset computation loop at `analyze_bag.py:1240-1247`, add a guard to skip latched topics. Insert `if ti.name in LATCHED_TOPICS: continue` immediately after the existing `if len(ti.stamps) < 2 ...` check at line 1241.
  - **Rationale:** This is the exact loop that produces the "Peak message clock delta = 14776ms on '/tf_static'" callout at line 1277. Excluding `/tf_static` from the candidate set means the peak delta will reflect a real transport topic instead.

- [x] **A3.** Scan the rest of `analyze_bag.py` for any other clock-offset / latency / transport-delta loops that iterate `rep.topics.values()` and apply the same `LATCHED_TOPICS` guard to each. Specifically check the bandwidth cross-reference section (line 1287+) and any per-topic gap/latency tables.
  - **Rationale:** Ensures no other code path inherits the same false-positive. Bandwidth calculations are fine (latched topics DO consume bandwidth), but clock-offset / transport-latency paths must be exempted.

### Issue A (related): analyze_log.py Plot & Metrics Paths

- [x] **A4.** In `analyze_log.py` `print_plot` clock-offset extraction (line 782-784), add `and topic not in LATCHED_TOPICS` to the list comprehension filter, matching the pattern already used in `print_timeline` (line 531-532).
  - **Current:** `vals = [abs(v[2]) for v in b.rates.values() if v[2] != -1.0]`
  - **Target:** `vals = [abs(v[2]) for topic, v in b.rates.items() if v[2] != -1.0 and topic not in LATCHED_TOPICS]`
  - **Rationale:** Without this, the plot's "Max |clock_off|" axis can spike to ~15s from `/tf_static`, compressing the real signal.

- [x] **A5.** In `analyze_log.py` `to_metrics` clock-offset extraction (line 875-876), apply the same `LATCHED_TOPICS` filter.
  - **Current:** `vals = [abs(v[2]) for v in b.rates.values() if v[2] != -1.0]`
  - **Target:** `vals = [abs(v[2]) for topic, v in b.rates.items() if v[2] != -1.0 and topic not in LATCHED_TOPICS]`
  - **Rationale:** The `clock_offset_peak_s` metric feeds automated pass/fail thresholds; a `/tf_static` spike would trigger false failures.

### Issue B: Hidden Arm DIAG Data (analyze_log.py)

- [x] **B1.** Extend the `VioAnchorStats` dataclass (line 150-158) with four new fields to capture the last-seen diagnostic snapshot:
  - `head_vio_status: str = "unknown"` — last `vio=` value for head
  - `head_pos_norm: float = 0.0` — last `pos_norm=` value for head
  - `arm_vio_status: str = "unknown"` — last `vio=` value for arm
  - `arm_pos_norm: float = 0.0` — last `pos_norm=` value for arm
  - **Rationale:** The regex already captures these (groups 4 and 5); we just need to store them. "Last seen" is the right semantic because markers/corrections are cumulative sums while vio/pos_norm are instantaneous snapshots — the final value is the most diagnostic.

- [x] **B2.** Update the DIAG parser block (line 362-377) to extract and store the new fields. After the existing `locked = vm.group(7)` assignment, add:
  - `vio_status = vm.group(4)` (the `vio=valid` or `vio=INVALID` token)
  - `pos_norm = float(vm.group(5))` (the position norm in meters)
  - In the arm branch: set `rep.vio_stats.arm_vio_status = vio_status` and `rep.vio_stats.arm_pos_norm = pos_norm`
  - In the head branch: set `rep.vio_stats.head_vio_status = vio_status` and `rep.vio_stats.head_pos_norm = pos_norm`
  - **Rationale:** Wires the already-captured regex groups into the data model. No regex change needed — `RE_VIO_DIAG` already matches all 7 groups correctly against the actual log format.

- [x] **B3.** Update the `print_vio_yield` table renderer (line 673-701) to:
  1. **Remove the early-return suppression** at line 677 (`if vs.head_markers == 0 and vs.arm_markers == 0: return`) — replace with a check that only returns if BOTH sides have zero markers AND unknown vio status (i.e., truly no DIAG lines were ever seen).
  2. **Remove the `markers == 0` hide-and-skip** at line 687-689. Instead, always render the row with actual values. When `markers == 0`, show `0` for markers, `0` for corrections, `0.0%` for yield, and the actual `vio_status` / `pos_norm`.
  3. **Add two new columns** to the table header and rows: `VIO` (status) and `PosNorm` (last-seen position norm in meters).
  - **Target table format:**
    ```
    ALGORITHMIC LAYER — VIO STATE ESTIMATION HEALTH
    ------------------------------------------------------------------------------
                      Markers    Corrections   Yield    Locked   VIO        PosNorm
                      -------    -----------   -----    ------   ---        -------
      Head VIO:      153          146       95.4%     YES     valid      0.53m
      Arm  VIO:        0            0        0.0%      no     INVALID    7.77m
    ```
  - **Rationale:** This directly addresses the user's observation that arm DIAG data "is not parsed" — it IS parsed, just hidden. Showing the arm row with `vio=INVALID pos_norm=7.77m` makes the failure state immediately visible and actionable.

- [x] **B4.** Add a `[CRITICAL WARNING]` flag for the arm side when `markers == 0` and `vio_status == "INVALID"` (matching the existing critical warning pattern at line 694-697 for 0% yield). Message should indicate the arm camera may be occluded or the arm node crashed before detecting markers.
  - **Rationale:** A side that never sees markers is a critical failure that should be called out explicitly, not hidden behind a "—" row.

---

## Verification Criteria

- [x] **V1.** Run `python3 -m py_compile scripts/analyze_bag.py` — exits 0 with no syntax errors.
- [x] **V2.** Run `python3 -m py_compile scripts/analyze_log.py` — exits 0 with no syntax errors.
- [x] **V3.** Re-run `analyze_bag.py` against the existing bag and confirm the "Peak message clock delta = 14776ms on '/tf_static'" callout no longer appears. The peak delta should now reflect a real transport topic with a plausible sub-50ms value.
- [x] **V4.** Re-run `analyze_log.py` against `logs/jetson-run-jetson-debug-20260619_174609.txt` and confirm:
  - The Arm VIO row is visible (not hidden with "—")
  - It shows `markers=0, corrections=0, yield=0.0%, locked=no, VIO=INVALID, PosNorm=7.77m`
  - A `[CRITICAL WARNING]` flag appears for the arm side
  - The Head VIO row still shows `markers=153, corrections=146, yield=95.4%` correctly
- [x] **V5.** Confirm the `clock_offset_peak_s` metric in `to_metrics` output no longer spikes from `/tf_static`.
- [x] **V6.** Confirm the plot's "Max |clock_off|" axis no longer compresses to near-zero due to a `/tf_static` outlier.

---

## Potential Risks and Mitigations

1. **Existing callout logic may reference the removed `/tf_static` peak.**
   Mitigation: The callout at `analyze_bag.py:1274-1285` uses `max(clock_offs, key=...)` — once `/tf_static` is excluded, it simply picks the next-highest topic. If no topic exceeds 50ms, no callout is emitted (clean behavior).

2. **Enriching VioAnchorStats may break downstream consumers of the dataclass.**
   Mitigation: All new fields have defaults (`"unknown"`, `0.0`), so any code that constructs `VioAnchorStats()` without the new fields will still work. The dataclass is only instantiated in one place (the `LogReport` default factory).

3. **Removing the `markers == 0` early-return may show noise for runs where the arm node was never launched.**
   Mitigation: The `vio_status` defaults to `"unknown"` — if no arm DIAG lines were ever seen, the row will show `VIO=unknown` which clearly distinguishes "never ran" from "ran but saw 0 markers" (`VIO=INVALID`).

---

## Alternative Approaches

1. **For Issue A — QoS introspection instead of hardcoded set:** Instead of a hardcoded `LATCHED_TOPICS` set, read the actual QoS durability profile from the bag metadata and auto-exempt any topic with `TRANSIENT_LOCAL` durability.
   - **Trade-off:** More robust (auto-detects future latched topics) but significantly more complex — requires parsing MCAP QoS metadata, which may not be available in all bag formats. The hardcoded set is simpler and covers the known case (`/tf_static`).

2. **For Issue B — Full DIAG timeseries instead of last-seen snapshot:** Store every `[DIAG]` line as a timeseries and render a timeline plot of `pos_norm` and `vio_status` over time.
   - **Trade-off:** Much richer visualization but requires a new data structure, a new plot, and significantly more code. The last-seen snapshot gives the key diagnostic signal (final state) with minimal change. The timeseries can be added as a future enhancement.
