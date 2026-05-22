# Fix Clock Sync Timezone Bug

## Objective

Fix the 2-hour clock drift between host and Jetson caused by the `timesync` Makefile target using UTC-formatted time but setting it as local time on the Jetson.

## Root Cause

`Makefile:322` uses `date -u '+%Y-%m-%d %H:%M:%S'` to get UTC time, but `Makefile:325` passes it to `date -s` on the Jetson which interprets the string as **local time**. If the host is in CEST (UTC+2), the Jetson clock ends up 2 hours behind.

## Implementation Plan

- [x] **Replace lines 322-326** in `Makefile` with epoch-based sync:

  Change FROM:
  ```makefile
  @HOST_TIME="$$(date -u '+%Y-%m-%d %H:%M:%S')" && \
      echo "Host time:  $${HOST_TIME} UTC" && \
      echo "Jetson before: $$(ssh $(JETSON_HOST) date)" && \
      ssh $(JETSON_HOST) "echo robotlab | sudo -S date -s '$${HOST_TIME}'" 2>/dev/null && \
      echo "Jetson after:  $$(ssh $(JETSON_HOST) date)"
  ```

  Change TO:
  ```makefile
  @HOST_EPOCH="$$(date +%s.%N)" && \
      echo "Host time:   $$(date)" && \
      echo "Jetson before: $$(ssh $(JETSON_HOST) date)" && \
      ssh $(JETSON_HOST) "echo robotlab | sudo -S date -s @$${HOST_EPOCH}" 2>/dev/null && \
      echo "Jetson after:  $$(ssh $(JETSON_HOST) date)"
  ```

  Key change: `date +%s.%N` produces seconds-since-epoch (always UTC), and `date -s @epoch` interprets the `@`-prefixed value as epoch seconds regardless of timezone. This eliminates the UTC/localtime mismatch.

- [x] **Verify** by running `make timesync && make timesync-check` — the two clocks should now show the same wall-clock time (accounting for SSH latency of ~0.1s).

## Verification Criteria

- `make timesync-check` shows host and Jetson times within 1-2 seconds of each other
- No 2-hour offset between the displayed times
- The displayed times are in the same timezone (both show local wall-clock time)

## Potential Risks and Mitigations

1. **Jetson BusyBox `date` doesn't support `@epoch` syntax**
   Mitigation: BusyBox date does support `date -s @<epoch>` — this is a POSIX-standard extension. Verified to work on Ubuntu/Debian ARM64.
2. **`date +%s.%N` not available on all systems**
   Mitigation: GNU coreutils `date` supports `%N` (nanoseconds). Both WSL2 Ubuntu and Jetson Ubuntu have GNU date. If `%N` is not available, fall back to `%s` (second precision, still accurate enough for TF2).

## Alternative Approaches

1. **Use `date` (no `-u`) on both sides**: Simpler but breaks if host and Jetson are in different timezones.
2. **Use `ntpdate`**: Requires an NTP server, which WSL2 can't provide (the original problem).
3. **Use `ssh` with `TZ=UTC date`**: Set TZ=UTC on the Jetson side when calling `date -s`, but this is fragile and harder to understand.
