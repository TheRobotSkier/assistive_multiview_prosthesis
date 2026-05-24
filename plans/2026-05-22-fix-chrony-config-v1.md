# Fix Chrony Configuration for Reliable Jetson-Host Time Sync

## Objective

Fix the chrony configuration files so that time synchronization actually works between the host (10.42.0.1) and the Jetson (10.42.0.2). The current configs were deployed via `make timesync` but have issues that prevent proper synchronization, particularly on WSL2 with mirrored networking.

## Context

- Host runs on WSL2 with mirrored networking (confirmed by `make timesync` output: "WSL2 interface (mirrored networking active)")
- Direct Ethernet link: host=10.42.0.1, Jetson=10.42.0.2
- Lab has no internet NTP access — the host must serve time from its local clock
- v3 log shows 0.17-0.88s clock skew between Jetson and host
- `make timesync` ran successfully but chrony is not actually syncing

## Implementation Plan

- [ ] **Fix `config/chrony-host.conf`**: Remove duplicate bare `local` directive (line 25), add `bindaddress 10.42.0.1` so chrony listens on the Ethernet interface, and add `bindcmdaddress 10.42.0.1` for chronyc. On WSL2, chrony may default to binding only to 127.0.0.1, making it unreachable from the Jetson. Also add `acquisitionport 123` and `allow 10.42.0.0/24` (already present, confirm it stays).
- [ ] **Fix `config/chrony-jetson.conf`**: Add `bindcmdaddress 127.0.0.1` and ensure `server 10.42.0.1 iburst` has `minpoll 0 maxpoll 4` for faster convergence on a direct link. The `local stratum 20` fallback is correct and should stay.
- [ ] **Update `Makefile` timesync-check target**: Add a more informative check that shows whether chrony is actually synced (not just sources), including the `chronyc tracking` output and a direct `ntpdate`-style offset check.
- [ ] **Re-run `make timesync` and `make timesync-check`**: Verify that chrony reports synchronized status and sub-10ms offset.

## Verification Criteria

- [ ] `chronyc sources` on Jetson shows `10.42.0.1` with `^*` (currently synced) mode
- [ ] `chronyc tracking` on Jetson reports "Last offset" < 10ms
- [ ] No "extrapolation into the past" errors in subsequent camera test logs
- [ ] `make timesync-check` shows both machines in sync

## Potential Risks and Mitigations

1. **WSL2 chrony may not be able to bind to 10.42.0.1** — On WSL2 mirrored networking, the 10.42.0.1 address may not exist as a separate interface. Mitigation: Also bind to `0.0.0.0` if `10.42.0.1` fails, and ensure the `allow` directive permits the Jetson subnet.
2. **Chrony port 123 may be blocked** — WSL2 or the Jetson's firewall could block NTP (UDP 123). Mitigation: The `make timesync-check` target should verify port reachability.
3. **Existing systemd-timesyncd conflict** — Some Ubuntu systems run systemd-timesyncd alongside chrony, which can cause conflicts. Mitigation: The Makefile should disable systemd-timesyncd when configuring chrony.

## Alternative Approaches

1. **Use `ntpdate` instead of chrony** — Simpler one-shot sync, but doesn't maintain ongoing synchronization. Chrony is better for continuous drift correction.
2. **Use PTP (IEEE 1588)** — More accurate than NTP but requires hardware support on both ends. Overkill for this use case.
3. **Use `use_sim_time` in ROS2** — Avoids clock sync entirely by using a common time source, but requires every node to be configured with it. More invasive change.
