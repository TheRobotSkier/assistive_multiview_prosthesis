# Enable NTP from WSL2 Host to Jetson via chrony

## Objective

Make `chrony` on the WSL2 host serve NTP on `10.42.0.1:123` so the Jetson's chrony can continuously sync via NTP, eliminating the need for repeated one-shot SSH clock syncs.

## Current State

- **Jetson**: chrony installed, configured to sync to `10.42.0.1`, but `chronyc sources` likely shows `^?` (unreachable) because nothing is listening on the host side.
- **Host (WSL2)**: chrony is NOT installed or configured. The `make timesync` target only touches the Jetson. The `timesync-host` target exists but is manual-only and never called by `timesync`.
- **Networking**: Mirrored networking is active (confirmed by `robotlab_connect.sh`). The `10.42.0.1` address is visible inside WSL2. This makes NTP serving feasible — the old "WSL2 can't serve NTP" comment predates the mirrored networking setup.
- **Configs exist**: `config/chrony-host.conf` already has `bindaddress 0.0.0.0`, `local stratum 10`, `allow 10.42.0.0/24` — it's ready for WSL2 use.

## Root Cause

The claim "WSL2 cannot run an NTP server (chrony can't bind UDP 123)" at `Makefile:310-311` and "On WSL2, chrony cannot serve NTP" at `Makefile:343` was written before mirrored networking was confirmed working. With mirrored networking, the WSL2 VM sees the host's IP addresses directly and should be able to bind to port 123 on `10.42.0.1`.

## Implementation Plan

- [ ] **1. Install chrony on the WSL2 host**
  - `sudo apt install -y chrony`
  - Rationale: chrony is not currently installed on the WSL2 side. The `timesync` target never installs it, and `timesync-host` checks for it but that target is never called by `timesync`.

- [ ] **2. Check for port 123 conflicts on the host**
  - Run `sudo ss -ulpn | grep :123` to see if anything (Windows Time service, systemd-timesyncd, etc.) is already bound to UDP 123 on the WSL2 side.
  - If Windows Time service is bound on the Windows side of the mirrored interface, it may block WSL2 chrony from binding. Mitigation: disable Windows Time service on the `10.42.0.1` interface only, or configure it to not listen on that interface.
  - Rationale: Port 123 is a privileged port — only one process can bind to it per IP. If w32time on Windows already holds it on `10.42.0.1`, chrony in WSL2 will fail to bind.

- [ ] **3. Deploy chrony host config**
  - Copy `config/chrony-host.conf` to `/etc/chrony/chrony.conf` on the host.
  - Set `SYNC_IN_CONTAINER="yes"` in `/etc/default/chrony` (the `timesync-host` target already handles this at `Makefile:348`).
  - Disable systemd-timesyncd on the host to avoid conflicts.
  - Rationale: The config already has the right settings (`bindaddress 0.0.0.0`, `local stratum 10`, `allow 10.42.0.0/24`). We just need to deploy it.

- [ ] **4. Start/restart chronyd on the host**
  - `sudo systemctl restart chronyd`
  - Verify with `sudo ss -ulpn | grep :123` that chronyd is listening on `0.0.0.0:123` or `10.42.0.1:123`.
  - Rationale: If chronyd fails to bind, we need to diagnose immediately (port conflict, WSL2 limitation, etc.).

- [ ] **5. Verify NTP reachability from Jetson**
  - From the Jetson: `chronyc sources` should show `10.42.0.1` transitioning from `^?` to `^*` (synced).
  - Also test UDP reachability directly: `sudo nmap -sU -p 123 10.42.0.1` from Jetson, or `echo | nc -u -w1 10.42.0.1 123` from Jetson and check chrony host logs for the query.
  - Rationale: Even if chronyd binds, Windows Firewall or the Jetson's firewall could block UDP 123 inbound on the Ethernet interface.

- [ ] **6. Update `Makefile` `timesync` target to also configure the host**
  - Call `timesync-host` (or its logic inline) as part of `timesync`.
  - Remove or update the outdated comments at lines 310-311 and 343 about WSL2 not being able to serve NTP.
  - Rationale: Currently `timesync` only configures the Jetson side. For NTP to work, the host must also run chrony.

- [ ] **7. Update `Makefile` `timesync-check` to show host chrony status**
  - Add host-side `chronyc sources` and `chronyc tracking` output.
  - Add a UDP port 123 reachability test from Jetson to host.
  - Show the Jetson's sync status more clearly (not just sources, but whether `^*` appears next to `10.42.0.1`).
  - Rationale: Better diagnostics help catch NTP failures early.

- [ ] **8. Handle Windows Firewall (if needed)**
  - If step 5 shows UDP 123 is unreachable from the Jetson, the Windows Firewall may be blocking inbound NTP on the Ethernet interface.
  - Fix: Add a Windows Firewall rule allowing UDP 123 inbound on the Ethernet adapter (this would be a one-time manual step on the Windows side, documented in a README or script).
  - Rationale: This is the most likely non-WSL2 blocker. Windows Firewall by default may not allow incoming UDP on public/private network profiles for the Ethernet adapter.

- [ ] **9. End-to-end verification**
  - Run `make timesync` — should complete without errors, host chrony running, Jetson chrony running.
  - Run `make timesync-check` — Jetson should show `10.42.0.1` with `^*` mode, "Last offset" < 10ms.
  - Wait 60 seconds, run `make timesync-check` again — offset should remain stable (chrony maintaining sync).
  - Run a camera test — "extrapolation into the past" errors should be eliminated or significantly reduced.

## Verification Criteria

- `chronyc sources` on Jetson shows `10.42.0.1` with `^*` (currently synced) mode
- `chronyc tracking` on Jetson reports "Last offset" < 10ms and "RMS offset" < 1ms
- `sudo ss -ulpn | grep :123` on host shows chronyd listening on 0.0.0.0:123
- `make timesync-check` shows both host and Jetson chrony synced
- Camera test logs show no "extrapolation into the past" errors (previously ~10-50 per run)

## Potential Risks and Mitigations

1. **Windows Time service (w32time) already bound to UDP 123 on `10.42.0.1`**
   Mitigation: Disable w32time entirely (if not needed), or configure it to not listen on the Ethernet interface. A PowerShell command can do this: `Set-Service w32time -StartupType Disabled; Stop-Service w32time`. If Windows time sync is still needed for the Windows host itself, configure w32time to only use the WiFi interface, freeing the Ethernet interface for WSL2 chrony.

2. **Windows Firewall blocks inbound UDP 123 on the Ethernet adapter**
   Mitigation: Add a firewall rule via PowerShell: `New-NetFirewallRule -DisplayName "NTP for Jetson" -Direction Inbound -Protocol UDP -LocalPort 123 -InterfaceAlias "Ethernet" -Action Allow`. This is the most likely blocker and needs to be documented.

3. **WSL2 with mirrored networking still can't bind privileged ports**
   Mitigation: WSL2 runs a full Linux kernel with `CAP_NET_BIND_SERVICE` — `sudo chronyd` should be able to bind port 123. If this fails, chrony can be configured to use an unprivileged port (>1024) and iptables can redirect, but this is unlikely to be needed.

4. **Chrony on host and Jetson both using `local stratum 10` creates a sync loop**
   Mitigation: The Jetson's config uses `server 10.42.0.1` with `local stratum 10` as fallback only. Chrony correctly prefers the server over local when reachable. No loop risk.

## Alternative Approaches

1. **Cron-based periodic `make timesync` instead of NTP**: Add a cron job that runs the one-shot SSH sync every hour. Simpler but less precise (clock can drift up to ~1s between syncs). Trade-off: zero host-side configuration, but drift during sessions.

2. **PTP (IEEE 1588)**: More accurate than NTP (sub-microsecond) but requires hardware timestamping support on both Ethernet adapters. Overkill for TF2 which only needs ~10ms precision.

3. **ROS2 `use_sim_time`**: Have the Jetson publish `/clock` and all nodes use simulation time. This eliminates clock sync entirely but requires every ROS node to be configured for sim time, which is a more invasive change across both codebases.
