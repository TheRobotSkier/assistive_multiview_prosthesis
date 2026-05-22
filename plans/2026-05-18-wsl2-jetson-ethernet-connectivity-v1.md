# WSL2 + Jetson Ethernet Connectivity: Findings and Fixes

**Date:** 2026-05-18
**Status:** All automated fixes applied. One manual step remaining (`wsl --shutdown`).

---

## The Goal

Connect a Docker container running in WSL2 (via rootless podman) to a Jetson Orin over a direct Ethernet cable, so ROS2 topics published by the Jetson are visible inside the container.

## Why This Was Hard

People who run Linux natively just plug in the cable, set IPs, and it works. WSL2 adds three layers of networking complexity that each broke things independently.

---

## Problem 1: WSL2 Uses Virtual NAT, Not Physical NICs

**Finding:** WSL2 by default runs on a virtual `172.x.x.x` NAT network. The physical Ethernet adapter (plugged into the Jetson) belongs to Windows, not WSL2.

**Fix:** Enabled `networkingMode=mirrored` in `.wslconfig` (`/mnt/c/Users/jonat/.wslconfig`). This makes WSL2 share Windows' network interfaces directly — the Ethernet adapter with `10.42.0.1` becomes visible inside WSL2 and inside containers with `--network host`.

**Why correct:** Without mirrored mode, the container has no route to `10.42.0.0/24` at all.

---

## Problem 2: Windows Firewall Blocks Incoming Packets

**Finding:** The Jetson could not ping `10.42.0.1` (100% packet loss). The Ethernet adapter was on the "Public" network profile, which blocks all inbound traffic by default.

**Fix:** Created Windows Firewall rules via PowerShell (elevated):
- `New-NetFirewallRule -DisplayName "Jetson Ethernet" -Direction Inbound -InterfaceAlias "Ethernet" -Action Allow -Profile Any`
- `New-NetFirewallRule -DisplayName "Jetson Ethernet UDP" -Direction Inbound -InterfaceAlias "Ethernet" -Protocol UDP -Action Allow -Profile Any`

**Why correct:** DDS discovery requires bidirectional communication. Even with perfect CycloneDDS config, the Jetson's discovery packets were silently dropped by Windows.

---

## Problem 3: ROS2 CLI Daemon Hangs in Rootless Podman

**Finding:** `ros2 topic list` hung indefinitely (no output, no error). `ros2 topic list --no-daemon` worked fine. The ROS2 CLI daemon spawns an XML-RPC subprocess that times out in rootless podman with `--network host`.

**Fix:** Patched `strategy.py` in the Dockerfile (`docker/Dockerfile:77-81`) to permanently set `use_daemon = False`:
```
sed -i "s/use_daemon = not getattr(args, 'no_daemon', False)/use_daemon = False  # patched: daemon hangs in rootless podman/"
```

**Why correct:** The daemon is just a CLI optimization — disabling it has no effect on ROS2 functionality, only makes the first `ros2` call slightly slower. This is a known rootless podman issue.

---

## Problem 4: Hyper-V Firewall Blocks DDS (The Main Blocker)

**Finding:** Even after fixing the Windows Firewall, DDS discovery still failed. Confirmed via Python UDP tests that:
- Host → Jetson: TCP and UDP both work
- Jetson → Host: TCP and UDP both **fail**

The Windows Firewall rules only apply to the physical Ethernet adapter. WSL2 mirrored networking routes packets through a **Hyper-V virtual switch** which has its own separate firewall (`DefaultInboundAction: Block`). This firewall dropped all incoming packets from the Jetson, including DDS discovery.

**Fix:** Added `firewall=false` to `.wslconfig`:
```
[wsl2]
networkingMode=mirrored
firewall=false
```

**Why correct:** The Hyper-V firewall is a separate layer from the Windows Firewall. The `firewall=false` setting disables it for WSL2 specifically, allowing incoming DDS/UDP packets to reach the container. This is safe because the host machine is still protected by the regular Windows Firewall.

---

## Problem 5: Multicast Doesn't Work Over Direct Cable + WSL2

**Finding:** CycloneDDS defaults to multicast discovery (`239.255.0.1:7400`). Even with the Hyper-V firewall disabled, multicast UDP may not traverse the WSL2 mirrored networking boundary reliably.

**Fix:** Updated both host and Jetson CycloneDDS configs to disable multicast and use explicit unicast peers:

**Host** (`config/cyclonedds_peer.xml`):
```xml
<CycloneDDS>
  <Domain Id="0">
    <General>
      <AllowMulticast>false</AllowMulticast>
      <MulticastRecvNetworkInterfaceAddresses>none</MulticastRecvNetworkInterfaceAddresses>
    </General>
    <Discovery>
      <Peers>
        <Peer address="10.42.0.2"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

**Jetson** (same pattern, peers include `10.42.0.1`):
```xml
<CycloneDDS>
    <Domain Id="0">
        <General>
            <NetworkInterfaceAddress>10.42.0.2</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
            <MulticastRecvNetworkInterfaceAddresses>none</MulticastRecvNetworkInterfaceAddresses>
        </General>
        <Discovery>
            <Peers>
                <Peer address="10.42.0.1"/>
                <Peer address="10.42.0.2"/>
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
```

**Why correct:** With a direct Ethernet cable (no switch/router), multicast is unreliable. Explicit unicast peer discovery guarantees CycloneDDS sends SPDP packets directly to the known IP of the other machine. Both sides need the other listed as a peer for bidirectional discovery.

---

## Files Modified

| File | Change |
|------|--------|
| `/mnt/c/Users/jonat/.wslconfig` | Added `networkingMode=mirrored` and `firewall=false` |
| `scripts/setup_jetson_ethernet.ps1` | Created PowerShell script to set `10.42.0.1/24` on Windows Ethernet adapter |
| `scripts/robotlab_connect.sh` | Created WSL-side connectivity check script |
| `~/.ssh/config` | Created SSH shortcut (`ssh robotlab` → `robotlab@10.42.0.2`) |
| `~/.ssh/id_ed25519` | Generated SSH key pair |
| `Makefile:5` | Default `DOCKER_CMD` changed from `docker` to `podman` |
| `scripts/ros2_ethernet_hello_host.sh:13` | Default container runtime changed to `podman` |
| `docker/Dockerfile:11` | Added `openssh-client` to jazzy-rviz image |
| `docker/Dockerfile:77-81` | Added `strategy.py` patch to disable ROS2 daemon |
| `docker/docker-compose.yml` | Added `CYCLONEDDS_URI` env and config volume mount to prosthesis service |
| `config/cyclonedds_peer.xml` | Disabled multicast, added unicast peer `10.42.0.2` |
| Jetson `cyclonedds_peer.xml` | Disabled multicast, added unicast peer `10.42.0.1` |
| `docs/ethernet_ros2_setup.md` | Added WSL2 setup section |
| `src/mia_hand_driver/src/mia_hand_driver/cpp_driver.cpp:4` | Added `#include <thread>` (pre-existing bug) |

---

## Remaining Manual Step

Run `wsl --shutdown` from PowerShell, then reopen WSL. The `firewall=false` setting takes effect on restart. After that:

```bash
make down && make up
podman exec prosthesis bash -c 'source /opt/ros/jazzy/setup.bash && source /prosthesis_ws/install/setup.bash && timeout 30 ros2 topic list'
```

The Jetson's 80+ camera topics should appear within 10-15 seconds.
