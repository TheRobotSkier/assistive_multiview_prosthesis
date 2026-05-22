# Connect to Jetson from Docker in WSL — Setup Plan

## Objective

Enable full network connectivity between a **Docker container running inside WSL2** on the host PC and the **Jetson Orin Nano** (`10.42.0.2`) over a direct Ethernet cable, so that:
1. You can **SSH into the Jetson** from within the WSL Docker container (and from WSL itself)
2. **ROS2 CycloneDDS** discovery and topic communication work between the Jetson and the Docker container

## Problem Analysis

### The Core Challenge: WSL2 Networking

WSL2 uses a **virtualized network adapter** (`eth0` inside WSL, visible as a vEthernet switch on the Windows side). This creates a NAT between WSL2 and the Windows host. The physical Ethernet adapter on the Windows host (the one plugged into the Jetson) is **not directly visible inside WSL2**.

This means:
- Inside WSL2, `ip addr` shows a virtual `eth0` with a `172.x.x.x` address — **not** the `10.42.0.1` address assigned to the physical Ethernet adapter
- Docker containers inside WSL2 use `--network host` to share the **WSL2** network namespace — not the Windows host's network
- A Docker container with `--network host` in WSL2 **cannot reach** `10.42.0.2` by default because WSL2 doesn't have a route to the `10.42.0.0/24` subnet

### What Needs to Happen

There are two main approaches, depending on how the Ethernet adapter is managed:

**Option A (Recommended): Windows-side Ethernet configuration + WSL2 mirrored networking**
- Configure the physical Ethernet adapter's IP to `10.42.0.1/24` on **Windows** (not inside WSL)
- Enable WSL2 **mirrored networking mode** so WSL2 shares the Windows host's network interfaces directly
- Docker containers with `--network host` will then see `10.42.0.1` and can reach `10.42.0.2`

**Option B (Fallback): Windows-side Ethernet + port forwarding/routing**
- Configure the Ethernet adapter on Windows
- Set up routing so WSL2 traffic to `10.42.0.0/24` goes through the Windows host
- More complex, less reliable for DDS multicast/unicast

---

## Implementation Plan

### Phase 1: Verify Current State

- [ ] **1.1. Check Windows build version** — WSL2 mirrored networking requires Windows 11 (build 22H2+) or Windows 10 recent builds. Run `winver` from Windows Run dialog (Win+R). Must be build 19044+ (Windows 10) or any Windows 11.
- [ ] **1.2. Check WSL2 version** — Run `wsl --version` from PowerShell. Should be WSL 2.0+. If outdated, run `wsl --update`.
- [ ] **1.3. Identify the physical Ethernet adapter name on Windows** — Run `Get-NetAdapter` in PowerShell. Look for the adapter connected to the Jetson (likely "Ethernet", "Ethernet 2", or a USB-Ethernet adapter name). Note its exact name.
- [ ] **1.4. Verify current IP on the Ethernet adapter** — Run `Get-NetIPAddress -InterfaceAlias "Ethernet"` (replace with actual name). It should currently have no IP or a different IP.
- [ ] **1.5. Check if Jetson is powered on and reachable from Windows** — After setting the IP on Windows (step 2.1), ping `10.42.0.2` from PowerShell.

### Phase 2: Configure Windows Ethernet Adapter

- [ ] **2.1. Set static IP on the Windows Ethernet adapter** — In PowerShell (Admin):
  ```
  New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 10.42.0.1 -PrefixLength 24
  ```
  Replace `"Ethernet"` with the actual adapter name from step 1.3. If an IP already exists, use `Set-NetIPAddress` or remove it first with `Remove-NetIPAddress`.
- [ ] **2.2. Verify connectivity from Windows** — Run `ping 10.42.0.2` from PowerShell. The Jetson must be powered on and have `10.42.0.2` configured on its Ethernet interface.
- [ ] **2.3. (Optional) Make the IP persistent** — The `New-NetIPAddress` command makes it persistent across reboots. Verify with `Get-NetIPAddress -InterfaceAlias "Ethernet"`.

### Phase 3: Enable WSL2 Mirrored Networking

- [ ] **3.1. Create or edit `.wslconfig`** — The file lives at `C:\Users\<YourUsername>\.wslconfig` (Windows path). In WSL, this is accessible at `/mnt/c/Users/<YourUsername>/.wslconfig`. Add or modify:
  ```ini
  [wsl2]
  networkingMode=mirrored
  ```
  This tells WSL2 to share the Windows host's network stack instead of using a separate virtual NAT.
- [ ] **3.2. Restart WSL2** — From PowerShell:
  ```
  wsl --shutdown
  ```
  Then reopen WSL. WSL2 will restart with mirrored networking.
- [ ] **3.3. Verify WSL2 sees the Ethernet interface** — Inside WSL, run `ip addr`. You should now see the physical Ethernet adapter (may appear as `eth0`, `eth1`, or with its Windows name) with the `10.42.0.1` address. Also run `ip route` to confirm there's a route to `10.42.0.0/24`.
- [ ] **3.4. Test ping from WSL2** — Run `ping 10.42.0.2` from inside WSL. This should succeed if mirrored networking is working correctly.

### Phase 4: SSH Access to Jetson

- [ ] **4.1. Test SSH from WSL2** — Run `ssh robotlab@10.42.0.2` from WSL. The default password should be accepted (check `docs/ROBOTLAB_SETUP.md` — credentials: user `robotlab`).
- [ ] **4.2. Set up SSH key-based auth (recommended)** — Generate a key if you don't have one, then copy it:
  ```bash
  ssh-keygen -t ed25519  # if no key exists
  ssh-copy-id robotlab@10.42.0.2
  ```
- [ ] **4.3. Add SSH config entry** — Edit `~/.ssh/config` inside WSL:
  ```
  Host robotlab
      HostName 10.42.0.2
      User robotlab
  ```
  Then you can simply `ssh robotlab`.
- [ ] **4.4. Test SSH from Docker container** — Run a Docker container with host networking and test:
  ```bash
  docker run --rm -it --network host localhost/ros2-jazzy-rviz:latest bash -c "apt-get update && apt-get install -y openssh-client && ssh robotlab@10.42.0.2"
  ```
  Or install `openssh-client` in the Docker image permanently (Phase 6).

### Phase 5: ROS2 / CycloneDDS Configuration

- [ ] **5.1. Verify CycloneDDS config is correct** — The existing `config/cyclonedds_peer.xml` has `NetworkInterfaceAddress` set to `10.42.0.1`. With mirrored networking, the Docker container (using `--network host`) will see this IP, so **the existing config should work as-is**.
- [ ] **5.2. Verify the Docker container can reach the Jetson** — Start the ROS2 shell:
  ```bash
  make ros2-ethernet-shell
  ```
  Inside the container, verify:
  ```bash
  ping 10.42.0.2
  source /opt/ros/jazzy/setup.bash
  ros2 topic list
  ```
  You should see topics published by the Jetson.
- [ ] **5.3. If topics don't appear — check CycloneDDS binding** — Inside the container, verify the IP:
  ```bash
  ip addr show | grep 10.42.0
  ```
  If the container doesn't see `10.42.0.1`, the CycloneDDS `NetworkInterfaceAddress` won't bind correctly. This means mirrored networking isn't fully working — revisit Phase 3.

### Phase 6: Optional — Add SSH Client to Docker Image

- [ ] **6.1. Modify `docker/Dockerfile.jazzy-rviz`** — Add `openssh-client` to the apt install line:
  ```
  RUN apt-get update && apt-get install -y --no-install-recommends \
          ros-jazzy-cyclonedds \
          ros-jazzy-rmw-cyclonedds-cpp \
          openssh-client \
      && rm -rf /var/lib/apt/lists/*
  ```
- [ ] **6.2. Rebuild the image** — `make build-jazzy-rviz`
- [ ] **6.3. Mount SSH keys into container** — Update `scripts/ros2_ethernet_hello_host.sh` to mount `~/.ssh` if you want SSH from inside the container. Alternatively, just SSH from WSL directly (outside Docker).

### Phase 7: Verify End-to-End

- [ ] **7.1. Full connectivity test** — From WSL:
  ```bash
  ping 10.42.0.2          # Layer 3
  ssh robotlab             # SSH
  ```
- [ ] **7.2. ROS2 topic test** — From the Docker container:
  ```bash
  make ros2-ethernet-shell
  # Inside container:
  source /opt/ros/jazzy/setup.bash
  ros2 topic list          # Should show Jetson topics
  ros2 topic echo /jetson_hello std_msgs/msg/String  # If Jetson is publishing
  ```
- [ ] **7.3. RViz test** — `make rviz` should show Jetson pointclouds if cameras are running.

---

## Verification Criteria

- [ ] `ping 10.42.0.2` succeeds from **WSL2**
- [ ] `ping 10.42.0.2` succeeds from a **Docker container** with `--network host` inside WSL2
- [ ] `ssh robotlab@10.42.0.2` works from **WSL2**
- [ ] `ros2 topic list` inside the Docker container shows Jetson topics
- [ ] `make rviz` launches and can display Jetson pointcloud data

## Potential Risks and Mitigations

1. **WSL2 mirrored networking not available (older Windows/WSL)**
   - Mitigation: Update Windows and WSL (`wsl --update`). If still unavailable, fall back to Option B: manual routing via `netsh` port forwarding or a bridge adapter on Windows.

2. **Mirrored networking breaks other WSL2 networking (VPN, DNS, etc.)**
   - Mitigation: Test your normal workflow after enabling mirrored mode. Some VPN clients conflict with mirrored networking. You can toggle back by removing `networkingMode=mirrored` and restarting WSL.

3. **Docker Desktop vs. native Docker in WSL2**
   - Mitigation: If using Docker Desktop for Windows, it manages its own network stack differently. The plan assumes Docker Engine running natively inside WSL2 (which is the default on modern WSL2 with `docker` installed via apt). If using Docker Desktop, the `--network host` flag maps to the Docker Desktop VM's network, not WSL2's — mirrored networking may not help. In that case, consider switching to native Docker in WSL2.

4. **Ethernet adapter name changes between reboots (USB adapters)**
   - Mitigation: Use the Windows "Network Connections" GUI to rename the adapter to something stable (e.g., "Jetson-Ethernet") and use that name in `New-NetIPAddress`.

5. **Firewall blocks DDS traffic**
   - Mitigation: Windows Defender Firewall may block incoming DDS traffic. Add a rule allowing UDP traffic on the DDS ports (typically 7400-7410) or temporarily disable the firewall for testing.

6. **CycloneDDS can't bind to the correct interface**
   - Mitigation: If `ip addr` inside the container shows `10.42.0.1` but CycloneDDS still fails, try using the interface name instead of IP in the XML config, or add debug output: `CYCLONEDDS_DEBUG=1` environment variable.

## Alternative Approaches

1. **Windows bridge adapter**: Instead of mirrored networking, create a network bridge in Windows between the physical Ethernet adapter and the WSL2 virtual switch. This is more complex and less reliable, but works on older Windows versions.

2. **WSL2 port forwarding with `netsh`**: Forward specific ports from Windows to WSL2. This works for SSH (port 22) but is **not suitable for ROS2/DDS** because DDS uses dynamic ports and multicast — you'd need to forward hundreds of ports.

3. **Run Docker on Windows directly (Docker Desktop with Hyper-V backend)**: Skip WSL2 entirely and run Docker containers on Windows with host networking. This avoids the WSL2 NAT issue but loses the native Linux development environment.

4. **Use WiFi hotspot instead of Ethernet**: The existing `scripts/setup_robotlab_wifi_linux.sh` creates a WiFi hotspot. However, the docs explicitly state that DDS discovery does **not** work reliably over WiFi with this adapter, so this is not viable for ROS2 communication.
