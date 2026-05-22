# Fix Jetson-Host Ethernet Connectivity

## Objective

Enable bidirectional IP communication between the Jetson (10.42.0.2) and the Windows/WSL host (10.42.0.1) over the direct Ethernet cable, so that DDS discovery can work.

## Root Cause

Windows Firewall blocks all incoming traffic on the Ethernet adapter because it's classified as a "Public" network. The Jetson cannot ping 10.42.0.1, which means DDS discovery packets from the Jetson never reach the host.

## Implementation Plan

- [ ] **Step 1. Allow all traffic on the Ethernet adapter in Windows Firewall**

  Run this in an **elevated PowerShell** on Windows:
  
  ```powershell
  # Option A: Allow all inbound traffic on the Ethernet adapter (simplest)
  New-NetFirewallRule -DisplayName "Jetson Ethernet" -Direction Inbound -InterfaceAlias "Ethernet" -Action Allow -Profile Any
  
  # Also allow ICMP (ping) explicitly
  New-NetFirewallRule -DisplayName "Jetson Ethernet ICMP" -Direction Inbound -InterfaceAlias "Ethernet" -Protocol ICMPv4 -Action Allow -Profile Any
  ```

  Rationale: This opens all inbound traffic on the specific Ethernet adapter connected to the Jetson. Since this is a dedicated point-to-point link with no internet access, there's no security risk.

- [ ] **Step 2. Verify bidirectional ping**

  From the Jetson:
  ```bash
  ping 10.42.0.1
  ```
  
  From WSL:
  ```bash
  ping 10.42.0.2
  ```
  
  Both must succeed before proceeding.

- [ ] **Step 3. Verify DDS interop (FastDDS ↔ CycloneDDS)**

  After ping works, test from inside the prosthesis container:
  ```bash
  podman exec prosthesis bash -c 'source /opt/ros/jazzy/setup.bash && source /prosthesis_ws/install/setup.bash && timeout 10 ros2 topic list'
  ```
  
  If Jetson topics appear, DDS interop is working. If not, we need to address the FastDDS↔CycloneDDS compatibility (Step 4).

- [ ] **Step 4. (If needed) Switch host to FastDDS for compatibility**

  If CycloneDDS cannot discover FastDDS nodes on the Jetson, switch the prosthesis container to use FastDDS instead:
  
  In `docker/docker-compose.yml`, change:
  ```yaml
  - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```
  to:
  ```yaml
  - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ```
  
  And remove the `CYCLONEDDS_URI` environment variable and volume mount. FastDDS uses multicast by default which should work on a point-to-point link.
  
  This requires `ros-jazzy-rmw-fastrtps-cpp` to be installed in the prosthesis image (check if it's already there).

- [ ] **Step 5. (If needed) Configure FastDDS XML on both sides**

  If multicast doesn't work over the direct cable, create a FastDDS peer config:
  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>
  <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="jetson_peer" is_default_profile="true">
      <rtps>
        <useBuiltinTransports>false</useBuiltinTransports>
        <userTransports>
          <transport_id>udp_transport</transport_id>
        </userTransports>
        <builtinTransports max_msg_size="65536"/>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>10.42.0.2</address>
            </udpv4>
          </locator>
        </initialPeersList>
      </rtps>
    </participant>
  </profiles>
  ```

## Verification Criteria

- [ ] Jetson can ping `10.42.0.1` (0% packet loss)
- [ ] WSL can ping `10.42.0.2` (0% packet loss)
- [ ] `ros2 topic list` from prosthesis container shows Jetson topics (`/arm/d435i_arm/...`, `/head/d435i_head/...`, etc.)
- [ ] `ros2 topic echo /arm/d435i_arm/color/image_raw --once` receives data

## Potential Risks and Mitigations

1. **Windows Firewall rule too broad**
   Mitigation: The rule is scoped to the specific "Ethernet" interface alias only. This adapter has no internet access (no default gateway), so the risk is minimal.

2. **FastDDS ↔ CycloneDDS interop failure**
   Mitigation: Switch host to FastDDS (Step 4). Since the Jetson runs FastDDS natively, using the same DDS on both sides guarantees compatibility.

3. **Docker on Jetson uses bridge network (172.17.0.0/16)**
   Mitigation: The Jetson's Docker containers likely use `--network host` (the processes show direct ROS2 nodes). If they use bridge mode, the Docker containers would need `--network host` or port forwarding. The `ip addr` output shows `docker0` at 172.17.0.1 but it's DOWN (no containers currently using bridge networking), which is consistent with `--network host`.

## Alternative Approaches

1. **Set Ethernet adapter to Private profile**: Instead of firewall rules, change the network profile to Private which allows all inbound traffic by default. `Set-NetConnectionProfile -InterfaceAlias "Ethernet" -NetworkCategory Private`. Simpler but less targeted.

2. **Disable Windows Firewall entirely**: `Set-NetFirewallProfile -Profile Public -Enabled False`. Works but reduces security on all interfaces. Not recommended.
