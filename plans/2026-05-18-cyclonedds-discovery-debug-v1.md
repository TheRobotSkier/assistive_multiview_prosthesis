# Fix CycloneDDS Discovery — UDP/Config Debug

## Objective

Get CycloneDDS discovery working between the WSL2 host container and the Jetson over the direct Ethernet link.

## Current State

- Network layer works: TCP to Jetson port 22 succeeds, routing table correct (`eth0` → `10.42.0.0/24`)
- CycloneDDS shows deprecation warning for `NetworkInterfaceAddress` element
- Both sides use CycloneDDS, ROS_DOMAIN_ID=0, have peer entries for each other
- But only 2 local topics appear, no Jetson topics

## Implementation Plan

- [ ] **Step 1. Test UDP connectivity on DDS port**

  DDS uses UDP, not TCP. The TCP test to port 22 succeeded but that doesn't prove UDP works. Test with Python:
  
  ```bash
  podman exec prosthesis python3 -c "
  import socket
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  s.settimeout(3)
  s.sendto(b'test', ('10.42.0.2', 7400))
  print('UDP sent OK')
  "
  ```
  
  Also test from Jetson to host (from WSL):
  ```bash
  ssh robotlab 'python3 -c "
  import socket
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  s.settimeout(3)
  s.sendto(b\"test\", (\"10.42.0.1\", 7400))
  print(\"UDP sent OK\")
  "'
  ```

- [ ] **Step 2. Update CycloneDDS config to non-deprecated syntax**

  Replace `config/cyclonedds_peer.xml` with minimal config using new syntax:
  ```xml
  <CycloneDDS>
    <Domain Id="0">
      <General>
        <Interfaces>
          <NetworkInterface address="10.42.0.1"/>
        </Interfaces>
      </General>
      <Discovery>
        <Peers>
          <Peer address="10.42.0.2"/>
        </Peers>
      </Discovery>
    </Domain>
  </CycloneDDS>
  ```
  
  If the `address` attribute doesn't work in this CycloneDDS version, try by interface name:
  ```xml
  <NetworkInterface name="eth0"/>
  ```

- [ ] **Step 3. If still failing, try with no interface binding at all**

  Strip config to absolute minimum:
  ```xml
  <CycloneDDS>
    <Domain Id="0">
      <Discovery>
        <Peers>
          <Peer address="10.42.0.2"/>
        </Peers>
      </Discovery>
    </Domain>
  </CycloneDDS>
  ```
  
  This lets CycloneDDS auto-detect the interface based on the kernel routing table.

- [ ] **Step 4. If still failing, enable CycloneDDS trace logging**

  Add to docker-compose.yml environment:
  ```yaml
  - CYCLONEDDS_LOG_LEVEL=finest
  ```
  
  Then check logs for what CycloneDDS is actually doing:
  ```bash
  podman exec prosthesis bash -c 'source /opt/ros/jazzy/setup.bash && source /prosthesis_ws/install/setup.bash && CYCLONEDDS_LOG_LEVEL=config timeout 10 ros2 topic list 2>&1'
  ```

- [ ] **Step 5. If UDP is blocked, add Windows firewall rule for UDP**

  The existing firewall rule may not cover UDP. Run in elevated PowerShell:
  ```powershell
  New-NetFirewallRule -DisplayName "Jetson Ethernet UDP" -Direction Inbound -InterfaceAlias "Ethernet" -Protocol UDP -Action Allow -Profile Any
  ```

## Verification Criteria

- [ ] `ros2 topic list` shows Jetson topics (`/arm/d435i_arm/...`, `/head/d435i_head/...`)
