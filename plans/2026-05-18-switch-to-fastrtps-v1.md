# Fix DDS Interop: Switch Prosthesis Container to FastDDS

## Objective

Switch the prosthesis container from CycloneDDS to FastDDS so it can discover the Jetson's ROS2 topics over the direct Ethernet link.

## Root Cause

The prosthesis container uses CycloneDDS (`rmw_cyclonedds_cpp`) while the Jetson uses FastDDS (`rmw_fastrtps_cpp`). These two DDS implementations cannot discover each other over a point-to-point Ethernet cable without complex bridging configuration. The simplest fix is to use the same DDS on both sides.

FastDDS is already available in the `osrf/ros:jazzy-desktop` base image — the `grasp_test` and `digital_twin` profiles in `docker-compose.yml` already use `rmw_fastrtps_cpp`.

## Implementation Plan

- [ ] **Step 1. Create a FastDDS peer config file**

  Create `config/fastrtps_peer.xml` with initial peer pointing to the Jetson:
  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>
  <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
      <participant profile_name="jetson_peer" is_default_profile="true">
          <rtps>
              <builtinTransports max_msg_size="65536"/>
          </rtps>
      </participant>
  </profiles>
  ```

  Rationale: The default FastDDS config uses multicast for discovery. Over a direct Ethernet cable, multicast usually works but can be unreliable. If it doesn't work, we'll add explicit initial peers. Start simple first.

- [ ] **Step 2. Update `docker-compose.yml` prosthesis service**

  In the `prosthesis` service environment section, change:
  ```yaml
  - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  - CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml
  ```
  to:
  ```yaml
  - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  - FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_peer.xml
  ```

  And change the volume mount from:
  ```yaml
  - ../config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro
  ```
  to:
  ```yaml
  - ../config/fastrtps_peer.xml:/tmp/fastrtps_peer.xml:ro
  ```

  Rationale: Switch to FastDDS with its own config file. The `FASTRTPS_DEFAULT_PROFILES_FILE` env var is the standard way to configure FastDDS.

- [ ] **Step 3. Restart and test**

  ```bash
  make down && make up
  podman exec prosthesis bash -c 'source /opt/ros/jazzy/setup.bash && source /prosthesis_ws/install/setup.bash && timeout 10 ros2 topic list'
  ```

  Should now show Jetson topics (`/arm/d435i_arm/...`, `/head/d435i_head/...`, etc.)

- [ ] **Step 4. (If multicast doesn't work) Add explicit initial peers**

  If `ros2 topic list` still doesn't show Jetson topics, update `config/fastrtps_peer.xml` to add explicit unicast peers:
  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>
  <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
      <participant profile_name="jetson_peer" is_default_profile="true">
          <rtps>
              <builtinTransports max_msg_size="65536"/>
              <initialPeersList>
                  <locator>
                      <udpv4>
                          <address>10.42.0.2</address>
                          <port>7400</port>
                      </udpv4>
                  </locator>
              </initialPeersList>
          </rtps>
      </participant>
  </profiles>
  ```

  Then restart: `make down && make up`

- [ ] **Step 5. Update Dockerfile default RMW (optional but recommended)**

  In `docker/Dockerfile:65`, change:
  ```
  ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```
  to:
  ```
  ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ```

  And remove the CycloneDDS install from `Dockerfile:14-15`:
  ```
  ros-jazzy-cyclonedds \
  ros-jazzy-rmw-cyclonedds-cpp \
  ```

  Rationale: Keeps the Dockerfile consistent with the compose config. Not strictly required since compose env vars override Dockerfile ENV, but avoids confusion.

## Verification Criteria

- [ ] `ros2 topic list` from prosthesis container shows Jetson topics
- [ ] `ros2 topic echo /arm/d435i_arm/color/image_raw --once` receives data
- [ ] `ros2 node list` shows Jetson nodes (e.g., `d435i_arm`, `d435i_head`)

## Potential Risks and Mitigations

1. **FastDDS multicast not working over direct cable**
   Mitigation: Step 4 adds explicit unicast initial peers to force discovery.

2. **Humble↔Jazzy message compatibility**
   Mitigation: ROS2 Humble and Jazzy use the same DDS message wire format for standard messages. Custom message types must have identical definitions on both sides. The Jetson's sensor_fusion_bringup package likely has its own message definitions — if there are version mismatches, topics may appear but subscription could fail.

3. **Existing local nodes may behave differently with FastDDS**
   Mitigation: The `grasp_test` and `digital_twin` profiles already use FastDDS, so this is a tested configuration.

## Alternative Approaches

1. **Keep CycloneDDS, configure interop**: Technically possible but complex — requires CycloneDDS to use the same discovery protocol as FastDDS. Not worth the effort when switching is trivial.

2. **Install CycloneDDS on the Jetson**: Would require modifying the Jetson's Docker setup, which is not your system to change. Switching the host side is simpler.

3. **Use ros2_bridge**: Run a bridge node that subscribes on one DDS and publishes on another. Adds latency and complexity. Unnecessary when both sides can use the same DDS.
