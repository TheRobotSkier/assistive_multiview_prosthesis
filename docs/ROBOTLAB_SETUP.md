# Robotlab Setup Reference

Concise reference for connecting to and operating the Jetson Orin Nano robotlab.

## Network Topology

```
Host PC (10.42.0.1) ──WiFi Hotspot──▶ Robotlab Jetson (10.42.0.2)
       │                                      │
       │  internet via NAT                    │  2x D435 cameras
       │  RViz2 via podman                    │  2x GY-91 IMUs (I2C)
       │                                      │  Docker-based ROS 2
       ▼                                      ▼
   Internet                            Sensor Fusion + EKF
```

The host creates a dedicated WiFi access point. The Jetson connects to it with a static IP. All ROS 2 communication uses CycloneDDS over this subnet — no router needed.

## Quick Connect

### 1. Create the Hotspot

**Linux (native NetworkManager):**
```bash
sudo scripts/setup_robotlab_wifi_linux.sh <wifi_interface>
# Example: sudo scripts/setup_robotlab_wifi_linux.sh wlan1
```

**Windows (PowerShell as Admin):**
```powershell
powershell -ExecutionPolicy Bypass -File scripts/setup_robotlab_wifi.ps1
```

### 2. SSH into Robotlab
```bash
ssh robotlab@10.42.0.2
```

### 3. Launch the Sensor Fusion Bringup (on robotlab)
```bash
cd ~/jetson_ws/jetson-docker
docker compose -f docker_ws/docker-deployment/docker-compose.yml run --rm miahand_ros2 \
  bash -c 'source /miahand_ws/install/setup.bash && ros2 launch sensor_fusion_bringup robotlab_bringup.launch.py'
```

### 4. Launch RViz on Host (optional)
```bash
scripts/launch_rviz_host.sh
```
Uses podman with host networking. Requires X11 and a local `osrf/ros:jazzy-desktop` image.

## Credentials & IPs

| Setting       | Value              |
|---------------|--------------------|
| SSID          | `robotlab-wifi`    |
| Password      | `labrobot123`      |
| Host IP       | `10.42.0.1/24`     |
| Robotlab IP   | `10.42.0.2/24`     |
| SSH user      | `robotlab`         |

## Hardware Reference

### D435 Cameras

| Camera | Serial Number   | Namespace     |
|--------|-----------------|---------------|
| Head   | `829212072207`  | `/head`       |
| Arm    | `827112072033`  | `/arm`        |

### GY-91 External IMUs (I2C)

| Camera Slot | I2C Bus | Address | Device         |
|-------------|---------|---------|----------------|
| cam0        | bus 7   | `0x68`  | GY-91 (MPU9250)|
| cam1        | bus 1   | `0x68`  | GY-91 (MPU9250)|

## Robotlab Repo Layout

```
~/jetson_ws/jetson-docker/
└── docker_ws/docker-deployment/
    └── docker-compose.yml    # miahand_ros2 service + bringup
```

## Troubleshooting

**Can't SSH:** Verify the hotspot is active (`nmcli connection show robotlab-hotspot` on Linux, `netsh wlan show hostednetwork` on Windows). The Jetson must be powered on and within range.

**No ROS discovery:** Ensure the host is on the same subnet (`10.42.0.0/24`) and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is set. The `launch_rviz_host.sh` script handles this.

**Hotspot stops internet on host:** The Linux script uses `ipv4.method shared` which creates a NAT automatically — your internet should still work. On Windows the script creates a NetNat rule. If both fail, check your firewall.

**IMU data missing:** Check I2C bus enumeration on the Jetson: `i2cdetect -y -r 1` and `i2cdetect -y -r 7`. Address `0x68` should appear.
