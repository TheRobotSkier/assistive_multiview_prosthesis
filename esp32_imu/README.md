# ESP32 Dual GY-91 IMU → ROS2 Bridge

## Overview

Two GY-91 modules (MPU9250 9-DOF IMU) read by an ESP32-WROOM and streamed
over USB serial to a ROS2 node that publishes `sensor_msgs/Imu` on:
- `/head/imu` — for the head-mounted camera
- `/arm/imu`  — for the arm-mounted camera

OpenVINS subscribes to these topics for visual-inertial odometry.

## Hardware Wiring

```
ESP32-WROOM          GY-91 (head)        GY-91 (arm)
────────────         ────────────         ────────────
GPIO21 (SDA)   ──── SDA              ─── SDA
GPIO22 (SCL)   ──── SCL              ─── SCL
3.3V           ──── VCC              ─── VCC
GND            ──── GND              ─── GND
GPIO19         ──── (not connected)  ─── SAO

USB (UART0)    ──── Host computer

The SAO (Slave Address 0, same as AD0) pin of the **arm** IMU is
connected to GPIO19. During boot, the ESP32 drives SAO LOW
(both IMUs at 0x68), configures them, then drives SAO HIGH
(arm IMU moves to 0x69).

## Quick Start

```bash
# 1. Build firmware
cd esp32_imu
make build

# 2. Flash to ESP32
make flash-docker SERIAL_PORT=/dev/ttyUSB0

# 3. Run the ROS2 bridge + OpenVINS
cd ..
podman-compose run imu_bridge &
podman-compose run openvins
```

## Configuration

Edit `config.yaml` to change:
- GPIO pins for I2C and AD0 control
- MPU9250 ranges and DLPF bandwidth
- IMU-to-camera offsets (used for Kalibr/OpenVINS calibration)

## Directory Layout

```
esp32_imu/
├── config.yaml              # Pin mappings, offsets, IMU settings
├── Makefile                 # Build & flash targets
├── firmware/                # ESP-IDF project
│   ├── CMakeLists.txt
│   ├── sdkconfig.defaults
│   ├── partitions.csv
│   ├── main/
│   │   ├── CMakeLists.txt
│   │   └── main.cpp         # Boot sequence + 200Hz loop
│   └── components/
│       └── gy91/
│           ├── CMakeLists.txt
│           ├── gy91.h        # MPU9250 driver
│           └── gy91.cpp
├── docker/
│   └── Dockerfile            # ESP-IDF v5.4 build container
└── ros2_serial_bridge/
    └── imu_bridge_node.py    # Serial → ROS2 Imu publisher
```

## Data Flow

```
GY-91 (head) ──I2C──┐
                     ├── ESP32 ──USB/UART──▶ Host
GY-91 (arm)  ──I2C──┘                         │
                                               ▼
                                    imu_bridge_node.py
                                    (ROS2 sensor_msgs/Imu)
                                               │
                                    ┌──────────┴──────────┐
                                    ▼                      ▼
                              /head/imu               /arm/imu
                                    │                      │
                                    ▼                      ▼
                              ov_msckf_head          ov_msckf_arm
                              (OpenVINS VIO)         (OpenVINS VIO)
```
