# MindRove EMG Armband — ROS 2 Integration

ROS 2 Jazzy driver and signal-routing stack for the **MindRove WiFi armband**.

## Packages

| Package | Description |
|---|---|
| `mia_hand_ros2_pkgs/mia_hand_msgs` | `EmgData.msg` — gesture + proportional value |
| `mia_hand_ros2_pkgs/emg_armband` | Hardware driver, ML inference, Docker |

## Quick start

```bash
cd mia_hand_ros2_pkgs/emg_armband/docker
docker compose run --rm emg_setup   # guided: collect → train → launch → monitor
# or, after training:
docker compose up --build emg        # launch ROS 2 nodes only
```

See **[`mia_hand_ros2_pkgs/emg_armband/README.md`](mia_hand_ros2_pkgs/emg_armband/README.md)** for full documentation.
