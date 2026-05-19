# emg_armband

ROS 2 Jazzy package for the **MindRove WiFi armband**.  
Classifies forearm EMG into gestures (REST / POWER / PINCH / OPEN / POINT) and
routes proportional control signals to any ROS 2 topics you configure in
`config/emg.yaml`.

---

## Package layout

```
emg_armband/
├── config/
│   └── emg.yaml          # all configuration (classifier + routing)
├── docker/
│   ├── Dockerfile        # minimal ROS 2 Jazzy + MindRove + ML image
│   ├── docker-compose.yml
│   └── ros_entrypoint.sh
├── emg_classifier/       # Python package: inference pipeline
│   ├── __init__.py
│   ├── config.py         # gesture names, feature settings
│   ├── features.py       # time/freq feature extraction
│   ├── inference.py      # BoardReader → filter → buffer → predict
│   └── smoother.py       # majority-vote prediction smoother
├── launch/
│   └── emg_launch.py     # starts emg_node + emg_parser_node
├── models/               # bind-mounted — persists outside the image
│   ├── classifier.pkl
│   ├── meta.pkl
│   └── prop_calibration.pkl
├── data/                 # bind-mounted — labelled training sessions
├── nodes/
│   ├── emg_node.py       # hardware → /emg/data (EmgData)
│   └── emg_parser_node.py# /emg/data → per-gesture Float32 topics
└── scripts/
    ├── collect_data.py   # record labelled sessions
    ├── train.py          # fit classifier + calibration
    ├── run_classifier.py # standalone live inference (no ROS)
    ├── emg_monitor.py    # ROS 2 terminal dashboard (10 Hz)
    └── setup.sh          # guided: collect → train → launch → monitor
```

---

## Quick start (Docker / Podman)

All commands run from `emg_armband/docker/`.

### 1 — First-time classifier training

```bash
# Step through collection, training, and a test run interactively:
docker compose run --rm emg_setup
```

Or run each step separately:

```bash
# Record labelled training data (follow the on-screen prompts):
docker compose run --rm emg_collect

# Fit the classifier and calibrate proportional output:
docker compose run --rm emg_train

# Verify live inference in the terminal (no ROS required):
docker compose run --rm emg_run
```

### 2 — Launch the ROS 2 nodes

```bash
docker compose up --build emg
```

This starts both `emg_node` and `emg_parser_node` via `emg_launch.py`.

Published topics (defaults, change in `emg.yaml`):

| Topic | Type | Description |
|---|---|---|
| `/emg/data` | `mia_hand_msgs/EmgData` | gesture + proportional value, ~10 Hz |
| `/emg/gesture` | `std_msgs/String` | current gesture name, every frame |
| *(per-gesture)* | `std_msgs/Float32` | routed signals, see `emg.yaml` |

### 3 — Live monitor (separate terminal)

```bash
docker compose run --rm emg bash -c \
  "source /emg_ws/install/setup.bash && \
   python /emg_ws/install/emg_armband/lib/emg_armband/scripts/emg_monitor.py"
```

---

## Configuration (`config/emg.yaml`)

The file has two top-level keys:

### `emg_node`

| Key | Default | Description |
|---|---|---|
| `model_dir` | `/emg_ws/src/emg_armband/models/` | classifier + calibration files |
| `emg_data_topic` | `/emg/data` | output topic |
| `publish_rate` | `10.0` | Hz |
| `confidence_threshold` | `0.55` | below this → REST |
| `smoothing_frames` | `3` | majority-vote window |

### `emg_parser` — per-gesture output routing

Each gesture can have a list of `outputs`.  Every output supports:

| Field | Default | Description |
|---|---|---|
| `topic` | *(required)* | `std_msgs/Float32` topic |
| `threshold` | `0.0` | minimum proportional before anything fires |
| `binary` | `false` | send `1.0` instead of the proportional value |
| `send_zero` | `true` | send `0.0` while below threshold |
| `send_on_change` | `false` | edge-triggered: fire only on rising/falling edge |
| `change_timeout` | `0.0` | seconds between consecutive rising-edge fires |
| `activation_count` | `0` | consecutive above-threshold frames required (debounce) |

#### Output modes

**Streaming (default)** — every incoming frame publishes a value:
- above threshold → publish `proportional` (or `1.0` if `binary`)
- below threshold → publish `0.0` if `send_zero`, else nothing

**Edge-triggered (`send_on_change: true`)** — publishes only on transitions:
- rising edge (inactive → active) → publish value, subject to `change_timeout`
- falling edge (active → inactive) → publish `0.0` if `send_zero`

Example — mode-switch toggle that fires once per intentional hold:

```yaml
POINT:
  outputs:
    - topic: "/mia_hand/mode_switch"
      binary: true
      threshold: 0.5
      send_zero: false
      send_on_change: true
      change_timeout: 1.0   # 1 s cooldown between triggers
      activation_count: 3   # hold ~300 ms at 10 Hz before activating
```

---

## Networking

The container uses `network_mode: host` so it shares the host's DDS domain.
`ROS_AUTOMATIC_DISCOVERY_RANGE: LOCALHOST` limits discovery to the local
machine — set it to `SUBNET` to talk to other hosts.

The MindRove board connects over WiFi; no USB serial device is needed.

---

## Building from source (without Docker)

```bash
cd <workspace>/src
# place mia_hand_msgs/ and emg_armband/ here
pip install mindrove numpy scipy scikit-learn joblib
colcon build --packages-select mia_hand_msgs emg_armband
source install/setup.bash
ros2 launch emg_armband emg_launch.py
```
