# Classical EMG Gesture Classifier

A classical signal-processing + ML pipeline for real-time EMG gesture recognition from the MindRove WiFi armband (8 channels, 500 Hz).

Recognizes **5 classes**: REST, POWER (cylindrical grip), PINCH (lateral), OPEN (extension), POINT (index hook).  
Additionally outputs a **proportional control signal** [0.0 – 1.0] derived from contraction intensity.

---

## Pipeline Overview

```
MindRove WiFi Armband  (8 ch, 500 Hz)
           │
           ▼
  ┌─ Pre-processing ───────────────────────────────┐
  │  Butterworth HP 20 Hz → Notch 50 Hz → LP 200 Hz│
  │  Sliding window: 200 ms, 50 % overlap (~10 Hz) │
  └────────────────────────────────────────────────┘
           │
           ▼
  ┌─ Feature Extraction  (per channel × 6) ────────┐
  │  MAV · RMS · Waveform Length · ZC · SSC · VAR  │
  │  → 48-dimensional feature vector               │
  └────────────────────────────────────────────────┘
           │
           ▼
  ┌─ Classification ────────────────────────────────┐
  │  StandardScaler → LDA  (SVM fallback if < 85 %) │
  │  Confidence threshold: 0.55  (below → REST)     │
  │  Majority-vote smoother: last 3 frames          │
  └─────────────────────────────────────────────────┘
           │
           ▼
  Terminal: Gesture │ Confidence │ Proportional ctrl
```

---

## File Layout

```
mindrove/
├── emg_classifier/          # Python package
│   ├── config.py            # All constants (sampling rate, window params, gestures)
│   ├── preprocessing.py     # SciPy filters; OnlineFilter + RingBuffer for streaming
│   ├── features.py          # Feature extraction (MAV, RMS, WL, ZC, SSC, VAR)
│   ├── classifier.py        # Train / load / predict; PredictionSmoother
│   ├── proportional.py      # RMS-based proportional control signal
│   └── board_reader.py      # Context-manager wrapper for MindRove BoardShim
├── scripts/
│   ├── collect_data.py      # Interactive data collection / calibration
│   ├── train.py             # Offline training + cross-validation
│   └── run_classifier.py    # Real-time inference loop
├── data/                    # Recorded .npz files (bind-mounted at runtime)
└── models/                  # Trained .pkl files (bind-mounted at runtime)
```

---

## Workflow

### Step 1 — Collect data

Connect to the armband's WiFi, then:

```bash
cd docker_ws/docker-deployment
docker compose run --rm mindrove_emg_collect
```

The script prompts you to perform each gesture in turn and records 5 seconds × 3 reps.  
Raw EMG is saved to `data/session_<timestamp>.npz`.

CLI flags (append after `--`):
```
--reps N          Repetitions per gesture (default: 3)
--duration S      Recording seconds per rep (default: 5)
--output-dir DIR  Where to save .npz files (default: /app/data)
```

### Step 2 — Train the classifier

```bash
docker compose run --rm mindrove_emg_train
```

Loads all `.npz` files in `data/`, runs 5-fold cross-validation, prints a confusion matrix, and saves `models/classifier.pkl` + `models/prop_calibration.pkl`.

### Step 3 — Run live inference

```bash
docker compose run --rm mindrove_emg_run
```

Displays a live terminal readout at ~10 Hz:

```
────────────────────────────────────────────────────
  Gesture  : POWER
  Confidence: [███████████····] 74%
  Prop. ctrl: [▓▓▓▓▓▓▓▓▓░░░░░░░░░░░░] 0.62
  REST:22%  POWER:74%  PINCH:1%  OPEN:2%  POINT:1%
────────────────────────────────────────────────────
  10.1 Hz  |  Ctrl-C to quit
```

---

## Tuning

Key constants in `emg_classifier/config.py`:

| Constant | Default | Effect |
|---|---|---|
| `GESTURE_NAMES` | `[REST, POWER, PINCH, OPEN, POINT]` | Labels shown in output |
| `WINDOW_LEN` | `100` (200 ms) | Longer = more stable, more latency |
| `WINDOW_STEP` | `50` (100 ms) | Smaller = more frequent predictions |
| `CONFIDENCE_THRESHOLD` | `0.55` | Lower = more activations; higher = fewer false positives |
| `PREDICTION_SMOOTHING_FRAMES` | `3` | Majority vote window |
