"""Shared constants for the EMG classifier pipeline."""

# ── Board ──────────────────────────────────────────────────────────────────────
SAMPLING_RATE: int = 500          # Hz — MindRove WiFi board
N_CHANNELS: int = 8               # EMG channels (raw_data[:8])

# ── Windowing ─────────────────────────────────────────────────────────────────
WINDOW_LEN: int = 100             # samples  →  200 ms at 500 Hz
WINDOW_STEP: int = 50             # samples  →  100 ms step  (50 % overlap, ~10 Hz)

# ── Filters ───────────────────────────────────────────────────────────────────
HIGHPASS_CUTOFF_HZ: float = 20.0  # DC / motion-artifact removal
LOWPASS_CUTOFF_HZ: float = 200.0  # anti-alias / HF noise (must be < Nyquist = 250 Hz)
NOTCH_FREQ_HZ: float = 50.0       # mains hum
NOTCH_Q: float = 30.0             # notch quality factor
FILTER_ORDER: int = 4             # Butterworth filter order

# ── Gestures ──────────────────────────────────────────────────────────────────
GESTURE_NAMES: list[str] = [
    "REST",       # 0 – relaxed, no contraction
    "POWER",      # 1 – power / cylindrical grip
    "PINCH",      # 2 – lateral pinch
    "OPEN",       # 3 – hand open / extension
    "POINT",      # 4 – index point / hook
]
N_GESTURES: int = len(GESTURE_NAMES)
REST_LABEL: int = 0

# ── Classifier ────────────────────────────────────────────────────────────────
CONFIDENCE_THRESHOLD: float = 0.55   # below → output REST
PREDICTION_SMOOTHING_FRAMES: int = 3  # majority-vote over last N predictions

# ── Data recording ────────────────────────────────────────────────────────────
DEFAULT_RECORD_DURATION_S: float = 5.0   # seconds per gesture per rep
DEFAULT_REPS: int = 3                    # repetitions per gesture
WARMUP_DURATION_S: float = 2.0          # board warm-up before first recording
