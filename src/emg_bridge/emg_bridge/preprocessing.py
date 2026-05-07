"""
EMG signal pre-processing: filters and windowing.

Provides both offline (batch) and online (stateful, streaming) variants so
that training and inference use identical filter parameters.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from scipy.signal import butter, iirnotch, sosfilt, sosfilt_zi

from .config import (
    FILTER_ORDER,
    HIGHPASS_CUTOFF_HZ,
    LOWPASS_CUTOFF_HZ,
    N_CHANNELS,
    NOTCH_FREQ_HZ,
    NOTCH_Q,
    SAMPLING_RATE,
    WINDOW_LEN,
    WINDOW_STEP,
)


# ── Filter coefficient builders ───────────────────────────────────────────────

def _make_highpass_sos() -> NDArray:
    return butter(FILTER_ORDER, HIGHPASS_CUTOFF_HZ / (SAMPLING_RATE / 2),
                  btype="high", output="sos")


def _make_lowpass_sos() -> NDArray:
    return butter(FILTER_ORDER, LOWPASS_CUTOFF_HZ / (SAMPLING_RATE / 2),
                  btype="low", output="sos")


def _make_notch_sos() -> NDArray:
    b, a = iirnotch(NOTCH_FREQ_HZ / (SAMPLING_RATE / 2), NOTCH_Q)
    # Convert ba → sos for consistent interface
    from scipy.signal import tf2sos
    return tf2sos(b, a)


# Pre-build at module import (cheap, reusable)
_HP_SOS = _make_highpass_sos()
_LP_SOS = _make_lowpass_sos()
_NOTCH_SOS = _make_notch_sos()


# ── Offline (batch) filtering ─────────────────────────────────────────────────

def filter_signal(emg: NDArray) -> NDArray:
    """Apply HP → notch → LP to a (N_samples, N_channels) EMG array.

    Returns array of same shape, filtered along axis 0.
    """
    out = emg.copy()
    for sos in (_HP_SOS, _NOTCH_SOS, _LP_SOS):
        out = sosfilt(sos, out, axis=0)
    return out


# ── Offline windowing ─────────────────────────────────────────────────────────

def extract_windows(
    emg: NDArray,
    window_len: int = WINDOW_LEN,
    step: int = WINDOW_STEP,
) -> NDArray:
    """Slice a filtered (N_samples, N_channels) array into overlapping windows.

    Returns:
        ndarray of shape (N_windows, N_channels, window_len)
    """
    n_samples = emg.shape[0]
    starts = range(0, n_samples - window_len + 1, step)
    windows = np.stack([emg[s : s + window_len].T for s in starts], axis=0)
    return windows  # (N_windows, N_channels, window_len)


# ── Online (stateful) filter ──────────────────────────────────────────────────

class OnlineFilter:
    """Stateful cascade of HP → notch → LP filters for real-time streaming.

    Maintains one independent filter state per channel so that short chunks
    of new samples can be processed incrementally without edge artifacts.
    """

    def __init__(self, n_channels: int = N_CHANNELS) -> None:
        self._n_channels = n_channels
        self._stages: list[NDArray] = [_HP_SOS, _NOTCH_SOS, _LP_SOS]
        # zi shape for each stage: (n_sections, 2)  → one per channel
        self._zi: list[list[NDArray]] = [
            [sosfilt_zi(sos) * 0.0 for _ in range(n_channels)]
            for sos in self._stages
        ]

    def process(self, chunk: NDArray) -> NDArray:
        """Filter a (n_new_samples, n_channels) chunk in-place and return it.

        Updates internal filter state so consecutive calls are seamless.
        """
        out = chunk.astype(float)
        for stage_idx, sos in enumerate(self._stages):
            result = np.empty_like(out)
            for ch in range(self._n_channels):
                filtered, zi = sosfilt(
                    sos, out[:, ch], zi=self._zi[stage_idx][ch]
                )
                result[:, ch] = filtered
                self._zi[stage_idx][ch] = zi
            out = result
        return out

    def reset(self) -> None:
        """Reset all filter states to zero (call between gestures or on reconnect)."""
        self._zi = [
            [sosfilt_zi(sos) * 0.0 for _ in range(self._n_channels)]
            for sos in self._stages
        ]


# ── Ring buffer ───────────────────────────────────────────────────────────────

class RingBuffer:
    """Fixed-length ring buffer that holds the last `capacity` samples.

    Used in the real-time loop to maintain a sliding window without copies.
    Shape convention: buffer is always (capacity, n_channels).
    """

    def __init__(self, capacity: int = WINDOW_LEN, n_channels: int = N_CHANNELS) -> None:
        self._capacity = capacity
        self._n_channels = n_channels
        self._buf: NDArray = np.zeros((capacity, n_channels), dtype=float)
        self._count: int = 0

    def push(self, chunk: NDArray) -> None:
        """Append (n_new, n_channels) chunk, discarding oldest samples."""
        n = chunk.shape[0]
        if n >= self._capacity:
            self._buf[:] = chunk[-self._capacity :]
            self._count = self._capacity
        else:
            self._buf = np.roll(self._buf, -n, axis=0)
            self._buf[-n:] = chunk
            self._count = min(self._capacity, self._count + n)

    def is_full(self) -> bool:
        """True once at least `capacity` samples have been pushed."""
        return self._count >= self._capacity

    def get_window(self) -> NDArray:
        """Return current (capacity, n_channels) window (copy)."""
        return self._buf.copy()

    def reset(self) -> None:
        self._buf[:] = 0.0
        self._count = 0
