"""
Thin wrapper around the MindRove BoardShim for easier use as a context manager.
"""

from __future__ import annotations

import logging

from mindrove.board_shim import BoardIds, BoardShim, MindRoveInputParams

from .config import N_CHANNELS, SAMPLING_RATE

logger = logging.getLogger(__name__)


class BoardReader:
    """Context-manager wrapper for the MindRove WiFi board.

    Usage::

        with BoardReader() as br:
            chunk = br.read(n_samples=50)   # shape (n_samples, N_CHANNELS)
            frame = br.read_frame(n_samples=50)  # dict with emg, gyro, accel, timestamps
    """

    def __init__(self, buffer_size: int = 450_000) -> None:
        self._buffer_size = buffer_size
        self._board: BoardShim | None = None
        self._gyro_channels: list[int] = []
        self._accel_channels: list[int] = []
        self._timestamp_channel: int = -1

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def connect(self) -> None:
        params = MindRoveInputParams()
        self._board = BoardShim(BoardIds.MINDROVE_WIFI_BOARD, params)
        self._board.prepare_session()
        self._board.start_stream(self._buffer_size)

        board_id = self._board.get_board_id()
        try:
            self._gyro_channels = BoardShim.get_gyro_channels(board_id)
            self._accel_channels = BoardShim.get_accel_channels(board_id)
            self._timestamp_channel = BoardShim.get_timestamp_channel(board_id)
        except Exception:
            self._gyro_channels = []
            self._accel_channels = []
            self._timestamp_channel = -1

        if not self._gyro_channels:
            logger.warning("No gyro channels discovered for board %s", board_id)
        if not self._accel_channels:
            logger.warning("No accel channels discovered for board %s", board_id)

    def disconnect(self) -> None:
        if self._board is not None and self._board.is_prepared():
            self._board.stop_stream()
            self._board.release_session()
        self._board = None

    def __enter__(self) -> "BoardReader":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.disconnect()

    # ── properties ────────────────────────────────────────────────────────────

    @property
    def sampling_rate(self) -> int:
        if self._board is None:
            return SAMPLING_RATE
        return int(BoardShim.get_sampling_rate(self._board.get_board_id()))

    @property
    def gyro_channels(self) -> list[int]:
        return list(self._gyro_channels)

    @property
    def accel_channels(self) -> list[int]:
        return list(self._accel_channels)

    # ── data access ───────────────────────────────────────────────────────────

    def available(self) -> int:
        """Number of samples currently in the board's ring buffer."""
        if self._board is None:
            return 0
        return int(self._board.get_board_data_count())

    def read(self, n_samples: int) -> "numpy.ndarray":
        """Read exactly n_samples from the board (blocks briefly if needed).

        Returns:
            ndarray of shape (n_samples, N_CHANNELS), dtype float64 — EMG only
        """
        import time
        import numpy as np

        assert self._board is not None, "Not connected"

        while self._board.get_board_data_count() < n_samples:
            time.sleep(0.001)

        raw = self._board.get_board_data(n_samples)
        return raw[:N_CHANNELS].T.astype(np.float64)  # (n_samples, N_CHANNELS)

    def read_frame(self, n_samples: int) -> dict:
        """Read exactly n_samples as a structured frame with EMG, gyro, accel, and timestamps.

        Returns:
            dict with keys:
                emg        — (n_samples, N_CHANNELS) float64, always present
                gyro       — (n_samples, N_GYRO_CH) float64, or None if unavailable
                accel      — (n_samples, N_ACCEL_CH) float64, or None if unavailable
                timestamps — (n_samples,) float64, or None if unavailable
        """
        import time
        import numpy as np

        assert self._board is not None, "Not connected"

        while self._board.get_board_data_count() < n_samples:
            time.sleep(0.001)

        raw = self._board.get_board_data(n_samples)

        emg = raw[:N_CHANNELS].T.astype(np.float64)

        timestamps = None
        if self._timestamp_channel >= 0:
            timestamps = raw[self._timestamp_channel].astype(np.float64)

        gyro = None
        if self._gyro_channels:
            gyro = raw[self._gyro_channels].T.astype(np.float64)

        accel = None
        if self._accel_channels:
            accel = raw[self._accel_channels].T.astype(np.float64)

        return {"emg": emg, "gyro": gyro, "accel": accel, "timestamps": timestamps}

    def flush(self) -> None:
        """Discard all buffered samples."""
        if self._board is not None:
            self._board.get_board_data()
