"""
Thin wrapper around the MindRove BoardShim for easier use as a context manager.
"""

from __future__ import annotations

from mindrove.board_shim import BoardIds, BoardShim, MindRoveInputParams

from .config import N_CHANNELS, SAMPLING_RATE


class BoardReader:
    """Context-manager wrapper for the MindRove WiFi board.

    Usage::

        with BoardReader() as br:
            chunk = br.read(n_samples=50)   # shape (n_samples, N_CHANNELS)
    """

    def __init__(self, buffer_size: int = 450_000) -> None:
        self._buffer_size = buffer_size
        self._board: BoardShim | None = None

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def connect(self) -> None:
        params = MindRoveInputParams()
        self._board = BoardShim(BoardIds.MINDROVE_WIFI_BOARD, params)
        self._board.prepare_session()
        self._board.start_stream(self._buffer_size)

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

    # ── data access ───────────────────────────────────────────────────────────

    def available(self) -> int:
        """Number of samples currently in the board's ring buffer."""
        if self._board is None:
            return 0
        return int(self._board.get_board_data_count())

    def read(self, n_samples: int) -> "numpy.ndarray":
        """Read exactly n_samples from the board (blocks briefly if needed).

        Returns:
            ndarray of shape (n_samples, N_CHANNELS), dtype float64
        """
        import time
        import numpy as np

        assert self._board is not None, "Not connected"

        while self._board.get_board_data_count() < n_samples:
            time.sleep(0.001)

        raw = self._board.get_board_data(n_samples)
        return raw[:N_CHANNELS].T.astype(np.float64)  # (n_samples, N_CHANNELS)

    def flush(self) -> None:
        """Discard all buffered samples."""
        if self._board is not None:
            self._board.get_board_data()
