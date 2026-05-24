"""Tests for IMU channel discovery in BoardReader."""

from __future__ import annotations

import sys
from unittest.mock import MagicMock, patch

import numpy as np
import pytest


# ── Helpers ────────────────────────────────────────────────────────────────────


def _build_mock_module(*, gyro_ch=None, accel_ch=None, ts_ch=-1, sampling_rate=500):
    """Build a mock mindrove.board_shim module with configurable IMU channels."""
    mod = MagicMock()
    mod.BoardShim = MagicMock()
    mod.BoardShim.get_gyro_channels.return_value = gyro_ch or []
    mod.BoardShim.get_accel_channels.return_value = accel_ch or []
    mod.BoardShim.get_timestamp_channel.return_value = ts_ch
    mod.BoardShim.get_sampling_rate.return_value = sampling_rate
    mod.BoardIds.MINDROVE_WIFI_BOARD = 0
    mod.MindRoveInputParams = MagicMock()
    return mod


def _import_board_reader(mock_mod):
    """Import BoardReader with mindrove.board_shim patched in sys.modules."""
    with patch.dict(sys.modules, {
        "mindrove": MagicMock(),
        "mindrove.board_shim": mock_mod,
    }):
        from emg_bridge.board_reader import BoardReader
        return BoardReader


def _make_reader_and_board(board_reader_cls, mock_mod, n_available=100):
    """Instantiate and connect BoardReader; return (reader, mock_board_instance)."""
    reader = board_reader_cls()
    reader.connect()
    board = mock_mod.BoardShim.return_value
    board.get_board_data_count.return_value = n_available
    board.is_prepared.return_value = True
    board.get_board_id.return_value = 0
    return reader, board


def _set_board_data(board, raw):
    """Configure mock board's get_board_data to return *raw*."""
    board.get_board_data.return_value = raw


# ── Tests ──────────────────────────────────────────────────────────────────────


class TestReadReturnsEmgOnly:
    """read() must always return only EMG data, never IMU."""

    def test_read_returns_emg_shape(self):
        mock_mod = _build_mock_module(gyro_ch=[8, 9, 10], accel_ch=[11, 12, 13], ts_ch=14)
        BR = _import_board_reader(mock_mod)
        reader, board = _make_reader_and_board(BR, mock_mod, n_available=10)

        # Raw data: 15 channels x 10 samples — only first 8 are EMG
        raw = np.random.randn(15, 10).astype(np.float64)
        _set_board_data(board, raw)

        result = reader.read(10)
        assert result.shape == (10, 8)
        np.testing.assert_array_almost_equal(result, raw[:8].T.astype(np.float64))


class TestReadFrame:
    """read_frame() returns a dict with emg, gyro, accel, timestamps."""

    def test_read_frame_with_imu(self):
        mock_mod = _build_mock_module(
            gyro_ch=[8, 9, 10], accel_ch=[11, 12, 13], ts_ch=14, sampling_rate=500,
        )
        BR = _import_board_reader(mock_mod)
        reader, board = _make_reader_and_board(BR, mock_mod, n_available=10)

        raw = np.arange(15 * 10, dtype=np.float64).reshape(15, 10)
        _set_board_data(board, raw)

        frame = reader.read_frame(10)

        assert isinstance(frame, dict)
        assert "emg" in frame
        assert "gyro" in frame
        assert "accel" in frame
        assert "timestamps" in frame

        np.testing.assert_array_almost_equal(frame["emg"], raw[:8].T)
        np.testing.assert_array_almost_equal(frame["gyro"], raw[8:11].T)
        np.testing.assert_array_almost_equal(frame["accel"], raw[11:14].T)
        np.testing.assert_array_almost_equal(frame["timestamps"], raw[14])

    def test_read_frame_emg_always_present(self):
        """EMG must always be present even if no IMU channels are discovered."""
        mock_mod = _build_mock_module(gyro_ch=None, accel_ch=None, ts_ch=-1)
        BR = _import_board_reader(mock_mod)
        reader, board = _make_reader_and_board(BR, mock_mod, n_available=5)

        raw = np.random.randn(8, 5).astype(np.float64)
        _set_board_data(board, raw)

        frame = reader.read_frame(5)
        assert frame["emg"].shape == (5, 8)
        assert frame["gyro"] is None
        assert frame["accel"] is None
        assert frame["timestamps"] is None


class TestChannelDiscovery:
    """gyro_channels and accel_channels properties reflect discovered channels."""

    def test_gyro_channels_property_returns_list(self):
        mock_mod = _build_mock_module(gyro_ch=[8, 9, 10], accel_ch=[11, 12, 13])
        BR = _import_board_reader(mock_mod)
        reader = BR()
        reader.connect()
        assert reader.gyro_channels == [8, 9, 10]
        assert reader.accel_channels == [11, 12, 13]

    def test_channels_empty_when_unavailable(self):
        mock_mod = _build_mock_module(gyro_ch=None, accel_ch=None)
        BR = _import_board_reader(mock_mod)
        reader = BR()
        reader.connect()
        assert reader.gyro_channels == []
        assert reader.accel_channels == []

    def test_channel_lists_are_copies(self):
        """Property returns a copy, not internal state reference."""
        mock_mod = _build_mock_module(gyro_ch=[8, 9, 10])
        BR = _import_board_reader(mock_mod)
        reader = BR()
        reader.connect()
        lst = reader.gyro_channels
        lst.append(999)
        assert reader.gyro_channels == [8, 9, 10]


class TestBackwardsCompat:
    """Existing API surfaces must not break."""

    def test_available_returns_int(self):
        mock_mod = _build_mock_module()
        BR = _import_board_reader(mock_mod)
        reader, board = _make_reader_and_board(BR, mock_mod, n_available=50)
        assert reader.available() == 50

    def test_flush_does_not_raise(self):
        mock_mod = _build_mock_module()
        BR = _import_board_reader(mock_mod)
        reader, board = _make_reader_and_board(BR, mock_mod)
        reader.flush()
        board.get_board_data.assert_called_with()

    def test_context_manager(self):
        mock_mod = _build_mock_module()
        BR = _import_board_reader(mock_mod)
        # Configure mock board before entering the context manager,
        # since connect() is called by __enter__.
        reader = BR()
        mock_board = mock_mod.BoardShim.return_value
        mock_board.get_board_data_count.return_value = 100
        mock_board.is_prepared.return_value = True
        with reader:
            assert reader.available() == 100
        mock_board.stop_stream.assert_called_once()
        mock_board.release_session.assert_called_once()

    def test_sampling_rate_property(self):
        mock_mod = _build_mock_module(sampling_rate=500)
        BR = _import_board_reader(mock_mod)
        reader, _ = _make_reader_and_board(BR, mock_mod)
        assert reader.sampling_rate == 500
