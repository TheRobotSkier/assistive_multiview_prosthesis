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

    def __init__(
        self,
        buffer_size: int = 450_000,
        ip_address: str = "",
        ip_port: int = 4210,
    ) -> None:
        self._buffer_size = buffer_size
        self._ip_address = ip_address
        self._ip_port = ip_port
        self._board: BoardShim | None = None

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Connect to the MindRove board, auto-discovering IP if needed.

        Tries the provided ``ip_address`` first.  On failure, scans the
        local subnet for a device listening on the MindRove port (4210)
        and retries with any discovered address.
        """
        ip = self._ip_address
        try:
            self._try_connect(ip or "", self._ip_port)
        except Exception:
            if ip:  # explicit IP was given and failed — don't scan
                raise
            discovered = self._discover_board()
            if discovered:
                self._try_connect(discovered, self._ip_port)
            else:
                raise

    def _try_connect(self, ip_address: str, ip_port: int) -> None:
        params = MindRoveInputParams()
        if ip_address:
            params.ip_address = ip_address
        params.ip_port = ip_port
        self._board = BoardShim(BoardIds.MINDROVE_WIFI_BOARD, params)
        self._board.prepare_session()
        self._board.start_stream(self._buffer_size)

    @staticmethod
    def _discover_board() -> str:
        """Scan the local subnet for a MindRove board (port 4210)."""
        import socket
        import struct
        import fcntl

        port = 4210
        candidates: list[str] = []

        # Build a list of /24 subnets from local interfaces
        subnets: set[str] = set()
        try:
            import netifaces  # type: ignore[import-untyped]
            for iface in netifaces.interfaces():
                addrs = netifaces.ifaddresses(iface)
                for addr in addrs.get(netifaces.AF_INET, []):
                    ip_str = addr["addr"]
                    if ip_str.startswith("127."):
                        continue
                    parts = ip_str.rsplit(".", 1)
                    subnets.add(parts[0] + ".")
        except ImportError:
            # Fallback: scan common private subnets
            subnets = {"192.168.4.", "192.168.1.", "192.168.0.", "10.0.0."}

        for subnet in sorted(subnets):
            for host in range(1, 255):
                ip = f"{subnet}{host}"
                try:
                    sock = socket.create_connection((ip, port), timeout=0.05)
                    sock.close()
                    candidates.append(ip)
                    if len(candidates) >= 2:  # enough
                        break
                except OSError:
                    continue
            if candidates:
                break

        # If multiple found, try reverse-DNS to pick the MindRove
        for ip in candidates:
            try:
                hostname = socket.gethostbyaddr(ip)[0]
                if "mindrove" in hostname.lower():
                    return ip
            except OSError:
                pass
        return candidates[0] if candidates else ""


    def disconnect(self) -> None:
        """Stop streaming and release the board session."""
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
