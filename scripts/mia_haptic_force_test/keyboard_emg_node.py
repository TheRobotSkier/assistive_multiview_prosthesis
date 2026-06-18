#!/usr/bin/env python3
"""Keyboard EMG emulation node.

Opt-in replacement for :mod:`emg_input_node` that lets the haptic force
test run **without the MindRove bracelet**.  Arrow keys and WASD are
mapped onto the existing ``/emg/*`` gesture contract so every downstream
node (supervisor, hand controller, haptics, logger, terminal UI) is
unchanged::

    left  / A  → OPEN
    right / D  → POWER
    down  / S  → FLEXION
    up    / W  → EXTENSION
    (no key held)        → REST

While a key is held, ``confidence=1.0`` and ``proportional=1.0`` are
published.  On release the stream returns to ``REST`` with
``confidence=0.0`` / ``proportional=0.0`` within one publish tick.

Terminal access
---------------
``ros2 launch`` does not reliably forward the parent terminal's stdin to
``ExecuteProcess`` children, and the terminal_ui_node reclaims the screen.
This node therefore prefers ``/dev/tty`` (the controlling terminal of the
session), but falls back to inherited stdin when the container runtime cannot
allocate a PTY for ``exec``.  The ``make test-grasp`` wrapper handles that
podman case by attaching stdin without ``-t`` and switching the host terminal
to raw mode first.  When no input source is available the node logs an error
and falls back to publishing REST, so the rest of the graph still runs.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
import tty
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String

_REPO_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if __name__ == "__main__" and _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.mia_haptic_force_test.common.constants import (
    GESTURES,
    KEYBOARD_GESTURE_MAP,
    REST_GESTURE_LABEL,
    TOPIC_EMG_AGE,
    TOPIC_EMG_CONFIDENCE,
    TOPIC_EMG_GESTURE,
    TOPIC_EMG_GESTURE_LABEL,
    TOPIC_EMG_PROPORTIONAL,
    TOPIC_EMG_SOURCE,
)
from scripts.mia_haptic_force_test.common.keyboard_idle import (
    DEFAULT_KEY_IDLE_TIMEOUT_S,
    keyboard_gesture_state,
)

_DEFAULT_PUBLISH_RATE_HZ: float = 50.0
_TTY_DEVICE: str = "/dev/tty"
_KEY_IDLE_TIMEOUT_S: float = DEFAULT_KEY_IDLE_TIMEOUT_S


class KeyboardEmgNode(Node):
    """Publishes emulated EMG gestures driven by terminal arrow keys."""

    def __init__(self, config_path: Optional[str] = None) -> None:
        super().__init__("keyboard_emg_node")
        self._config_path = config_path or os.path.join(
            _REPO_ROOT, "config", "mia_haptic_force_test.yaml"
        )

        publish_rate_hz = self._load_publish_rate()
        self.declare_parameter("publish_rate_hz", publish_rate_hz)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)

        # ── Publishers (identical topic contract to emg_input_node) ────────
        self._gesture_pub = self.create_publisher(String, TOPIC_EMG_GESTURE, 10)
        self._label_pub = self.create_publisher(Int32, TOPIC_EMG_GESTURE_LABEL, 10)
        self._conf_pub = self.create_publisher(Float32, TOPIC_EMG_CONFIDENCE, 10)
        self._prop_pub = self.create_publisher(Float32, TOPIC_EMG_PROPORTIONAL, 10)
        self._source_pub = self.create_publisher(String, TOPIC_EMG_SOURCE, 10)
        self._age_pub = self.create_publisher(Float32, TOPIC_EMG_AGE, 10)

        # ── State ──────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._gesture: str = "REST"
        self._last_key_time: float = 0.0
        self._running = True

        # ── Terminal + key reader thread ───────────────────────────────────
        self._tty_fd: Optional[int] = None
        self._tty_owned = False
        self._input_name: Optional[str] = None
        self._old_termios = None
        self._reader_thread: Optional[threading.Thread] = None
        self._init_terminal()

        self._source = "keyboard" if self._tty_fd is not None else "keyboard (no input)"
        self.get_logger().info(
            "Keyboard EMG emulation ready — "
            "←/A OPEN   →/D POWER   ↓/S FLEXION   ↑/W EXTENSION  (release→REST)"
        )
        if self._tty_fd is None:
            self.get_logger().error(
                "No keyboard input source available (/dev/tty or stdin). "
                "Arrow keys will not be read."
            )
        elif self._input_name == "stdin":
            self.get_logger().info(
                "Reading keyboard input from stdin fallback "
                "(host terminal must already be in raw mode)."
            )

        self._thread = threading.Thread(target=self._publish_loop, daemon=True, name="emg_pub")
        self._thread.start()

    # ── Config ────────────────────────────────────────────────────────────

    def _load_publish_rate(self) -> float:
        """Read ``emg.publish_rate_hz`` from the YAML config, else default."""
        import yaml

        try:
            with open(self._config_path) as f:
                raw = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(
                f"Config load failed ({self._config_path}): {exc}; using defaults"
            )
            return _DEFAULT_PUBLISH_RATE_HZ
        emg = raw.get("emg", {}) if isinstance(raw, dict) else {}
        return float(emg.get("publish_rate_hz", _DEFAULT_PUBLISH_RATE_HZ))

    # ── Terminal handling ─────────────────────────────────────────────────

    def _init_terminal(self) -> None:
        """Prefer /dev/tty; fall back to stdin when exec has no PTY."""
        import fcntl
        import termios

        errors: list[str] = []
        candidates: list[tuple[str, int, bool]] = []
        try:
            candidates.append((_TTY_DEVICE, os.open(_TTY_DEVICE, os.O_RDWR), True))
        except OSError as exc:
            errors.append(f"{_TTY_DEVICE}: {exc}")

        try:
            stdin_fd = sys.stdin.fileno()
        except (AttributeError, OSError, ValueError) as exc:
            errors.append(f"stdin: {exc}")
        else:
            candidates.append(("stdin", stdin_fd, False))

        for name, fd, owned in candidates:
            try:
                if os.isatty(fd):
                    self._old_termios = termios.tcgetattr(fd)
                    tty.setraw(fd)
                flags = fcntl.fcntl(fd, fcntl.F_GETFL)
                fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
            except Exception as exc:
                errors.append(f"{name}: {exc}")
                self._old_termios = None
                if owned:
                    try:
                        os.close(fd)
                    except Exception:
                        pass
                continue

            self._tty_fd = fd
            self._tty_owned = owned
            self._input_name = name
            self._reader_thread = threading.Thread(
                target=self._read_keys, daemon=True, name="key_reader"
            )
            self._reader_thread.start()
            return

        self._tty_fd = None
        self._tty_owned = False
        self._input_name = None
        if errors:
            self.get_logger().warn(
                "Keyboard input unavailable: " + "; ".join(errors)
            )

    def _restore_terminal(self) -> None:
        fd = self._tty_fd
        if self._old_termios is not None and fd is not None and os.isatty(fd):
            try:
                import termios

                termios.tcsetattr(fd, termios.TCSADRAIN, self._old_termios)
            except Exception:
                pass
        if fd is not None and self._tty_owned:
            try:
                os.close(fd)
            except Exception:
                pass
        self._tty_fd = None
        self._tty_owned = False
        self._input_name = None
        self._old_termios = None

    def _read_keys(self) -> None:
        """Background thread: read arrow-key escape sequences from the active input."""
        assert self._tty_fd is not None
        while self._running:
            try:
                ch = os.read(self._tty_fd, 1)
            except BlockingIOError:
                time.sleep(0.005)
                continue
            except OSError:
                # TTY closed (e.g. container teardown) — stop reading.
                self._running = False
                break
            if not ch:
                time.sleep(0.005)
                continue
            seq = ch
            if ch == b"\x1b":
                # Attempt to read the two remaining bytes of an arrow sequence
                # with a tiny grace window; non-blocking so we never stall.
                deadline = time.monotonic() + 0.02
                while len(seq) < 3 and time.monotonic() < deadline:
                    try:
                        seq += os.read(self._tty_fd, 1)
                    except BlockingIOError:
                        time.sleep(0.002)
                    except OSError:
                        break
            self._handle_seq(seq.decode("utf-8", errors="ignore"))

    def _handle_seq(self, seq: str) -> None:
        """Map a raw key sequence to a gesture and stamp the key time."""
        gesture = KEYBOARD_GESTURE_MAP.get(seq)
        if gesture is None:
            # Treat Ctrl-C / q as a no-op here; SIGINT/term already handled.
            return
        with self._lock:
            self._gesture = gesture
            self._last_key_time = time.monotonic()

    # ── Publish loop ──────────────────────────────────────────────────────

    def _current_state(self) -> tuple[str, int, float, float, float]:
        """Return (gesture, label, confidence, proportional, age)."""
        now = time.monotonic()
        with self._lock:
            gesture = self._gesture
            last_key = self._last_key_time
        return keyboard_gesture_state(gesture, last_key, now)


    def _publish_loop(self) -> None:
        while rclpy.ok() and self._running:
            t0 = time.monotonic()
            gesture, label, confidence, proportional, age = self._current_state()

            self._gesture_pub.publish(String(data=gesture))
            self._label_pub.publish(Int32(data=label))
            self._conf_pub.publish(Float32(data=confidence))
            self._prop_pub.publish(Float32(data=proportional))
            self._source_pub.publish(String(data=self._source))
            self._age_pub.publish(Float32(data=age))

            elapsed = time.monotonic() - t0
            remaining = self._dt - elapsed
            if remaining > 0.0:
                time.sleep(remaining)

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def stop(self) -> None:
        self._running = False
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._restore_terminal()

    def destroy_node(self) -> None:
        self.stop()
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config-path", default=None)
    known, remaining = parser.parse_known_args(args or [])
    rclpy.init(args=remaining)
    node = KeyboardEmgNode(config_path=known.config_path)

    def _signal_handler(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _signal_handler)
    signal.signal(signal.SIGINT, _signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
