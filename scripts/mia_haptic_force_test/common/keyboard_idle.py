"""Pure keyboard idle helper extracted from ``keyboard_emg_node``.

The split-node ``keyboard_emg_node`` publishes emulated EMG gestures
driven by terminal arrow keys and WASD.  Because terminals report
key-down events but not key-up events, the node treats a gesture as
"released" after a short idle window (default 0.08s).  This module
exposes the pure decision function so it can be unit-tested without a
TTY, without ``rclpy``, and without spinning up a ROS graph.
"""

from __future__ import annotations

from .constants import GESTURES, REST_GESTURE_LABEL

DEFAULT_KEY_IDLE_TIMEOUT_S: float = 0.08


def keyboard_gesture_state(
    gesture: str,
    last_key_time: float,
    now: float,
    *,
    timeout_s: float = DEFAULT_KEY_IDLE_TIMEOUT_S,
) -> tuple[str, int, float, float, float]:
    """Return ``(gesture_name, label, confidence, proportional, age_s)``.

    * ``last_key_time <= 0`` → REST, age=-1.0.
    * ``now - last_key_time <= timeout_s`` → active gesture, age=now-last_key.
    * Otherwise → REST, age=now-last_key.
    """
    if last_key_time > 0.0 and (now - last_key_time) <= timeout_s:
        label = GESTURES.get(gesture, REST_GESTURE_LABEL)
        return gesture, label, 1.0, 1.0, now - last_key_time
    if last_key_time > 0.0:
        return "REST", REST_GESTURE_LABEL, 0.0, 0.0, now - last_key_time
    return "REST", REST_GESTURE_LABEL, 0.0, 0.0, -1.0


__all__ = ["DEFAULT_KEY_IDLE_TIMEOUT_S", "keyboard_gesture_state"]
