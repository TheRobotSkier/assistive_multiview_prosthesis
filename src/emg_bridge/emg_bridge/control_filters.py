"""
Proportional control slew / acceleration limiting.

Wraps a raw proportional value in [0, 1] and outputs a stable,
limited value suitable for driving prosthetic hardware.
"""

from __future__ import annotations

import numpy as np

from .config import REST_LABEL
from .experiment_config import ProportionalSlewConfig


class ProportionalLimiter:
    """Stateful slew / acceleration limiter for proportional control signals.

    Applies configurable velocity, acceleration, and per-step caps so that
    aggressive EMG predictions do not cause jerky or unsafe prosthetic
    movements.
    """

    def __init__(self, config: ProportionalSlewConfig) -> None:
        self._config = config
        self._current: float = config.initial_value
        self._velocity: float = 0.0
        self._initialized: bool = False

    def step(
        self,
        raw: float,
        dt: float,
        current_gesture_label: int,
    ) -> float:
        """Advance the limiter one time-step.

        Args:
            raw: Raw proportional value in [0, 1].
            dt: Seconds since the last call to ``step``.
            current_gesture_label: Integer gesture label (REST=0).

        Returns:
            Limited proportional value in [0, 1].
        """
        if not self._config.enabled:
            return max(0.0, min(1.0, raw))

        raw = np.clip(raw, 0.0, 1.0)

        # First-call / stall detection
        if not self._initialized or dt > 1.0:
            self._current = self._config.initial_value
            self._velocity = 0.0
            self._initialized = True

        if dt <= 0.0:
            return self._current

        # REST gesture: cancel internal state, drive toward zero
        if self._config.reset_on_rest and current_gesture_label == REST_LABEL:
            self._velocity = 0.0
            target = 0.0
        else:
            target = raw

        desired_delta = target - self._current

        # ── Velocity limiting ──────────────────────────────────────────────
        vel_limit = self._config.max_velocity_per_s

        if self._config.max_fall_velocity_per_s > 0.0 and desired_delta < 0.0:
            vel_limit = self._config.max_fall_velocity_per_s

        if vel_limit > 0.0:
            max_step = vel_limit * dt
            desired_delta = np.clip(desired_delta, -max_step, max_step)

        # ── Acceleration limiting ──────────────────────────────────────────
        if self._config.max_accel_per_s2 > 0.0:
            desired_velocity = desired_delta / dt
            max_vel_change = self._config.max_accel_per_s2 * dt
            desired_velocity = np.clip(
                desired_velocity,
                self._velocity - max_vel_change,
                self._velocity + max_vel_change,
            )
            desired_delta = desired_velocity * dt

        # ── Per-step hard cap ──────────────────────────────────────────────
        if self._config.max_delta_per_step > 0.0:
            desired_delta = np.clip(
                desired_delta,
                -self._config.max_delta_per_step,
                self._config.max_delta_per_step,
            )

        # ── Clamp & snap ───────────────────────────────────────────────────
        output = np.clip(self._current + desired_delta, 0.0, 1.0)

        if self._config.snap_to_zero_below > 0.0 and output < self._config.snap_to_zero_below:
            output = 0.0

        # ── Update state ───────────────────────────────────────────────────
        self._velocity = (output - self._current) / dt
        self._current = output

        return output
