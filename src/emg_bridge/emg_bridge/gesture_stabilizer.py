"""
Confidence-hysteresis (sticky) gesture stabilizer.

Applied after the classifier backend. The stabilizer prevents transient
misclassifications from flipping the output gesture. Backend-neutral —
operates on labels and confidence/probability values only.

When disabled, acts as a pure pass-through.
"""

from __future__ import annotations

from typing import Any

from .experiment_config import GestureStabilityConfig

RELEASE_GESTURE_LABELS = {0, 2}


class GestureStabilizer:
    """Sticky gesture selection with confidence hysteresis.

    Constructor takes a GestureStabilityConfig.  When enabled=False the
    stabilizer is a no-op pass-through; the returned label always matches
    the candidate.
    """

    def __init__(self, config: GestureStabilityConfig) -> None:
        self._config = config
        self._current_label: int = 0
        self._consecutive_frames: int = 0
        self._last_candidate: int | None = None
        self._time_since_last_switch: float = 999.0
        self._frames_held: int = 0

    @property
    def enabled(self) -> bool:
        return self._config.enabled

    def update(
        self,
        candidate_label: int,
        confidence: float,
        probabilities: list[float] | None,
        dt: float,
    ) -> tuple[int, dict[str, Any]]:
        """Accept a candidate label and return the stabilized output.

        Args:
            candidate_label: Raw label from the classifier backend.
            confidence: Top-class probability from the classifier.
            probabilities: Full posterior vector (can be None if backend
                does not supply it; margin checks are skipped in that case).
            dt: Seconds since the last call to update().

        Returns:
            (stabilized_label, debug_info) where debug_info contains:
                raw_label, stabilized, confidence, changed, frames_held,
                consecutive_frames, time_since_last_switch, enabled.
        """
        if not self._config.enabled:
            return candidate_label, {
                "raw_label": candidate_label,
                "stabilized": candidate_label,
                "confidence": confidence,
                "changed": False,
                "frames_held": 0,
                "consecutive_frames": 0,
                "time_since_last_switch": 0.0,
                "enabled": False,
            }

        # Bookkeeping
        self._time_since_last_switch += dt
        self._frames_held += 1

        # Track consecutive frames of the same candidate
        if candidate_label == self._last_candidate:
            self._consecutive_frames += 1
        else:
            self._consecutive_frames = 1
            self._last_candidate = candidate_label

        cfg = self._config
        is_release = candidate_label in RELEASE_GESTURE_LABELS

        # Effective confidence threshold
        effective_threshold = cfg.min_confidence_to_switch
        if is_release and cfg.release_lower_threshold > 0:
            effective_threshold = cfg.release_lower_threshold

        confidence_ok = confidence >= effective_threshold

        # Margin check
        margin_ok = True
        margin_val = 0.0
        if cfg.min_margin_to_switch > 0 and probabilities is not None and len(probabilities) >= 2:
            sorted_probs = sorted(probabilities, reverse=True)
            margin_val = sorted_probs[0] - sorted_probs[1]
            margin_ok = margin_val >= cfg.min_margin_to_switch

        frames_ok = self._consecutive_frames >= cfg.min_frames
        hold_ok = self._time_since_last_switch >= cfg.min_hold_s

        changed = False
        stabilized = self._current_label

        # Only attempt switch if candidate is different from current
        if candidate_label != self._current_label:
            # Release behavior
            if is_release and cfg.release_behavior == "allow_rest_immediately":
                self._current_label = candidate_label
                self._time_since_last_switch = 0.0
                self._frames_held = 0
                changed = True
            elif cfg.release_behavior == "hold_previous_when_uncertain" and not confidence_ok:
                pass  # stay on previous label
            elif confidence_ok and margin_ok and frames_ok and hold_ok:
                self._current_label = candidate_label
                self._time_since_last_switch = 0.0
                self._frames_held = 0
                changed = True
            else:
                # Uncertain — apply fallback behavior
                if cfg.fallback_behavior == "rest":
                    self._current_label = 0
                    self._time_since_last_switch = 0.0
                    self._frames_held = 0
                    changed = True

        debug: dict[str, Any] = {
            "raw_label": candidate_label,
            "stabilized": self._current_label,
            "confidence": confidence,
            "changed": changed,
            "frames_held": self._frames_held,
            "consecutive_frames": self._consecutive_frames,
            "time_since_last_switch": self._time_since_last_switch,
            "enabled": True,
            "margin": margin_val,
            "confidence_ok": confidence_ok,
            "margin_ok": margin_ok,
            "frames_ok": frames_ok,
            "hold_ok": hold_ok,
        }

        return self._current_label, debug
