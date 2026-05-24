"""Tests for the GestureStabilizer confidence-hysteresis module."""

from __future__ import annotations

import pytest

from emg_bridge.gesture_stabilizer import GestureStabilizer, RELEASE_GESTURE_LABELS
from emg_bridge.experiment_config import GestureStabilityConfig


def make_config(**overrides) -> GestureStabilityConfig:
    """Build a GestureStabilityConfig with sensible test defaults."""
    defaults = {
        "enabled": True,
        "min_confidence_to_switch": 0.55,
        "min_margin_to_switch": 0.0,
        "min_frames": 3,
        "min_hold_s": 0.0,
        "release_behavior": "allow_rest_immediately",
        "fallback_behavior": "hold_previous",
        "release_lower_threshold": 0.0,
    }
    defaults.update(overrides)
    return GestureStabilityConfig(**defaults)


# ── disabled = pass-through ─────────────────────────────────────────────────────

class TestDisabled:
    def test_pass_through(self):
        cfg = make_config(enabled=False)
        gs = GestureStabilizer(cfg)
        label, info = gs.update(1, 0.3, [0.1, 0.3, 0.1, 0.4, 0.1], 0.1)
        assert label == 1
        assert info["raw_label"] == 1
        assert info["stabilized"] == 1
        assert info["changed"] is False
        assert info["enabled"] is False

    def test_pass_through_zero_confidence(self):
        cfg = make_config(enabled=False)
        gs = GestureStabilizer(cfg)
        label, info = gs.update(3, 0.0, None, 0.1)
        assert label == 3
        assert info["enabled"] is False

    def test_enabled_property(self):
        gs = GestureStabilizer(make_config(enabled=False))
        assert gs.enabled is False
        gs2 = GestureStabilizer(make_config(enabled=True))
        assert gs2.enabled is True


# ── min_confidence_to_switch ────────────────────────────────────────────────────

class TestMinConfidence:
    def test_low_confidence_holds(self):
        cfg = make_config(min_confidence_to_switch=0.55, min_frames=1, min_hold_s=0.0)
        gs = GestureStabilizer(cfg)
        label, info = gs.update(1, 0.40, None, 0.1)
        assert label == 0  # defaults to REST
        assert info["changed"] is False

    def test_high_confidence_switches(self):
        cfg = make_config(min_confidence_to_switch=0.55, min_frames=1, min_hold_s=0.0)
        gs = GestureStabilizer(cfg)
        label, info = gs.update(1, 0.80, None, 0.1)
        assert label == 1
        assert info["changed"] is True


# ── min_frames ──────────────────────────────────────────────────────────────────

class TestMinFrames:
    def test_needs_n_consecutive(self):
        cfg = make_config(min_frames=3, min_confidence_to_switch=0.55)
        gs = GestureStabilizer(cfg)
        # First 2 frames — should not switch
        label1, info1 = gs.update(1, 0.80, None, 0.1)
        assert label1 == 0
        assert info1["changed"] is False

        label2, info2 = gs.update(1, 0.80, None, 0.1)
        assert label2 == 0
        assert info2["changed"] is False

        # 3rd frame — switch
        label3, info3 = gs.update(1, 0.80, None, 0.1)
        assert label3 == 1
        assert info3["changed"] is True
        assert info3["consecutive_frames"] == 3

    def test_resets_on_different_candidate(self):
        cfg = make_config(min_frames=3, min_confidence_to_switch=0.55)
        gs = GestureStabilizer(cfg)
        gs.update(1, 0.80, None, 0.1)
        gs.update(1, 0.80, None, 0.1)
        # Interrupt with different candidate
        gs.update(3, 0.80, None, 0.1)
        label, info = gs.update(3, 0.80, None, 0.1)
        assert info["consecutive_frames"] == 2  # reset to 2


# ── min_hold_s ──────────────────────────────────────────────────────────────────

class TestMinHold:
    def test_cant_switch_within_hold_time(self):
        cfg = make_config(min_hold_s=0.5, min_frames=1, min_confidence_to_switch=0.55)
        gs = GestureStabilizer(cfg)
        label1, _ = gs.update(1, 0.80, None, 0.1)
        assert label1 == 1  # switched

        # Try switching immediately to another gesture
        label2, info2 = gs.update(3, 0.80, None, 0.05)
        assert label2 == 1  # still held
        assert info2["changed"] is False

    def test_switches_after_hold_time(self):
        cfg = make_config(min_hold_s=0.2, min_frames=1, min_confidence_to_switch=0.55)
        gs = GestureStabilizer(cfg)
        gs.update(1, 0.80, None, 0.1)
        # Wait enough time
        label, info = gs.update(3, 0.80, None, 0.2)
        assert label == 3
        assert info["changed"] is True


# ── release_behavior = "allow_rest_immediately" ─────────────────────────────────

class TestReleaseAllowRestImmediately:
    def test_release_switches_immediately_even_low_confidence(self):
        cfg = make_config(
            release_behavior="allow_rest_immediately",
            min_confidence_to_switch=0.80,
            min_frames=5,
        )
        gs = GestureStabilizer(cfg)
        # First switch away from REST to establish a non-release gesture
        for _ in range(5):
            gs.update(1, 0.90, None, 0.1)
        label_on, _ = gs.update(1, 0.90, None, 0.1)
        assert label_on == 1
        # Now REST should switch immediately despite low confidence and min_frames=5
        label, info = gs.update(0, 0.1, None, 0.1)
        assert label == 0
        assert info["changed"] is True

    def test_release_switches_to_rest(self):
        cfg = make_config(
            release_behavior="allow_rest_immediately",
            min_confidence_to_switch=0.80,
            min_frames=5,
        )
        gs = GestureStabilizer(cfg)
        # First switch to gesture 1
        gs.update(1, 0.90, None, 0.1)
        gs.update(1, 0.90, None, 0.1)
        gs.update(1, 0.90, None, 0.1)
        gs.update(1, 0.90, None, 0.1)
        gs.update(1, 0.90, None, 0.1)
        # REST should switch immediately even though min_frames=5
        label, info = gs.update(0, 0.1, None, 0.1)
        assert label == 0
        assert info["changed"] is True

    def test_release_to_label_2_immediately(self):
        cfg = make_config(
            release_behavior="allow_rest_immediately",
            min_confidence_to_switch=0.80,
            min_frames=5,
        )
        gs = GestureStabilizer(cfg)
        label, info = gs.update(2, 0.2, None, 0.1)
        assert label == 2
        assert info["changed"] is True


# ── release_behavior = "require_threshold" ──────────────────────────────────────

class TestReleaseRequireThreshold:
    def test_release_needs_threshold_too(self):
        cfg = make_config(
            release_behavior="require_threshold",
            min_confidence_to_switch=0.55,
            min_frames=1,
            release_lower_threshold=0.0,
        )
        gs = GestureStabilizer(cfg)
        # First switch to gesture 1 with high confidence
        gs.update(1, 0.80, None, 0.1)
        label_after, _ = gs.update(1, 0.80, None, 0.1)
        assert label_after == 1
        # REST with low confidence — should not switch (needs threshold)
        label, info = gs.update(0, 0.30, None, 0.1)
        assert label == 1  # held at gesture 1
        assert info["changed"] is False
        # REST with enough confidence — should switch
        label2, info2 = gs.update(0, 0.60, None, 0.1)
        assert label2 == 0
        assert info2["changed"] is True

    def test_release_with_enough_confidence_switches(self):
        cfg = make_config(
            release_behavior="require_threshold",
            min_confidence_to_switch=0.55,
            min_frames=1,
        )
        gs = GestureStabilizer(cfg)
        # First switch to a non-release gesture
        gs.update(1, 0.80, None, 0.1)
        label_after, _ = gs.update(1, 0.80, None, 0.1)
        assert label_after == 1
        # REST with enough confidence should switch
        label, info = gs.update(0, 0.60, None, 0.1)
        assert label == 0
        assert info["changed"] is True

    def test_release_lower_threshold_used(self):
        cfg = make_config(
            release_behavior="require_threshold",
            min_confidence_to_switch=0.80,
            release_lower_threshold=0.30,
            min_frames=1,
        )
        gs = GestureStabilizer(cfg)
        # First switch to gesture 1 with high confidence
        gs.update(1, 0.90, None, 0.1)
        label_on, _ = gs.update(1, 0.90, None, 0.1)
        assert label_on == 1

        # Non-release candidate with 0.50 confidence (< min_confidence_to_switch=0.80)
        # should not cause switch from gesture 1
        label_non, _ = gs.update(3, 0.50, None, 0.1)
        assert label_non == 1  # held at 1

        # Release (REST) only needs release_lower_threshold=0.30, confidence=0.40 works
        label_rel, info_rel = gs.update(0, 0.40, None, 0.1)
        assert label_rel == 0
        assert info_rel["changed"] is True


# ── release_behavior = "hold_previous_when_uncertain" ───────────────────────────

class TestHoldPreviousWhenUncertain:
    def test_holds_previous_when_confidence_low(self):
        cfg = make_config(
            release_behavior="hold_previous_when_uncertain",
            min_confidence_to_switch=0.55,
            min_frames=1,
        )
        gs = GestureStabilizer(cfg)
        # Switch to gesture 1 first
        gs.update(1, 0.80, None, 0.1)
        label_after, _ = gs.update(1, 0.80, None, 0.1)
        assert label_after == 1

        # Low confidence — should hold gesture 1
        label, info = gs.update(0, 0.30, None, 0.1)
        assert label == 1  # held previous
        assert info["changed"] is False


# ── fallback_behavior ───────────────────────────────────────────────────────────

class TestFallbackBehavior:
    def test_rest_fallback(self):
        cfg = make_config(
            fallback_behavior="rest",
            release_behavior="require_threshold",
            min_confidence_to_switch=0.90,
            min_frames=1,
        )
        gs = GestureStabilizer(cfg)
        # Switch to gesture 1 with high confidence
        gs.update(1, 0.95, None, 0.1)
        label_after, _ = gs.update(1, 0.95, None, 0.1)
        assert label_after == 1

        # New non-release candidate with low confidence → fallback to rest
        label, info = gs.update(3, 0.30, None, 0.1)
        assert label == 0
        assert info["changed"] is True

    def test_hold_previous_fallback(self):
        cfg = make_config(
            fallback_behavior="hold_previous",
            release_behavior="require_threshold",
            min_confidence_to_switch=0.90,
            min_frames=1,
        )
        gs = GestureStabilizer(cfg)
        gs.update(1, 0.95, None, 0.1)
        gs.update(1, 0.95, None, 0.1)

        # Low confidence → hold previous
        label, info = gs.update(3, 0.30, None, 0.1)
        assert label == 1
        assert info["changed"] is False


# ── min_margin_to_switch ────────────────────────────────────────────────────────

class TestMargin:
    def test_margin_prevents_ambiguous_switch(self):
        cfg = make_config(
            min_margin_to_switch=0.20,
            min_confidence_to_switch=0.40,
            min_frames=1,
        )
        gs = GestureStabilizer(cfg)
        probs = [0.1, 0.45, 0.05, 0.35, 0.05]  # best=1(0.45), second=3(0.35), margin=0.10
        label, info = gs.update(1, 0.45, probs, 0.1)
        assert label == 0  # margin too small, held
        assert info["changed"] is False
        assert info["margin"] == pytest.approx(0.10)

    def test_wide_margin_allows_switch(self):
        cfg = make_config(
            min_margin_to_switch=0.20,
            min_confidence_to_switch=0.40,
            min_frames=1,
        )
        gs = GestureStabilizer(cfg)
        probs = [0.1, 0.60, 0.05, 0.10, 0.15]  # best=1(0.60), second=4(0.15), margin=0.45
        label, info = gs.update(1, 0.60, probs, 0.1)
        assert label == 1
        assert info["changed"] is True
        assert info["margin"] == pytest.approx(0.45)

    def test_margin_skipped_when_probs_is_none(self):
        cfg = make_config(
            min_margin_to_switch=0.20,
            min_confidence_to_switch=0.40,
            min_frames=1,
        )
        gs = GestureStabilizer(cfg)
        label, info = gs.update(1, 0.60, None, 0.1)
        assert label == 1
        assert info["changed"] is True
        assert info["margin"] == 0.0

    def test_margin_skipped_when_config_zero(self):
        cfg = make_config(
            min_margin_to_switch=0.0,
            min_confidence_to_switch=0.40,
            min_frames=1,
        )
        gs = GestureStabilizer(cfg)
        probs = [0.1, 0.45, 0.05, 0.35, 0.05]
        label, info = gs.update(1, 0.45, probs, 0.1)
        assert label == 1
        assert info["changed"] is True


# ── edge cases ──────────────────────────────────────────────────────────────────

class TestEdgeCases:
    def test_initial_state_is_rest(self):
        gs = GestureStabilizer(make_config())
        label, info = gs.update(0, 0.90, None, 0.1)
        assert label == 0

    def test_confidence_zero(self):
        cfg = make_config(
            min_confidence_to_switch=0.55,
            min_frames=1,
            release_behavior="require_threshold",
        )
        gs = GestureStabilizer(cfg)
        label, info = gs.update(1, 0.0, None, 0.1)
        assert label == 0
        assert info["changed"] is False

    def test_confidence_one(self):
        cfg = make_config(min_confidence_to_switch=0.55, min_frames=1)
        gs = GestureStabilizer(cfg)
        label, info = gs.update(3, 1.0, None, 0.1)
        assert label == 3
        assert info["changed"] is True

    def test_rapid_switching_prevented_by_frames(self):
        cfg = make_config(min_frames=3, min_confidence_to_switch=0.55)
        gs = GestureStabilizer(cfg)
        # Switch to 1
        for _ in range(3):
            gs.update(1, 0.80, None, 0.1)
        label, _ = gs.update(1, 0.80, None, 0.1)
        assert label == 1

        # Try to switch to 4 with only 1 frame — should be held
        label, info = gs.update(4, 0.80, None, 0.1)
        assert label == 1
        assert info["changed"] is False

    def test_same_label_no_change_flag(self):
        cfg = make_config(min_frames=1, min_confidence_to_switch=0.55)
        gs = GestureStabilizer(cfg)
        gs.update(1, 0.80, None, 0.1)
        label, info = gs.update(1, 0.90, None, 0.1)
        assert label == 1
        assert info["changed"] is False  # already on 1

    def test_frames_held_increments(self):
        cfg = make_config(min_frames=1, min_confidence_to_switch=0.55)
        gs = GestureStabilizer(cfg)
        # call 1: switches to 1, _frames_held reset to 0
        _, info1 = gs.update(1, 0.80, None, 0.1)
        assert info1["frames_held"] == 0
        # call 2: _frames_held = 1
        _, info2 = gs.update(1, 0.80, None, 0.2)
        assert info2["frames_held"] == 1
        # call 3: _frames_held = 2
        _, info3 = gs.update(1, 0.80, None, 0.15)
        assert info3["frames_held"] == 2

    def test_debug_info_keys_present(self):
        gs = GestureStabilizer(make_config())
        _, info = gs.update(1, 0.80, None, 0.1)
        expected_keys = {
            "raw_label", "stabilized", "confidence", "changed",
            "frames_held", "consecutive_frames", "time_since_last_switch",
            "enabled", "margin", "confidence_ok", "margin_ok",
            "frames_ok", "hold_ok",
        }
        assert expected_keys.issubset(set(info.keys()))

    def test_release_labels_set(self):
        assert RELEASE_GESTURE_LABELS == {0, 2}

    def test_full_cycle_power_to_rest(self):
        """Simulate a full gesture cycle: REST -> POWER -> REST."""
        cfg = make_config(min_frames=3, min_hold_s=0.1, min_confidence_to_switch=0.55)
        gs = GestureStabilizer(cfg)

        # Hold POWER for 3 frames
        for _ in range(3):
            gs.update(1, 0.80, None, 0.1)
        label, _ = gs.update(1, 0.80, None, 0.1)
        assert label == 1  # switched to POWER

        # Wait hold time
        gs.update(1, 0.80, None, 0.2)  # dt > min_hold_s

        # REST — should switch immediately (allow_rest_immediately)
        label2, info2 = gs.update(0, 0.1, None, 0.1)
        assert label2 == 0
        assert info2["changed"] is True


# ── Integration with GestureStabilityConfig defaults ────────────────────────────

class TestConfigDefaults:
    def test_default_config_values(self):
        cfg = GestureStabilityConfig()
        assert cfg.enabled is False
        assert cfg.min_confidence_to_switch == 0.55
        assert cfg.min_margin_to_switch == 0.0
        assert cfg.min_frames == 3
        assert cfg.min_hold_s == 0.0
        assert cfg.release_behavior == "allow_rest_immediately"
        assert cfg.fallback_behavior == "hold_previous"
        assert cfg.release_lower_threshold == 0.0

    def test_frozen_dataclass(self):
        cfg = GestureStabilityConfig()
        with pytest.raises(Exception):
            cfg.enabled = True  # type: ignore[misc]
