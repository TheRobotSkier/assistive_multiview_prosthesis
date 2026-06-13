#!/usr/bin/env python3
"""Unit tests for the keyframe buffer pure-logic core.

These tests exercise the ROS-free :class:`KeyframeBufferCore` and the
:class:`Keyframe` dataclass without rclpy.  They verify:

- Spatial gate logic (translation + rotation thresholds).
- Ring buffer eviction at capacity.
- ROI query correctness.
- Memory accounting via the ``memory_mb`` property.

Usage:
    python3 -m pytest src/keyframe_buffer/test/test_keyframe_buffer.py -v
"""

import numpy as np
import pytest

from keyframe_buffer.keyframe import Keyframe
from keyframe_buffer.keyframe_buffer_node import KeyframeBufferCore


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_pose(translation=(0.0, 0.0, 0.0), rot_z_deg=0.0) -> np.ndarray:
    """Build a (4, 4) SE(3) pose with a z-axis rotation."""
    T = np.eye(4)
    theta = np.deg2rad(rot_z_deg)
    T[:3, :3] = np.array([
        [np.cos(theta), -np.sin(theta), 0.0],
        [np.sin(theta),  np.cos(theta), 0.0],
        [0.0,            0.0,           1.0],
    ])
    T[:3, 3] = translation
    return T


def _make_cloud(n=100, seed=0) -> tuple[np.ndarray, np.ndarray]:
    """Generate a small synthetic (N, 3) xyz + (N, 3) rgb cloud."""
    rng = np.random.default_rng(seed)
    xyz = rng.uniform(-1, 1, (n, 3)).astype(np.float32)
    rgb = rng.integers(0, 255, (n, 3)).astype(np.uint8)
    return xyz, rgb


def _make_image(h=8, w=8) -> np.ndarray:
    return np.zeros((h, w, 3), dtype=np.uint8)


def _make_keyframe(
    translation=(0.0, 0.0, 0.0),
    rot_z_deg=0.0,
    camera_id="head",
    n_points=50,
    timestamp=0.0,
) -> Keyframe:
    xyz, rgb = _make_cloud(n_points)
    return Keyframe(
        timestamp=timestamp,
        camera_id=camera_id,
        cloud_xyz=xyz,
        cloud_rgb=rgb,
        image=_make_image(),
        K=np.eye(3),
        pose=_make_pose(translation, rot_z_deg),
        organized=False,
    )


# ---------------------------------------------------------------------------
# Keyframe dataclass
# ---------------------------------------------------------------------------

class TestKeyframeDataclass:
    def test_memory_mb_known_arrays(self):
        """memory_mb sums the nbytes of all stored arrays."""
        n = 1000
        xyz = np.zeros((n, 3), dtype=np.float32)   # 12,000 bytes
        rgb = np.zeros((n, 3), dtype=np.uint8)     #  3,000 bytes
        image = np.zeros((10, 10, 3), dtype=np.uint8)  # 300 bytes
        K = np.eye(3, dtype=np.float64)            # 72 bytes
        pose = np.eye(4, dtype=np.float64)         # 128 bytes

        kf = Keyframe(
            timestamp=0.0, camera_id="head",
            cloud_xyz=xyz, cloud_rgb=rgb, image=image,
            K=K, pose=pose, organized=False)

        expected_bytes = (12000 + 3000 + 300 + 72 + 128)
        expected_mb = expected_bytes / (1024.0 * 1024.0)
        assert abs(kf.memory_mb - expected_mb) < 1e-9

    def test_translation_property(self):
        kf = _make_keyframe(translation=(1.0, 2.0, 3.0))
        np.testing.assert_allclose(kf.translation, [1.0, 2.0, 3.0])

    def test_num_points_unorganized(self):
        kf = _make_keyframe(n_points=42)
        assert kf.num_points == 42

    def test_num_points_organized(self):
        cloud = np.zeros((10, 20, 3), dtype=np.float32)
        kf = Keyframe(
            timestamp=0.0, camera_id="head",
            cloud_xyz=cloud, cloud_rgb=np.zeros((10, 20, 3), dtype=np.uint8),
            image=_make_image(), K=np.eye(3), pose=np.eye(4),
            organized=True)
        assert kf.num_points == 200

    def test_slots_memory_efficiency(self):
        """Keyframe uses __slots__ via dataclass(slots=True)."""
        kf = _make_keyframe()
        # With slots, there should be no __dict__.
        assert not hasattr(kf, "__dict__")


# ---------------------------------------------------------------------------
# Spatial gate
# ---------------------------------------------------------------------------

class TestSpatialGate:
    def test_first_keyframe_always_accepted(self):
        """The first keyframe for a camera passes the gate (no reference)."""
        core = KeyframeBufferCore(
            max_keyframes_per_camera=10,
            spatial_gate_translation_m=0.10,
            spatial_gate_rotation_deg=15.0)
        core.update_pose("head", _make_pose((0, 0, 0)), stamp=0.0)

        xyz, rgb = _make_cloud()
        kf = core.try_create_keyframe(
            "head", xyz, rgb, _make_image(), np.eye(3), cloud_stamp=0.0)
        assert kf is not None
        assert len(core.all_keyframes("head")) == 1

    def test_reject_insufficient_translation(self):
        """Motion below the translation threshold is rejected."""
        core = KeyframeBufferCore(
            max_keyframes_per_camera=10,
            spatial_gate_translation_m=0.10,
            spatial_gate_rotation_deg=15.0)
        # First keyframe at origin
        core.update_pose("head", _make_pose((0, 0, 0)), stamp=0.0)
        xyz, rgb = _make_cloud()
        core.try_create_keyframe("head", xyz, rgb, _make_image(),
                                 np.eye(3), cloud_stamp=0.0)

        # Move only 0.05 m (below 0.10 threshold) with 20° rotation
        core.update_pose("head", _make_pose((0.05, 0, 0), rot_z_deg=20.0),
                         stamp=0.01)
        kf2 = core.try_create_keyframe("head", xyz, rgb, _make_image(),
                                       np.eye(3), cloud_stamp=0.01)
        assert kf2 is None
        assert len(core.all_keyframes("head")) == 1

    def test_reject_insufficient_rotation(self):
        """Motion below the rotation threshold is rejected."""
        core = KeyframeBufferCore(
            max_keyframes_per_camera=10,
            spatial_gate_translation_m=0.10,
            spatial_gate_rotation_deg=15.0)
        core.update_pose("head", _make_pose((0, 0, 0)), stamp=0.0)
        xyz, rgb = _make_cloud()
        core.try_create_keyframe("head", xyz, rgb, _make_image(),
                                 np.eye(3), cloud_stamp=0.0)

        # Move 0.20 m (above threshold) but only 5° rotation (below threshold)
        core.update_pose("head", _make_pose((0.20, 0, 0), rot_z_deg=5.0),
                         stamp=0.01)
        kf2 = core.try_create_keyframe("head", xyz, rgb, _make_image(),
                                       np.eye(3), cloud_stamp=0.01)
        assert kf2 is None
        assert len(core.all_keyframes("head")) == 1

    def test_accept_both_thresholds_exceeded(self):
        """Motion exceeding BOTH thresholds is accepted."""
        core = KeyframeBufferCore(
            max_keyframes_per_camera=10,
            spatial_gate_translation_m=0.10,
            spatial_gate_rotation_deg=15.0)
        core.update_pose("head", _make_pose((0, 0, 0)), stamp=0.0)
        xyz, rgb = _make_cloud()
        core.try_create_keyframe("head", xyz, rgb, _make_image(),
                                 np.eye(3), cloud_stamp=0.0)

        # Move 0.20 m AND 20° — both above thresholds
        core.update_pose("head", _make_pose((0.20, 0, 0), rot_z_deg=20.0),
                         stamp=0.01)
        kf2 = core.try_create_keyframe("head", xyz, rgb, _make_image(),
                                       np.eye(3), cloud_stamp=0.01)
        assert kf2 is not None
        assert len(core.all_keyframes("head")) == 2

    def test_gate_sequence(self):
        """Feed a sequence of poses; verify accept/reject pattern."""
        core = KeyframeBufferCore(
            max_keyframes_per_camera=50,
            spatial_gate_translation_m=0.10,
            spatial_gate_rotation_deg=15.0)

        # Pose 0: origin — accepted (first)
        core.update_pose("head", _make_pose((0, 0, 0), 0.0), stamp=0.0)
        xyz, rgb = _make_cloud()
        assert core.try_create_keyframe("head", xyz, rgb, _make_image(),
                                        np.eye(3), cloud_stamp=0.0) is not None

        # Pose 1: tiny move — rejected
        core.update_pose("head", _make_pose((0.01, 0, 0), 1.0), stamp=0.01)
        assert core.try_create_keyframe("head", xyz, rgb, _make_image(),
                                        np.eye(3), cloud_stamp=0.01) is None

        # Pose 2: big translation, tiny rotation — rejected
        core.update_pose("head", _make_pose((0.5, 0, 0), 2.0), stamp=0.02)
        assert core.try_create_keyframe("head", xyz, rgb, _make_image(),
                                        np.eye(3), cloud_stamp=0.02) is None

        # Pose 3: big translation + big rotation — accepted
        core.update_pose("head", _make_pose((0.5, 0.3, 0), 30.0), stamp=0.03)
        assert core.try_create_keyframe("head", xyz, rgb, _make_image(),
                                        np.eye(3), cloud_stamp=0.03) is not None

        assert len(core.all_keyframes("head")) == 2

    def test_no_pose_rejected(self):
        """Cloud without a pose is rejected."""
        core = KeyframeBufferCore()
        xyz, rgb = _make_cloud()
        kf = core.try_create_keyframe(
            "head", xyz, rgb, _make_image(), np.eye(3), cloud_stamp=0.0)
        assert kf is None
        diag = core.diagnostics()
        assert diag["per_camera"]["head"]["rejected_no_pose"] == 1


# ---------------------------------------------------------------------------
# Ring buffer eviction
# ---------------------------------------------------------------------------

class TestRingBufferEviction:
    def test_eviction_at_capacity(self):
        """Filling beyond max_keyframes drops the oldest."""
        core = KeyframeBufferCore(
            max_keyframes_per_camera=3,
            spatial_gate_translation_m=0.0,   # accept everything
            spatial_gate_rotation_deg=0.0)

        for i in range(5):
            pose = _make_pose((float(i) * 0.5, 0, 0), rot_z_deg=float(i) * 30)
            core.update_pose("head", pose, stamp=float(i) * 0.01)
            xyz, rgb = _make_cloud()
            core.try_create_keyframe("head", xyz, rgb, _make_image(),
                                     np.eye(3), cloud_stamp=float(i) * 0.01)

        kfs = core.all_keyframes("head")
        assert len(kfs) == 3  # capped at maxlen
        # Oldest two (i=0, i=1) should be evicted; remaining are i=2,3,4
        translations = [tuple(kf.translation) for kf in kfs]
        assert (1.0, 0.0, 0.0) in translations  # i=2
        assert (0.0, 0.0, 0.0) not in translations  # i=0 evicted

    def test_independent_cameras(self):
        """Each camera has its own ring buffer."""
        core = KeyframeBufferCore(
            max_keyframes_per_camera=2,
            spatial_gate_translation_m=0.0,
            spatial_gate_rotation_deg=0.0)

        for cam in ("head", "arm"):
            for i in range(4):
                pose = _make_pose((float(i), 0, 0), rot_z_deg=float(i) * 45)
                core.update_pose(cam, pose, stamp=float(i) * 0.01)
                xyz, rgb = _make_cloud()
                core.try_create_keyframe(cam, xyz, rgb, _make_image(),
                                         np.eye(3), cloud_stamp=float(i) * 0.01)

        assert len(core.all_keyframes("head")) == 2
        assert len(core.all_keyframes("arm")) == 2
        assert len(core.all_keyframes()) == 4


# ---------------------------------------------------------------------------
# ROI query
# ---------------------------------------------------------------------------

class TestROIQuery:
    def test_roi_returns_correct_subset(self):
        """Insert keyframes at known positions, query a region."""
        core = KeyframeBufferCore(
            max_keyframes_per_camera=50,
            spatial_gate_translation_m=0.0,
            spatial_gate_rotation_deg=0.0)

        positions = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
                     (2.0, 0.0, 0.0), (0.5, 0.5, 0.0)]
        for i, pos in enumerate(positions):
            pose = _make_pose(pos, rot_z_deg=float(i) * 45)
            core.update_pose("head", pose, stamp=float(i) * 0.01)
            xyz, rgb = _make_cloud()
            core.try_create_keyframe("head", xyz, rgb, _make_image(),
                                     np.eye(3), cloud_stamp=float(i) * 0.01)

        # Query around origin with radius 0.8 → should get positions 0 and 3
        # (0.5, 0.5, 0) is at distance sqrt(0.5) ≈ 0.707 from origin
        result = core.get_in_roi(np.array([0, 0, 0]), 0.8)
        result_t = sorted([tuple(kf.translation) for kf in result])
        assert (0.0, 0.0, 0.0) in result_t
        assert (0.5, 0.5, 0.0) in result_t
        assert (1.0, 0.0, 0.0) not in result_t

    def test_roi_empty_region(self):
        """A region with no keyframes returns an empty list."""
        core = KeyframeBufferCore(
            max_keyframes_per_camera=50,
            spatial_gate_translation_m=0.0,
            spatial_gate_rotation_deg=0.0)
        pose = _make_pose((0, 0, 0), rot_z_deg=0.0)
        core.update_pose("head", pose, stamp=0.0)
        xyz, rgb = _make_cloud()
        core.try_create_keyframe("head", xyz, rgb, _make_image(),
                                 np.eye(3), cloud_stamp=0.0)

        result = core.get_in_roi(np.array([10, 10, 10]), 0.5)
        assert result == []

    def test_roi_across_cameras(self):
        """ROI query spans both cameras."""
        core = KeyframeBufferCore(
            max_keyframes_per_camera=50,
            spatial_gate_translation_m=0.0,
            spatial_gate_rotation_deg=0.0)
        for cam in ("head", "arm"):
            pose = _make_pose((0.1, 0, 0), rot_z_deg=0.0)
            core.update_pose(cam, pose, stamp=0.0)
            xyz, rgb = _make_cloud()
            core.try_create_keyframe(cam, xyz, rgb, _make_image(),
                                     np.eye(3), cloud_stamp=0.0)

        result = core.get_in_roi(np.array([0, 0, 0]), 0.5)
        assert len(result) == 2

    def test_roi_boundary(self):
        """A keyframe exactly at radius distance is included (<=)."""
        core = KeyframeBufferCore(
            max_keyframes_per_camera=50,
            spatial_gate_translation_m=0.0,
            spatial_gate_rotation_deg=0.0)
        pose = _make_pose((0.5, 0, 0), rot_z_deg=0.0)
        core.update_pose("head", pose, stamp=0.0)
        xyz, rgb = _make_cloud()
        core.try_create_keyframe("head", xyz, rgb, _make_image(),
                                 np.eye(3), cloud_stamp=0.0)

        # Exactly at radius 0.5 → included
        result = core.get_in_roi(np.array([0, 0, 0]), 0.5)
        assert len(result) == 1


# ---------------------------------------------------------------------------
# Memory accounting & diagnostics
# ---------------------------------------------------------------------------

class TestMemoryAndDiagnostics:
    def test_diagnostics_reports_counts(self):
        core = KeyframeBufferCore(
            max_keyframes_per_camera=50,
            spatial_gate_translation_m=0.0,
            spatial_gate_rotation_deg=0.0)
        for i in range(3):
            pose = _make_pose((float(i), 0, 0), rot_z_deg=float(i) * 45)
            core.update_pose("head", pose, stamp=float(i) * 0.01)
            xyz, rgb = _make_cloud()
            core.try_create_keyframe("head", xyz, rgb, _make_image(),
                                     np.eye(3), cloud_stamp=float(i) * 0.01)

        diag = core.diagnostics()
        assert diag["total_count"] == 3
        assert diag["per_camera"]["head"]["count"] == 3
        assert diag["per_camera"]["head"]["accepted"] == 3
        assert diag["total_memory_mb"] > 0.0

    def test_memory_under_budget_at_capacity(self):
        """At 100 keyframes (50×2 cameras) memory must be < 350 MB.

        Uses realistic unorganised cloud sizes (~115k pts, V6 §10).
        """
        n_points = 115000
        core = KeyframeBufferCore(
            max_keyframes_per_camera=50,
            spatial_gate_translation_m=0.0,
            spatial_gate_rotation_deg=0.0)
        for cam in ("head", "arm"):
            for i in range(50):
                pose = _make_pose(
                    (float(i) * 0.2, 0, 0), rot_z_deg=float(i) * 20)
                core.update_pose(cam, pose, stamp=float(i) * 0.01)
                xyz, rgb = _make_cloud(n_points, seed=i)
                core.try_create_keyframe(cam, xyz, rgb, _make_image(),
                                         np.eye(3), cloud_stamp=float(i) * 0.01)

        diag = core.diagnostics()
        assert diag["total_count"] == 100
        # ~115k pts × (12 + 3) bytes ≈ 1.7 MB/kf + image ≈ 0.0003 MB
        # 100 keyframes ≈ ~175 MB — well under 350 MB
        assert diag["total_memory_mb"] < 350.0, (
            f"Memory {diag['total_memory_mb']:.1f} MB exceeds 350 MB budget")


# ---------------------------------------------------------------------------
# Cloud organisation detection
# ---------------------------------------------------------------------------

class TestCloudOrganizationDetection:
    def test_detect_unorganized(self):
        core = KeyframeBufferCore()
        organized = core.detect_cloud_organization("head", height=1)
        assert organized is False

    def test_detect_organized(self):
        core = KeyframeBufferCore()
        organized = core.detect_cloud_organization("head", height=480)
        assert organized is True

    def test_detection_cached(self):
        """Organisation is detected once and cached."""
        core = KeyframeBufferCore()
        core.detect_cloud_organization("head", height=1)
        # Subsequent detection returns cached value even with different height
        organized = core.detect_cloud_organization("head", height=480)
        assert organized is False  # cached as unorganised
