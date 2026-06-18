#!/usr/bin/env python3
"""Pure unit tests for the run-directory retention helper."""

from __future__ import annotations

import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.mia_haptic_force_test.common.log_retention import (
    list_run_dirs,
    prune_run_dirs,
)


_PREFIX = "mia_haptic_force_test"


def _make_run(base: Path, name: str) -> Path:
    d = base / name
    d.mkdir(parents=True, exist_ok=True)
    (d / "samples.csv").write_text("run_id\n" + name + "\n")
    return d


class ListRunDirsTest(unittest.TestCase):
    def test_empty_base_dir(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            result = list_run_dirs(Path(tmp), _PREFIX)
            self.assertEqual(result, [])

    def test_returns_only_matching_prefix(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            _make_run(base, f"{_PREFIX}_20260101_000000")
            _make_run(base, "other_prefix_20260101_000000")
            result = list_run_dirs(base, _PREFIX)
            self.assertEqual(len(result), 1)
            self.assertEqual(result[0].name, f"{_PREFIX}_20260101_000000")

    def test_newest_first_by_timestamp(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            _make_run(base, f"{_PREFIX}_20260101_000000")
            _make_run(base, f"{_PREFIX}_20260102_000000")
            _make_run(base, f"{_PREFIX}_20260103_000000")
            result = list_run_dirs(base, _PREFIX)
            self.assertEqual(
                [p.name for p in result],
                [
                    f"{_PREFIX}_20260103_000000",
                    f"{_PREFIX}_20260102_000000",
                    f"{_PREFIX}_20260101_000000",
                ],
            )

    def test_falls_back_to_mtime(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            old = _make_run(base, "mia_haptic_force_test_old")
            time.sleep(0.02)
            new = _make_run(base, "mia_haptic_force_test_new")
            result = list_run_dirs(base, _PREFIX)
            self.assertEqual(result[0].name, new.name)
            self.assertEqual(result[1].name, old.name)


class PruneRunDirsTest(unittest.TestCase):
    def test_keeps_newest_n(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            for i in range(9):
                _make_run(base, f"{_PREFIX}_2026010{i}_{i:06d}")
            deleted = prune_run_dirs(base, _PREFIX, keep_last=7)
            self.assertEqual(
                [p.name for p in deleted],
                [
                    f"{_PREFIX}_20260100_000000",
                    f"{_PREFIX}_20260101_000001",
                ],
            )
            remaining = sorted(p.name for p in base.iterdir())
            self.assertEqual(len(remaining), 7)
            self.assertNotIn(f"{_PREFIX}_20260100_000000", remaining)
            self.assertNotIn(f"{_PREFIX}_20260101_000001", remaining)
            self.assertIn(f"{_PREFIX}_20260108_000008", remaining)

    def test_preserved_dir_kept_even_if_old(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            # 8 newer runs and 1 very old run we want to preserve.
            for i in range(8):
                _make_run(base, f"{_PREFIX}_2026010{i}_{i:06d}")
            preserved = _make_run(base, f"{_PREFIX}_20250101_000000")
            # Set keep_last=7: without preserve, 20250101_000000 would be deleted.
            deleted = prune_run_dirs(
                base, _PREFIX, keep_last=7, preserve=preserved
            )
            self.assertNotIn(preserved, deleted)
            self.assertTrue(preserved.exists())

    def test_no_deletion_when_under_limit(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            for i in range(5):
                _make_run(base, f"{_PREFIX}_2026010{i}_{i:06d}")
            deleted = prune_run_dirs(base, _PREFIX, keep_last=7)
            self.assertEqual(deleted, [])
            self.assertEqual(len(list(base.iterdir())), 5)

    def test_preserved_dir_outside_base(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp) / "base"
            other = Path(tmp) / "other"
            base.mkdir()
            other.mkdir()
            for i in range(9):
                _make_run(base, f"{_PREFIX}_2026010{i}_{i:06d}")
            preserved = _make_run(other, f"{_PREFIX}_20250101_000000")
            deleted = prune_run_dirs(
                base, _PREFIX, keep_last=7, preserve=preserved
            )
            self.assertNotIn(preserved, deleted)
            self.assertTrue(preserved.exists())
            # The 2 oldest in base should be deleted.
            self.assertEqual(len(deleted), 2)

    def test_keep_last_zero_deletes_all(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            for i in range(3):
                _make_run(base, f"{_PREFIX}_2026010{i}_{i:06d}")
            deleted = prune_run_dirs(base, _PREFIX, keep_last=0)
            self.assertEqual(len(deleted), 3)
            self.assertEqual(len(list(base.iterdir())), 0)

    def test_invalid_keep_last_raises(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            with self.assertRaises(ValueError):
                prune_run_dirs(base, _PREFIX, keep_last=-1)


if __name__ == "__main__":
    unittest.main()
