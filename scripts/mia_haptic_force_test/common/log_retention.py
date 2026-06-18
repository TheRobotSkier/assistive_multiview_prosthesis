"""Log-directory retention for the mia_haptic_force_test run tree.

Both the split-node ``logger_node.py`` and the monolithic ``CsvLogger`` in
``scripts/mia_haptic_force_test.py`` create one run directory per launch
under the configured ``output_dir``.  This helper trims that tree so only
the newest *keep_last* runs survive (default 7), keeping the current run
even if it would otherwise be pruned.

The helper is intentionally pure: it does not import ROS, the legacy
``yaml`` config, or the runtime nodes.  Tests exercise it on a
``tempfile.TemporaryDirectory`` populated with synthetic run directories.
"""

from __future__ import annotations

import re
import shutil
from pathlib import Path
from typing import Iterable


_RUN_TIMESTAMP_RE = re.compile(r"_(\d{8}_\d{6})$")


def _parse_timestamp(name: str) -> tuple[int, int, int]:
    """Return (timestamp_int, mtime_ns, original_index) for ordering *name*.

    The timestamp suffix ``_%Y%m%d_%H%M%S`` is preferred because it matches
    what both the split-node logger and the monolith CsvLogger emit
    (``logger_node.py:131-133`` and ``mia_haptic_force_test.py:451-453``
    share the same ``f"{prefix}_{stamp}"`` format).  When the suffix is
    missing we fall back to the directory's mtime so unbranded run dirs
    still get a stable order.
    """
    match = _RUN_TIMESTAMP_RE.search(name)
    if match:
        # Convert YYYYMMDD_HHMMSS to a sortable integer.
        stamp_str = match.group(1)
        try:
            return (int(stamp_str), 0, 0)
        except ValueError:
            pass
    return (0, 0, 0)


def _mtime_ns(path: Path) -> int:
    try:
        return path.stat().st_mtime_ns
    except OSError:
        return 0


def list_run_dirs(
    base_dir: Path, prefix: str, *, include: Iterable[Path] | None = None
) -> list[Path]:
    """Return matching run directories under *base_dir*, newest first.

    The *include* iterable lets callers add ad-hoc paths (e.g. the current
    run dir, which lives outside *base_dir*) without changing the sort
    behaviour.
    """
    if not base_dir.exists():
        candidates: list[Path] = list(include or [])
    else:
        candidates = [p for p in base_dir.iterdir() if p.is_dir() and p.name.startswith(prefix)]
        if include:
            candidates.extend(include)
    keyed: list[tuple[tuple[int, int, int], int, Path]] = []
    for idx, path in enumerate(candidates):
        if not path.exists():
            continue
        ts = _parse_timestamp(path.name)
        keyed.append((ts, _mtime_ns(path), path))
    # Newest first: largest timestamp first; for ties, largest mtime first;
    # final tiebreaker preserves discovery order so the result is stable.
    keyed.sort(key=lambda item: (item[0][0], item[1]), reverse=True)
    return [item[2] for item in keyed]


def prune_run_dirs(
    base_dir: Path,
    prefix: str,
    *,
    keep_last: int = 7,
    preserve: Path | None = None,
) -> list[Path]:
    """Delete old run directories so only the newest *keep_last* survive.

    Returns the list of deleted directories in deletion order (oldest
    first).  *preserve* is always kept even if it would otherwise be
    pruned; pass the current run directory to avoid deleting the run in
    progress.
    """
    if keep_last < 0:
        raise ValueError("keep_last must be >= 0")
    runs = list_run_dirs(base_dir, prefix, include=[preserve] if preserve else None)
    # Deduplicate (preserve may already be in the list).
    seen: set[Path] = set()
    deduped: list[Path] = []
    for path in runs:
        try:
            resolved = path.resolve()
        except OSError:
            resolved = path
        if resolved in seen:
            continue
        seen.add(resolved)
        deduped.append(path)
    if preserve is not None:
        try:
            preserve_resolved = preserve.resolve()
        except OSError:
            preserve_resolved = preserve
        deduped = [p for p in deduped if p.resolve() != preserve_resolved] if False else deduped
    # The first keep_last are kept; the rest are deleted, oldest first.
    keep = deduped[:keep_last]
    delete = list(reversed(deduped[keep_last:]))
    deleted: list[Path] = []
    for path in delete:
        if preserve is not None:
            try:
                if path.resolve() == preserve.resolve():
                    continue
            except OSError:
                pass
        try:
            shutil.rmtree(path)
            deleted.append(path)
        except OSError:
            # If deletion fails, leave the directory in place; the next
            # call to prune_run_dirs will try again.
            continue
    return deleted


__all__ = ["list_run_dirs", "prune_run_dirs"]
