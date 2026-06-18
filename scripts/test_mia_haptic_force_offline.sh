#!/usr/bin/env bash
# ── Thin shell wrapper for the PTY-driven offline end-to-end suite ──
#
# Spawns the Python suite (tests/mia_haptic_force_test/offline_suite.py)
# with a TIMEOUT default of 180s.  The suite itself opens a PTY, spawns
# the split-node launch under it, drives the keyboard via raw bytes,
# and verifies the full happy-path scenario.
#
# Usage:
#     bash scripts/test_mia_haptic_force_offline.sh
#     TIMEOUT=300 bash scripts/test_mia_haptic_force_offline.sh
#
# Exits 0 on PASS, non-zero on FAIL.  The suite writes its transcript,
# topic snapshot, and config overlay to the output dir on exit.

set -euo pipefail

# ── Resolve repo root ───────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── Defaults ────────────────────────────────────────────────────────────
TIMEOUT="${TIMEOUT:-180}"
OUTPUT_BASE="${OUTPUT_BASE:-/tmp/mia_haptic_force_test_offline}"
RUN_ID="${RUN_ID:-offline_suite_$(date +%Y%m%d_%H%M%S)}"
OUTPUT_DIR="$OUTPUT_BASE/$RUN_ID"
CONFIG_PATH="${CONFIG_PATH:-$REPO_ROOT/config/mia_haptic_force_test.yaml}"
PYTHONPATH="${PYTHONPATH:-}:$REPO_ROOT"
export PYTHONPATH

# ── Pre-flight ─────────────────────────────────────────────────────────
if [[ ! -f "$CONFIG_PATH" ]]; then
    echo "ERROR: config not found: $CONFIG_PATH" >&2
    exit 2
fi

mkdir -p "$OUTPUT_DIR"

# ── Run the suite with a wall-clock timeout ───────────────────────────
echo "[offline-shell] run_id=$RUN_ID output_dir=$OUTPUT_DIR timeout=${TIMEOUT}s"
timeout "${TIMEOUT}s" \
    python3 -m tests.mia_haptic_force_test.offline_suite \
        --config-path "$CONFIG_PATH" \
        --run-id "$RUN_ID" \
        --output-dir "$OUTPUT_DIR"
rc=$?

if [[ $rc -eq 0 ]]; then
    echo "[offline-shell] PASS — artifacts in $OUTPUT_DIR"
    exit 0
fi

echo "[offline-shell] FAIL (rc=$rc) — artifacts in $OUTPUT_DIR" >&2
exit "$rc"
