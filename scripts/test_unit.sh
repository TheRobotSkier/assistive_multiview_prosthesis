#!/bin/bash
# test_unit.sh — run pytest unit tests across all packages
#
# Produces JUnit XML output for timing analysis and baseline comparison.
# Exit code is non-zero if any test fails.
#
# The tests use two import styles:
#   - Flat:    `from twist_propagation_node import ...`  (needs source dir on PYTHONPATH)
#   - Nested:  `from gtsam_tracker.factor_graph import ...` (needs package root on PYTHONPATH)
# We set PYTHONPATH to cover both.

# Source ROS workspace BEFORE set -euo pipefail — ROS setup scripts
# reference unset variables (e.g. AMENT_TRACE_SETUP_FILES) that would
# trigger the -u (nounset) guard.
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f /prosthesis_ws/install/setup.bash ]; then
    source /prosthesis_ws/install/setup.bash
fi

set -euo pipefail

cd /prosthesis_ws

# ── Build PYTHONPATH for both flat and nested imports ────────────────────────
# Flat-import source dirs (the module file lives one level below the package):
SRC_FLAT_DIRS="
    src/twist_propagation/twist_propagation
    src/segmentation/segmentation_bridge/segmentation_bridge
"
# Nested-import package roots (the package dir is directly importable):
SRC_PKG_DIRS="
    src/gtsam_tracker
    src/keyframe_buffer
    src/tsdf_fusion
    src/cross_camera_features
    src/pointcloud_fusion
"

EXTRA_PATH=""
for d in $SRC_FLAT_DIRS $SRC_PKG_DIRS; do
    if [ -d "$d" ]; then
        EXTRA_PATH="${EXTRA_PATH:+$EXTRA_PATH:}/prosthesis_ws/$d"
    fi
done
export PYTHONPATH="${EXTRA_PATH}${PYTHONPATH:+:$PYTHONPATH}"

# ── Discover test directories ────────────────────────────────────────────────
TEST_DIRS=$(find src -type d -name test | sort)

if [ -z "$TEST_DIRS" ]; then
    echo "FAIL: no test directories found under src/"
    exit 1
fi

# ── Output directory for JUnit XML ───────────────────────────────────────────
XML_DIR="/prosthesis_ws/logs/test-results"
mkdir -p "$XML_DIR"

echo "=========================================="
echo "  Prosthesis Unit Tests (pytest)"
echo "=========================================="
echo ""

TOTAL_PASS=0
TOTAL_FAIL=0
TOTAL_SKIP=0
FAILED_DIRS=()

for test_dir in $TEST_DIRS; do
    # Only run pytest if there are test files
    if ! ls "$test_dir"/test_*.py >/dev/null 2>&1; then
        continue
    fi

    pkg_name=$(basename "$(dirname "$test_dir")")
    xml_file="$XML_DIR/${pkg_name}.xml"

    echo "  Running: $test_dir"

    # Run pytest with JUnit XML output. -q keeps output concise.
    # --tb=short gives concise tracebacks on failure.
    if python3 -m pytest "$test_dir" \
        -q \
        --tb=short \
        --junitxml="$xml_file" \
        --no-header \
        2>&1 | tail -5; then
        echo "    \033[32mPASS\033[0m"
    else
        echo "    \033[31mFAIL\033[0m"
        FAILED_DIRS+=("$test_dir")
    fi
    echo ""

    # Parse pass/fail/skip counts from the XML if possible
    if [ -f "$xml_file" ]; then
        stats=$(python3 -c "
import xml.etree.ElementTree as ET
tree = ET.parse('$xml_file')
root = tree.getroot()
# JUnit XML: <testsuite tests=\"N\" failures=\"N\" errors=\"N\" skipped=\"N\">
t = int(root.get('tests', 0))
f = int(root.get('failures', 0)) + int(root.get('errors', 0))
s = int(root.get('skipped', 0))
print(f'{t-f-s} {f} {s}')
" 2>/dev/null || echo "0 0 0")
        p=$(echo "$stats" | awk '{print $1}')
        fl=$(echo "$stats" | awk '{print $2}')
        sk=$(echo "$stats" | awk '{print $3}')
        TOTAL_PASS=$((TOTAL_PASS + p))
        TOTAL_FAIL=$((TOTAL_FAIL + fl))
        TOTAL_SKIP=$((TOTAL_SKIP + sk))
    fi
done

# ── Summary ──────────────────────────────────────────────────────────────────
echo "=========================================="
echo "  Unit Test Summary"
echo "=========================================="
printf "  Passed:   %d\n" "$TOTAL_PASS"
printf "  Failed:   %d\n" "$TOTAL_FAIL"
printf "  Skipped:  %d\n" "$TOTAL_SKIP"
echo ""

if [ ${#FAILED_DIRS[@]} -gt 0 ]; then
    echo "  Failed test directories:"
    for d in "${FAILED_DIRS[@]}"; do
        echo "    - $d"
    done
    echo ""
fi

# ── Baseline comparison (if baseline exists) ─────────────────────────────────
BASELINE_FILE="/prosthesis_ws/tests/baselines/timings.json"
if [ -f "$BASELINE_FILE" ]; then
    echo "=========================================="
    echo "  Timing Comparison vs Baseline"
    echo "=========================================="
    python3 /prosthesis_ws/scripts/compare_timings.py \
        --baseline "$BASELINE_FILE" \
        --current-dir "$XML_DIR" \
        --threshold 50 \
        2>&1 || true
    echo ""
fi

echo "=========================================="

if [ "$TOTAL_FAIL" -gt 0 ]; then
    exit 1
fi
exit 0
