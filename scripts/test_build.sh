#!/bin/bash
# test_build.sh — verify colcon build succeeded
# The Dockerfile already runs colcon build. This test verifies the artifacts.

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

# Check that the build produced an install directory
if [ ! -d install/ ]; then
    echo "FAIL: install/ directory not found — colcon build did not run"
    exit 1
fi

# Count installed packages
PKG_COUNT=$(ls -d install/*/ 2>/dev/null | wc -l)
if [ "$PKG_COUNT" -eq 0 ]; then
    echo "FAIL: no packages found in install/"
    exit 1
fi

echo "OK: ${PKG_COUNT} packages installed"

# Verify critical packages exist
CRITICAL_PKGS="grasp_preshaping mia_hand_driver mia_hand_description mia_hand_msgs"
for pkg in $CRITICAL_PKGS; do
    if [ ! -d "install/${pkg}" ]; then
        echo "FAIL: critical package '${pkg}' not found in install/"
        exit 1
    fi
done

echo "OK: all critical packages present"

# Verify the pre-built .so was installed
if [ ! -f install/grasp_preshaping/lib/libgrasp_preshaping.so ]; then
    echo "WARN: libgrasp_preshaping.so not in install/ (may be in lib/)"
fi

# Verify the LUT data file exists
if [ ! -f src/grasp_preshaping/data/finger_contact_lut.npz ]; then
    echo "FAIL: finger_contact_lut.npz not found"
    exit 1
fi

echo "OK: finger_contact_lut.npz present"

echo "PASS: build verification complete"
