#!/bin/bash
# test_launch_syntax.sh — verify all launch files parse without errors

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

LAUNCH_DIR="/prosthesis_ws/src/prosthesis_launch/launch"
ERRORS=0

for launch_file in "$LAUNCH_DIR"/*.launch.py; do
    if [ ! -f "$launch_file" ]; then
        continue
    fi
    name=$(basename "$launch_file")
    # Try to parse the launch file — this catches import errors and syntax issues
    if python3 -c "
import sys
sys.path.insert(0, '$LAUNCH_DIR')
import importlib.util
spec = importlib.util.spec_from_file_location('launch_test', '$launch_file')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
print(f'  {\"$name\"}: parsed OK')
" 2>&1; then
        echo "  $name: OK"
    else
        echo "  $name: FAIL"
        ERRORS=$((ERRORS + 1))
    fi
done

if [ "$ERRORS" -gt 0 ]; then
    echo "FAIL: $ERRORS launch file(s) had parse errors"
    exit 1
fi
echo "All launch files parsed successfully"
