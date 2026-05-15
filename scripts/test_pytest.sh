#!/bin/bash
# test_pytest.sh — run Python unit tests under tests/

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f /prosthesis_ws/install/setup.bash ]; then
    source /prosthesis_ws/install/setup.bash
fi

set -euo pipefail

cd /prosthesis_ws

python3 -m pytest tests/ -q --tb=short
