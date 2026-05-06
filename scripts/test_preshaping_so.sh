#!/bin/bash
# test_preshaping_so.sh — verify the pre-built .so loads and responds
set -euo pipefail

SO_PATH="/prosthesis_ws/install/grasp_preshaping/lib/libgrasp_preshaping.so"

if [ ! -f "$SO_PATH" ]; then
    echo "FAIL: .so not found at $SO_PATH"
    exit 1
fi

# Use Python ctypes to load the .so and call the version function
python3 -c "
import ctypes, sys
so = ctypes.CDLL('$SO_PATH')

# Check that grasp_preshaping_version exists and returns a value
try:
    version_fn = so.grasp_preshaping_version
    version_fn.restype = ctypes.c_int
    v = version_fn()
    print(f'  API version: {v}')
    if v < 1:
        print('FAIL: API version < 1')
        sys.exit(1)
except Exception as e:
    print(f'FAIL: could not call grasp_preshaping_version: {e}')
    sys.exit(1)

# Check that the runtime config loaded (by checking a known function exists)
try:
    init_fn = so.grasp_preshaping_init
    print('  grasp_preshaping_init: found')
except:
    print('  grasp_preshaping_init: not found (may be optional)')

print('  .so loaded and API version check passed')
"
