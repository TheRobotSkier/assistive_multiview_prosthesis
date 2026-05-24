#!/usr/bin/env bash
# scripts/jetson_build_openvins.sh
#
# Build the OpenVINS overlay (ov_msckf) on the Jetson inside a Docker container.
#
# Problem: The Jetson has limited RAM (7.4GB). Compiling OpenVINS with
# Boost+Ceres+Eigen templates in StateHelper.cpp and VioManagerHelper.cpp
# exceeds 8GB+ RSS, triggering the kernel OOM killer even with -j1 because
# colcon/make spawns compiler processes in the background.
#
# Solution: This script uses cmake to configure, then compiles each .o file
# one-at-a-time with no background jobs, and links the final .so.
#
# Usage (run on Jetson):
#   chmod +x scripts/jetson_build_openvins.sh
#   ./scripts/jetson_build_openvins.sh
#
# Environment variables (all optional):
#   OPENVINS_SRC_DIR     Path to open_vins source (default: auto-detect)
#   OVERLAY_INSTALL_DIR  Path to overlay install (default: ../install_overlay)
#   DOCKER_IMAGE         Docker image to use (default: prosthesis:latest)
#   BUILD_TYPE           Build type (default: Release)
#   OPT_LEVEL            Optimization level (default: O1, use O0 if still OOM)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── Paths ────────────────────────────────────────────────────────────────────
DOCKER_WS="${DOCKER_WS:-${REPO_ROOT}/docker_ws}"
OPENVINS_SRC_DIR="${OPENVINS_SRC_DIR:-${DOCKER_WS}/src/open_vins}"
MULTI_CAM_DIR="${MULTI_CAM_DIR:-${DOCKER_WS}/multi_cam_localization}"
OVERLAY_INSTALL_DIR="${OVERLAY_INSTALL_DIR:-${DOCKER_WS}/install_overlay}"
DOCKER_IMAGE="${DOCKER_IMAGE:-prosthesis:latest}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
OPT_LEVEL="${OPT_LEVEL:-O1}"

BUILD_DIR="${OVERLAY_INSTALL_DIR}/../build_overlay"
LOG_DIR="${OVERLAY_INSTALL_DIR}/../log_overlay"

OV_MSKF_SRC="${OPENVINS_SRC_DIR}/ov_msckf"
OV_CORE_SRC="${OPENVINS_SRC_DIR}/ov_core"
OV_INIT_SRC="${OPENVINS_SRC_DIR}/ov_init"

# ── Sanity checks ────────────────────────────────────────────────────────────
if [ ! -f "${OV_MSKF_SRC}/CMakeLists.txt" ]; then
    echo "ERROR: ov_msckf source not found at ${OV_MSKF_SRC}/CMakeLists.txt"
    echo "Set OPENVINS_SRC_DIR to the open_vins source directory."
    exit 1
fi

if [ ! -f "${MULTI_CAM_DIR}/sensor_fusion_bringup/CMakeLists.txt" ]; then
    echo "ERROR: sensor_fusion_bringup not found at ${MULTI_CAM_DIR}"
    exit 1
fi

# ── Prepare directories ──────────────────────────────────────────────────────
mkdir -p "${BUILD_DIR}" "${LOG_DIR}" "${OVERLAY_INSTALL_DIR}"

# ── Docker arguments ─────────────────────────────────────────────────────────
DOCKER_MOUNTS=(
    -v "${OPENVINS_SRC_DIR}:/ws/src/open_vins:ro"
    -v "${MULTI_CAM_DIR}:/ws/src/multi_cam_localization:ro"
    -v "${OVERLAY_INSTALL_DIR}:/ws/install_overlay"
    -v "${BUILD_DIR}:/ws/build_overlay"
    -v "${LOG_DIR}:/ws/log_overlay"
)

DOCKER_OPTS=(
    --rm
    --runtime nvidia
    --network host
    --user root
    --memory 0 --memory-swap -1
    -w /ws
    -e MAKEFLAGS="-j1"
)

# ── Helper: run a command inside the build container ─────────────────────────
docker_build() {
    docker run "${DOCKER_OPTS[@]}" "${DOCKER_MOUNTS[@]}" \
        "${DOCKER_IMAGE}" bash -c "$1"
}

# ── Step 1: Build dependencies (ov_core, ov_init, sensor_fusion_msgs) ───────
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Step 1/4: Configure & build dependencies"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

docker_build '
source /opt/ros/humble/setup.bash
colcon build \
    --packages-select ov_core ov_init sensor_fusion_msgs sensor_fusion_bringup imu_driver \
    --cmake-args -DCMAKE_BUILD_TYPE='"${BUILD_TYPE}"' \
    --install-base /ws/install_overlay \
    --build-base /ws/build_overlay \
    --executor sequential \
    --parallel-workers 1 \
    2>&1
' 2>&1 | head -100
DEP_RC=${PIPESTATUS[0]}
if [ $DEP_RC -ne 0 ]; then
    echo ""
    echo "WARNING: Dependency build had non-zero exit ($DEP_RC)."
    echo "If only ov_msckf is needed, this may be OK."
    echo "Check logs in ${LOG_DIR}"
fi

# ── Step 2: Configure ov_msckf with cmake ────────────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Step 2/4: Configure ov_msckf (cmake)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

docker_build '
source /opt/ros/humble/setup.bash
source /ws/install_overlay/setup.bash 2>/dev/null || true
mkdir -p /ws/build_overlay/ov_msckf
cd /ws/build_overlay/ov_msckf

# Set O1 optimization + omit frame pointer to save RAM
export CXXFLAGS="-'"${OPT_LEVEL}"' -pipe -fno-omit-frame-pointer -g0"

cmake /ws/src/open_vins/ov_msckf \
    -DCMAKE_BUILD_TYPE='"${BUILD_TYPE}"' \
    -DCMAKE_INSTALL_PREFIX=/ws/install_overlay/ov_msckf \
    -DCMAKE_PREFIX_PATH="/opt/ros/humble;/ws/install_overlay/ov_core;/ws/install_overlay/ov_init;/ws/install_overlay/sensor_fusion_msgs" \
    -DBUILD_TESTING=OFF \
    2>&1
' 2>&1 | tail -20

# ── Step 3: Compile each .o file ONE AT A TIME ───────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Step 3/4: Compile .o files (one at a time to avoid OOM)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

OV_MSKF_BUILD_DIR="${BUILD_DIR}/ov_msckf"
OBJ_DIR="${OV_MSKF_BUILD_DIR}/CMakeFiles/ov_msckf_lib.dir"

# Source files for the library (from ROS2.cmake)
LIB_SOURCES=(
    src/dummy.cpp
    src/sim/Simulator.cpp
    src/state/State.cpp
    src/state/StateHelper.cpp
    src/state/Propagator.cpp
    src/core/VioManager.cpp
    src/core/VioManagerHelper.cpp
    src/update/UpdaterHelper.cpp
    src/update/UpdaterDynamicArmPose.cpp
    src/update/UpdaterMSCKF.cpp
    src/update/UpdaterMarkerPose.cpp
    src/update/UpdaterSLAM.cpp
    src/update/UpdaterZeroVelocity.cpp
    src/ros/ROS2Visualizer.cpp
    src/ros/ROSVisualizerHelper.cpp
)

TOTAL=${#LIB_SOURCES[@]}
COMPILED=0
FAILED=()

for src_file in "${LIB_SOURCES[@]}"; do
    COMPILED=$((COMPILED + 1))
    obj_file="${OBJ_DIR}/${src_file}.o"
    obj_dir="$(dirname "${obj_file}")"
    mkdir -p "${obj_dir}"

    echo "  [${COMPILED}/${TOTAL}] ${src_file}"

    set +e
    docker_build "
source /opt/ros/humble/setup.bash
source /ws/install_overlay/setup.bash 2>/dev/null || true
cd /ws/build_overlay/ov_msckf

# Remove old object file for clean recompile
rm -f 'CMakeFiles/ov_msckf_lib.dir/${src_file}.o'

# Compile with explicit flags, no optimization body (saves RAM on templates)
/usr/bin/c++ \
    -DROS_AVAILABLE=2 -DENABLE_ARUCO_TAGS=1 \
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
    -std=c++14 -fPIC -${OPT_LEVEL} -pipe -g0 \
    -fno-omit-frame-pointer \
    -I/ws/src/open_vins/ov_msckf/src \
    -I/opt/ros/humble/include \
    -I/ws/install_overlay/ov_core/include \
    -I/ws/install_overlay/ov_init/include \
    -I/ws/install_overlay/sensor_fusion_msgs/include \
    -I/usr/include/eigen3 \
    -I/usr/include/opencv4 \
    -I/usr/include/boost \
    -I/usr/include/suitesparse \
    -c /ws/src/open_vins/ov_msckf/${src_file} \
    -o 'CMakeFiles/ov_msckf_lib.dir/${src_file}.o' \
    2>&1
" 2>&1 | grep -v "^========$\|^== CUDA\|Container image\|This container\|By pulling\|A copy\|WARNING\|NGC-DL\|site:" || true

    rc=${PIPESTATUS[0]}
    set -e

    if [ "$rc" -ne 0 ]; then
        echo "    FAILED (exit code $rc)"
        FAILED+=("${src_file}")
    fi
done

if [ ${#FAILED[@]} -gt 0 ]; then
    echo ""
    echo "ERROR: ${#FAILED[@]} file(s) failed to compile:"
    for f in "${FAILED[@]}"; do
        echo "  - ${f}"
    done
    exit 1
fi

echo ""
echo "  All ${TOTAL} files compiled successfully."

# ── Step 4: Link and install ─────────────────────────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Step 4/4: Link libov_msckf_lib.so and install"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Build the object list for the linker
OBJ_FILES=""
for src_file in "${LIB_SOURCES[@]}"; do
    OBJ_FILES="${OBJ_FILES} CMakeFiles/ov_msckf_lib.dir/${src_file}.o"
done

docker_build "
source /opt/ros/humble/setup.bash
source /ws/install_overlay/setup.bash 2>/dev/null || true
cd /ws/build_overlay/ov_msckf

echo 'Linking libov_msckf_lib.so...'
/usr/bin/c++ -fPIC -${OPT_LEVEL} \
    -Wl,-rpath,/ws/install_overlay/ov_core/lib:/ws/install_overlay/ov_init/lib \
    -shared -Wl,-soname,libov_msckf_lib.so -o libov_msckf_lib.so \
    ${OBJ_FILES} \
    -L/ws/install_overlay/ov_core/lib -lov_core_lib \
    -L/ws/install_overlay/ov_init/lib -lov_init_lib \
    /opt/ros/humble/lib/librclcpp.so \
    /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_cpp.so \
    -lceres -lglog -lgflags \
    -lboost_system -lboost_thread -lboost_filesystem -lboost_date_time \
    -lopencv_core -lopencv_imgproc -lopencv_calib3d -lopencv_aruco -lopencv_video \
    -lpthread -ldl \
    2>&1

echo 'Installing to overlay...'
# Copy .so
mkdir -p /ws/install_overlay/ov_msckf/lib/
cp libov_msckf_lib.so /ws/install_overlay/ov_msckf/lib/

# Install share files via cmake install (generates hooks, package files)
cmake --install . --prefix /ws/install_overlay/ov_msckf 2>&1 || {
    echo 'cmake --install failed, copying share files manually...'
    mkdir -p /ws/install_overlay/ov_msckf/share/ov_msckf/
    cp /ws/src/open_vins/ov_msckf/package.xml /ws/install_overlay/ov_msckf/share/ov_msckf/ 2>/dev/null || true
}

echo ''
echo 'Ov_msckf build complete!'
echo '.so: /ws/install_overlay/ov_msckf/lib/libov_msckf_lib.so'
" 2>&1 | tail -20

# ── Verify ───────────────────────────────────────────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Verification"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

SO_SIZE=$(stat -c%s "${OVERLAY_INSTALL_DIR}/ov_msckf/lib/libov_msckf_lib.so" 2>/dev/null || echo "0")
echo "  libov_msckf_lib.so: ${SO_SIZE} bytes"

docker_build '
source /opt/ros/humble/setup.bash
source /ws/install_overlay/setup.bash 2>/dev/null || true

echo "Checking symbols for new features..."
for sym in marker_noise_multiplier marker_reset_bias_policy initial_lock_zero_velocity_fallback zupt_noise_multiplier dynamic_arm_noise_multiplier; do
    if strings /ws/install_overlay/ov_msckf/lib/libov_msckf_lib.so 2>/dev/null | grep -q "$sym"; then
        echo "  OK: $sym"
    else
        echo "  MISSING: $sym"
    fi
done
' 2>&1 | tail -10

echo ""
echo "Build complete. Overlay installed at: ${OVERLAY_INSTALL_DIR}"
