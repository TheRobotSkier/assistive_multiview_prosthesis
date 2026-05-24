#!/usr/bin/env bash
# scripts/jetson_fix_overlay_symlinks.sh
#
# Fix broken symlinks in the OpenVINS overlay install directory.
#
# The original overlay was built on a different machine with paths like
# /miahand_ws/src/build_overlay/... and later rsynced to the Jetson.
# The symlinks in share/<pkg>/ point to paths that don't exist.
#
# This script deletes broken symlinks and replaces them with minimal
# working ament hooks that reference the correct Jetson paths.
#
# Usage (run on Jetson):
#   ./scripts/jetson_fix_overlay_symlinks.sh [overlay_dir]

set -euo pipefail

OVERLAY_DIR="${1:-$(dirname "$0")/../docker_ws/install_overlay}"
OVERLAY_DIR="$(cd "$OVERLAY_DIR" 2>/dev/null && pwd || echo "$OVERLAY_DIR")"

if [ ! -d "$OVERLAY_DIR" ]; then
    echo "ERROR: Overlay directory not found: $OVERLAY_DIR"
    echo "Usage: $0 [path/to/install_overlay]"
    exit 1
fi

echo "Fixing overlay at: $OVERLAY_DIR"

fix_package_share() {
    local pkg="$1"
    local share_dir="${OVERLAY_DIR}/${pkg}/share/${pkg}"

    if [ ! -d "$share_dir" ]; then
        echo "  SKIP $pkg (no share dir)"
        return
    fi

    # Delete broken symlinks
    echo "  Fixing $pkg..."
    find "$share_dir" -type l -delete 2>/dev/null || true

    # Ensure package.xml exists
    if [ ! -f "${OVERLAY_DIR}/${pkg}/share/${pkg}/package.xml" ]; then
        # Try to find in source
        local pkg_xml
        pkg_xml=$(find /home/robotlab/wt-simplified-jetson/jetson_docker_branch/docker_ws/src -path "*/${pkg}/package.xml" 2>/dev/null | head -1)
        if [ -n "${pkg_xml}" ]; then
            cp "${pkg_xml}" "${share_dir}/"
            echo "    Copied package.xml from source"
        fi
    fi

    # Generate environment hooks
    mkdir -p "${share_dir}/environment" "${share_dir}/hook"

    # cmake_prefix_path hook
    cat > "${share_dir}/hook/cmake_prefix_path.sh" << EOFSH
# generated
_colcon_prepend_unique_value CMAKE_PREFIX_PATH "\$COLCON_CURRENT_PREFIX"
EOFSH
    cat > "${share_dir}/hook/cmake_prefix_path.dsv" << EOFDSV
prepend-non-duplicate;CMAKE_PREFIX_PATH;
EOFDSV

    # ament_prefix_path hook
    cat > "${share_dir}/hook/ament_prefix_path.sh" << EOFSH
# generated
ament_prepend_unique_value AMENT_PREFIX_PATH "\$AMENT_CURRENT_PREFIX"
EOFSH
    cat > "${share_dir}/hook/ament_prefix_path.dsv" << EOFDSV
prepend-non-duplicate;AMENT_PREFIX_PATH;
EOFDSV

    # environment/ament_prefix_path
    cat > "${share_dir}/environment/ament_prefix_path.sh" << EOFSH
ament_prepend_unique_value AMENT_PREFIX_PATH "\$AMENT_CURRENT_PREFIX"
EOFSH

    # local_setup files
    for ext in bash sh; do
        cat > "${share_dir}/local_setup.${ext}" << EOFSH
# generated
AMENT_CURRENT_PREFIX="${OVERLAY_DIR}/${pkg}"
export AMENT_CURRENT_PREFIX
EOFSH
    done

    # package.sh / package.bash
    for ext in sh bash; do
        cat > "${share_dir}/package.${ext}" << EOFSH
AMENT_CURRENT_PREFIX="${OVERLAY_DIR}/${pkg}"
_colcon_package_sh_COLCON_CURRENT_PREFIX="${OVERLAY_DIR}/${pkg}"
export AMENT_CURRENT_PREFIX
export _colcon_package_sh_COLCON_CURRENT_PREFIX
EOFSH
    done

    # package.dsv — only reference .sh/.bash files (avoid .dsv recursion)
    cat > "${share_dir}/package.dsv" << EOFDSV
source;share/${pkg}/hook/cmake_prefix_path.sh
source;share/${pkg}/hook/ament_prefix_path.sh
source;share/${pkg}/local_setup.bash
source;share/${pkg}/local_setup.sh
source;share/${pkg}/package.bash
source;share/${pkg}/package.sh
EOFDSV

    echo "    Done."
}

# Fix all packages in the overlay
for pkg_dir in "${OVERLAY_DIR}"/*/; do
    pkg="$(basename "$pkg_dir")"
    # Skip non-package directories
    [ "$pkg" = "_local_setup_util_ps1.py" ] && continue
    [ "$pkg" = "_local_setup_util_sh.py" ] && continue
    [ -f "${OVERLAY_DIR}/${pkg}/share/${pkg}/package.xml" ] || continue
    fix_package_share "$pkg"
done

echo ""
echo "Overlay fix complete."
echo "To verify:"
echo "  source ${OVERLAY_DIR}/setup.bash"
echo "  ros2 pkg prefix ov_msckf"
echo "  ros2 pkg prefix sensor_fusion_bringup"
