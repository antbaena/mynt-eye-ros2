#!/usr/bin/env bash
# ============================================================================
# build_d_sdk_ubuntu24.sh
# Builds and installs MYNT-EYE-D-SDK for Ubuntu 24.04 (GCC 13+, C++14/17).
#
# The SDK uses the Etron eSPDI closed-source library (libeSPDI.so) which is
# included pre-built in the SDK repository and is binary-compatible with
# Ubuntu 24.04 (depends only on glibc/libstdc++/libm/libgcc).
#
# Output: <repo_root>/d_install/
#   lib/libmynteye_depth.so
#   lib/3rdparty/libeSPDI.so
#   include/mynteyed/
#   lib/cmake/mynteyed/
#
# Usage:
#   cd <repo_root>
#   bash scripts/build_d_sdk_ubuntu24.sh
#
# After building, compile the ROS 2 workspace:
#   source /opt/ros/jazzy/setup.bash
#   cd mynteye_ws
#   colcon build --packages-select mynteye_camera \
#     --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
# ============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_DIR="${REPO_ROOT}/MYNT-EYE-D-SDK"
BUILD_DIR="${SDK_DIR}/_build"
INSTALL_DIR="${REPO_ROOT}/d_install"

echo "=== MYNT-EYE-D-SDK build script for Ubuntu 24.04 ==="
echo "  Repo root  : ${REPO_ROOT}"
echo "  SDK source : ${SDK_DIR}"
echo "  Install to : ${INSTALL_DIR}"
echo ""

# ---- 1. Clone SDK if not already present -----------------------------------
if [[ ! -d "${SDK_DIR}" ]]; then
    echo "[1/5] Cloning MYNT-EYE-D-SDK ..."
    git clone https://github.com/slightech/MYNT-EYE-D-SDK.git "${SDK_DIR}"
else
    echo "[1/5] SDK directory already exists, skipping clone."
fi

# ---- 2. Install system dependencies ----------------------------------------
echo "[2/5] Installing dependencies ..."
if command -v apt-get &>/dev/null; then
    sudo apt-get update -qq
    sudo apt-get install -y --no-install-recommends \
        build-essential cmake \
        libopencv-dev \
        libusb-1.0-0-dev \
        libusb-dev
else
    echo "  WARNING: apt-get not available. Make sure build-essential, cmake,"
    echo "           libopencv-dev, libusb-dev are installed."
fi

# ---- 3. Apply Ubuntu 24.04 compatibility patch (GCC 13 stdint) -------------
echo "[3/5] Applying compatibility patch ..."
TYPES_CALIB="${SDK_DIR}/include/mynteyed/stubs/types_calib.h"
if grep -q '#include <cstdint>' "${TYPES_CALIB}"; then
    echo "  Patch already applied."
else
    # Insert #include <cstdint> after the first #pragma once or header guard
    if grep -q '#pragma once' "${TYPES_CALIB}"; then
        sed -i 's|#pragma once|#pragma once\n#include <cstdint>  // PATCH: Ubuntu 24.04 GCC 13 requires explicit include|' "${TYPES_CALIB}"
    else
        # Prepend at top
        sed -i '1s|^|#include <cstdint>  // PATCH: Ubuntu 24.04 GCC 13 requires explicit include\n|' "${TYPES_CALIB}"
    fi
    echo "  Patch applied: added #include <cstdint> to ${TYPES_CALIB}"
fi

# ---- 4. Configure and build ------------------------------------------------
echo "[4/5] Configuring and building (this may take a few minutes) ..."
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=14 \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
    -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON

JOBS=$(nproc 2>/dev/null || echo 4)
echo "  Building with ${JOBS} parallel jobs ..."
make -j"${JOBS}"

# ---- 5. Install ------------------------------------------------------------
echo "[5/5] Installing to ${INSTALL_DIR} ..."
make install

# ---- Summary ---------------------------------------------------------------
echo ""
echo "=== Build complete! ==="
echo "  Headers  : ${INSTALL_DIR}/include/mynteyed/"
echo "  Library  : ${INSTALL_DIR}/lib/libmynteye_depth.so"
echo "  eSPDI    : ${INSTALL_DIR}/lib/3rdparty/libeSPDI.so"
echo "  CMake    : ${INSTALL_DIR}/lib/cmake/mynteyed/"
echo ""
echo "Next steps:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  cd ${REPO_ROOT}/mynteye_ws"
echo "  colcon build --packages-select mynteye_camera \\"
echo "    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3"
echo "  source install/setup.bash"
echo "  ros2 launch mynteye_camera mynteye_d1000_hw_depth.launch.py"
