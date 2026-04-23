#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${ROOT_DIR}/build"

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release "$@"
cmake --build "${BUILD_DIR}" -j4

echo
echo "Build completed."
echo "Core library: ${ROOT_DIR}/lib/libORB_SLAM3_Monocular_Package.dylib"
echo "API library: ${ROOT_DIR}/lib/libmonocular_slam_api.dylib"
echo "C API library: ${ROOT_DIR}/lib/libmonocular_slam_capi.dylib"
