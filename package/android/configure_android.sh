#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ANDROID_SDK_ROOT="${ANDROID_SDK_ROOT:-${ANDROID_HOME:-$HOME/Library/Android/sdk}}"
ANDROID_NDK_ROOT="${ANDROID_NDK_ROOT:-}"
ANDROID_ABI="${ANDROID_ABI:-arm64-v8a}"
ANDROID_PLATFORM="${ANDROID_PLATFORM:-android-24}"
BUILD_DIR="${1:-${PACKAGE_ROOT}/build-android-${ANDROID_ABI}}"
BUILD_TYPE="${BUILD_TYPE:-Release}"

if [[ -z "${ANDROID_NDK_ROOT}" ]]; then
  if [[ -d "${ANDROID_SDK_ROOT}/ndk" ]]; then
    ANDROID_NDK_ROOT="$(find "${ANDROID_SDK_ROOT}/ndk" -mindepth 1 -maxdepth 1 -type d | sort | tail -n 1)"
  fi
fi

if [[ -z "${ANDROID_NDK_ROOT}" || ! -d "${ANDROID_NDK_ROOT}" ]]; then
  echo "Could not determine ANDROID_NDK_ROOT."
  echo "Set ANDROID_NDK_ROOT explicitly before running this script."
  exit 1
fi

if [[ -z "${ANDROID_OPENCV_SDK:-}" || ! -d "${ANDROID_OPENCV_SDK}" ]]; then
  echo "ANDROID_OPENCV_SDK is not set to a valid OpenCV Android SDK directory."
  echo "Example:"
  echo "  export ANDROID_OPENCV_SDK=\$HOME/Android/OpenCV-android-sdk"
  exit 1
fi

TOOLCHAIN_FILE="${ANDROID_NDK_ROOT}/build/cmake/android.toolchain.cmake"
if [[ ! -f "${TOOLCHAIN_FILE}" ]]; then
  echo "Android NDK toolchain file not found at:"
  echo "  ${TOOLCHAIN_FILE}"
  exit 1
fi

cmake -S "${PACKAGE_ROOT}/android" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
  -DANDROID_ABI="${ANDROID_ABI}" \
  -DANDROID_PLATFORM="${ANDROID_PLATFORM}" \
  -DOpenCV_DIR="${ANDROID_OPENCV_SDK}/sdk/native/jni" \
  -DMONOCULAR_SLAM_ENABLE_DA2=OFF

echo
echo "Configured Android build in:"
echo "  ${BUILD_DIR}"
echo
echo "Build with:"
echo "  cmake --build ${BUILD_DIR} -j4"
