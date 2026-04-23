cmake_minimum_required(VERSION 3.22.1)
project(monocular_slam_android LANGUAGES C CXX)

if(NOT ANDROID)
  message(FATAL_ERROR "package/android/CMakeLists.txt is intended to be used with the Android NDK toolchain.")
endif()

set(PACKAGE_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")

if(NOT OpenCV_DIR AND DEFINED ENV{ANDROID_OPENCV_SDK} AND NOT "$ENV{ANDROID_OPENCV_SDK}" STREQUAL "")
  set(OpenCV_DIR "$ENV{ANDROID_OPENCV_SDK}/sdk/native/jni" CACHE PATH "OpenCV Android SDK CMake config path" FORCE)
endif()

if(NOT OpenCV_DIR)
  message(FATAL_ERROR
    "OpenCV Android SDK not configured. Set OpenCV_DIR to <OpenCV-android-sdk>/sdk/native/jni "
    "or export ANDROID_OPENCV_SDK=<OpenCV-android-sdk> before configuring.")
endif()

find_package(OpenCV 4.4 REQUIRED)

set(MONOCULAR_SLAM_ENABLE_PANGOLIN_VIEWER OFF CACHE BOOL "" FORCE)
set(MONOCULAR_SLAM_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MONOCULAR_SLAM_BUILD_DATASET_APP OFF CACHE BOOL "" FORCE)
set(MONOCULAR_SLAM_BUILD_C_API ON CACHE BOOL "" FORCE)
set(MONOCULAR_SLAM_ENABLE_NATIVE_OPTIMIZATION OFF CACHE BOOL "" FORCE)

if(NOT DEFINED MONOCULAR_SLAM_ENABLE_DA2)
  set(MONOCULAR_SLAM_ENABLE_DA2 OFF CACHE BOOL "" FORCE)
endif()

add_subdirectory("${PACKAGE_ROOT}" "${CMAKE_BINARY_DIR}/monocular_slam_core")

add_library(monocular_slam_jni SHARED
  jni/MonocularSlamJni.cpp
)

target_include_directories(monocular_slam_jni
PRIVATE
  "${PACKAGE_ROOT}/api/include"
)

find_library(log-lib log)

target_link_libraries(monocular_slam_jni
PRIVATE
  monocular_slam_capi
  ${OpenCV_LIBS}
  ${log-lib}
)
