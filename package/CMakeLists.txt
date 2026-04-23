cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3_Monocular_Package)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

option(MONOCULAR_SLAM_ENABLE_NATIVE_OPTIMIZATION "Enable host-specific CPU optimizations such as -march=native" OFF)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
if(MONOCULAR_SLAM_ENABLE_NATIVE_OPTIMIZATION)
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
  message(STATUS "Native CPU optimization enabled for Release builds.")
else()
  message(STATUS "Using portable Release flags (native CPU optimization disabled).")
endif()

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(OpenCV 4.4)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV >= 4.4 not found.")
endif()

message("OPENCV VERSION:")
message(${OpenCV_VERSION})

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)
set(PANGOLIN_LOCAL_BUILD_DIR ${PROJECT_SOURCE_DIR}/Pangolin/build)
if(EXISTS "${PANGOLIN_LOCAL_BUILD_DIR}/PangolinConfig.cmake")
  set(Pangolin_DIR "${PANGOLIN_LOCAL_BUILD_DIR}")
endif()
find_package(Pangolin REQUIRED)
find_package(realsense2)

set(DA2_CODE_DIR ${PROJECT_SOURCE_DIR}/da2-code)
set(ONNXRUNTIME_ROOT ${DA2_CODE_DIR}/onnxruntime-osx-arm64-1.17.1)
set(ONNXRUNTIME_INCLUDE_DIR ${ONNXRUNTIME_ROOT}/include)
set(ONNXRUNTIME_LIB_DIR ${ONNXRUNTIME_ROOT}/lib)
find_library(ONNXRUNTIME_LIB onnxruntime PATHS ${ONNXRUNTIME_LIB_DIR} NO_DEFAULT_PATH)

if(EXISTS "${ONNXRUNTIME_INCLUDE_DIR}/onnxruntime_cxx_api.h" AND ONNXRUNTIME_LIB)
  message(STATUS "Using ONNX Runtime from ${ONNXRUNTIME_ROOT}")
else()
  message(FATAL_ERROR "ONNX Runtime not found under ${ONNXRUNTIME_ROOT}.")
endif()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${ONNXRUNTIME_INCLUDE_DIR}
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
src/DebugMapPublisher.cc
src/DepthAnythingV2.cc
src/YOLO.cc
include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Settings.h
include/DebugMapPublisher.h
include/DepthAnythingV2.h
include/YOLO.h)

target_compile_definitions(${PROJECT_NAME} PRIVATE ORB_SLAM3_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_LIB_DIR}"
  INSTALL_RPATH "${ONNXRUNTIME_LIB_DIR}")

add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
DBoW2
g2o
${Boost_LIBRARIES}
OpenSSL::Crypto
${ONNXRUNTIME_LIB}
)

add_library(monocular_slam_api SHARED
api/src/MonocularPipeline.cpp
api/src/MonocularSession.cpp
api/include/monocular_slam/Config.h
api/include/monocular_slam/Types.h
api/include/monocular_slam/MonocularPipeline.h
api/include/monocular_slam/MonocularSession.h
)

target_include_directories(monocular_slam_api
PUBLIC
${PROJECT_SOURCE_DIR}/api/include
)

target_link_libraries(monocular_slam_api
${PROJECT_NAME}
)

add_executable(dataset_debug
        examples/dataset_debug.cpp)
target_link_libraries(dataset_debug
        monocular_slam_api
        ${OpenCV_LIBS})
set_target_properties(dataset_debug PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/examples)

add_executable(dataset_debug_session
        examples/dataset_debug_session.cpp)
target_link_libraries(dataset_debug_session
        monocular_slam_api
        ${OpenCV_LIBS})
set_target_properties(dataset_debug_session PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/examples)

add_executable(video_debug_session
        examples/video_debug_session.cpp)
target_link_libraries(video_debug_session
        monocular_slam_api
        ${OpenCV_LIBS})
set_target_properties(video_debug_session PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/examples)

add_executable(video_ar_session
        examples/video_ar_session.cpp)
target_link_libraries(video_ar_session
        monocular_slam_api
        ${OpenCV_LIBS})
set_target_properties(video_ar_session PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/examples)

if(realsense2_FOUND)
    include_directories(${PROJECT_NAME}
    ${realsense_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
    )
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/apps/Monocular)

add_executable(mono_tum
        apps/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})
