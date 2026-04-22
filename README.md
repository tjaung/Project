<div align="center">
  <h1>NGD-SLAM: Towards Real-Time Dynamic SLAM <br>without GPU</h1>
  <strong>IROS 2025</strong>
  <br>
    <a href="https://yuhaozhang7.github.io" target="_blank">Yuhao Zhang</a><sup>1,2</sup>,
    <a href="#" target="_blank">Mihai Bujanca</a><sup>3</sup>,
    <a href="#" target="_blank">Mikel Luján</a><sup>1</sup>
  <p>
    <sup>1</sup>University of Manchester &nbsp;&nbsp;
    <sup>2</sup>University of Cambridge &nbsp;&nbsp;
    <br>
    <sup>3</sup>Qualcomm Technologies XR Labs &nbsp;&nbsp;
  </p>

  [<img src="https://img.shields.io/badge/Preprint-arXiv-990000" alt="Arxiv">](https://arxiv.org/abs/2405.07392)
  [<img src="https://img.shields.io/badge/Paper-IEEE_Xplore-blue" alt="IEEE">](https://ieeexplore.ieee.org/abstract/document/11246202)
  [<img src="https://img.shields.io/badge/Video-Bilibili-pink" alt="Bilibili">](https://www.bilibili.com/video/BV1XKT5eaEsT/)
</div>

<p align="center">
  <img src="assets/bonn_crowd_small.gif" alt="GIF 1" width="240">
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <img src="assets/bonn_mov_small.gif" alt="GIF 2" width="240">
</p>

This repository is a dynamic visual SLAM system derived from ORB-SLAM3. On this branch, the monocular pipeline has been extended with in-process, ONNX Runtime based Depth Anything v2 depth estimation so it can follow the RGB-D dynamic masking logic more closely while still using a monocular camera input.

## Overview

There are now two main ways to run the TUM Freiburg 3 walking sequences:

- `RGB-D`: the original sensor-depth pipeline.
- `Monocular + DA2`: monocular RGB input with asynchronous per-frame Depth Anything v2 estimation, YOLO-based dynamic masking, mask propagation, optical-flow tracking, and masked ORB extraction.

The monocular TUM runner also:

- saves `CameraTrajectory.txt`
- saves `KeyFrameTrajectory.txt`
- evaluates `ATE`, `RPE`, and `RPE %` automatically if `<dataset>/groundtruth.txt` exists
- appends one row per run to `out/mono_tum_metrics.csv`

## Tested Setup

These instructions reflect the setup used to build and run this branch locally:

- Machine: Apple Silicon MacBook Air, `arm64`
- OS: macOS on Darwin `24.6.0`
- Python: `3.12.0`
- CMake: `3.31.11`

The repo still contains the original paper code and third-party dependencies, but the monocular DA2 path documented below is specifically set up for this Apple Silicon macOS environment.

## Requirements

Required native dependencies:

- C++14-capable compiler
- CMake
- OpenCV `>= 4.4`
- Eigen3
- Boost with `serialization`
- OpenSSL
- Pangolin

Included in the repository:

- YOLO-fastest config and weights in `Thirdparty`
- DBoW2 in `Thirdparty/DBoW2`
- g2o in `Thirdparty/g2o`
- Sophus in `Thirdparty/Sophus`
- Depth Anything v2 ONNX model in [da2-code/model_fp16.onnx](da2-code/model_fp16.onnx)
- ONNX Runtime for Apple Silicon macOS in [da2-code/onnxruntime-osx-arm64-1.17.1](</Users/timjaung/Documents/NEU/CS5330/Projects/Final/Project/da2-code/onnxruntime-osx-arm64-1.17.1>)

Required Python packages:

- `numpy` for trajectory evaluation

Optional Python packages:

- `opencv-python` if you use any local Python visualization tools that rely on OpenCV

## Installation

### 1. Install system dependencies

On macOS with Homebrew, the closest setup to this branch is:

```bash
brew install cmake opencv eigen boost openssl pangolin
```

If `pangolin` is not available through your package manager on your machine, build Pangolin from source and make sure CMake can find it.

### 2. Set up Python for evaluation

From the repo root:

```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install numpy
```

### 3. Verify DA2 assets are present

These paths must exist before configuring:

- [da2-code/model_fp16.onnx](da2-code/model_fp16.onnx)
- [da2-code/onnxruntime-osx-arm64-1.17.1/include/onnxruntime_cxx_api.h](da2-code/onnxruntime-osx-arm64-1.17.1/include/onnxruntime_cxx_api.h)
- [da2-code/onnxruntime-osx-arm64-1.17.1/lib/libonnxruntime.dylib](da2-code/onnxruntime-osx-arm64-1.17.1/lib/libonnxruntime.dylib)

## Build

### Option A: use the provided build script

```bash
chmod +x build.sh
./build.sh
```

### Option B: use CMake directly

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target ORB_SLAM3 mono_tum rgbd_tum -j4
cd ..
```

The vocabulary archive must be unpacked once:

```bash
tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary
```

## Dataset Download

The commands below download and extract the four Freiburg 3 walking sequences into `data/`.

```bash
mkdir -p data
```

```bash
curl -L https://cvg.cit.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_walking_xyz.tgz -o data/rgbd_dataset_freiburg3_walking_xyz.tgz && tar -xzf data/rgbd_dataset_freiburg3_walking_xyz.tgz -C data
```

```bash
curl -L https://cvg.cit.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_walking_rpy.tgz -o data/rgbd_dataset_freiburg3_walking_rpy.tgz && tar -xzf data/rgbd_dataset_freiburg3_walking_rpy.tgz -C data
```

```bash
curl -L https://cvg.cit.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_walking_halfsphere.tgz -o data/rgbd_dataset_freiburg3_walking_halfsphere.tgz && tar -xzf data/rgbd_dataset_freiburg3_walking_halfsphere.tgz -C data
```

```bash
curl -L https://cvg.cit.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_walking_static.tgz -o data/rgbd_dataset_freiburg3_walking_static.tgz && tar -xzf data/rgbd_dataset_freiburg3_walking_static.tgz -C data
```

After extraction, you should have:

- `data/rgbd_dataset_freiburg3_walking_xyz`
- `data/rgbd_dataset_freiburg3_walking_rpy`
- `data/rgbd_dataset_freiburg3_walking_halfsphere`
- `data/rgbd_dataset_freiburg3_walking_static`

## Running the Monocular DA2 Pipeline

Enable the DA2 monocular path:

```bash
export ORB_SLAM3_MONO_DA2=1
export ORB_SLAM3_DA2_MODEL=$PWD/da2-code/model_fp16.onnx
export ORB_SLAM3_DA2_MIN_SIDE=256
```

Optional debug / visualization environment variables:

```bash
export ORB_SLAM3_DEBUG_MAP_FILE=debug_map_state.txt
export ORB_SLAM3_FRAME_TRACE_ROOT=$PWD/out/frame_traces
```

### Freiburg 3 walking xyz

```bash
./Examples/Monocular/mono_tum \
  Vocabulary/ORBvoc.txt \
  Examples/Monocular/TUM3.yaml \
  data/rgbd_dataset_freiburg3_walking_xyz
```

### Freiburg 3 walking rpy

```bash
./Examples/Monocular/mono_tum \
  Vocabulary/ORBvoc.txt \
  Examples/Monocular/TUM3.yaml \
  data/rgbd_dataset_freiburg3_walking_rpy
```

### Freiburg 3 walking halfsphere

```bash
./Examples/Monocular/mono_tum \
  Vocabulary/ORBvoc.txt \
  Examples/Monocular/TUM3.yaml \
  data/rgbd_dataset_freiburg3_walking_halfsphere
```

### Freiburg 3 walking static

```bash
./Examples/Monocular/mono_tum \
  Vocabulary/ORBvoc.txt \
  Examples/Monocular/TUM3.yaml \
  data/rgbd_dataset_freiburg3_walking_static
```

### Monocular outputs

Each successful run produces:

- `CameraTrajectory.txt`
- `KeyFrameTrajectory.txt`
- metrics printed in the terminal
- a CSV row appended to [out/mono_tum_metrics.csv](/Users/timjaung/Documents/NEU/CS5330/Projects/Final/Project/out/mono_tum_metrics.csv:1)

The CSV includes:

- dataset name
- `ATE`
- `RPE`
- `RPE %`
- mean tracking time in ms
- median tracking time in ms

## Running the RGB-D Pipeline

Use the RGB-D executable with the matching association file.

### Freiburg 3 walking xyz

```bash
./Examples/RGB-D/rgbd_tum \
  Vocabulary/ORBvoc.txt \
  Examples/RGB-D/TUM3.yaml \
  data/rgbd_dataset_freiburg3_walking_xyz \
  Examples/RGB-D/associations/fr3_walking_xyz.txt
```

### Freiburg 3 walking rpy

```bash
./Examples/RGB-D/rgbd_tum \
  Vocabulary/ORBvoc.txt \
  Examples/RGB-D/TUM3.yaml \
  data/rgbd_dataset_freiburg3_walking_rpy \
  Examples/RGB-D/associations/fr3_walking_rpy.txt
```

### Freiburg 3 walking halfsphere

```bash
./Examples/RGB-D/rgbd_tum \
  Vocabulary/ORBvoc.txt \
  Examples/RGB-D/TUM3.yaml \
  data/rgbd_dataset_freiburg3_walking_halfsphere \
  Examples/RGB-D/associations/fr3_walking_halfsphere.txt
```

### Freiburg 3 walking static

```bash
./Examples/RGB-D/rgbd_tum \
  Vocabulary/ORBvoc.txt \
  Examples/RGB-D/TUM3.yaml \
  data/rgbd_dataset_freiburg3_walking_static \
  Examples/RGB-D/associations/fr3_walking_static.txt
```

## Custom 3D Debug Viewer

The repository includes a standalone viewer in [DebugViewerNative](DebugViewerNative/README.md).

Run the viewer in one terminal:

```bash
./DebugViewerNative/debug_map_viewer_gl debug_map_state.txt
```

Run ORB-SLAM3 in another terminal with snapshot publishing enabled:

```bash
ORB_SLAM3_DEBUG_MAP_FILE=debug_map_state.txt \
./Examples/Monocular/mono_tum \
  Vocabulary/ORBvoc.txt \
  Examples/Monocular/TUM3.yaml \
  data/rgbd_dataset_freiburg3_walking_xyz
```

You can use the same `ORB_SLAM3_DEBUG_MAP_FILE` environment variable with `rgbd_tum` as well.

## Notes on the Monocular DA2 Pipeline

The current monocular pipeline is:

- RGB frame in
- frame submitted to a dedicated Depth Anything v2 worker thread
- latest completed estimated depth used to refresh YOLO dynamic masks
- dynamic mask propagated forward using the RGB-D-style logic
- optical-flow tracking used on lightweight frames
- masked ORB extraction used on keyframes
- normal ORB-SLAM tracking, mapping, and trajectory output downstream

This is meant to emulate the RGB-D dynamic logic as closely as possible, while replacing sensor depth with estimated depth.

## Troubleshooting

### `Not enough associated trajectory pairs to evaluate`

The evaluator now auto-normalizes oversized timestamps in `CameraTrajectory.txt`, so this should no longer happen for the current `mono_tum` output format. If it does, make sure:

- the dataset directory contains `groundtruth.txt`
- `CameraTrajectory.txt` was actually produced
- the run did not fail before shutdown

### ONNX Runtime not found

The CMake configure step expects:

- `da2-code/onnxruntime-osx-arm64-1.17.1/include`
- `da2-code/onnxruntime-osx-arm64-1.17.1/lib`

If those are missing, configuration will fail.

### Pangolin crashes on macOS

This branch is set up to use the separate `DebugViewerNative` tool for interactive inspection. Use `ORB_SLAM3_DEBUG_MAP_FILE=...` with the standalone viewer instead of relying on Pangolin windows for long runs.

## Citation

If you find this work useful in your research, please consider citing:

```bibtex
@inproceedings{zhang2025ngdslam,
  title={NGD-SLAM: Towards Real-Time Dynamic SLAM without GPU},
  author={Zhang, Yuhao and Bujanca, Mihai and Luján, Mikel},
  booktitle={2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={3467--3473},
  year={2025},
  doi={10.1109/IROS60139.2025.11246202},
  publisher={IEEE}
}
```
