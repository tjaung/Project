# 3d_viewer

Standalone live 3D point-cloud viewer for the backend stream emitted by `package/`.

It reads the text snapshot written by the backend at:

```bash
package/out/live_cloud_stream.txt
```

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

## Run

In one terminal, run the backend demo from `package/`:

```bash
./examples/monocular_video_demo \
  Vocabulary/ORBvoc.txt \
  da2-code/model_fp16.onnx \
  0
```

In another terminal, run the viewer from repo root:

```bash
./3d_viewer/build/3d_viewer_gl package/out/live_cloud_stream.txt
```

## Controls

- Left drag: orbit
- Right drag: pan
- Scroll: zoom
- `A`: auto-frame
- `F`: follow current camera
- `P`: toggle point cloud
- `T`: toggle trajectory
- `C`: toggle current camera frustum
- `1`: perspective reset
- `2`: top view
- `3`: front view
- `Esc` or `Q`: quit
