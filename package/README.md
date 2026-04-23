# Package

Fresh rebuild of the monocular package.

This package is being rebuilt with one rule:

- the monocular pipeline is the source of truth

Build portability:

- default builds avoid host-specific CPU flags like `-march=native`
- this makes the package safer to build for broader hardware targets
- if you want machine-specific tuning for a local-only build, configure with:
  - `-DMONOCULAR_SLAM_ENABLE_NATIVE_OPTIMIZATION=ON`

Design goals:

- preserve the research monocular pipeline behavior
- use the RGB-D path as the control-flow reference
- substitute DA2 anywhere the monocular path needs depth
- expose pipeline telemetry without re-implementing pipeline logic in the wrapper
- support both live video sources and dataset runs
- support dataset metrics at end of run
- keep AR helpers in a separate API layer on top of the core

Planned layout:

- `core/`: package architecture notes and pipeline parity notes
- `api/`: high-level control API and telemetry/AR API design
- `apps/`: thin runners for video and dataset modes
- `include/`, `src/`: extracted monocular core implementation
- `evaluation/`: dataset metric helpers
- `examples/`: example consumers of the API
- `config/`: package-local configs/templates

Immediate rebuild order:

1. restore minimal monocular core package
2. verify pipeline parity
3. add a thin high-level API around the working core
4. add richer telemetry exposure
5. add dataset metrics path
6. add AR sub-API

Current status:

- monocular core copied from the research pipeline
- standalone package build configured around package-local dependencies
- thin API wrapper added:
  - `api/include/monocular_slam/Config.h`
  - `api/include/monocular_slam/Types.h`
  - `api/include/monocular_slam/MonocularPipeline.h`
- tracker-owned telemetry is now exposed through the API for:
  - depth
  - dynamic mask
  - YOLO detections
  - optical-flow tracks
  - mask-propagation points and clusters
  - ORB keypoints
  - keyframe request/status
  - current and previous frame snapshots

The current API is still intentionally thin:

- initialize the monocular system
- process a monocular frame
- retrieve current and previous frame data

It reads from the real core pipeline and does not add wrapper-side fallback logic.

## C API Layer

The package now includes a thin `extern "C"` wrapper for external consumers such as:

- Android Studio / Kotlin via JNI
- C applications
- other languages that bind to C ABIs more easily than to C++

The C layer is intentionally thin:

- it wraps the current top-level `MonocularSession` API
- it does not re-implement SLAM logic
- it uses an opaque handle so callers do not need C++ types

Files:

- header: `api/include/monocular_slam/c_api.h`
- implementation: `api/src/c_api.cpp`
- library output: `lib/libmonocular_slam_capi.dylib`

### Build

From `package/`:

```bash
./build.sh
```

Or build the C wrapper target directly:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target monocular_slam_capi -j4
```

The C wrapper depends on:

- `libORB_SLAM3_Monocular_Package.dylib`
- `libmonocular_slam_api.dylib`
- package-local dependencies already referenced by the build

### Include and Link

For an external native consumer, include:

```c
#include "monocular_slam/c_api.h"
```

And link against:

- `package/lib/libmonocular_slam_capi.dylib`

You will also need the package include path:

- `package/api/include`

### API Shape

The C layer is handle-based:

- `mdslam_create`
- `mdslam_destroy`
- `mdslam_initialize`
- `mdslam_shutdown`

It supports three main usage styles:

1. Direct frame processing
2. Stepped live video
3. Stepped dataset processing

### Direct Frame Processing

Use this when your platform already owns camera delivery and you want to push frames in manually.

Main calls:

- `mdslam_initialize`
- `mdslam_process_frame_bgr8`
- `mdslam_get_snapshot`

Example:

```c
mdslam_handle* handle = mdslam_create();

mdslam_session_config config = {0};
config.vocabulary_path = "Vocabulary/ORBvoc.txt";
config.settings_path = "apps/Monocular/TUM3.yaml";
config.use_viewer = 0;
config.sensor_mode = MDSLAM_SENSOR_MONOCULAR;

if (!mdslam_initialize(handle, &config)) {
    fprintf(stderr, "init failed: %s\n", mdslam_get_last_error(handle));
}

mdslam_frame frame = {0};
frame.bgr_data = image_data;
frame.width = width;
frame.height = height;
frame.stride_bytes = stride_bytes;
frame.timestamp = timestamp_sec;
frame.frame_name = "camera_frame";

if (!mdslam_process_frame_bgr8(handle, &frame)) {
    fprintf(stderr, "frame failed: %s\n", mdslam_get_last_error(handle));
}

mdslam_snapshot snapshot = {0};
if (mdslam_get_snapshot(handle, &snapshot)) {
    printf("frame=%llu tracking_state=%d tracked_pts=%zu\n",
           (unsigned long long)snapshot.frame_id,
           snapshot.tracking_state,
           snapshot.tracked_map_point_count);
}

mdslam_shutdown(handle);
mdslam_destroy(handle);
```

### Live Video Mode

Use this when you want the native package to own the video source directly.

Main calls:

- `mdslam_open_video`
- `mdslam_step_video`
- `mdslam_finalize_video`

Example:

```c
mdslam_video_config video = {0};
video.source = "0";
video.realtime_playback = 1;

mdslam_open_video(handle, &video);

while (mdslam_step_video(handle)) {
    mdslam_snapshot snapshot = {0};
    mdslam_get_snapshot(handle, &snapshot);
}

mdslam_finalize_video(handle);
```

`video.source` can be:

- webcam index as a string, such as `"0"`
- video file path
- full URL, such as `"http://10.0.0.198:8080/video"`
- shorthand IP webcam source such as `"10.0.0.198:8080"`

### Dataset Mode

Use this when you want the package to own a TUM-style dataset run.

Main calls:

- `mdslam_open_dataset`
- `mdslam_step_dataset`
- `mdslam_finalize_dataset`
- `mdslam_get_run_metrics`

Example:

```c
mdslam_dataset_config dataset = {0};
dataset.dataset_path = "../data/rgbd_dataset_freiburg3_walking_xyz";
dataset.realtime_playback = 0;
dataset.save_trajectories = 1;
dataset.evaluate_metrics = 1;

mdslam_open_dataset(handle, &dataset);
while (mdslam_step_dataset(handle)) {
}
mdslam_finalize_dataset(handle);

mdslam_run_metrics metrics = mdslam_get_run_metrics(handle);
printf("frames=%zu mean=%f median=%f\n",
       metrics.frames_processed,
       metrics.mean_tracking_time_sec,
       metrics.median_tracking_time_sec);
```

### Snapshot Data

`mdslam_get_snapshot(...)` gives a compact state view of the current session, including:

- runner state
- current frame id and timestamp
- tracking state
- pose availability
- dynamic mask / estimated depth availability
- keyframe flags
- tracked keypoint / map point counts
- active local map counts
- current pose matrix

This is the main C-facing polling function for external apps.

### AR Functions

The C layer exposes the current anchor API:

- `mdslam_create_anchor_at_current_pose`
- `mdslam_create_anchor`
- `mdslam_get_anchor_count`
- `mdslam_get_anchor`
- `mdslam_clear_anchors`

This mirrors the session-level AR layer without exposing any C++ classes.

### Error Handling

Most functions return `0` on failure and `1` on success.

To inspect errors:

- `mdslam_get_last_error`
- `mdslam_get_last_error_code`

Common error categories:

- `MDSLAM_ERROR_INVALID_ARGUMENT`
- `MDSLAM_ERROR_NOT_INITIALIZED`
- `MDSLAM_ERROR_BUSY`
- `MDSLAM_ERROR_EXCEPTION`

### Android / JNI Note

For Android Studio and Kotlin, the intended integration path is:

1. Kotlin calls JNI
2. JNI calls this C API
3. this C API calls the C++ session/pipeline layer

That keeps the ABI stable and avoids exposing C++ templates, STL types, or name-mangled symbols directly to Java/Kotlin.
