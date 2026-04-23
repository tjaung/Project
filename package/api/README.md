# API

Planned API shape:

- high-level lifecycle at the top level
- pipeline telemetry as structured frame/map data
- AR helpers in a dedicated sub-API

Top-level control:

- `Initialize(...)`
- `StartVideo(...)`
- `StartDatasetRun(...)`
- `Stop()`
- `Pause()`
- `Resume()`
- `Reset()`
- `Shutdown()`

Top-level data:

- `GetCurrentFrameData()`
- `GetPreviousFrameData()`
- `GetMapState()`
- `GetRunMetrics()`

AR sub-API:

- `api.ar.CreateAnchorAtCurrentPose()`
- `api.ar.CreateAnchor(...)`
- `api.ar.GetAnchors()`
- `api.ar.RemoveAnchor(...)`

Current first-pass API:

- `monocular_slam::SessionConfig`
- `monocular_slam::FrameData`
- `monocular_slam::MonocularPipeline`
- `monocular_slam::MonocularSession`

Implemented now:

- `Initialize(...)`
- `Shutdown()`
- `IsInitialized()`
- `ProcessFrame(...)`
- `GetCurrentFrameData()`
- `GetPreviousFrameData()`
- `GetLastFrameData()`
- `StartVideo(...)`
- `StartDatasetRun(...)`
- `Stop()`
- `Pause()`
- `Resume()`
- `Wait()`
- `GetRunMetrics()`
- `session.ar.CreateAnchorAtCurrentPose()`
- `session.ar.CreateAnchor(...)`
- `session.ar.GetAnchors()`
- `session.ar.RemoveAnchor(...)`
- `session.ar.ClearAnchors()`

This first pass exposes:

- current pose
- tracking state
- current/previous frame telemetry snapshots
- raw ORB keypoints before mask filtering
- filtered ORB keypoints after mask filtering
- ORB keypoints for the current frame
- tracked keypoints and tracked map-point count
- current dynamic mask
- current estimated depth
- YOLO detections
- optical-flow tracks
- mask-propagation seed points, flowed points, tracked 3D points, and clusters
- keyframe request / keyframe status
- active map keyframe and map-point counts

AR notes:

- anchors are managed at the session layer
- `CreateAnchorAtCurrentPose()` uses the latest valid tracked pose
- `CreateAnchor(...)` accepts an explicit world pose
- anchors are intentionally separate from the generic telemetry/session API

Debug example:

- `package/examples/dataset_debug`
  - runs a TUM-style dataset through `monocular_slam::MonocularPipeline`
  - overlays current-frame telemetry on each dataset image
