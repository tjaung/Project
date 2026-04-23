# Core

This directory tracks the package-core contract.

The core must remain:

- monocular-pipeline first
- tracker/system owned
- RGB-D logic as reference
- DA2 substituted for depth needs
- no wrapper-side fallback tracking, masking, or depth logic

Core outputs to expose later:

- current and previous frame telemetry
- estimated depth
- YOLO detections
- dynamic mask
- mask propagation sampled points
- optical-flow predictions/tracks
- ORB raw and filtered points
- tracked map-point projections
- keyframe decisions
- pose, keyframes, trajectory, map state
