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
