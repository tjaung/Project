# DebugViewerNative

Native standalone OpenGL debug viewer for ORB-SLAM3 snapshots.

Controls:

- Left drag: orbit
- Right drag: pan
- Scroll: zoom
- `A`: auto-frame the current map cloud
- `F`: follow the latest SLAM camera
- `I`: toggle inertial graph edges
- `1`: perspective view
- `2`: top view
- `3`: front view
- `P`: toggle map points
- `K`: toggle keyframes
- `G`: toggle graph edges
- `Q` or `Esc`: quit

Viewer notes:

- The window title shows live counts for points, reference points, keyframes, and edges.
- The first successful snapshot auto-centers and auto-scales the camera to the map.
- A second `ORB-SLAM3: Current Frame` window mirrors the tracked-image overlay from the original viewer.
- Reference points are red, non-reference points are black, regular keyframes are blue, and the current camera is green.

Run the viewer in one terminal:

```bash
./DebugViewerNative/debug_map_viewer_gl debug_map_state.txt
```

Run ORB-SLAM3 in another terminal with snapshot publishing enabled:

```bash
ORB_SLAM3_DEBUG_MAP_FILE=debug_map_state.txt ./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml data/rgbd_dataset_freiburg1_xyz
```
