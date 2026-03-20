# ros2bag_replay

A ROS 2 utility package for replaying camera bag data during early-stage
development and testing of ROS 2 systems — no physical sensors required.

The package ships a C++ `camera_info_fixer_node` and an XML launch file that
plays a bag on loop, fixes common recording artefacts in the CameraInfo
messages, and (optionally) opens RViz2 with a pre-configured depth-cloud view.
New bag datasets can be dropped in as the project grows; currently one
RealSense dataset is included.

**Intended use-case:** Testing ROS 2 perception pipelines (e.g. nvblox TSDF
mapping) with publicly available, non-application-specific bag data before
physical hardware is available.

---

## Package structure

```
ros2bag_replay/
├── src/
│   └── camera_info_fixer_node.cpp   C++ node — corrects bag CameraInfo headers
├── launch/
│   └── bag_depth_viewer.launch.xml  Main launch file (bag + fixer + optional RViz2)
├── config/
│   └── depth_viewer.rviz            RViz2 configuration (DepthCloud + Image)
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Prerequisites

| Requirement | Version | Notes                              |
|-------------|---------|-------------------------------------|
| ROS 2       | Jazzy   |                                     |
| rviz2       | any     | Only needed when `use_rviz:=true`   |
| CMake       | ≥ 3.20  |                                     |

---

## Build

```bash
# From the workspace root (e.g. /home/vscode/dev)
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros2bag_replay
source install/setup.bash
```

---

## Available bag datasets

### Sophia Dataset — RealSense front/side camera

| Property     | Value                                                                   |
|--------------|-------------------------------------------------------------------------|
| Path         | `some_test_data/sophia_dataset_example/data/cameras/realsense_front_side_ros2` |
| Sensor       | Intel RealSense (depth + colour)                                        |
| Depth topic  | `/device_0/sensor_0/Depth_0/image/data`                                 |
| Colour topic | `/device_0/sensor_1/Color_0/image/data`                                 |
| CameraInfo   | `/device_0/sensor_0/Depth_0/info/camera_info`                           |
| Frame ID     | `"0"` (depth sensor header)                                             |
| Notes        | CameraInfo has zero timestamp — `camera_info_fixer_node` corrects this  |

---

## Launch: `bag_depth_viewer.launch.xml`

Plays a ROS 2 bag on loop, runs `camera_info_fixer_node`, and optionally
opens RViz2 with the depth-cloud view.

### Launch arguments

| Argument                   | Type   | Default                                              | Description                                                                              |
|----------------------------|--------|------------------------------------------------------|------------------------------------------------------------------------------------------|
| `bag_path`                 | string | **required**                                         | Path to the ROS 2 bag directory or `.db3` file.                                          |
| `depth_topic`              | string | `/device_0/sensor_0/Depth_0/image/data`              | Depth image topic (`sensor_msgs/Image`, `32FC1` or `16UC1`).                             |
| `image_topic`              | string | `/device_0/sensor_1/Color_0/image/data`              | RGB/colour image topic (`sensor_msgs/Image`). Used by RViz2 DepthCloud for colourisation.|
| `depth_camera_info_topic`  | string | `/device_0/sensor_0/Depth_0/info/camera_info`        | Depth camera intrinsics topic (`sensor_msgs/CameraInfo`).                                |
| `camera_frame`             | string | `0`                                                  | TF frame of the depth sensor. Must match the `frame_id` in the depth image header.       |
| `use_rviz`                 | bool   | `true`                                               | Launch RViz2 with the pre-configured depth viewer. Set `false` for headless use.         |

### Usage examples

**Sophia dataset (defaults apply — just supply the bag path):**

```bash
ros2 launch ros2bag_replay bag_depth_viewer.launch.xml \
  bag_path:=some_test_data/sophia_dataset_example/data/cameras/realsense_front_side_ros2
```

**Headless / CI — suppress RViz2:**

```bash
ros2 launch ros2bag_replay bag_depth_viewer.launch.xml \
  bag_path:=/path/to/bag \
  use_rviz:=false
```

**Custom bag with different topic names:**

```bash
ros2 launch ros2bag_replay bag_depth_viewer.launch.xml \
  bag_path:=/path/to/bag \
  depth_topic:=/camera/depth/image_rect_raw \
  depth_camera_info_topic:=/camera/depth/camera_info \
  camera_frame:=camera_depth_optical_frame \
  use_rviz:=true
```

---

## Node: `camera_info_fixer_node`

### Why is this node needed?

Many bags record `sensor_msgs/CameraInfo` with a **zero timestamp** and/or an
**empty `frame_id`**.  RViz2's DepthCloud display requires:
1. The CameraInfo stamp to match (within a small tolerance) the depth image stamp.
2. The `frame_id` to match the TF frame published by the depth sensor.

Without correcting these fields the depth cloud simply does not render.

### How it works

1. Subscribes to the bag's CameraInfo topic using a **TRANSIENT_LOCAL /
   RELIABLE** QoS (equivalent to "latching"), so the single stored message is
   received even when this node connects after the bag player has already
   published it.
2. On each incoming depth `Image`, copies the latched `CameraInfo`, patches in
   the image's `stamp` and the configured `frame_id`, then republishes on
   `/viewer/depth_info` (configurable).

### Parameters

| Parameter        | Type   | Default                                              | Description                                      |
|------------------|--------|------------------------------------------------------|--------------------------------------------------|
| `camera_info_in` | string | `/device_0/sensor_0/Depth_0/info/camera_info`        | Source CameraInfo topic from the bag.            |
| `depth_image_in` | string | `/device_0/sensor_0/Depth_0/image/data`              | Depth image topic (used only for stamp copying). |
| `camera_info_out`| string | `/viewer/depth_info`                                 | Republished, corrected CameraInfo topic.         |
| `frame_id`       | string | `0`                                                  | `frame_id` to inject into the output CameraInfo. |

---

## RViz2 configuration

`config/depth_viewer.rviz` is pre-configured with:

- **Grid** — XY ground plane at the world origin.
- **Depth Cloud** — reads `/viewer/depth_info` (CameraInfo) and
  `/device_0/sensor_0/Depth_0/image/data` (depth map); colourises using
  `/device_0/sensor_1/Color_0/image/data` (RGB).
- **Depth Image** — 2D panel showing the raw depth image.
- **TF** — shows the static `world → 0` transform broadcast by the launch file.
- Fixed Frame: `world`.

The configuration is bag-independent — the `/viewer/depth_info` indirection
means changing the bag's native topic names only requires updating the
`camera_info_fixer_node` parameters, not the RViz2 config.

---

## Integration with nvblox_base

This package provides the bag replay layer only.  To feed the replayed depth
stream into the nvblox TSDF mapper, launch `nvblox_base/depth_stream.launch.xml`
alongside or after this package:

```bash
# Terminal 1 — replay the bag (RViz2 optional)
ros2 launch ros2bag_replay bag_depth_viewer.launch.xml \
  bag_path:=some_test_data/sophia_dataset_example/data/cameras/realsense_front_side_ros2 \
  use_rviz:=false

# Terminal 2 — run TSDF fusion with static pose (no TF tree in bag)
ros2 launch nvblox_base depth_stream.launch.xml \
  depth_topic:=/device_0/sensor_0/Depth_0/image/data \
  camera_info_topic:=/device_0/sensor_0/Depth_0/info/camera_info \
  use_static_pose:=true
```

---

## Adding new bag datasets

1. Place the bag directory anywhere accessible (e.g. `some_test_data/<name>/`).
2. Identify the depth, colour, and CameraInfo topic names in the bag:
   ```bash
   ros2 bag info /path/to/bag
   ```
3. Launch with the appropriate topic overrides:
   ```bash
   ros2 launch ros2bag_replay bag_depth_viewer.launch.xml \
     bag_path:=/path/to/bag \
     depth_topic:=/your/depth/topic \
     depth_camera_info_topic:=/your/camera_info/topic \
     camera_frame:=your_frame_id
   ```
4. Document the bag source, topic names, and frame ID in this README under
   **Available bag datasets**.

---

## License

MIT — see [LICENSE](LICENSE).
