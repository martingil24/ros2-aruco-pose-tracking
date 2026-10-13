# ROS 2 ArUco Pose Tracking

Real-time 3D localization of ArUco fiducial markers using **ROS 2 Jazzy**, **OpenCV** and **TF2**.
Developed as part of my Bachelor's Thesis (TFG) at the Budapest University of Technology and Economics (BME), working with the lab's robotic platforms under weekly supervision.

<!-- DEMO: add a screenshot/GIF of the 3D axes drawn over the marker here -->

## Overview

A C++ ROS 2 node subscribes to a camera feed, detects ArUco markers, estimates each marker's 6-DoF pose relative to the camera and publishes it to the TF2 tree, so any other node in the system can query where the marker is in space.

**Pipeline:**
`/image_raw` → ArUco detection (custom 4x4 dictionary) → pose estimation (`estimatePoseSingleMarkers`) → rotation vector → rotation matrix (Rodrigues) → quaternion → TF2 broadcast `camera_frame → marker_<id>`

## ROS interface

| Type | Name | Description |
|------|------|-------------|
| Subscribed topic | `/image_raw` (`sensor_msgs/Image`) | Camera feed |
| Published topic | `/aruco/image_out` (`sensor_msgs/Image`) | Feed annotated with marker outlines, IDs and 3D axes |
| TF transform | `camera_frame → marker_<id>` | Pose of every detected marker |

## Tech stack

C++17 · ROS 2 Jazzy · OpenCV 4.6 (aruco module) · TF2 · cv_bridge · Ubuntu 24.04

## Build and run

```bash
cd ~/ros2_ws
colcon build --packages-select aruco_perception
source install/setup.bash

# Terminal 1: camera driver
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:=mjpeg2rgb

# Terminal 2: detector
ros2 run aruco_perception detector_node

# Inspect the output
ros2 run rqt_image_view rqt_image_view /aruco/image_out
ros2 run tf2_ros tf2_echo camera_frame marker_0
```

## Progress log

The thesis is developed in weekly iterations agreed with my supervisor.

| Week | Focus | Log |
|------|-------|-----|
| 1 | Environment setup, camera pipeline, first detection attempts | [Week 1](docs/weeks/week-01.md) |
| 2 | Custom dictionary, 3D pose estimation, TF2 integration | [Week 2](docs/weeks/week-02.md) |

## Known limitations and next steps

- **Camera intrinsics are placeholders** (fx = fy = 800, principal point at 320×240, zero distortion), so distances are approximate. Next: calibrate the real camera with the `camera_calibration` package.
- Marker size, dictionary and frame names are hard-coded. Next: expose them as ROS parameters.
- `estimatePoseSingleMarkers` is deprecated from OpenCV 4.7. Next: migrate to `cv::solvePnP`.
