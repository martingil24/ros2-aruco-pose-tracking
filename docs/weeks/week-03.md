# Week 3: Multi-robot 2D localization architecture

## Goal
Implement the node architecture proposed by my supervisor: an overhead camera, a reference marker on the floor (ID 1, 195 mm) and markers on the robots (IDs 0, 2, 3, 100 mm). The system must output every robot's x, y and heading on the floor. Requirement: keep it simple, no quaternions.

## Work done
- **`aruco_interfaces` package** with custom messages: `Marker`, `MarkerArray`, `RobotPose2D` and `RobotPose2DArray`.
- **Detector** now publishes all detections on `/aruco/markers`. Pose is estimated per marker with its own physical size (ID-dependent). The dictionary is created once at startup instead of every frame. TF2 and quaternions were removed.
- **`reference_node`**: selects the reference marker, whose plane defines the ground, and publishes it on `/reference_tf`. It keeps the last known pose so a robot covering the marker briefly does not break the system.
- **`robot_pos_node`**: converts each robot marker into the ground frame using rotation matrices (`cv::Rodrigues`) and publishes x, y and heading on `/positions`.
- Marker sizes and the reference ID are ROS parameters.

## Verification
Reference marker fixed, robot marker ID 0 moved by hand (distances estimated by eye):

| Test | Expected | Measured |
|------|----------|----------|
| Slide about 10 cm along the ground X axis | Δx ≈ 0.10 m, Δy ≈ 0, heading unchanged | Δx = 0.103 m, Δy = 0.004 m, Δheading = 0.6° |
| Rotate 90° in place | Δheading ≈ 1.57 rad, position unchanged | Δheading = 1.56 rad (89°), position change < 3 mm |

## Limitations
- Placeholder camera intrinsics (no calibration yet).
- Reference pose taken from a single detection, without filtering.
- Optional task (micro-ROS on ESP32, issue #4) not started yet.
