# Week 2: 3D pose estimation and TF2 integration

## Goal
Get the physical marker detected and compute its real position and orientation in space, integrated with the ROS 2 transform system.

## Work done
- **Detection fixed:** the lab marker matches a custom dictionary, `generateCustomDictionary(10, 4, 0)` (10 markers, 4x4 bits, seed 0). Marker ID 0 is now detected reliably.
- **Pose estimation:** added the physical marker size (10 cm) and a camera intrinsics matrix so OpenCV can estimate translation and rotation in metres.
- **Visualization:** 3D axes (X red, Y green, Z blue) drawn over each detected marker in the output image.
- **TF2 integration:** updated `package.xml` and `CMakeLists.txt` and added a `tf2_ros::TransformBroadcaster`. Each detection is published as `camera_frame → marker_<id>`.
- **Rotation conversion:** OpenCV's rotation vector is converted to a rotation matrix (`cv::Rodrigues`), then to a `tf2::Matrix3x3`, and finally to a quaternion (x, y, z, w).

## Verification
Transforms checked in real time with `ros2 run tf2_ros tf2_echo camera_frame marker_0`.

## Limitations
The camera matrix uses placeholder values rather than a real calibration, so distances are approximate.
