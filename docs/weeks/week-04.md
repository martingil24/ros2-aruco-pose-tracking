# Week 4: Launch file to start the whole system

## Goal
Replace the personal bash aliases (one terminal per node) with a standard ROS 2 launch file stored in the repository, so the whole system starts with a single command on any machine.

## Work done
- **`launch/localization.launch.py`** (Python launch file) starts the camera driver, `detector_node`, `reference_node` and `robot_pos_node`.
- **Parameters in one place:** the camera pixel format, the marker sizes and the reference ID are defined in the launch file. The reference ID is written once and passed to every node that needs it.
- **Build configuration:** `CMakeLists.txt` installs the `launch/` folder, and `package.xml` declares `launch_ros` and `usb_cam` as runtime dependencies.
- Reviewed how the previous setup worked (`.bashrc`, workspace sourcing and aliases) and why a launch file is better: aliases only exist on my machine, while the launch file is part of the repository.

## Usage
```bash
ros2 launch aruco_perception localization.launch.py
```
All node logs appear in a single terminal, and `Ctrl+C` stops every node cleanly.

## Verification
The four processes start with one command and `/positions` is published as before.

## Next
- micro-ROS on the ESP32 (issue #4), once the cable is available.
