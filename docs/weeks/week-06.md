# Week 6: ESP32 message layer with dummy sensor data

## Goal
Prepare the ESP32 firmware that will run on each robot: periodically send all sensor data to ROS 2 with FreeRTOS tasks, and receive velocity commands and the robot's pose. Use dummy data, so it can be done without the robot hardware. Messages must work with several robots at the same time.

## Work done
- Reviewed the robot code of the other student and the board design: 3 ultrasonic sensors, 3 wheel encoders, DIP switch for the robot ID, battery measurement and an MPU-9250 IMU.
- **Sensor and actuator layer** with dummy implementations, so the real drivers can be plugged in later without changing the rest of the firmware.
- **One FreeRTOS task per sensor group** at its own rate, and a single micro-ROS task that publishes and receives (micro-ROS is not thread-safe).
- **Standard ROS 2 messages** and a **namespace per robot** (`/robot_<id>/...`). Details in [`esp32/robot_firmware`](../../esp32/robot_firmware).
- **`robot_pos_node`** now also publishes each robot's pose on `/robot_<id>/pose`, so every robot only receives its own pose.

## Verification

| Test | Result |
|------|--------|
| Publishing rates (messages per 2 s) | ultrasonic 20, wheel_speeds 40, imu 100, battery 2: exactly 10, 20, 50 and 1 Hz |
| `ros2 topic hz /robot_2/imu` | 50.0 Hz |
| `cmd_vel` and `pose` sent with `ros2 topic pub` | Printed on the ESP32 serial monitor with the same values |
| Full chain: camera → `robot_pos_node` → `/robot_2/pose` → ESP32 | The ESP32 prints its pose and it changes when marker 2 moves; robot 3 gets its own topic |

## Problem solved
The fast topics first reached only about 17 Hz. Each executor wait lasted at least one FreeRTOS tick (10 ms). Calling the executor without timeout and yielding one tick per loop gave the designed rates.

## Next
- Replace the dummy sensor layer with the real drivers.
- Inverse kinematics of the omnidirectional base: `cmd_vel` → wheel speeds → `setMotor()`.
