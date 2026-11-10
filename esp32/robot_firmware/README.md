# ESP32 robot firmware (micro-ROS)

Message layer of the robot: the ESP32 sends all sensor data to ROS 2 and receives velocity commands and its pose from the camera system. Sensor values are **dummy data** for now, so the communication can be developed and tested without the robot hardware.

## Architecture

- **Sensor layer** (`robot_sensors.h`): getters in SI units. `robot_sensors_dummy.c` returns made-up values; the real implementation will call the robot drivers with the same functions.
- **Actuator layer** (`robot_actuators.h`): `actuators_set_velocity()` prints the command (dummy).
- **One FreeRTOS task per sensor group**, each at its own rate, leaves the latest reading in a one-slot queue (mailbox).
- **One micro-ROS task** publishes every new reading and handles incoming messages. Only this task uses micro-ROS, which is not thread-safe.
- **Namespace per robot:** the robot ID (DIP switch, dummy value `CONFIG_ROBOT_DUMMY_ID`) gives `/robot_<id>/...`, so several robots can share the network. The ID must match the ArUco marker ID on the robot.

## Topics

| Topic | Direction | Type | Rate |
|-------|-----------|------|------|
| `/robot_<id>/ultrasonic` | ESP32 → ROS | `std_msgs/Float32MultiArray` (3 distances, m) | 10 Hz |
| `/robot_<id>/wheel_speeds` | ESP32 → ROS | `std_msgs/Float32MultiArray` (3 wheels, rad/s) | 20 Hz |
| `/robot_<id>/imu` | ESP32 → ROS | `sensor_msgs/Imu` (orientation not provided) | 50 Hz |
| `/robot_<id>/battery` | ESP32 → ROS | `std_msgs/Float32` (V) | 1 Hz |
| `/robot_<id>/cmd_vel` | ROS → ESP32 | `geometry_msgs/Twist` (vx, vy, omega) | on demand |
| `/robot_<id>/pose` | ROS → ESP32 | `geometry_msgs/Pose2D` (x, y, heading from the camera) | camera rate |

Sensor topics are best effort: a lost sample is replaced by the next one.

## Build and flash

Same setup as [`microros_publisher`](../microros_publisher): ESP-IDF v5.5, in a shell without ROS 2 sourced. `app-colcon.meta` raises the micro-ROS limits to 6 publishers and 4 subscriptions.

```bash
bash --norc
. ~/esp/esp-idf-v5.5/export.sh
cd esp32/robot_firmware
idf.py menuconfig   # micro-ROS Settings (WiFi, Agent IP) and Robot firmware settings (robot ID)
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Test

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
ros2 topic hz /robot_2/imu
ros2 topic pub --once /robot_2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.1}, angular: {z: 0.5}}"
ros2 topic pub --once /robot_2/pose geometry_msgs/msg/Pose2D "{x: 0.5, y: -0.3, theta: 1.57}"
```

The serial monitor prints the received messages and, every 2 s, how many messages of each type were sent.

## Notes

- `rclc_executor_spin_some()` is called with no timeout plus a one-tick `vTaskDelay`. With a timeout, every wait lasted at least one FreeRTOS tick (10 ms) and the fast topics stayed below their rate (IMU about 17 Hz instead of 50 Hz).
- WiFi power save is disabled to reduce latency.
