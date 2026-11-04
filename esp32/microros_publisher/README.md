# ESP32 micro-ROS publisher

micro-ROS firmware for the ESP32 (ESP32-WROOM-32D). It connects over WiFi to a micro-ROS Agent and publishes a counter (`std_msgs/Int32`) on `/freertos_int32_publisher` every second.
Based on the official `int32_publisher` example of [micro_ros_espidf_component](https://github.com/micro-ROS/micro_ros_espidf_component) (Jazzy).

## Requirements

- ESP-IDF v5.5
- Python packages in the ESP-IDF environment: `pip3 install catkin_pkg colcon-common-extensions lark empy==3.3.4`
- micro-ROS Agent for ROS 2 Jazzy

The micro-ROS component is downloaded automatically from the ESP Component Registry on the first build.

## Configure, build and flash

Build in a shell without ROS 2 sourced:

```bash
bash --norc
. ~/esp/esp-idf-v5.5/export.sh
cd esp32/microros_publisher
idf.py set-target esp32
idf.py menuconfig   # micro-ROS Settings: Agent IP and port, WiFi credentials
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

`sdkconfig` stores the WiFi password, so it is git-ignored and never committed.

## Run

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
ros2 topic echo /freertos_int32_publisher
```

Start the Agent first. If the ESP32 booted before it, press EN to restart the board.

## Network setup

The ESP32 and the laptop are on the same 2.4 GHz WiFi network. ROS 2 runs in a VirtualBox VM in NAT mode: UDP port 8888 is forwarded from the Windows host to the VM, and a Windows firewall rule allows it. The Agent IP configured on the ESP32 is the Windows host IP.
