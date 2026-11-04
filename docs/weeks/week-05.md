# Week 5: ESP32 with micro-ROS over WiFi

## Goal
Connect the ESP32 to ROS 2 on my laptop: start with a simple program, then send data from the ESP32 over WiFi and see it in ROS 2.

## Work done
- **Board:** ESP32-WROOM-32D DevKit with a CP2102 USB-serial chip, passed to the Ubuntu VM with a VirtualBox USB filter (`/dev/ttyUSB0`).
- **First program:** the ESP-IDF blink example, verified through the serial monitor (LED ON/OFF messages every second). This board has no LED on GPIO 2.
- **Tools:** installed the micro-ROS Agent (ROS 2 Jazzy) and ESP-IDF v5.5, the version supported by micro-ROS for Jazzy.
- **Network:** the ESP32 and the laptop share the same WiFi. The VM stays in NAT mode with UDP port 8888 forwarded from Windows, plus a Windows firewall rule.
- **Firmware:** the micro-ROS `int32_publisher` example, now in [`esp32/microros_publisher`](../../esp32/microros_publisher). The micro-ROS component is downloaded from the ESP Component Registry, and the WiFi password stays out of the repository.

## Verification
The Agent logs `session established` and `publisher created`, and `ros2 topic echo /freertos_int32_publisher` shows the counter increasing every second.

## Next
ESP32 firmware with a software layer for the robot sensors, publishing dummy data on several topics.
