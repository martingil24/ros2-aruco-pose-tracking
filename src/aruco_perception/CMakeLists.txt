cmake_minimum_required(VERSION 3.8)
project(aruco_perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(aruco_interfaces REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(detector_node src/detector_node.cpp)
ament_target_dependencies(detector_node rclcpp sensor_msgs cv_bridge aruco_interfaces)
target_link_libraries(detector_node ${OpenCV_LIBS})

add_executable(reference_node src/reference_node.cpp)
ament_target_dependencies(reference_node rclcpp aruco_interfaces)

add_executable(robot_pos_node src/robot_pos_node.cpp)
ament_target_dependencies(robot_pos_node rclcpp aruco_interfaces)
target_link_libraries(robot_pos_node ${OpenCV_LIBS})

install(TARGETS detector_node reference_node robot_pos_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
