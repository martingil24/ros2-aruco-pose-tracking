# Week 1: Environment setup and first detection attempts

## Goal
Set up the development environment and build a first ROS 2 node able to receive camera images and detect ArUco markers.

## Work done
- Set up Ubuntu 24.04 (VirtualBox) with ROS 2 Jazzy and OpenCV.
- Created the C++ package `aruco_perception` with a node subscribed to the webcam feed.
- Fixed video decoding issues by configuring the camera driver with the `mjpeg2rgb` pixel format.
- Implemented the base detection logic and published the processed image on `/aruco/image_out` for visualization in `rqt_image_view`.

## Problem found
The image pipeline worked end to end, but OpenCV did not recognize the printed physical marker. Standard predefined 4x4 and 5x5 dictionaries were tested without success, which suggested the marker was not generated from any of them.

## Outcome
Working camera → ROS 2 → OpenCV pipeline. The marker design was handed over to my supervisor for review.
