// Actuator layer: receives commands from ROS 2 and drives the hardware.
#pragma once

// Velocity command in the robot frame: vx, vy [m/s] and omega [rad/s].
// Real implementation: omnidirectional inverse kinematics -> setMotor() in motor.c.
void actuators_set_velocity(float vx, float vy, float omega);
