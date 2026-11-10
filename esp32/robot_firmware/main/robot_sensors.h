// Sensor layer: the only place that knows where sensor data comes from.
// The rest of the firmware calls these functions and never touches hardware.
// All values use SI units, as ROS 2 expects.
#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    float accel[3];  // linear acceleration x, y, z [m/s^2]
    float gyro[3];   // angular velocity x, y, z [rad/s]
} imu_data_t;

esp_err_t sensors_init(void);

// Robot ID (0-7). Real source: 3-bit DIP switch on the I2C expander
// ((readbyte() & sw_mask) >> 5 in i2c_expander.c).
uint8_t sensors_get_robot_id(void);

// Distances of the 3 ultrasonic sensors [m].
// Real source: ultrasonic_sensor_t.distance in ultrasonic.c (cm -> m).
void sensors_get_ultrasonic(float distances_m[3]);

// Angular velocity of the 3 wheels [rad/s].
// Real source: motors[i].omega in motor.c (updated every 20 ms).
void sensors_get_wheel_speeds(float omega_rad_s[3]);

// IMU readings. Real source: MPU-9250 on the robot board (driver not written yet).
void sensors_get_imu(imu_data_t *imu);

// Battery voltage [V]. Real source: battery_read_task() in adc_read.c.
float sensors_get_battery_voltage(void);
