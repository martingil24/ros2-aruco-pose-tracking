// Dummy implementation of the sensor layer: made-up values that change over
// time, so the whole ROS 2 communication can be tested without the robot.
// To use the real sensors, replace this file with one that implements the
// same functions on top of the hardware drivers.
#include <math.h>
#include "esp_timer.h"
#include "sdkconfig.h"
#include "robot_sensors.h"

static float time_s(void)
{
    return (float)esp_timer_get_time() / 1e6f;
}

esp_err_t sensors_init(void)
{
    return ESP_OK;
}

uint8_t sensors_get_robot_id(void)
{
    return (uint8_t)CONFIG_ROBOT_DUMMY_ID;
}

void sensors_get_ultrasonic(float distances_m[3])
{
    const float t = time_s();
    for (int i = 0; i < 3; i++) {
        // Between 0.2 m and 0.8 m, each sensor shifted by a third of a period
        distances_m[i] = 0.50f + 0.30f * sinf(0.5f * t + 2.094f * i);
    }
}

void sensors_get_wheel_speeds(float omega_rad_s[3])
{
    const float t = time_s();
    for (int i = 0; i < 3; i++) {
        omega_rad_s[i] = 5.0f * sinf(0.2f * t + 2.094f * i);
    }
}

void sensors_get_imu(imu_data_t *imu)
{
    const float t = time_s();
    imu->accel[0] = 0.1f * sinf(t);
    imu->accel[1] = 0.1f * cosf(t);
    imu->accel[2] = 9.81f;               // gravity: robot standing on the floor
    imu->gyro[0] = 0.0f;
    imu->gyro[1] = 0.0f;
    imu->gyro[2] = 0.5f * sinf(0.3f * t); // turning left and right
}

float sensors_get_battery_voltage(void)
{
    // Slowly discharging from 7.4 V, never below 6.0 V
    const float v = 7.4f - 0.001f * time_s();
    return v > 6.0f ? v : 6.0f;
}
