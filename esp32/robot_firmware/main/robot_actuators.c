// Dummy actuator layer: only prints the command on the serial port, so it
// can be verified that the message arrived.
#include "esp_log.h"
#include "robot_actuators.h"

static const char *TAG = "actuators";

void actuators_set_velocity(float vx, float vy, float omega)
{
    ESP_LOGI(TAG, "Velocity command (dummy): vx=%.2f m/s, vy=%.2f m/s, omega=%.2f rad/s",
             vx, vy, omega);
}
