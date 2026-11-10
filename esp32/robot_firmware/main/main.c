// Robot firmware: sends the robot sensor data to ROS 2 and receives commands.
//
// - One FreeRTOS task per sensor group samples its sensors at its own rate and
//   leaves the latest reading in a one-slot queue (a "mailbox").
// - A single micro-ROS task publishes every new reading and runs the executor
//   that handles incoming messages. Only this task touches micro-ROS, because
//   micro-ROS is not thread-safe.
// - All topics live under the robot namespace /robot_<id>.
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "sdkconfig.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose2_d.h>

#include "robot_sensors.h"
#include "robot_actuators.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); vTaskDelete(NULL); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); } }

// Publishing period of each sensor group
#define ULTRASONIC_PERIOD_MS 100   // 10 Hz
#define WHEELS_PERIOD_MS      50   // 20 Hz
#define IMU_PERIOD_MS         20   // 50 Hz
#define BATTERY_PERIOD_MS   1000   //  1 Hz
#define REPORT_PERIOD_MS    2000   // serial report of published messages

static const char *TAG = "robot";

typedef struct {
    float v[3];
} vec3_t;

static uint8_t robot_id;
static QueueHandle_t ultrasonic_q, wheels_q, imu_q, battery_q;

// ---------- Sensor tasks (one per sensor group) ----------

static void ultrasonic_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    vec3_t data;
    while (1) {
        sensors_get_ultrasonic(data.v);
        xQueueOverwrite(ultrasonic_q, &data);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(ULTRASONIC_PERIOD_MS));
    }
}

static void wheels_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    vec3_t data;
    while (1) {
        sensors_get_wheel_speeds(data.v);
        xQueueOverwrite(wheels_q, &data);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(WHEELS_PERIOD_MS));
    }
}

static void imu_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    imu_data_t data;
    while (1) {
        sensors_get_imu(&data);
        xQueueOverwrite(imu_q, &data);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}

static void battery_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    float voltage;
    while (1) {
        voltage = sensors_get_battery_voltage();
        xQueueOverwrite(battery_q, &voltage);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(BATTERY_PERIOD_MS));
    }
}

// ---------- Incoming messages ----------

static void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    actuators_set_velocity((float)msg->linear.x, (float)msg->linear.y, (float)msg->angular.z);
}

static void pose_callback(const void *msgin)
{
    const geometry_msgs__msg__Pose2D *msg = (const geometry_msgs__msg__Pose2D *)msgin;
    ESP_LOGI(TAG, "Pose from camera: x=%.3f m, y=%.3f m, heading=%.3f rad",
             msg->x, msg->y, msg->theta);
}

// ---------- micro-ROS task ----------

static void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Synchronise the clock with the Agent so the IMU timestamps are meaningful
    RCSOFTCHECK(rmw_uros_sync_session(1000));

    // Node in the robot namespace: every topic becomes /robot_<id>/<name>
    static char ns[16];
    snprintf(ns, sizeof(ns), "robot_%u", (unsigned)robot_id);
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "robot_firmware", ns, &support));

    // Publishers (best effort: a lost sample is replaced by the next one)
    rcl_publisher_t ultrasonic_pub, wheels_pub, imu_pub, battery_pub;
    RCCHECK(rclc_publisher_init_best_effort(&ultrasonic_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "ultrasonic"));
    RCCHECK(rclc_publisher_init_best_effort(&wheels_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "wheel_speeds"));
    RCCHECK(rclc_publisher_init_best_effort(&imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu"));
    RCCHECK(rclc_publisher_init_best_effort(&battery_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "battery"));

    // Subscriptions
    rcl_subscription_t cmd_vel_sub, pose_sub;
    static geometry_msgs__msg__Twist cmd_vel_msg;
    static geometry_msgs__msg__Pose2D pose_msg;
    RCCHECK(rclc_subscription_init_default(&cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    RCCHECK(rclc_subscription_init_default(&pose_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), "pose"));

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &pose_sub, &pose_msg, &pose_callback, ON_NEW_DATA));

    // Outgoing messages, with static memory for their arrays and strings
    static float ultrasonic_buf[3], wheels_buf[3];
    static std_msgs__msg__Float32MultiArray ultrasonic_msg, wheels_msg;
    static sensor_msgs__msg__Imu imu_msg;
    static std_msgs__msg__Float32 battery_msg;
    static char imu_frame[24];
    memset(&ultrasonic_msg, 0, sizeof(ultrasonic_msg));
    memset(&wheels_msg, 0, sizeof(wheels_msg));
    memset(&imu_msg, 0, sizeof(imu_msg));
    memset(&battery_msg, 0, sizeof(battery_msg));

    ultrasonic_msg.data.data = ultrasonic_buf;
    ultrasonic_msg.data.size = 3;
    ultrasonic_msg.data.capacity = 3;
    wheels_msg.data.data = wheels_buf;
    wheels_msg.data.size = 3;
    wheels_msg.data.capacity = 3;

    snprintf(imu_frame, sizeof(imu_frame), "robot_%u/imu", (unsigned)robot_id);
    imu_msg.header.frame_id.data = imu_frame;
    imu_msg.header.frame_id.size = strlen(imu_frame);
    imu_msg.header.frame_id.capacity = sizeof(imu_frame);
    imu_msg.orientation_covariance[0] = -1.0;  // ROS convention: orientation not provided

    ESP_LOGI(TAG, "micro-ROS ready, publishing under /%s", ns);

    vec3_t v3;
    imu_data_t imu;
    float voltage;
    uint32_t sent_ultrasonic = 0, sent_wheels = 0, sent_imu = 0, sent_battery = 0;
    TickType_t last_report = xTaskGetTickCount();

    while (1) {
        if (xQueueReceive(ultrasonic_q, &v3, 0) == pdTRUE) {
            memcpy(ultrasonic_buf, v3.v, sizeof(ultrasonic_buf));
            RCSOFTCHECK(rcl_publish(&ultrasonic_pub, &ultrasonic_msg, NULL));
            sent_ultrasonic++;
        }
        if (xQueueReceive(wheels_q, &v3, 0) == pdTRUE) {
            memcpy(wheels_buf, v3.v, sizeof(wheels_buf));
            RCSOFTCHECK(rcl_publish(&wheels_pub, &wheels_msg, NULL));
            sent_wheels++;
        }
        if (xQueueReceive(imu_q, &imu, 0) == pdTRUE) {
            const int64_t now_ns = rmw_uros_epoch_nanos();
            imu_msg.header.stamp.sec = (int32_t)(now_ns / 1000000000LL);
            imu_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);
            imu_msg.linear_acceleration.x = imu.accel[0];
            imu_msg.linear_acceleration.y = imu.accel[1];
            imu_msg.linear_acceleration.z = imu.accel[2];
            imu_msg.angular_velocity.x = imu.gyro[0];
            imu_msg.angular_velocity.y = imu.gyro[1];
            imu_msg.angular_velocity.z = imu.gyro[2];
            RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
            sent_imu++;
        }
        if (xQueueReceive(battery_q, &voltage, 0) == pdTRUE) {
            battery_msg.data = voltage;
            RCSOFTCHECK(rcl_publish(&battery_pub, &battery_msg, NULL));
            sent_battery++;
        }

        // Handle incoming messages (no wait, then yield one tick)
        rclc_executor_spin_some(&executor, 0);
        vTaskDelay(1);  // yield one tick (10 ms) so the sensor tasks can run

        if ((xTaskGetTickCount() - last_report) >= pdMS_TO_TICKS(REPORT_PERIOD_MS)) {
            ESP_LOGI(TAG, "Sent in last 2 s -> ultrasonic: %" PRIu32 ", wheel_speeds: %" PRIu32
                     ", imu: %" PRIu32 ", battery: %" PRIu32,
                     sent_ultrasonic, sent_wheels, sent_imu, sent_battery);
            sent_ultrasonic = sent_wheels = sent_imu = sent_battery = 0;
            last_report = xTaskGetTickCount();
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(sensors_init());
    robot_id = sensors_get_robot_id();
    ESP_LOGI(TAG, "Robot ID %u -> topics under /robot_%u", (unsigned)robot_id, (unsigned)robot_id);

    ultrasonic_q = xQueueCreate(1, sizeof(vec3_t));
    wheels_q = xQueueCreate(1, sizeof(vec3_t));
    imu_q = xQueueCreate(1, sizeof(imu_data_t));
    battery_q = xQueueCreate(1, sizeof(float));

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    // Keep the WiFi radio always awake: power save adds latency to every message
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
#endif

    xTaskCreate(ultrasonic_task, "ultrasonic", 3072, NULL, 4, NULL);
    xTaskCreate(wheels_task, "wheels", 3072, NULL, 4, NULL);
    xTaskCreate(imu_task, "imu", 3072, NULL, 4, NULL);
    xTaskCreate(battery_task, "battery", 3072, NULL, 4, NULL);
    xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}
