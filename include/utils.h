#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <geometry_msgs/msg/quaternion.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/semphr.h>
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "esp_heap_caps.h"

/**
 * @brief Print current heap memory usage statistics
 *
 * Logs the current free heap, total heap, and percentage usage
 * to the ESP logging system for monitoring memory usage.
 */
void printHeapMemoryUsage();

/**
 * @brief Print current stack memory usage for key tasks
 *
 * Logs the stack usage statistics for the PID and MicroROS tasks,
 * showing remaining stack space to help detect stack overflow issues.
 */
void printStackMemoryUsage();

/**
 * @brief Convert Euler angles to ROS2 quaternion message
 * @param roll Roll angle in radians
 * @param pitch Pitch angle in radians
 * @param yaw Yaw angle in radians
 * @return ROS2 quaternion message with computed quaternion values
 *
 * Converts Euler angles (roll, pitch, yaw) to a quaternion representation
 * suitable for ROS2 geometry_msgs/Quaternion messages.
 */
geometry_msgs__msg__Quaternion eulerToQuaternion(float roll, float pitch, float yaw);

#endif