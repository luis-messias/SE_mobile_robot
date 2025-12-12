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

void printHeapMemoryUsage();
void printStackMemoryUsage();

// Convert Euler angles (roll, pitch, yaw) in radians to geometry_msgs quaternion
geometry_msgs__msg__Quaternion eulerToQuaternion(float roll, float pitch, float yaw);

#endif