#ifndef UTILS_H
#define UTILS_H

#include <geometry_msgs/msg/quaternion.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/semphr.h>
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "esp_heap_caps.h"

void printHeapMemoryUsage() {
    uint32_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    uint32_t totalHeap = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    uint32_t usedHeap = totalHeap - freeHeap;
    float heapUsagePercent = (float)usedHeap / totalHeap * 100.0f;
    
    ESP_LOGI("MEMORY", "Heap Usage: %.1f%% (%lu / %lu bytes)", heapUsagePercent, usedHeap, totalHeap);
}

void printStackMemoryUsage() {
    TaskHandle_t pidEnginesHandle = xTaskGetHandle("pidEngines");
    TaskHandle_t urosTaskHandle = xTaskGetHandle("uros_task");

    if (pidEnginesHandle != NULL) {
        UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(pidEnginesHandle);
        float stackUsagePercent = (1.0f - (float)stackRemaining / 2048.0f) * 100.0f;
        ESP_LOGI("MEMORY", "pidEngines Stack: %.1f%% (%u / 1024 bytes free)", stackUsagePercent, stackRemaining);
    }
    
    if (urosTaskHandle != NULL) {
        UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(urosTaskHandle);
        float stackUsagePercent = (1.0f - (float)stackRemaining / 10096.0f) * 100.0f;
        ESP_LOGI("MEMORY", "uros_task Stack: %.1f%% (%u / 10096 bytes free)", stackUsagePercent, stackRemaining);
    }
}

// Convert Euler angles (roll, pitch, yaw) in radians to geometry_msgs quaternion
geometry_msgs__msg__Quaternion eulerToQuaternion(float roll, float pitch, float yaw) {
    // Compute half angles
    float roll_2 = roll / 2.0f;
    float pitch_2 = pitch / 2.0f;
    float yaw_2 = yaw / 2.0f;
    
    // Precompute sin and cos of half angles
    float sin_roll_2 = sinf(roll_2);
    float cos_roll_2 = cosf(roll_2);
    float sin_pitch_2 = sinf(pitch_2);
    float cos_pitch_2 = cosf(pitch_2);
    float sin_yaw_2 = sinf(yaw_2);
    float cos_yaw_2 = cosf(yaw_2);
    
    // Apply quaternion formula
    geometry_msgs__msg__Quaternion q;
    q.x = sin_roll_2 * cos_pitch_2 * cos_yaw_2 - cos_roll_2 * sin_pitch_2 * sin_yaw_2;
    q.y = cos_roll_2 * sin_pitch_2 * cos_yaw_2 + sin_roll_2 * cos_pitch_2 * sin_yaw_2;
    q.z = cos_roll_2 * cos_pitch_2 * sin_yaw_2 - sin_roll_2 * sin_pitch_2 * cos_yaw_2;
    q.w = cos_roll_2 * cos_pitch_2 * cos_yaw_2 + sin_roll_2 * sin_pitch_2 * sin_yaw_2;
    
    return q;
}

#endif