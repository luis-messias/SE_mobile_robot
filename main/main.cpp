
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <freertos/semphr.h>

// PID timing monitoring
static uint32_t pid_call_count = 0;
static int64_t last_pid_time = 0;

#include <rmw_microros/rmw_microros.h>
#include <uros_network_interfaces.h>

#include <MicroROS.h>
#include <RobotHandle.h>
#include <utils.h>

#include <config.h>


// Watchdog timeout constant
static const TickType_t CMD_VEL_TIMEOUT =
    CMD_VEL_TIMEOUT_MS / portTICK_PERIOD_MS;

// Global MicroROS instance
RobotHandle * robot = new RobotHandle();
MicroROS* microROS = nullptr;

void micro_ros_task(void *arg) {
    ESP_LOGI("MICROROS", "MicroROS task started");
    static int loop_count = 0;

    while (1) {
        loop_count++;
        if (loop_count % 1000 == 0) {
            ESP_LOGI("MICROROS", "ROS task running, loop #%d", loop_count);
        }

        // Handle ROS2 communication
        microROS->spinOnce(100);

        // Check for cmd_vel timeout and reset velocity if needed
        microROS->checkCmdVelTimeout(CMD_VEL_TIMEOUT);

        // Publish
        microROS->publishPose();
        microROS->publishPIDStatus();

        vTaskDelay(MAIN_LOOP_DELAY_MS / portTICK_PERIOD_MS);
    }
}

void pidEnginesTimerCallback(void *args) {
    pid_call_count++;

    // Measure timing every 100 calls
    if (pid_call_count % 100 == 0) {
        int64_t current_time = esp_timer_get_time();
        if (last_pid_time != 0) {
            int64_t interval_us = current_time - last_pid_time;
            float frequency = 1000000.0f / (interval_us / 100.0f); // Hz over 100 calls
            ESP_LOGD("PID", "Frequency: %.1f Hz (target: %.1f Hz), %.1f ms,calls: %lu",
                     frequency, 1000.0f / PID_LOOP_DELAY_MS, interval_us / 1000.0f/100, pid_call_count);
        }
        last_pid_time = current_time;
    }

    robot->updateWheelsPID();
}

extern "C" void app_main() {
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    robot = new RobotHandle();
    microROS = new MicroROS(robot);

    // Initialize MicroROS before creating tasks
    ESP_LOGI("MAIN", "Initializing MicroROS...");
    if (!microROS->initialize()) {
        ESP_LOGE("MAIN", "Failed to initialize MicroROS in main task");
        return;
    }
    ESP_LOGI("MAIN", "MicroROS initialized successfully");

    // Initialize PID control timer
    const esp_timer_create_args_t pid_timer_args = {
        .callback = &pidEnginesTimerCallback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "pid_timer",
        .skip_unhandled_events = true
    };

    esp_timer_handle_t pid_timer;
    ESP_ERROR_CHECK(esp_timer_create(&pid_timer_args, &pid_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_timer, PID_LOOP_DELAY_MS * 1000)); // Convert ms to us
    ESP_LOGI("MAIN", "PID control timer started (%d ms interval)", PID_LOOP_DELAY_MS);

    xTaskCreate(&micro_ros_task, "uros_task", UROS_TASK_STACK_SIZE, NULL,
                TASK_PRIORITY, NULL);

    printf("=====================================================================\n");
    printHeapMemoryUsage();
    printStackMemoryUsage();

    while (1) {
        printf("=====================================================================\n");
        printHeapMemoryUsage();
        printStackMemoryUsage();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}