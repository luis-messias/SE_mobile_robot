#ifndef CONFIG_H
#define CONFIG_H

#include <freertos/FreeRTOS.h>

// Robot configuration constants
constexpr float ENGINE_DEAD_ZONE = 0.05f;
constexpr float ENCODER_RESOLUTION = 823.1f;
constexpr float WHEEL_RADIUS = 0.130f / 2.0f; // 130mm diameter / 2
constexpr float WHEEL_DISTANCE = 0.190f;      // 190mm between wheels

// PID controller gains
constexpr float PID_KP = 0.003f;
constexpr float PID_KI = 0.03f;
constexpr float PID_KD = 0.0003f;
constexpr float PID_MAX_INT_ERROR = 1000.0f; // Anti-windup limit

// Timing constants (in milliseconds)
constexpr TickType_t CMD_VEL_TIMEOUT_MS = 2000;
constexpr int EXECUTOR_TIMEOUT_MS = 1000;
constexpr int MAIN_LOOP_DELAY_MS = 200;
constexpr int PID_LOOP_DELAY_MS = 50;

// Task configuration
constexpr uint32_t PID_ENGINES_STACK_SIZE = 2048;
constexpr uint32_t UROS_TASK_STACK_SIZE = 10096;
constexpr UBaseType_t TASK_PRIORITY = 5;

#endif // CONFIG_H
