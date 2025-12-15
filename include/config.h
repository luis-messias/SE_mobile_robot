#ifndef CONFIG_H
#define CONFIG_H

#include <freertos/FreeRTOS.h>

// Robot configuration constants
constexpr float ENGINE_DEAD_ZONE = 0.00f;
constexpr float ENCODER_RESOLUTION = 823.1f;
constexpr float WHEEL_RADIUS = 0.118f / 2.0f; // 118mm diameter / 2
constexpr float WHEEL_DISTANCE = 0.189f;      // 189mm between wheels

// PID controller gains
constexpr float PID_KP = 0.01f;
constexpr float PID_KI = 0.08f;
constexpr float PID_KD = 0.0001;
constexpr float PID_MAX_INT_ERROR = 5.0f; // Anti-windup limit

// Safety limits
constexpr float MAX_LINEAR_VELOCITY = 2.0f;    // m/s
constexpr float MAX_ANGULAR_VELOCITY = 10.0f;  // rad/s

// Timing constants (in milliseconds)
constexpr TickType_t CMD_VEL_TIMEOUT_MS = 2000;
constexpr int EXECUTOR_TIMEOUT_MS = 1000;
constexpr int MAIN_LOOP_DELAY_MS = 50;
constexpr int PID_LOOP_DELAY_MS = 10;

// Task configuration
constexpr uint32_t PID_ENGINES_STACK_SIZE = 2048;
constexpr uint32_t UROS_TASK_STACK_SIZE = 10096;
constexpr UBaseType_t TASK_PRIORITY = 20;

#endif // CONFIG_H
