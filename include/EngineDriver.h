#ifndef ENGINEDRIVER_H
#define ENGINEDRIVER_H

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include <freertos/semphr.h>
#include <esp_log.h>
#include <utility>

/**
 * @brief Motor driver class for H-bridge motor control using ESP32 LEDC PWM
 *
 * This class provides PWM-based motor control with dead zone compensation
 * for differential drive robot motors. It handles bidirectional control
 * and prevents motor chatter at low speeds.
 */
class EngineDriver {
public:
    /**
     * @brief Constructor - initializes PWM channels and timer for motor control
     * @param gpios Pair of GPIO pins (INA, INB) for H-bridge control
     * @param timer LEDC timer to use for PWM generation
     * @param channels Pair of LEDC channels (forward, reverse) for PWM output
     * @param deadZone Dead zone percentage to prevent motor chatter (0.0-1.0)
     */
    EngineDriver(std::pair<int, int> gpios, ledc_timer_t timer,
                 std::pair<ledc_channel_t, ledc_channel_t> channels,
                 float deadZone);

    /**
     * @brief Set motor output as percentage of maximum speed
     * @param percentage Motor speed (-1.0 to 1.0, negative = reverse)
     *
     * Values outside [-1.0, 1.0] are clamped. Dead zone compensation
     * is automatically applied to prevent motor chatter.
     */
    void setOutput(float percentage);

private:
    std::pair<ledc_channel_t, ledc_channel_t>
        m_channels;
    float m_deadZone;
    SemaphoreHandle_t xSemaphore;
};

#endif