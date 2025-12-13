#ifndef ENCODERDRIVER_H
#define ENCODERDRIVER_H

#include "driver/pulse_cnt.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "driver/pulse_cnt.h"

/**
 * @brief Quadrature encoder driver using ESP32 pulse counter
 *
 * This class provides RPM measurement for rotary encoders with automatic
 * overflow handling and glitch filtering. It uses the ESP32's hardware
 * pulse counter for accurate, low-latency encoder reading.
 */
class EncoderDriver{
public:
    /**
     * @brief Constructor - initializes pulse counter for quadrature encoder
     * @param portA GPIO pin for encoder channel A
     * @param portB GPIO pin for encoder channel B
     * @param encoderResolution Pulses per revolution of the encoder
     */
    EncoderDriver(int portA, int portB, float encoderResolution);

    /**
     * @brief Get current motor speed in revolutions per minute
     * @return Motor speed in RPM
     *
     * This method calculates RPM based on encoder pulses over time.
     * The calculation accounts for encoder resolution and timing.
     */
    float getRPM();

private:
    float getRotations();
    int64_t getCount();

    pcnt_unit_handle_t pcnt_unit;
    float lastRotation = 0;
    float lastRPM;
    TickType_t xLastTick;
    float m_encoderResolution;

    int64_t accumulated_count = 0;
    int last_hardware_count = 0;
    SemaphoreHandle_t xSemaphore;
};

#endif