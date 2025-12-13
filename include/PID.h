#ifndef PID_H
#define PID_H

#include "freertos/FreeRTOS.h"
#include <freertos/semphr.h>

/**
 * @brief PID (Proportional-Integral-Derivative) controller implementation
 *
 * Standard PID controller with anti-windup protection and configurable gains.
 * Integrator windup is prevented by clamping the integral term.
 */
class PID {
public:
    /**
     * @brief Constructor
     * @param k Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    PID(float k, float ki, float kd);

    /**
     * @brief Calculate PID output
     * @param y Current process value (measurement)
     * @param dt Time step since last calculation
     * @return Control output
     */
    float getOut(float y, float dt);

    /**
     * @brief Set the desired setpoint
     * @param ref Target value for the process
     */
    void setSetPoint(float ref);

    /**
     * @brief Update PID gains
     * @param k New proportional gain
     * @param ki New integral gain
     * @param kd New derivative gain
     */
    void setGains(float k, float ki, float kd);

private:
    float m_k, m_ki, m_kd;
    float m_setPoint;
    float m_intError;
    float m_lastError;
    SemaphoreHandle_t xSemaphore;
};

#endif