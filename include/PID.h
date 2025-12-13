#ifndef PID_H
#define PID_H

#include "freertos/FreeRTOS.h"
#include <freertos/semphr.h>

class PID {
public:

    PID(float k, float ki, float kd);

    float getOut(float y, float dt);
    void setSetPoint(float ref);
    void setGains(float k, float ki, float kd);

private:
    float m_k, m_ki, m_kd;
    float m_setPoint;
    float m_intError;
    float m_lastError;
    SemaphoreHandle_t xSemaphore;
};

#endif