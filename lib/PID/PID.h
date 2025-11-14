#ifndef PID_H
#define PID_H

#include <freertos/semphr.h>

class PID{
public:
    PID(float k, float ki, float kd) : m_k(k), m_ki(ki), m_kd(kd){
        xSemaphore = xSemaphoreCreateMutex();
    }
    float getOut(float y, float dt){
        float out = 0;
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            float erro = m_setPoint - y;
            m_intErro += erro * dt;
            out = m_k * erro + m_ki * m_intErro + m_kd * (erro - m_lastErro)/dt;
            m_lastErro = erro;
            xSemaphoreGive(xSemaphore);
        }
        return out;
    }
    void setSetPoint(float ref){
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            m_setPoint = ref;
            xSemaphoreGive(xSemaphore);
        }
    }

private:
    float m_k, m_ki, m_kd;
    float m_setPoint;
    float m_intErro;
    float m_lastErro;
    SemaphoreHandle_t xSemaphore;
};

#endif