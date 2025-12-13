#include "PID.h"
#include <freertos/semphr.h>
#include <config.h>
#include <esp_log.h>

PID::PID(float k, float ki, float kd) : m_k(k), m_ki(ki), m_kd(kd) {
    xSemaphore = xSemaphoreCreateMutex();
    if (xSemaphore == NULL) {
        ESP_LOGE("PID", "Failed to create mutex semaphore!");
    }
}

float PID::getOut(float y, float dt) {
    float out = 0;
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
        float erro = m_setPoint - y;
        m_intError += erro * dt;

        // Anti-windup
        if (m_intError > PID_MAX_INT_ERROR) {
            m_intError = PID_MAX_INT_ERROR;
        } else if (m_intError < -PID_MAX_INT_ERROR) {
            m_intError = -PID_MAX_INT_ERROR;
        }

        // Prevent division by zero in derivative term
        float derivative = (dt > 0.0f) ? m_kd * (erro - m_lastError) / dt : 0.0f;
        out = m_k * erro + m_ki * m_intError + derivative;
        m_lastError = erro;
        xSemaphoreGive(xSemaphore);
    }
    return out;
}

void PID::setSetPoint(float ref) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
        m_setPoint = ref;
        xSemaphoreGive(xSemaphore);
    }
}

void PID::setGains(float k, float ki, float kd) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
        m_k = k;
        m_ki = ki;
        m_kd = kd;
        xSemaphoreGive(xSemaphore);
    }
}