#include "RobotHandle.h"
#include <cmath>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <config.h>

RobotHandle::RobotHandle() {
        m_wheelsRadius = WHEEL_RADIUS;
        m_wheelsDistance = WHEEL_DISTANCE;
        engineDriverLeft =
            new EngineDriver({12, 4}, LEDC_TIMER_0,
                std::pair{LEDC_CHANNEL_0, LEDC_CHANNEL_1}, ENGINE_DEAD_ZONE);
        engineDriverRight =
            new EngineDriver({25, 26}, LEDC_TIMER_0,
                std::pair{LEDC_CHANNEL_2, LEDC_CHANNEL_3}, ENGINE_DEAD_ZONE);
        encoderDriverLeft = new EncoderDriver(21, 22, ENCODER_RESOLUTION);
        encoderDriverRight = new EncoderDriver(13, 32, ENCODER_RESOLUTION);
        pidLeft = new PID(PID_KP, PID_KI, PID_KD);
        pidRight = new PID(PID_KP, PID_KI, PID_KD);
        m_odometry = {0,0,0,0,0};
        xLastUpdateTick = xTaskGetTickCount();
    }

void RobotHandle::setVelocity(float vx, float w){
    if (fabs(vx) > 2.0f || fabs(w) > 10.0f) {
        ESP_LOGW("ROBOTHANDLE", "Velocity command exceeds limits: vx=%.2f m/s, w=%.2f rad/s", vx, w);
    }

    ESP_LOGI("ROBOTHANDLE", "Velocity set to vx: %.2f m/s, w: %.2f rad/s", vx, w);
    float vLeft = vx + w * m_wheelsDistance / 2;
    float vRight = vx - w * m_wheelsDistance / 2;

    // Convert linear velocity (m/s) to RPM for PID controllers
    m_rmpLeftSetPoint = (60 * vLeft) / (m_wheelsRadius * 2 * PI);
    m_rpmRightSetPoint = (60 * vRight) / (m_wheelsRadius * 2 * PI);

    pidLeft->setSetPoint(m_rmpLeftSetPoint);
    pidRight->setSetPoint(m_rpmRightSetPoint);
}

void RobotHandle::setPIDGains(float k, float ki, float kd){
    ESP_LOGI("ROBOTHANDLE", "Setting PID gains: k=%.4f, ki=%.4f, kd=%.4f", k, ki, kd);
    pidLeft->setGains(k, ki, kd);
    pidRight->setGains(k, ki, kd);
}

void RobotHandle::stop(){
    m_rmpLeftSetPoint = 0;
    m_rpmRightSetPoint = 0;

    pidLeft->setSetPoint(m_rmpLeftSetPoint);
    pidRight->setSetPoint(m_rpmRightSetPoint);
}

void RobotHandle::updateOdometry(){
    auto [vx, w] = getVelocity();
    float dx = 0;
    float dy = 0;
    float dAlpha = 0;

    TickType_t xCurrentTick = xTaskGetTickCount();

    // Always update velocity components, only update position if time has passed
    if(xCurrentTick > xLastUpdateTick){
        float dt = ((float)xCurrentTick - xLastUpdateTick) / configTICK_RATE_HZ;

        // Prevent large integration steps that could cause instability
        if(dt > 0.1f) {  // Max 100ms integration step
            dt = 0.1f;
            ESP_LOGW("ODOMETRY", "Large dt detected (%.3f s), clamping to prevent instability", dt);
        }

        float ds = vx * dt;
        dAlpha = w * dt;
        dx = std::cos(m_odometry.alpha) * ds;
        dy = std::sin(m_odometry.alpha) * ds;

        xLastUpdateTick = xCurrentTick;
    }
    m_odometry = {vx, w, m_odometry.x + dx, m_odometry.y + dy, m_odometry.alpha + dAlpha};
}

Odometry RobotHandle::getOdometry(){
    return m_odometry;
}

std::pair<float, float> RobotHandle::getVelocity(){
    float vLeft = m_rpmLeft * 2 * PI * m_wheelsRadius / 60;
    float vRight = m_rpmRight * 2 * PI * m_wheelsRadius/ 60;
    return {(vLeft + vRight) / 2, (vLeft - vRight) / m_wheelsDistance};
}

void RobotHandle::updateWheelsPID(){
    m_rpmLeft = encoderDriverLeft->getRPM();
    m_rpmRight = encoderDriverRight->getRPM();

    m_outPIDLeft = pidLeft->getOut(m_rpmLeft, PID_LOOP_DELAY_MS / 1000.0f);
    m_outPIDRight = pidRight->getOut(m_rpmRight, PID_LOOP_DELAY_MS / 1000.0f);

    engineDriverLeft->setOutput(m_outPIDLeft);
    engineDriverRight->setOutput(m_outPIDRight);

    updateOdometry();
}

pidStatus RobotHandle::getPIDStatus(){
    return {m_rpmLeft,
        m_rpmRight,
        m_rmpLeftSetPoint,
        m_rpmRightSetPoint,
        m_outPIDLeft,
        m_outPIDRight};
}