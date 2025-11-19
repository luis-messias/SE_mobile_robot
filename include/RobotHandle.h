#ifndef ROBOTHANDLE_H
#define ROBOTHANDLE_H

#include <EngineDriver.h>
#include <EncoderDriver.h>
#include <utility>

#define PI 3.14159265359

typedef struct {
    EngineDriver* engineDriver;
    EncoderDriver* encoderDriver;
    PID* pid;
} wheelHandle;

typedef struct {
    float vx;
    float w;
    float x;
    float y;
    float alpha;
} Odometry;

class robotHandle{
public:
    robotHandle(wheelHandle* leftWheel, wheelHandle* rightWheel, float wheelsRadius, float wheelsDistance) : 
        m_leftWheel(leftWheel),
        m_rightWheel(rightWheel),
        m_wheelsRadius(wheelsRadius),
        m_wheelsDistance(wheelsDistance) {
            m_odometry = {0,0,0,0,0};
        }
    
    void setVelocity(float vx, float w){
        ESP_LOGI("ROBOTHANDLE", "Velocity set to vx: %.2f m/s, w: %.2f rad/s", vx, w);
        float vLeft = vx + w * m_wheelsDistance / 2;
        float vRight = vx - w * m_wheelsDistance / 2;
        m_leftWheel->pid->setSetPoint((60 * vLeft) / (m_wheelsRadius * 2 * PI));
        m_rightWheel->pid->setSetPoint((60 * vRight) / (m_wheelsRadius * 2 * PI));
    }

    void updateOdometry(){
        auto [vx, w] = getVelocity();
        float dx = 0;
        float dy = 0;
        float dAlpha = 0;

        TickType_t xCurrentTick = xTaskGetTickCount();
        if(xCurrentTick != xLastUpdateTick){
            float dt = ((float)xCurrentTick - xLastUpdateTick)/configTICK_RATE_HZ;
            float ds = vx * dt;

            dAlpha = w * dt;
            dx = std::cos(m_odometry.alpha) * ds;
            dy = std::sin(m_odometry.alpha) * ds;   

            xLastUpdateTick = xCurrentTick;
        }
        m_odometry = {vx, w, m_odometry.x + dx, m_odometry.y + dy, m_odometry.alpha + dAlpha};
    }

    Odometry getOdometry(){return m_odometry;}

private:
    std::pair<float, float> getVelocity(){
        float vLeft = m_leftWheel->encoderDriver->getRPM() * 2 * PI * m_wheelsRadius / 60;
        float VRight = m_rightWheel->encoderDriver->getRPM() * 2 * PI * m_wheelsRadius/ 60;
        return {(vLeft + VRight) / 2, (vLeft - VRight) / m_wheelsDistance};
    }

    wheelHandle* m_leftWheel;
    wheelHandle* m_rightWheel;
    float m_wheelsRadius;
    float m_wheelsDistance;
    Odometry m_odometry;
    TickType_t xLastUpdateTick;
};

#endif