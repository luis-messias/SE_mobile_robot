#ifndef ROBOTHANDLE_H
#define ROBOTHANDLE_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <EngineDriver.h>
#include <EncoderDriver.h>
#include <PID.h>
#include <utility>

#define PI 3.14159265359

typedef struct {
    float vx;
    float w;
    float x;
    float y;
    float alpha;
} Odometry;

typedef struct {
    float rpmLeft;
    float rpmRight;
    float rmpLeftSetPoint;
    float rpmRightSetPoint;
    float outPIDLeft;
    float outPIDRight;
} pidStatus;


class RobotHandle{
public:
    RobotHandle();

    void setVelocity(float vx, float w);
    void setPIDGains(float k, float ki, float kd);
    void stop();
    
    Odometry getOdometry();

    void updateWheelsPID();
    pidStatus getPIDStatus();

private:
    std::pair<float, float> getVelocity();
    void updateOdometry();

    EngineDriver* engineDriverLeft;
    EngineDriver* engineDriverRight;
    EncoderDriver* encoderDriverLeft;
    EncoderDriver* encoderDriverRight;
    PID* pidLeft;
    PID* pidRight;

    float m_wheelsRadius;
    float m_wheelsDistance;

    Odometry m_odometry;
    TickType_t xLastUpdateTick;

    float m_rpmLeft;
    float m_rpmRight;
    float m_rmpLeftSetPoint;
    float m_rpmRightSetPoint;
    float m_outPIDLeft;
    float m_outPIDRight;
    SemaphoreHandle_t xSemaphore;
};

#endif