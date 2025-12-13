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
    float rpmLeftSetPoint;
    float rpmRightSetPoint;
    float outPIDLeft;
    float outPIDRight;
} PidStatus;


/**
 * @brief Main robot control class managing differential drive kinematics,
 *        PID control, and hardware interfaces.
 *
 * This class provides high-level control of a differential drive robot,
 * handling velocity commands, odometry tracking, and motor control through
 * PID controllers.
 */
class RobotHandle{
public:
    /**
     * @brief Constructor - initializes all hardware interfaces and PID controllers
     */
    RobotHandle();

    /**
     * @brief Set robot velocity commands
     * @param vx Linear velocity in m/s (forward/backward)
     * @param w Angular velocity in rad/s (rotation)
     *
     * Velocities are clamped to safe limits defined in config.h
     */
    void setVelocity(float vx, float w);

    /**
     * @brief Update PID controller gains for both wheels
     * @param k Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setPIDGains(float k, float ki, float kd);

    /**
     * @brief Stop - sets all velocities to zero
     */
    void stop();

    /**
     * @brief Get current odometry estimate
     * @return Odometry struct with position, velocity, and orientation
     */
    Odometry getOdometry();

    /**
     * @brief Update PID controllers and apply motor commands
     *
     * This method should be called at regular intervals (defined by PID_LOOP_DELAY_MS)
     * to maintain stable motor control.
     */
    void updateWheelsPID();

    /**
     * @brief Get current PID status for monitoring and debugging
     * @return PidStatus struct with RPM measurements, setpoints, and PID outputs
     */
    PidStatus getPIDStatus();

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
    float m_rpmLeftSetPoint;
    float m_rpmRightSetPoint;
    float m_outPIDLeft;
    float m_outPIDRight;
    SemaphoreHandle_t xSemaphore;
};

#endif