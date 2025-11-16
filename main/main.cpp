#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <freertos/semphr.h>
#include "sdkconfig.h"
#include "driver/ledc.h"
#include <utility>
#include <math.h>

#include <EngineDriver.h>
#include <EncoderDriver.h>
#include <PID.h>

#include <rclc/executor.h>

#define PI 3.14159265359

extern "C" {
    void app_main(void);
}

typedef struct {
    EngineDriver* engineDriver;
    EncoderDriver* encoderDriver;
    PID* pid;
} wheelHandle;

void pidEngineTask(void *args) {
    wheelHandle* pidParams = (wheelHandle*)args;

    EngineDriver* engineDriver = pidParams->engineDriver;
    EncoderDriver* encoderDriver = pidParams->encoderDriver;
    PID* pid = pidParams->pid;

    float out;
    float rpm;

    while(1){
        rpm = encoderDriver->getRPM();
        out = pid->getOut(rpm, 0.1);
        engineDriver->setOutput(out); 
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

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
        float vLeft = vx + w * m_wheelsDistance / 2;
        float vRight = vx - w * m_wheelsDistance / 2;
        m_leftWheel->pid->setSetPoint((60 * vLeft) / (m_wheelsRadius * 2 * PI));
        m_rightWheel->pid->setSetPoint((60 * vRight) / (m_wheelsRadius * 2 * PI));
    }

    Odometry updateOdometry(){
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
        return m_odometry;
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


void updateOdometry(void *args) {
    auto robot = (robotHandle *) args;

    while(1){
        robot->updateOdometry();
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}

void app_main()
{	   
    auto engineDriverLeft = EngineDriver({12, 27}, 
                                LEDC_TIMER_0, 
                                std::pair{LEDC_CHANNEL_0, LEDC_CHANNEL_1});
    auto encoderDriverLeft = EncoderDriver(21, 22, 823.1);
    auto pidLeft = PID(0.003, 0.03, 0.0003);
    auto wheelLeft = wheelHandle(&engineDriverLeft, &encoderDriverLeft, &pidLeft);

    auto engineDriverRight = EngineDriver({26, 25}, 
                                LEDC_TIMER_0, 
                                std::pair{LEDC_CHANNEL_2, LEDC_CHANNEL_3});
    auto encoderDriverRight = EncoderDriver(32, 13, 823.1);
    auto pidRight = PID(0.003, 0.03, 0.0003);
    auto wheelRight = wheelHandle(&engineDriverRight, &encoderDriverRight, &pidRight);
    
    auto robot = robotHandle(&wheelLeft, &wheelRight, 0.130/2, 0.020);

    robot.setVelocity(0, 20);
    // pidLeft.setSetPoint(30);
    // pidRight.setSetPoint(-30);

    vTaskDelay(3000/portTICK_PERIOD_MS);
    xTaskCreate(&pidEngineTask, "pidEngineLeft", 2048, &wheelLeft, 5, NULL);
    xTaskCreate(&pidEngineTask, "pidEngineRight", 2048, &wheelRight, 5, NULL);
    xTaskCreate(&updateOdometry, "odometryUpdate", 2048, &robot, 5, NULL);

    while(1){
        printf("RPM: %f %f\n", encoderDriverLeft.getRPM(),encoderDriverRight.getRPM());

        auto [vx, w, x, y, alpha] = robot.getOdometry();
        printf("vx: %f w: %f\n", vx, w);
        printf("x: %f y: %f alpha: %f\n", x, y, alpha);

        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}