#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <freertos/semphr.h>
#include "sdkconfig.h"
#include "driver/ledc.h"
#include <utility>

#include <EngineDriver.h>
#include <EncoderDriver.h>
#include <PID.h>

extern "C" {
    void app_main(void);
}

typedef struct {
    EngineDriver* engineDriver;
    EncoderDriver* encoderDriver;
    PID* pid;
} pidTaskParam;

void pidEngineTask(void *args) {
    pidTaskParam* pidParams = (pidTaskParam*)args;

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

void app_main()
{	   
    EngineDriver engineDriverLeft = {{12, 27}, 
                                LEDC_TIMER_0, 
                                std::pair{LEDC_CHANNEL_0, LEDC_CHANNEL_1}};
    EncoderDriver encoderDriverLeft(21, 22, 823.1);
    PID pidLeft = PID(0.003, 0.03, 0.0003);
    pidTaskParam pidParamsLeft = {&engineDriverLeft, &encoderDriverLeft, &pidLeft};

    EngineDriver engineDriverRight = {{26, 25}, 
                                LEDC_TIMER_0, 
                                std::pair{LEDC_CHANNEL_2, LEDC_CHANNEL_3}};
    EncoderDriver encoderDriverRight(32, 13, 823.1);
    PID pidRight = PID(0.003, 0.03, 0.0003);
    pidTaskParam pidParamsRight = {&engineDriverRight, &encoderDriverRight, &pidRight};

    pidLeft.setSetPoint(30);
    pidRight.setSetPoint(60);

    vTaskDelay(3000/portTICK_PERIOD_MS);
    xTaskCreate(&pidEngineTask, "pidEngineLeft", 2048, &pidParamsLeft, 5, NULL);
    xTaskCreate(&pidEngineTask, "pidEngineRight", 2048, &pidParamsRight, 5, NULL);

    while(1){
        printf("RPM: %f %f\n", encoderDriverLeft.getRPM(),encoderDriverRight.getRPM());
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}