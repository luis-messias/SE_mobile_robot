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

extern "C" {
    void app_main(void);
}

void task(void *args) {
	while(1){
        vTaskDelay(1000/portTICK_PERIOD_MS);
	} 
}

void app_main()
{	   
    EngineDriver engineDriver = {{14, 27}, 
                                LEDC_TIMER_0, 
                                std::pair{LEDC_CHANNEL_0, LEDC_CHANNEL_1}};
    engineDriver.setOutput(0); 

    EncoderDriver encoderDriver(21, 22, 823.1);
    xTaskCreate(&task, "task", 2048, NULL, 5, NULL);

    float setPoint = 60;
    float rpm;
    float erro = 0;
    float sumErro = 0;
    float lastErro = 0;
    float k = 0.003;
    float kd = 0.003;
    float ki = 0.003;
    float out;

    vTaskDelay(3000/portTICK_PERIOD_MS);
    while(1){
        rpm = encoderDriver.getRPM();
        erro = setPoint - rpm;
        sumErro += erro;

        out = k * erro + ki * sumErro + kd * (erro - lastErro);
        printf("%f %f %f %f %f\n", rpm, erro, sumErro, erro - lastErro, out);
        engineDriver.setOutput(out); 

        lastErro = erro;
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}