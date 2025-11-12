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
    EngineDriver engineDriver = {{27, 14}, 
                                LEDC_TIMER_0, 
                                std::pair{LEDC_CHANNEL_0, LEDC_CHANNEL_1}};
    engineDriver.setOutput(1); 

    EncoderDriver encoderDriver(21, 22, 823.1);
    
    xTaskCreate(&task, "task", 2048, NULL, 5, NULL);
    while(1){
        printf("%f\n", encoderDriver.getRPM());
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}