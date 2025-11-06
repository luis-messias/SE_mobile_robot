#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <freertos/semphr.h>
#include "sdkconfig.h"
#include "driver/ledc.h"
#include <utility>

#include<EngineDriver.h>

extern "C" {
    void app_main(void);
}


void engineMain(void *args) {

    EngineDriver engineDriver = {{25, 26}, 
                                LEDC_TIMER_0, 
                                std::pair{LEDC_CHANNEL_0, LEDC_CHANNEL_1}};

    float p = -1;
    engineDriver.setOutput(p); 

	while(1){
        p += 0.1;
        if(p > 1){
            p = -1;
        }
        printf("%f \n", p);
        engineDriver.setOutput(p); 
        vTaskDelay(1000/portTICK_PERIOD_MS);
	} 
}

void app_main()
{	   
    xTaskCreate(&engineMain,"engineMain",2048,NULL,5,NULL);
}