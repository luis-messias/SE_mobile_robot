#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sdkconfig.h"
#include "driver/ledc.h"

extern "C" {
    void app_main(void);
}

class EngineDriver {
public:
    EngineDriver(int gpio) : m_gpio(gpio) {
        ledc_timer_config_t timer_conf = {
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_15_BIT,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = 1000,
            .clk_cfg = LEDC_AUTO_CLK,
        };

        ledc_timer_config(&timer_conf); // Apply timer config

        ledc_channel_config_t ledc_conf = {
            .gpio_num = gpio,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
        };

        ledc_channel_config(&ledc_conf); //  Apply channel config
    }

    void setOutput(float percentage){
        printf("%f %d \n", percentage, (int) (percentage * 32768));
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (int)((1 - percentage) * 32768));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    }

private:
    const int m_gpio;
};

void engineMain(void *args) {

    EngineDriver driver1 = {25};

	while(1) {
        driver1.setOutput(0.10);     
        vTaskDelay(10/portTICK_PERIOD_MS);
	} 
}

void app_main()
{	   
    xTaskCreate(&engineMain,"engineMain",2048,NULL,5,NULL);
}