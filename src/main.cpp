#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sdkconfig.h"
#include "driver/ledc.h"
#include <utility>

extern "C" {
    void app_main(void);
}

class EngineDriver {
public:
    EngineDriver(std::pair<int, int> gpios,
                 ledc_timer_t timer,
                 std::pair<ledc_channel_t, ledc_channel_t> channels) : m_channels(channels) {

        ledc_timer_config_t timer_conf = {
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_15_BIT,
            .timer_num = timer,
            .freq_hz = 1000,
            .clk_cfg = LEDC_AUTO_CLK,
        };

        ledc_channel_config_t motorConfA = {
            .gpio_num = gpios.first,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = m_channels.first,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = timer,
            .duty = 32768,
        };

        ledc_channel_config_t motorConfB = {
            .gpio_num = gpios.second,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = m_channels.second,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = timer,
            .duty = 32768,
        };

        ledc_timer_config(&timer_conf);
        ledc_channel_config(&motorConfA);
        ledc_channel_config(&motorConfB);
    }

    void setOutput(float percentage){
        if(percentage > 1.0){
            return;
        }
        if(percentage < -1.0){
            return;
        }
        if(percentage == 0.0){
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.first, 32768);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.second, 32768);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.first);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
            return;
        }
        if(percentage > 0){
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.first, (int)((1 - percentage) * 32768));
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.second, 32768);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.first);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.second);
            return;
        }
        if(percentage < 0){
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.first, 32768);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.second, (int)((1 + percentage) * 32768));
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.first);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.second);
            return;
        }
    }

private:
    std::pair<ledc_channel_t, ledc_channel_t> m_channels;
};

void engineMain(void *args) {

    EngineDriver engineDriver = {{25, 26}, LEDC_TIMER_0, std::pair{LEDC_CHANNEL_0, LEDC_CHANNEL_1}};
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