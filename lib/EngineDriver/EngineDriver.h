#ifndef ENGINEDRIVER_H
#define ENGINEDRIVER_H

#include "freertos/FreeRTOS.h"
#include <freertos/semphr.h>
#include "driver/ledc.h"
#include <utility>

class EngineDriver {
public:
    EngineDriver(std::pair<int, int> gpios,
                 ledc_timer_t timer,
                 std::pair<ledc_channel_t, ledc_channel_t> channels) : m_channels(channels) {

        xSemaphore = xSemaphoreCreateMutex();
        xSemaphoreTake(xSemaphore, portMAX_DELAY);

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

        xSemaphoreGive(xSemaphore);
    }
    void setOutput(float percentage); 

private:
    std::pair<ledc_channel_t, ledc_channel_t> m_channels;
    SemaphoreHandle_t xSemaphore;
};

#endif