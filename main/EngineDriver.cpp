#include<EngineDriver.h>

static SemaphoreHandle_t xSemaphore = xSemaphoreCreateMutex();;

EngineDriver::EngineDriver(std::pair<int, int> gpios,
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
        .duty = 0,
    };

    ledc_channel_config_t motorConfB = {
        .gpio_num = gpios.second,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = m_channels.second,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = 0,
    };

    ledc_timer_config(&timer_conf);
    ledc_channel_config(&motorConfA);
    ledc_channel_config(&motorConfB);
}

void EngineDriver::setOutput(float percentage){
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
        if(percentage > 1.0){
            percentage = 1;
        }
        if(percentage < -1.0){
            percentage = -1;
        }
        
        if(percentage == 0.0){
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.first, 0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.second, 0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.first);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
            xSemaphoreGive(xSemaphore);
            return;
        }
        if(percentage > 0){
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.first, (int)(percentage * 32768));
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.second, 0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.first);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.second);
            xSemaphoreGive(xSemaphore);
            return;
        }
        if(percentage < 0){
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.first, 0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.second, (int)(-percentage * 32768));
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.first);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.second);
            xSemaphoreGive(xSemaphore);
            return;
        }
    }
}