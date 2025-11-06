#include<EngineDriver.h>

void EngineDriver::setOutput(float percentage){
    xSemaphoreTake(xSemaphore, portMAX_DELAY);

    if(percentage > 1.0){
        xSemaphoreGive(xSemaphore);
        return;
    }
    if(percentage < -1.0){
        xSemaphoreGive(xSemaphore);
        return;
    }
    if(percentage == 0.0){
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.first, 32768);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.second, 32768);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.first);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        xSemaphoreGive(xSemaphore);
        return;
    }
    if(percentage > 0){
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.first, (int)((1 - percentage) * 32768));
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.second, 32768);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.first);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.second);
        xSemaphoreGive(xSemaphore);
        return;
    }
    if(percentage < 0){
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.first, 32768);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.second, (int)((1 + percentage) * 32768));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.first);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.second);
        xSemaphoreGive(xSemaphore);
        return;
    }
}