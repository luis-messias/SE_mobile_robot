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
                std::pair<ledc_channel_t, ledc_channel_t> channels);
    void setOutput(float percentage); 

private:
    std::pair<ledc_channel_t, ledc_channel_t> m_channels;
    SemaphoreHandle_t xSemaphore;
};

#endif