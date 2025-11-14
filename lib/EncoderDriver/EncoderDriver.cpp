#include<EncoderDriver.h>

#define BDC_ENCODER_PCNT_HIGH_LIMIT   1000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -1000
#define PI 3.14159265359

static SemaphoreHandle_t xSemaphore = xSemaphoreCreateMutex();;

EncoderDriver::EncoderDriver(int portA, int portB, float encoderResolution){
    m_encoderResolution = encoderResolution;

    unit_config = {
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .flags = {1}, // enable counter accumulation
    };
    pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = portA,
        .level_gpio_num = portB,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = portB,
        .level_gpio_num = portA,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

int EncoderDriver::getCount(){
    int cur_pulse_count;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    return cur_pulse_count;
}

float EncoderDriver::getRotations(){
    return ((float)getCount())/m_encoderResolution;
}

float EncoderDriver::getRPM(){
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
        float currentRotation = getRotations();
        TickType_t xCurrentTick = xTaskGetTickCount();
        if(xCurrentTick != xLastTick){
            lastRPM = 60 * configTICK_RATE_HZ * (currentRotation - lastRotation)/(xCurrentTick - xLastTick);
            lastRotation = currentRotation;
        }
        xLastTick = xCurrentTick;

        xSemaphoreGive(xSemaphore);
    }
    return lastRPM;
}

// float EncoderDriver::getRadSec(){
//     float currentRotation = getRotations();
//     TickType_t xCurrentTick = xTaskGetTickCount();

//     float radSec = PI * configTICK_RATE_HZ * (currentRotation - lastRotation)/(xCurrentTick - xLastTick);

//     lastRotation = currentRotation;
//     xLastTick = xCurrentTick;

//     return radSec;
// }
