#include <EncoderDriver.h>
#include <esp_log.h>

#define BDC_ENCODER_PCNT_HIGH_LIMIT 1000
#define BDC_ENCODER_PCNT_LOW_LIMIT -1000
#define PI 3.14159265359

EncoderDriver::EncoderDriver(int portA, int portB, float encoderResolution) {
    // Input validation
    if (portA < 0 || portB < 0) {
        ESP_LOGE("ENCODERDRIVER", "Invalid GPIO pins: portA=%d, portB=%d", portA, portB);
        return;
    }
    if (encoderResolution <= 0.0f) {
        ESP_LOGE("ENCODERDRIVER", "Invalid encoder resolution: %.2f (must be > 0)", encoderResolution);
        m_encoderResolution = 1.0f; // Set safe default
    } else {
        m_encoderResolution = encoderResolution;
    }

    xSemaphore = xSemaphoreCreateMutex();
    if (xSemaphore == NULL) {
        ESP_LOGE("ENCODERDRIVER", "Failed to create mutex semaphore!");
        return;
    }

    pcnt_unit_config_t unit_config = {
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .intr_priority = 3,
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
        .flags = {0, 0,0, 0, 0},
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = portB,
        .level_gpio_num = portA,
        .flags = {0, 0,0, 0, 0},
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

int64_t EncoderDriver::getCount() {
    int cur_pulse_count;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);

    // Calculate delta and handle overflow/underflow
    int delta = cur_pulse_count - last_hardware_count;

    // Check for wrap-around
    // If delta is very large positive (e.g. -990 -> 10), it's actually a small
    // negative change If delta is very large negative (e.g. 10 -> -990), it's
    // actually a small positive change Threshold is half the range (1000 -
    // (-1000) = 2000, so 1000)
    const int range = BDC_ENCODER_PCNT_HIGH_LIMIT - BDC_ENCODER_PCNT_LOW_LIMIT;

    if (delta > range / 2) {
        delta -= range;
    } else if (delta < -range / 2) {
        delta += range;
    }

    accumulated_count += delta;
    last_hardware_count = cur_pulse_count;

    return accumulated_count;
}

float EncoderDriver::getRotations() {
    return ((float)getCount()) / m_encoderResolution;
}

float EncoderDriver::getRPM() {
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