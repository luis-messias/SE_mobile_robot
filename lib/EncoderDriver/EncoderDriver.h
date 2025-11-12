#ifndef ENCODERDRIVER_H
#define ENCODERDRIVER_H

#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"

class EncoderDriver{
public:
    EncoderDriver(int portA, int portB);
    int getCount();
    float getRotations();
    float getRPM();
    float getRadSec();

private:
    pcnt_unit_config_t unit_config;
    pcnt_unit_handle_t pcnt_unit;
    float lastRotation = 0;
    TickType_t xLastTick;
};


#endif