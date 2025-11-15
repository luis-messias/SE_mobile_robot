#ifndef ENCODERDRIVER_H
#define ENCODERDRIVER_H

#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"

class EncoderDriver{
public:
    EncoderDriver(int portA, int portB, float encoderResolution);
    float getRPM();

private:
    float getRotations();
    int getCount();

    pcnt_unit_config_t unit_config;
    pcnt_unit_handle_t pcnt_unit;
    float lastRotation = 0;
    float lastRPM;
    TickType_t xLastTick;
    float m_encoderResolution;
};


#endif