#ifndef ENCODERDRIVER_H
#define ENCODERDRIVER_H

#include "freertos/FreeRTOS.h"
#include "driver/pulse_cnt.h"

class EncoderDriver{
public:
    EncoderDriver(int portA, int portB, float encoderResolution);
    void updateRPM();
    float getRPM();

private:
    float getRotations();
    int getCount();

    pcnt_unit_handle_t pcnt_unit;
    float lastRotation = 0;
    float lastRPM;
    TickType_t xLastTick;
    float m_encoderResolution;
};


#endif