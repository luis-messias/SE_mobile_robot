#ifndef ENCODERDRIVER_H
#define ENCODERDRIVER_H

#include "driver/pulse_cnt.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "driver/pulse_cnt.h"

class EncoderDriver{
public:
    EncoderDriver(int portA, int portB, float encoderResolution);
    float getRPM();

private:
    float getRotations();
    int64_t getCount();

    pcnt_unit_handle_t pcnt_unit;
    float lastRotation = 0;
    float lastRPM;
    TickType_t xLastTick;
    float m_encoderResolution;

    int64_t accumulated_count = 0;
    int last_hardware_count = 0;
    SemaphoreHandle_t xSemaphore;
};

#endif