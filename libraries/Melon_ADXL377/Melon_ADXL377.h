/***************************************************************************
This is a library for the ADXL377 200g accelerometer

These sensors are polled over analog

Written by Paul Guenette

MIT License
***************************************************************************/
#pragma once
#include "Arduino.h"

#define X_AXIS_PIN (A0)
#define Y_AXIS_PIN (A1)
#define Z_AXIS_PIN (A2)

class Melon_ADXL377
{
public:
    int16_t getXAxis();
    int16_t getYAxis();
    int16_t getZAxis();
};