// Salus_Baro.h

#pragma once
#define SALUS_BARO_ADDRESS 0x76

#include "Salus_Common.h"
#include <i2c_t3.h>
#include <Melon_MS5607.h>

class Salus_Baro{
public:
    Salus_Baro(void);
    void begin(void);
    void setReferencePressure(double pres);
    void setLocalAsReference();
    void setOversamplingRate(ms5607_osr osr);
    void baroTask();

    float getTemperature();
    float getPressure();
    float getAltitude();
    float getReferencePressure();

private:
    Melon_MS5607 _ms5607;
    float _altRefPressure = 1013.25;   // Sea level pressure, in mbar
    float _pressure;
    float _altitude;
    float _temperature;
    uint8_t _baroState;

    enum{
        BARO_STATE_PRESSURE,
        BARO_STATE_TEMP
    };
};
