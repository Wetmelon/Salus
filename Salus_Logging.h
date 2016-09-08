// Salus_Logging.h

#pragma once

#include "Salus_Common.h"
#include <SystemInclude.h>
#include <SdFatUtil.h>
#include <SdFatConfig.h>
#include <SdFat.h>
#include <MinimumSerial.h>

typedef struct __attribute__ ((packed)) {

    // Floats are 32-bit on Teensy 3.x with Teensyduino 1.6.x
    // Doubles are 64-bit on Teensy 3.x with Teensyduino 1.6.x

    uint8_t dataVersion = 2;
    
    uint32_t clock;
    
    // GPS data
    uint8_t hour;
    uint8_t minute;
    uint8_t seconds;
    uint8_t satellites;

    float latitude;
    float longitude;
    float gpsSpeed;
    float gpsAltitude;

    // Barometer data
    float pressure;
    float altitude;
    float temperature;

    // ADXL Data
    float adxlX;
    float adxlY;
    float adxlZ;

    // IMU Data
    float bnoAx;
    float bnoAy;
    float bnoAz;
    float bnoGx;
    float bnoGy;
    float bnoGz;

    double quatW;
    double quatX;
    double quatY;
    double quatZ;

} salus_data_t;

salus_data_t* getBuffer();

void writeData();
void writeHeader();
void startLogger();
void loggingTask();

void startBinLogger(void (*dateTime)(uint16_t*,uint16_t*));
void fastLog();