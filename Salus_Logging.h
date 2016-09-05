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

    // GPS data
    uint8_t hour;
    uint8_t minute;
    uint8_t seconds;
    uint16_t milliseconds;

    float latitude;
    float longitude;
    float gpsSpeed;
    float gpsAngle;
    float gpsAltitude;

    uint8_t satellites;

    // Barometer data
    float pressure;
    float temperature;

    // ADXL Data
    double xAccel;
    double yAccel;
    double zAccel;

    // IMU Data
    float xOrient;
    float yOrient;
    float zOrient;

} salus_data_t;

salus_data_t* getBuffer();

void writeData();
void writeHeader();
void startLogger();
void loggingTask();

void startBinLogger(void (*dateTime)(uint16_t*,uint16_t*));
void fastLog();