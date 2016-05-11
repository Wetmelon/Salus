// Salus_Accel.h

#pragma once

#include "Salus_Common.h"
#include <i2c_t3.h>
#include <Melon_ADXL377.h>

#define XGAIN       (.006645)               // mV / g
#define YGAIN       (.007065)
#define ZGAIN       (.007158)

#define XBIAS       (-15)
#define YBIAS       (-3)
#define ZBIAS       (-52)

int32_t xAxis = 32768;
int32_t yAxis = 32768;
int32_t zAxis = 32768;

float xG = 0;
float yG = 0;
float zG = 0;
float Kp = 1.0;

void accelTask(){
    xAxis = Kp*(analogRead(A0) + XBIAS) + (1 - Kp)*xAxis;
    xG = XGAIN*((xAxis)-32768);

    yAxis = Kp*(analogRead(A1) + YBIAS) + (1 - Kp)*yAxis;
    yG = YGAIN*((yAxis)-32768);

    zAxis = Kp*(analogRead(A2) + ZBIAS) + (1 - Kp)*zAxis;
    zG = ZGAIN*((zAxis)-32768);

#ifdef ADXL_DEBUG
    Serial.print("X: ");
    Serial.print(xG, 4);
    Serial.print("\t\tY: ");
    Serial.print(yG, 4);
    Serial.print("\t\tZ: ");
    Serial.println(zG, 4);
#endif
}