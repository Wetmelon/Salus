// Salus_GPS.h

#pragma once
#include "Salus_Common.h"
#include <Adafruit_GPS.h>


Adafruit_GPS* getGPS();
bool gpsBegin();
bool gpsTask();
char readGPS();
void dateTime(uint16_t *date, uint16_t *time);