// Salus_Common.h
#pragma once

#include "Arduino.h"

// Pins
#define BUZZER_PIN 2

//#define GPS_DEBUG
//#define BARO_DEBUG
//#define IMU_DEBUG
//#define ADXL_DEBUG
//#define LOGGER_DEBUG
//#define STATE_DEBUG

#define WAIT_GPS
#define BEEP

void shortBeep();
void longBeep();
void beepTask();
bool queueBeep(uint32_t);