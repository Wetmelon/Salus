#include <Melon_ADXL377.h>
#include <i2c_t3.h>
#include <Melon_MS5607.h>
#include <Arduino.h>
#include "Salus_Common.h"
#include "Salus_GPS.h"
#include "Salus_Accel.h"
#include "Salus_Inertial.h"
#include "Salus_Logging.h"

#define TIMER_RATE  (1000)          // Check the timer every 1 millisecond
#define GPS_RATE    (200)           // Process GPS at 5Hz
#define ADXL_RATE   (10)            // Process ADXL377 data at 100Hz
#define BNO055_RATE (10)            // Process BNO055 data at 100Hz

IntervalTimer loopTimer;            // Multi-rate main loop timer object
uint32_t timer = 0;                 // Main loop timer value.  Increments by 1 every time loopTimer is called

bool gps_flag = false;
bool adxl_flag = false;
bool bno055_flag = false;



// Runs as an interrupt and sets the flags for our multi-rate main loop
void multiRateISR(){
    timer++;
    if (timer % GPS_RATE == 0) { gps_flag = 1; }
    if (timer % ADXL_RATE == 0) { adxl_flag = 1; }
    if (timer % BNO055_RATE == 0) { bno055_flag = 1; }
    if (timer >= 200) { timer = 0; }
}


void setup()
{
    loopTimer.begin(multiRateISR, TIMER_RATE);        // Start the main loop timer
}

void loop()
{
    if (bno055_flag) { }
}
