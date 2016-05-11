
#include <Arduino.h>
#include "Salus_Common.h"
#include "Salus_GPS.h"
#include "Salus_Accel.h"
#include "Salus_Inertial.h"
#include "Salus_Logging.h"
#include "Salus_Baro.h"
#include <Melon_ADXL377.h>

#define TIMER_RATE      (1000)                  // Check the timer every 1 millisecond
#define GPS_RATE        (TIMER_RATE / 100)      // Check GPS for new NMEA at 100Hz.  Actual refresh rate is 5Hz.
#define ADXL_RATE       (TIMER_RATE / 100)      // Process ADXL377 data at 100Hz
#define BNO055_RATE     (TIMER_RATE / 100)      // Process BNO055 data at 100Hz
#define BARO_RATE       (TIMER_RATE / 200)      // Process MS5607 data at 200Hz
#define LOGGING_RATE    (TIMER_RATE / 100)      // Log data at 100Hz

IntervalTimer loopTimer;            // Multi-rate main loop timer object
uint32_t timer = 0;                 // Main loop timer value.  Increments by 1 every time loopTimer is called
int count;

bool gps_flag = false;
bool adxl_flag = false;
bool bno055_flag = false;
bool baro_flag = false;
bool log_flag = false;

Salus_Baro myBaro;

salus_data_t* bufferPt;
Adafruit_GPS* gpsPt;

// Runs as an interrupt and sets the flags for our multi-rate main loop
void multiRateISR(){
    timer++;
    readGPS();
    if (timer % GPS_RATE == 0) { gps_flag = 1; }
    if (timer % ADXL_RATE == 0) { adxl_flag = 1; }
    if (timer % BNO055_RATE == 0) { bno055_flag = 1; }
    if (timer % BARO_RATE == 0) { baro_flag = 1; }
    if (timer % LOGGING_RATE == 0) { log_flag = 1; }
    if (timer >= TIMER_RATE) { timer = 0; }
}

void setup()
{
    // Set Pin modes
    pinMode(BUZZER_PIN, OUTPUT);

    // Set default pin states
    digitalWriteFast(BUZZER_PIN, LOW);

    // Setup Analog Accelerometer
    analogReadRes(16);
    analogReadAveraging(4);
    
    // Start USB serial comms
    Serial.begin(115200);
    delay(500);
    Serial.println("\nSalus Testing.");

    // Initialize the Barometer
    Serial.println("Initializing Barometer...");
    myBaro.begin();
    shortBeep();
    myBaro.setReferencePressure(1013.6);
    Serial.println("Barometer Initialized.\n");
    
    // Initialize Data logger
    //startLogger();
    startBinLogger();

    // Initialize the IMU
    Serial.println("Initializing IMU...");
    imuBegin();
    shortBeep();
    Serial.println("IMU Initialized.\n");

    // Initialize the GPS module
    Serial.println("Initializing GPS...");
    gpsBegin();
    shortBeep();
    Serial.println("GPS Initialized.\n");

    Wire.setRate(I2C_RATE_400);

    Serial.println("Starting tasks...");
    shortBeep();
    delay(1000);
    loopTimer.begin(multiRateISR, TIMER_RATE);        // Start the main loop timer
    Serial.println("Started.\n\n");

    // Healthy!
    shortBeep();
    delay(75);
    shortBeep();
    delay(75);
    shortBeep();
    delay(200);

    gpsPt = getGPS();
}

void loop()
{
    if (gps_flag)
    {
        gpsTask();
        gps_flag = 0;
    }
    else if (baro_flag)
    {
        myBaro.baroTask();
        baro_flag = 0;
    }
    else if (adxl_flag)
    {
        accelTask();
        adxl_flag = 0;
    }
    else if (bno055_flag)
    {
        imuTask();
        bno055_flag = 0;
    }
    else if (log_flag){

        bufferPt = getBuffer();

        // Load the data buffer with current data
        bufferPt->hour = gpsPt->hour;
        bufferPt->minute = gpsPt->minute;
        bufferPt->seconds = gpsPt->seconds;
        bufferPt->milliseconds = gpsPt->milliseconds;

        bufferPt->latitude = gpsPt->latitudeDegrees;
        bufferPt->longitude = gpsPt->longitudeDegrees;
        bufferPt->gpsSpeed = gpsPt->speed;
        bufferPt->gpsAngle = gpsPt->angle;
        bufferPt->gpsAltitude = gpsPt->altitude;
        bufferPt->satellites = gpsPt->satellites;

        bufferPt->pressure = myBaro.getPressure();
        bufferPt->temperature = myBaro.getTemperature();

        bufferPt->xAccel = xG;
        bufferPt->yAccel = yG;
        bufferPt->zAccel = zG;

        bufferPt->xOrient = event.orientation.x;
        bufferPt->yOrient = event.orientation.y;
        bufferPt->zOrient = event.orientation.z;

        fastLog();

        log_flag = 0;
    }
}

int freeRam()
{
    extern int __heap_start, *__brkval;
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}