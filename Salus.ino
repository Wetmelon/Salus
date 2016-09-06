
#include <i2c_t3.h>
#include <Arduino.h>
#include "Salus_Common.h"
#include "Salus_GPS.h"
#include "Salus_Accel.h"
#include "Salus_Inertial.h"
#include "Salus_Logging.h"
#include "Salus_Baro.h"


/* ---------------------------------------------------------------------------- +
*
*      Set up the multi-rate main loop timer
*
* ---------------------------------------------------------------------------- */
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
    Serial.begin(250000);
    delay(500);
    Serial.println("\nSalus Testing.");

    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);

    // Initialize the Barometer
    Serial.println("Initializing Barometer...");
    myBaro.begin();
    delay(10);
    myBaro.baroTask();
    delay(10);
    myBaro.baroTask();
    delay(10);
    myBaro.baroTask();
    delay(10);
    shortBeep();
    Serial.println("Barometer Initialized.\n");
    
    // Initialize the IMU
    Serial.println("Initializing IMU...");
    imuBegin();
    shortBeep();
    Serial.println("IMU Initialized.\n");

    // Initialize the GPS module
    Serial.println("Initializing GPS...");
    gpsBegin();
    gpsPt = getGPS();
    shortBeep();
    Serial.println("GPS Initialized.\n");

    // Initialize Data logger
    //startLogger();
    Serial.println("Starting binary logger...");
    startBinLogger(&dateTime);
    Serial.println("Logger started.\n");

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

        // Get the current buffer being used by Salus_Logging
        bufferPt = getBuffer();

        // Load GPS data into buffer
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

        // Load barometer data into buffer
        bufferPt->pressure = myBaro.getPressure();
        bufferPt->temperature = myBaro.getTemperature();

        // Load ADXL data into buffer
        bufferPt->adxlX = xG;
        bufferPt->adxlY = yG;
        bufferPt->adxlZ = zG;

        // Load BNO-055 data into struct
        bufferPt->bnoAx = accelEvent.acceleration.x;
        bufferPt->bnoAy = accelEvent.acceleration.y;
        bufferPt->bnoAz = accelEvent.acceleration.z;
        bufferPt->bnoGx = gyroEvent.gyro.x;
        bufferPt->bnoGy = gyroEvent.gyro.y;
        bufferPt->bnoGz = gyroEvent.gyro.z;
        bufferPt->xOrient = orientEvent.orientation.x;
        bufferPt->yOrient = orientEvent.orientation.y;
        bufferPt->zOrient = orientEvent.orientation.z;

        // Add to buffer array, or write if array is full
        fastLog();

        // Reset logging timer
        log_flag = 0;
    }
}

int freeRam()
{
    extern int __heap_start, *__brkval;
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void dateTime(uint16_t *date, uint16_t *time){
    *date = FAT_DATE(gpsPt->year, gpsPt->month, gpsPt->day);
    *time = FAT_TIME(gpsPt->hour, gpsPt->minute, gpsPt->seconds);
}