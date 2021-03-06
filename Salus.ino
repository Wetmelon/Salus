
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
#define BARO_RATE       (TIMER_RATE / 200)      // Process MS5607 data at 100Hz
#define LOGGING_RATE    (TIMER_RATE / 100)      // Log data at 100Hz

IntervalTimer loopTimer;            // Multi-rate main loop timer object
uint32_t timer = 0;                 // Main loop timer value.  Increments by 1 every time loopTimer is called
uint32_t globalClock = 0;

uint32_t maxBaro = 0;
uint32_t maxImu = 0;
uint32_t maxAdxl = 0;
uint32_t maxLog = 0;
uint32_t maxGps = 0;

float startAlt = 0;

enum control_state {
    WAITING_STATE,
    FLIGHT_STATE
};
enum control_state state = WAITING_STATE;

bool gps_flag = false;
bool adxl_flag = false;
bool bno055_flag = false;
bool baro_flag = false;
bool log_flag = false;

bool beepState = false;
uint32_t beepTimer = millis();
uint32_t beepPace = 10000;

Salus_Baro myBaro;

salus_data_t* bufferPt;
Adafruit_GPS* gpsPt;

// Runs as an interrupt and sets the flags for our multi-rate main loop
void multiRateISR(){
    timer++;
    globalClock++;
    
    readGPS();

    if (timer % GPS_RATE == 0) { gps_flag = true; }
    if (timer % ADXL_RATE == 0) { adxl_flag = true; }
    if (timer % BNO055_RATE == 0) { bno055_flag = true; }
    if (timer % BARO_RATE == 0) { baro_flag = true; }
    if (timer % LOGGING_RATE == 0) { log_flag = true; }
    if (timer >= TIMER_RATE) { timer = 0; }
}

void setup()
{
    // Set Pin modes
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, INPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(19, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(24, OUTPUT);
    pinMode(25, OUTPUT);

    // Set default pin states
    digitalWriteFast(BUZZER_PIN, LOW);
    digitalWriteFast(5, LOW);
    digitalWriteFast(6, LOW);
    digitalWriteFast(7, LOW);
    digitalWriteFast(9, HIGH);
    digitalWriteFast(10, HIGH);
    digitalWriteFast(22, LOW);
    digitalWriteFast(23, LOW);
    digitalWriteFast(24, LOW);
    digitalWriteFast(25, LOW);

    // Setup Analog Accelerometer
    analogReadRes(16);
    analogReadAveraging(4);
    
    // Start USB serial comms
    Serial.begin(250000);
    delay(500);
    Serial.println(F("\nSalus Testing."));

    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
    
    // Initialize the IMU
    Serial.println(F("Initializing IMU..."));
    imuBegin();
    shortBeep();
    Serial.println(F("IMU Initialized.\n"));

    // Initialize the Barometer
    Serial.println(F("Initializing Barometer..."));
    myBaro.begin();
    startAlt = myBaro.getAltitude();
    Serial.print(F("Start Alt: "));
    Serial.println(startAlt);
    shortBeep();
    Serial.println(F("Barometer Initialized.\n"));

    // Initialize the GPS module
    Serial.println(F("Initializing GPS..."));
    gpsBegin();
    gpsPt = getGPS();
    shortBeep();
    Serial.println(F("GPS Initialized.\n"));

    // Initialize Data logger
    //startLogger();
    Serial.println(F("Starting binary logger..."));
    startBinLogger(&dateTime);
    Serial.println(F("Logger started.\n"));

    Wire.setRate(I2C_RATE_400);

    Serial.println(F("Starting tasks..."));
    shortBeep();
    loopTimer.begin(multiRateISR, TIMER_RATE);        // Start the main loop timer
    Serial.println(F("Started.\n\n"));

    // Healthy!
    shortBeep();
    shortBeep();
    shortBeep();
    globalClock = 0;
    beepTimer = millis();
}

void loop()
{
    if (beepState == false){
        if (millis() - beepTimer > beepPace)
        {
            digitalWriteFast(BUZZER_PIN, HIGH);
            beepState = true;
            beepTimer = millis();
        }
    }
    else if (beepState == true){
        if (millis() - beepTimer > 150){
            digitalWriteFast(BUZZER_PIN, LOW);
            beepState = false;
            beepTimer = millis();
#ifdef STATE_DEBUG
            Serial.print("State: ");
            Serial.println(state);
#endif
        }
    }

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
    else if (log_flag && state == FLIGHT_STATE){
        loggingTask();
        // Add to buffer array, or write if array is full
        fastLog();
        // Reset logging timer
        log_flag = 0;
    }
    if (state == WAITING_STATE){
        if (abs(accelEvent.acceleration.y) > 24.0)
        {
#ifdef STATE_DEBUG
            Serial.println("State changed to FLIGHT!");
            Serial.print("Accel: ");
            Serial.println(abs(accelEvent.acceleration.y));
#endif
            state = FLIGHT_STATE;
            beepPace = 750;
            startAlt = myBaro.getAltitude();
            globalClock = 0;
        }
    }
}

void loggingTask(){
    // Get the current buffer being used by Salus_Logging
    bufferPt = getBuffer();
    bufferPt->clock = globalClock;

    // Load GPS data into buffer
    bufferPt->hour = gpsPt->hour;
    bufferPt->minute = gpsPt->minute;
    bufferPt->seconds = gpsPt->seconds;

    bufferPt->satellites = gpsPt->satellites;

    bufferPt->latitude = gpsPt->latitudeDegrees;
    bufferPt->longitude = gpsPt->longitudeDegrees;
    bufferPt->gpsSpeed = gpsPt->speed;
    bufferPt->gpsAltitude = gpsPt->altitude;

    // Load barometer data into buffer
    bufferPt->pressure = myBaro.getPressure();
    bufferPt->altitude = myBaro.getAltitude() - startAlt;
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
    bufferPt->quatW = bnoQuat.w();
    bufferPt->quatX = bnoQuat.x();
    bufferPt->quatY = bnoQuat.y();
    bufferPt->quatZ = bnoQuat.z();
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