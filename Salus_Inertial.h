// Salus_Inertial.h

#pragma once

#include "Salus_Common.h"
#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(54, 0x28);
sensors_event_t event;
/**************************************************************************/
/*
Displays some basic information on this sensor from the unified
sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displayIMUDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
}

/**************************************************************************/
/*
Display some basic info about the sensor status
*/
/**************************************************************************/
void displayIMUStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
}

/**************************************************************************/
/*
Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
Display the raw calibration offset and radius data
*/
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print((int16_t)calibData.accel_offset_x); Serial.print(" ");
    Serial.print((int16_t)calibData.accel_offset_y); Serial.print(" ");
    Serial.print((int16_t)calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print((int16_t)calibData.gyro_offset_x); Serial.print(" ");
    Serial.print((int16_t)calibData.gyro_offset_y); Serial.print(" ");
    Serial.print((int16_t)calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print((int16_t)calibData.mag_offset_x); Serial.print(" ");
    Serial.print((int16_t)calibData.mag_offset_y); Serial.print(" ");
    Serial.print((int16_t)calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print((int16_t)calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print((int16_t)calibData.mag_radius);
}

/**************************************************************************/
/*
Print error to screen
*/
/**************************************************************************/
void displayError(uint8_t system_error)
{
    Serial.print("\nERROR: 0x");
    Serial.print(system_error, HEX);
}


void writeToEEPROM(int eeAddress, int bnoID, adafruit_bno055_offsets_t &newCalib)
{
    eeAddress = 0;
    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);

    Serial.println("Data stored to EEPROM.");
}

/**************************************************************************/
/*
Loop until the BNO-055 is fully calibrated,
and return the calibration data via the struct parameter
*/
/**************************************************************************/
void calibrateSensor(Adafruit_BNO055 bno, adafruit_bno055_offsets_t &newCalib)
{
    sensors_event_t event;
    Serial.println("\nPlease Calibrate Sensor:\n");
    while (!bno.isFullyCalibrated())
    {
        bno.getEvent(&event);

        Serial.print("X: ");
        Serial.print(event.orientation.x, 4);
        Serial.print("\tY: ");
        Serial.print(event.orientation.y, 4);
        Serial.print("\tZ: ");
        Serial.print(event.orientation.z, 4);

        /* Optional: Display calibration status */
        displayCalStatus();

        /* New line for the next sample */
        Serial.println("");

        /* Wait the specified delay before requesting new data */
        delay(BNO055_SAMPLERATE_DELAY_MS);
    }

    Serial.println("\nFully calibrated!");
    Serial.println("\nCalibration Results: ");

    if (bno.getSensorOffsets(newCalib))
        displaySensorOffsets(newCalib);
    else
        Serial.println("ERROR:  Sensor may not be fully calibrated.");


}

void imuBegin(){
    sensor_t sensor;
    sensors_event_t event;

    adafruit_bno055_offsets_t calibrationData;
    adafruit_bno055_offsets_t newCalib;

    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;

    int eeAddress = 0;
    long bnoID;

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1){};
    }


    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    EEPROM.get(eeAddress, bnoID);

    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        bnoID = sensor.sensor_id;
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM.");
        calibrateSensor(bno, newCalib);
        writeToEEPROM(eeAddress, sensor.sensor_id, newCalib);
    }
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM:");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);
        displaySensorOffsets(calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");

        if (!bno.setSensorOffsets(calibrationData))
        {
            bnoID = sensor.sensor_id;
            Serial.println("\n\nERROR: Sensor did not enter config mode.");
            calibrateSensor(bno, newCalib);
            writeToEEPROM(eeAddress, sensor.sensor_id, newCalib);
        }
    }

    delay(1000);

    /* Display some basic information on this sensor */
    displayIMUDetails();
    delay(500);

    /* Optional: Display current status */
    displayIMUStatus();
    delay(500);

    bno.setExtCrystalUse(true);
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);
    if (system_status == 0x01)
    {
        displayError(system_status);
    }

    /* Wait until the sensor's Sensor Fusion Algorithm starts */
    Serial.println("Waiting for the Fusion Algorithm to start...\n");
    while (system_status != 0x05)
    {
        bno.getSystemStatus(&system_status, &self_test_results, &system_error);
        bno.getEvent(&event);
        delay(BNO055_SAMPLERATE_DELAY_MS);
    }
}


void imuTask(){
    /* Get a new sensor event */
    bno.getEvent(&event);

#ifdef IMU_DEBUG
    /* Display the floating point data */
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);

    /* Optional: Display calibration status */
    displayCalStatus();

    /* Optional: Display sensor status (debug only) */
    //displaySensorStatus();

    /* New line for the next sample */
    Serial.println("");
#endif
}