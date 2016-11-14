/***************************************************************************
This is a library for the MS5607-02BA03 barometric pressure sensor

These sensors use I2C or SPI to communicate, though only I2C is implemented

Written by Paul Guenette
Apr 8, 2016

MIT License
***************************************************************************/

#ifndef __MS5607_H__
#define __MS5607_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#ifdef __AVR_ATtiny85__
 #include "TinyWireM.h"
 #define Wire TinyWireM
#elif defined(TEENSYDUINO)
 #include <i2c_t3.h>
#else
 #include <Wire.h>
#endif


// Oversampling rates
typedef enum
{
    MS5607_OSR256   = 0x00, // 750  uS
    MS5607_OSR512   = 0x02, // 1250 uS
    MS5607_OSR1024  = 0x04, // 2500 uS
    MS5607_OSR2048  = 0x06, // 4750 uS
    MS5607_OSR4096  = 0x08  // 9250 uS
} ms5607_osr;

class Melon_MS5607
{
public:
    Melon_MS5607(void);
    bool begin(uint8_t addr);
    void reset();
    void printCalibData();
    void setOversamplingRate(uint8_t rate);

    int32_t getTemperature();                 // Returns the temperature, in Celsius, as a double
    int32_t getPressure();                    // Returns the pressure, in mbar, as a double
    
    bool startTemperatureConversion();
    bool startPressureConversion();

    bool readTemperature();
    bool readPressure();

    void getPressureBlocking();

private:

/*=========================================================================
COMMANDS
-----------------------------------------------------------------------*/
    enum MS5607_CMD
    {
        MS5607_RESET                = 0x1E,
        MS5607_CONVERT_D1           = 0x40,
        MS5607_CONVERT_D2           = 0x50,
        MS5607_ADC_READ             = 0x00,
        MS5607_PROM_READ_C1         = 0xA2,
        MS5607_PROM_READ_C2         = 0xA4,
        MS5607_PROM_READ_C3         = 0xA6,
        MS5607_PROM_READ_C4         = 0xA8,
        MS5607_PROM_READ_C5         = 0xAA,
        MS5607_PROM_READ_C6         = 0xAC
    };
/*=========================================================================*/


/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
    typedef struct 
    {
        uint16_t C1 = 0;            // C1 - Pressure Sensitivity
        uint16_t C2 = 0;            // C2 - Pressure Offset
        uint16_t C3 = 0;            // C3 - Temperature coefficient of pressure sensitivity
        uint16_t C4 = 0;            // C4 - Temperature coefficient of pressure offset
        uint16_t C5 = 0;            // C5 - Reference temperature
        uint16_t C6 = 0;            // C6 - Temperature coefficient of the temperature
    } ms5607_calibration;
/*=========================================================================*/
    
    ms5607_calibration _calibData;

    // Actual values
    uint32_t D1 = 0;
    uint32_t D2 = 0;
    int64_t dT = 0;
    int32_t TEMP = 0;
    int64_t OFF = 0;
    int64_t SENS = 0;
    int32_t P = 0;

    uint8_t _i2caddr;
    uint8_t _oversamplingRate = MS5607_OSR256;
    uint32_t _osrdelay = 1000;  // 1ms delay for OSR256
    uint32_t _lastConversion;

    void compensateSecondOrder();
    void getCalibrationData(ms5607_calibration &calib);
    void setDelay();

    void write8(uint8_t val);
    uint8_t read8(uint8_t reg);
    uint16_t read16(uint8_t reg);
    uint32_t read24(uint8_t reg);
};


#endif