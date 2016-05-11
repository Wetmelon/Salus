// 
// 
// 

#include "Salus_Baro.h"


Salus_Baro::Salus_Baro(){
    // Void
}

void Salus_Baro::begin(){

    _ms5607.begin(SALUS_BARO_ADDRESS);
    _ms5607.setOversamplingRate(MS5607_OSR1024);
    _baroState = BARO_STATE_TEMP;

#ifdef DEBUG_BARO
    Serial.println();
    _ms5607.printCalibData();
    Serial.println();
#endif
}

/* 
    This function is designed to allow the Teensy to do other stuff while the
    MS5607 is doing a conversion.  This keeps us from missing deadlines
*/
void Salus_Baro::baroTask(){
    switch (_baroState)
    {
    case BARO_STATE_TEMP:
        if (_ms5607.readTemperature()){
            _temperature = _ms5607.getTemperature() / 100.0;
            _ms5607.startPressureConversion();
            _baroState = BARO_STATE_PRESSURE;
        }
        break;
    case BARO_STATE_PRESSURE:
        if (_ms5607.readPressure()){
            _pressure = _ms5607.getPressure() / 100.0;
            _altitude = 44330*(1 - pow((_pressure / _altRefPressure), .190284));
            _ms5607.startTemperatureConversion();
            _baroState = BARO_STATE_TEMP;
        }
        break;
    }
#ifdef BARO_DEBUG
    Serial.print("T: ");
    Serial.print(_temperature);
    Serial.print("\t\tP: ");
    Serial.print(_pressure);
    Serial.print("\t\tA: ");
    Serial.println(_altitude);
#endif
}
// Return the current temperature from the MS5607, in C
float Salus_Baro::getTemperature(){
    return _temperature;
}

// Return the current pressure from the MS5607, in mbar
float Salus_Baro::getPressure(){
    return _pressure;
}

// Return the current altitude, using the reference pressure (1013.25 by default)
float Salus_Baro::getAltitude(){
    return _altitude;
}

// Returns the altitude reference pressure.
float Salus_Baro::getReferencePressure(){
    return _altRefPressure;
}

// Set the given value as the altitude reference pressure.
void Salus_Baro::setReferencePressure(double pres)
{
    _altRefPressure = pres;
}

// Set the current local pressure as the altitude reference pressure
void Salus_Baro::setLocalAsReference(){
    _altRefPressure = _ms5607.getPressure();
}

// Sets the oversampling rate
void Salus_Baro::setOversamplingRate(ms5607_osr osr){
    _ms5607.setOversamplingRate(osr);
}