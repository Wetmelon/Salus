// 
// 
// 

#include "Salus_GPS.h"

Adafruit_GPS myGPS(&Serial2);

Adafruit_GPS* getGPS(){
    return &myGPS;
}

char readGPS(){
    return myGPS.read();
}

bool gpsBegin(){

    Serial2.begin(9600);
    delay(1000);
    myGPS.sendCommand(PMTK_SET_BAUD_115200);
    delay(100);
    Serial2.end();
    delay(100);
    Serial2.begin(115200);
    delay(100);
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);     // RMC and GGA data
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);        // 5 Hz update rate
    Serial.println("Waiting for GPS Fix...");

#ifdef WAIT_GPS
    uint32_t count = 0;
    while (1){
        myGPS.read();
        if (myGPS.newNMEAreceived()){
            if (myGPS.parse(myGPS.lastNMEA()))
            {
                if (myGPS.fixquality)
                    break;
            }
        }
        delay(1);

        if (++count % 2000 == 0){
            shortBeep();
            count = 0;
        }
    }

#endif
    return true;
}

bool gpsTask(){
    if (myGPS.newNMEAreceived()){
        if (myGPS.parse(myGPS.lastNMEA()))
        {
#ifdef GPS_DEBUG
            Serial.print("\nTime: ");
            Serial.print(myGPS.hour, DEC); Serial.print(':');
            Serial.print(myGPS.minute, DEC); Serial.print(':');
            Serial.print(myGPS.seconds, DEC); Serial.print('.');
            Serial.println(myGPS.milliseconds);
            Serial.print("Date: ");
            Serial.print(myGPS.day, DEC); Serial.print('/');
            Serial.print(myGPS.month, DEC); Serial.print("/20");
            Serial.println(myGPS.year, DEC);
            Serial.print("Fix: "); Serial.print((int)myGPS.fix);
            Serial.print(" quality: "); Serial.println((int)myGPS.fixquality);
            if (myGPS.fix) {
                Serial.print("Location: ");
                Serial.print(myGPS.latitude, 4); Serial.print(myGPS.lat);
                Serial.print(", ");
                Serial.print(myGPS.longitude, 4); Serial.println(myGPS.lon);
                Serial.print("Location (in degrees, works with Google Maps): ");
                Serial.print(myGPS.latitudeDegrees, 4);
                Serial.print(", ");
                Serial.println(myGPS.longitudeDegrees, 4);

                Serial.print("Speed (knots): "); Serial.println(myGPS.speed);
                Serial.print("Angle: "); Serial.println(myGPS.angle);
                Serial.print("Altitude: "); Serial.println(myGPS.altitude);
                Serial.print("Satellites: "); Serial.println((int)myGPS.satellites);
            }
#endif
            return true;
        }
        return false;
    }
    return false;
}
