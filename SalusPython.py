import struct
import os

    # // Floats are 32-bit on Teensy 3.x with Teensyduino 1.6.x
    # // Doubles are 64-bit on Teensy 3.x with Teensyduino 1.6.x

    # uint8_t dataVersion = 2;
    
    # uint32_t clock;
    
    # // GPS data
    # uint8_t hour;
    # uint8_t minute;
    # uint8_t seconds;
    # uint8_t satellites;

    # float latitude;
    # float longitude;
    # float gpsSpeed;
    # float gpsAltitude;

    # // Barometer data
    # float pressure;
    # float altitude;
    # float temperature;

    # // ADXL Data
    # float adxlX;
    # float adxlY;
    # float adxlZ;

    # // IMU Data
    # float bnoAx;
    # float bnoAy;
    # float bnoAz;
    # float bnoGx;
    # float bnoGy;
    # float bnoGz;

    # double quatW;
    # double quatX;
    # double quatY;
    # double quatZ;

keys = (
    'dataVersion', 'milliseconds',
    'hour', 'minute', 'seconds',
    'satellites', 'latitude', 'longitude',
    'gpsSpeed', 'gpsAltitude',
    'pressure', 'AGL', 'temperature',
    'adxlX', 'adxlY', 'adxlZ',
    'imuAx', 'imuAy', 'imuAz',
    'imuGx', 'imuGy', 'imuGz',
    'quatW', 'quatX', 'quatY', 'quatZ',
)
structFormat = "<BI4B16f4d"
dataSize = 512//struct.calcsize(structFormat)*struct.calcsize(structFormat)

for filename in os.listdir():
    if ".bin" in filename:
        dataEntries = []
        fileSize = os.stat(filename).st_size
        with open(filename, "rb") as inFile:
            for x in range(fileSize//512):
                for entry in struct.iter_unpack(structFormat, inFile.read(dataSize)):
                    if not(entry[0] == 0 or entry[0] == 255):
                        dataEntries.append(entry)
                inFile.read(512-dataSize)

        import csv
        with open(filename.replace(".bin",".csv"),'w', newline='') as csvfile:
            csvWriter = csv.writer(csvfile, dialect='excel')
            csvWriter.writerow(keys)
            for entry in dataEntries:
                csvWriter.writerow(entry)