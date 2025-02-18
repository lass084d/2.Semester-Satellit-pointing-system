#include <Arduino.h>
#include <Wire.h>
#include <TimeLib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>

struct trimming_parameters
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    int32_t t_fine; // t_fine carries fine temperature as global value
};

struct AccelData {
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;

    float accelX;
    float accelY;
    float accelZ;
};

struct AltitudeData {
    float temperature;
    float pressure;
    float altitude;
};

#define BMP_280_ADDRESS 0x76
#define TEST
#define CONVERT

void scanForAdress();
void initMPU6050();
void initBMP280(trimming_parameters *trimmingParameters);
void accData(struct AccelData *AccelData);
void tempData(struct AltitudeData *AltitudeData, struct trimming_parameters *trimming_parameters);
void preasureData(struct AltitudeData *AltitudeData, struct trimming_parameters *trimming_parameters);