#include <Arduino.h>
#include <Wire.h>
#include <TimeLib.h>

struct AccelData {
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;

    float accelX;
    float accelY;
    float accelZ;
};

//#define TEST
#define CONVERT

void scanForAdress();
void initMPU6050();
void accData(struct AccelData *AccelData);