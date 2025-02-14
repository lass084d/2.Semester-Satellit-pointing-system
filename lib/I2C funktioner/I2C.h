#include <Arduino.h>
#include <Wire.h>

struct AccelData {
    int16_t ax;
    int16_t ay;
    int16_t az;
};

//#define TEST

void scanForAdress();
void initMPU6050();
void accData(struct AccelData *AccelData);