#include <I2C.h>

struct AccelData myAccelData = {0, 0, 0};
struct trimming_parameters myTrimmingParameters;
struct AltitudeData myAltitudeData = {0, 0, 0};

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  initBMP280(&myTrimmingParameters);
  initMPU6050();
}

void loop()
{  
  accData(&myAccelData);
  tempData(&myAltitudeData, &myTrimmingParameters);
  preasureData(&myAltitudeData, &myTrimmingParameters);
  Serial.println();
  delay(1000);
  
}
