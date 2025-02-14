#include <I2C.h>

struct AccelData myAccelData = {0, 0, 0};

void setup()
{
  Wire.begin();
  Serial.begin(9600);

#ifdef TEST
  scanForAdress();
#endif
  Serial.println("bitch ass");
  void initMPU6050();
}

void loop()
{
  accData(&myAccelData);
  delay(200);

  // Wire.beginTransmission(0x40);
  // Wire.write(0x00);
}
