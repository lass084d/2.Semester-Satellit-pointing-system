#include <I2C.h>

void scanForAdress()
{
  Serial.println("Scanning for I2C devices...");

  // Scan I2C addresses from 1 to 127
  for (byte i = 1; i < 127; i++)
  {
    Wire.beginTransmission(i);
    byte error = Wire.endTransmission();

    if (error == 0)
    {
      // Print the I2C address if the device is found
      Serial.print("I2C device found at address 0x");
      if (i < 16)
      {
        Serial.print("0");
      }
      Serial.println(i, HEX);
    }
  }
  Serial.println("Scan complete.");
}

// initializes the BMP280 and sets it to normal mode
void initBMP280(struct trimming_parameters *trimming_parameters)
{
  Wire.beginTransmission(BMP_280_ADDRESS);
  Wire.write(0xF4);       // Specify register over sample and power mode
  Wire.write(0b00101111); // Write value to register to set it to 4 oversampeling for preasur and 1 for temp and normal mode
  Wire.endTransmission(false);
  Wire.write(0xF5);       // Specify register iir filter and T_standby
  Wire.write(0b00011100); // Write value to register to set T_standby to 0.5ms and iir filter 16
  Wire.endTransmission(true);

  uint8_t registers[] = {0x88, 0x8E, 0x8A, 0x8C, 0x90, 0x92, 0x94, 0x96, 0x98, 0x9A, 0x9C, 0x9E};
  int16_t *signed_trimming_values[] = {
      &trimming_parameters->dig_T2, &trimming_parameters->dig_T3,
      &trimming_parameters->dig_P2, &trimming_parameters->dig_P3,
      &trimming_parameters->dig_P4, &trimming_parameters->dig_P5,
      &trimming_parameters->dig_P6, &trimming_parameters->dig_P7,
      &trimming_parameters->dig_P8, &trimming_parameters->dig_P9};
  uint16_t *unsigned_trimming_values[] = {
      &trimming_parameters->dig_T1, &trimming_parameters->dig_P1};

  for (int i = 0; i < 12; i++)
  {
    Wire.beginTransmission(BMP_280_ADDRESS);
    Wire.write(registers[i]); // Send the starting register for each parameter
    Wire.endTransmission(false);
    Wire.requestFrom(BMP_280_ADDRESS, 2, true); // Request 2 bytes

    int var1 = Wire.read();
    int var2 = Wire.read();

    // Combine the two bytes into one value and store it in the corresponding trimming parameter
    if (i < 2)
    { // For dig_T1 and dig_P1, store as unsigned
      *unsigned_trimming_values[i] = (var2 << 8) | var1;
    }
    else
    { // For other values, store as signed
      *signed_trimming_values[i - 2] = (var2 << 8) | var1;
    }
  }
}

// MPU6050 initialization
void initMPU6050()
{
  Wire.beginTransmission(0x68); // MPU6050 address
  Wire.write(0x6B);             // Power management register
  Wire.write(0);                // Wake up the MPU6050
  Wire.endTransmission(true);
};

void accData(struct AccelData *AccelData)
{
  // Reading the accelerometer values (for example)
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true); // Request 6 bytes (acceleration data)

  // Reading the values
  int16_t ax_raw = Wire.read() << 8 | Wire.read();
  int16_t ay_raw = Wire.read() << 8 | Wire.read();
  int16_t az_raw = Wire.read() << 8 | Wire.read();

#ifndef CONVERT
  // Print acceleration values
  Serial.print("raw Accel X: ");
  Serial.print(ax_raw);
  Serial.print("\tY: ");
  Serial.print(ay_raw);
  Serial.print("\tZ: ");
  Serial.println(az_raw);

#endif

  AccelData->ax_raw = ax_raw;
  AccelData->ay_raw = ay_raw;
  AccelData->az_raw = az_raw;

#ifdef CONVERT
  // Convert the raw values to g
  float accelX = ax_raw / 16384.0;
  float accelY = ay_raw / 16384.0;
  float accelZ = az_raw / 16384.0;

  // the g into m/s^2
  accelX = accelX * 9.81;
  accelY = accelY * 9.81;
  accelZ = accelZ * 9.81;

  // Print acceleration values in g
  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.print(second());
  Serial.print("\tAccel X: ");
  Serial.print(accelX);
  Serial.print(" m/s^2");
  Serial.print("\tY: ");
  Serial.print(accelY);
  Serial.print(" m/s^2");
  Serial.print("\tZ: ");
  Serial.print(accelZ);
  Serial.println(" m/s^2");

  AccelData->accelX = accelX;
  AccelData->accelY = accelY;
  AccelData->accelZ = accelZ;

#endif
}

void tempData(struct AltitudeData *AltitudeData, struct trimming_parameters *trimming_parameters)
{
  // Reading the altitude values (for example)
  Wire.beginTransmission(BMP_280_ADDRESS);
  Wire.write(0xFA); // Starting register for altitude data
  Wire.endTransmission(false);
  Wire.requestFrom(BMP_280_ADDRESS, 3, true); // Request 3 bytes (altitude data)

  // Reading the values
  int32_t temperature_raw = (int32_t)Wire.read() << 16 | (int32_t)Wire.read() << 8 | Wire.read();

  temperature_raw >>= 4;

  int32_t var1, var2;

  var1 = ((((temperature_raw >> 3) - ((int32_t)trimming_parameters->dig_T1 << 1))) * ((int32_t)trimming_parameters->dig_T2)) >> 11;
  var2 = (((((temperature_raw >> 4) - ((int32_t)trimming_parameters->dig_T1)) * ((temperature_raw >> 4) - ((int32_t)trimming_parameters->dig_T1))) >> 12) * ((int32_t)trimming_parameters->dig_T3)) >> 14;

  trimming_parameters->t_fine = var1 + var2;
  float T = (trimming_parameters->t_fine * 5 + 128) >> 8;

  float temperature = T / 100;

  AltitudeData->temperature = temperature;

  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.print(second());

  Serial.print("\tTemperature: ");
  Serial.print(temperature);
  Serial.println("C");
}

void preasureData(struct AltitudeData *AltitudeData, struct trimming_parameters *trimming_parameters)
{
  // Reading the altitude values (for example)
  Wire.beginTransmission(BMP_280_ADDRESS);
  Wire.write(0xF7); // Starting register for altitude data
  Wire.endTransmission(false);
  Wire.requestFrom(BMP_280_ADDRESS, 3, true); // Request 3 bytes (altitude data)

  // Reading the values
  int32_t preasure_raw = (int32_t)Wire.read() << 16 | (int32_t)Wire.read() << 8 | Wire.read();

  preasure_raw >>= 4;

  int64_t var1, var2, p;

  var1 = ((int64_t)trimming_parameters->t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)trimming_parameters->dig_P6;
  var2 = var2 + ((var1 * (int64_t)trimming_parameters->dig_P5) << 17);
  var2 = var2 + (((int64_t)trimming_parameters->dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)trimming_parameters->dig_P3) >> 8) + ((var1 * (int64_t)trimming_parameters->dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)trimming_parameters->dig_P1) >> 33;

  if (var1 == 0)
  {
    AltitudeData->pressure = 0;
    return; // avoid exception caused by division by zero
  }

  p = 1048576 - preasure_raw;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)trimming_parameters->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)trimming_parameters->dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)trimming_parameters->dig_P7) << 4);

  AltitudeData->pressure = (float)p / 256;

  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.print(second());
  Serial.print("\tPressure: ");
  Serial.print(AltitudeData->pressure);
  Serial.println("Pa");
}