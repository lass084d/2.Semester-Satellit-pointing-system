#include <I2C.h>

void scanForAdress()
{
  Serial.println("Scanning for I2C devices...");

  // Scan I2C addresses from 1 to 127
  for (byte i = 1; i <= 127; i++)
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

void setSamplingSettings(sensor_mode mode,
                         sensor_sampling tempSampling,
                         sensor_sampling pressSampling,
                         sensor_filter filter,
                         standby_duration duration)
{

  Wire.beginTransmission(BMP_ADDRESS);
  Wire.write(0xF4);                                                               // Specify register over sample and power mode
  Wire.write(((tempSampling << 5) | (pressSampling << 2) | (mode)) & 0b11111111); // Write value to register to set the settings
  Wire.endTransmission(true);                                                     // End the transmission, if set to false it will bug out and not send to the next register
  delay(100);
  Wire.beginTransmission(BMP_ADDRESS);
  Wire.write(0xF5);                                           // Specify register for iir filter and T_standby
  Wire.write(((duration << 5) | (filter << 2)) & 0b11111111); // Write value to register to set T_standby to 0.5ms and iir filter 16
  Wire.endTransmission(true);
}

void initBMP(struct trimming_parameters *trimming_parameters,
             sensor_mode mode,
             sensor_sampling tempSampling,
             sensor_sampling pressSampling,
             sensor_filter filter,
             standby_duration duration)
{
  setSamplingSettings(mode,
                      tempSampling,
                      pressSampling,
                      filter,
                      duration);
  // Registers for the trimming parameters
  uint8_t registers[] = {0x88, 0x8E, 0x8A, 0x8C, 0x90, 0x92, 0x94, 0x96, 0x98, 0x9A, 0x9C, 0x9E};
  // Pointers to the trimming parameters
  int16_t *signed_trimming_values[] = {
      &trimming_parameters->dig_T2, &trimming_parameters->dig_T3,
      &trimming_parameters->dig_P2, &trimming_parameters->dig_P3,
      &trimming_parameters->dig_P4, &trimming_parameters->dig_P5,
      &trimming_parameters->dig_P6, &trimming_parameters->dig_P7,
      &trimming_parameters->dig_P8, &trimming_parameters->dig_P9};
  uint16_t *unsigned_trimming_values[] = {
      &trimming_parameters->dig_T1, &trimming_parameters->dig_P1};

  // Read the trimming parameters from the BMP280 and store them in the struct
  for (int i = 0; i < 12; i++)
  {
    Wire.beginTransmission(BMP_ADDRESS);
    Wire.write(registers[i]); // Send the starting register for each parameter
    Wire.endTransmission(false);
    Wire.requestFrom(BMP_ADDRESS, 2, true); // Request 2 bytes

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

  Wire.beginTransmission(BMP_ADDRESS);
  Wire.write(0xF4);                       // Specify first register
  Wire.endTransmission(false);            // End the transmission and initiate a new one emediatly
  Wire.requestFrom(BMP_ADDRESS, 2, true); // Request 2 bytes from 0xF4 and 0xF5
  Serial2.println("BMP280 initialized");
  Serial2.println("Curent settings:");

  int8_t settings_0xF4 = Wire.read();
  int8_t settings_0xF5 = Wire.read();

  Serial2.print("0xF4: ");
  for (int i = 7; i >= 0; i--)
  {                                          // Loop from 7 to 0 (for 8-bit binary representation)
    Serial2.print((settings_0xF4 >> i) & 1); // Shift the bits and print each bit
  }
  Serial2.println(); // Print a new line after the binary representation
  Serial2.print("0xF5: ");
  for (int i = 7; i >= 0; i--)
  {                                          // Loop from 7 to 0 (for 8-bit binary representation)
    Serial2.print((settings_0xF5 >> i) & 1); // Shift the bits and print each bit
  }
  Serial2.println("\n");
}

void initHMC()
{

  uint8_t abe;
  Wire.beginTransmission(HMC_ADDRESS); // HMC address
  Wire.write(0x00);                    // Sets Configuration Register A
  Wire.write(0b01110000);              // Set sample rate to 15Hz, sample size to 1 and normal measurement mode
  Wire.endTransmission(true);
  delay(10);

#ifdef SETTINGS
  Wire.beginTransmission(HMC_ADDRESS);    // Re-init transmission for register read
  Wire.write(0x00);                       // Configuration Register A
  Wire.endTransmission(false);            // Don't release the bus to do a repeated start
  Wire.requestFrom(HMC_ADDRESS, 1, true); // Reads settings from register A
  abe = Wire.read();
  Serial.println("Configuration Register A 0x00: ");
  if (abe == 0b00010000)
  {
    Serial.print("sample rate set to 15Hz, sample size to 1 and normal measurement mode: ");
    // Loop through each bit (8 bits) and print it
  }
  Serial.print("0b");
  for (int i = 7; i >= 0; i--)
  {
    // Use bitRead to read each bit starting from the most significant bit (7)
    Serial.print(bitRead(abe, i));
  };
  Serial.println("");
#endif

  Wire.beginTransmission(HMC_ADDRESS); // HMC address
  Wire.write(0x01);                    // Sets Configuration Register B
  Wire.write(0b11100000);              // Set the range to +-8.1Ga
  Wire.endTransmission(true);
  delay(10);

#ifdef SETTINGS
  Wire.beginTransmission(HMC_ADDRESS);    // Re-init transmission for register read
  Wire.write(0x01);                       // Configuration Register B
  Wire.endTransmission(false);            // Don't release the bus to do a repeated start
  Wire.requestFrom(HMC_ADDRESS, 1, true); // Reads settings from register B
  abe = Wire.read();
  Serial.print("Configuration Register B 0x01: ");
  Serial.println(abe);
#endif

  Wire.beginTransmission(HMC_ADDRESS); // HMC address
  Wire.write(0x02);                    // Sets Mode Register
  Wire.write(0b00000000);              // Set the mode to continuous measurement mode
  Wire.endTransmission(true);
  delay(10);

#ifdef SETTINGS
  Wire.beginTransmission(HMC_ADDRESS);    // Re-init transmission for register read
  Wire.write(0x02);                       // Mode Register
  Wire.endTransmission(false);            // Don't release the bus to do a repeated start
  Wire.requestFrom(HMC_ADDRESS, 1, true); // Reads settings from mode register
  abe = Wire.read();
  Serial.print("Mode Register 0x02: ");
  Serial.println(abe);
#endif

  Serial.println("HMC5883L initialized\n");
}

void initMPU()
{

#ifdef TEST

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 1, true); // Request 1 byte from the WHO_AM_I register

  Serial.println(Wire.read(), HEX); // Read the WHO_AM_I register to check if the MPU6050 is connected

  delay(250);

#endif

  Wire.beginTransmission(MPU_ADDRESS); // MPU address
  Wire.write(0x6B);                    // Power management register
  Wire.write(0b10000000);              // Write a one to bit 7 reset bit; toggle reset device
  Wire.endTransmission(true);
  delay(500);

  Wire.beginTransmission(MPU_ADDRESS); // MPU address
  Wire.write(0x6B);                    // Power management register
  Wire.write(0b00000000);              // Clear sleep mode bit (6), enable all sensors
  Wire.endTransmission(true);
  delay(250);

  Wire.beginTransmission(MPU_ADDRESS); // MPU address
  Wire.write(0x6B);                    // Power management register
  Wire.write(0b00000001);              // sets it to the X-axis gyroscope reference (it is recomended by the datasheet to use one of the gyroscope axies as the clock source) and disable the temperature sensor
  Wire.endTransmission(true);
  delay(250);

  Wire.beginTransmission(MPU_ADDRESS); // MPU address
  Wire.write(0x1A);                    // DLPF (Digital low pass filter) configuration register
  Wire.write(0b00000100);              // Set the DLPF to 20Hz with a total delay of 16.8ms across accelerometer and gyroscope
  Wire.endTransmission(true);
  delay(250);

  Wire.beginTransmission(MPU_ADDRESS); // MPU address
  Wire.write(0x1B);                    // Gyroscope configuration register
  Wire.write(0b00010000);              // Set the gyroscope to full scale range of +-1000 degrees per second
  Wire.endTransmission(true);
  delay(250);

  Wire.beginTransmission(MPU_ADDRESS); // MPU address
  Wire.write(0x1c);                    // accelerometer configuration register
  Wire.write(0b00000000);              // Set the accelerometer to full scale range of +-2g
  Wire.endTransmission(true);

  delay(250);

  Wire.beginTransmission(MPU_ADDRESS); // MPU address
  Wire.write(0x37);                    // Interrupt configuration register
  Wire.write(0x22);                    // Set the interrupt to active high and neabels I2C BYPASS mode
  Wire.endTransmission(true);

  delay(250);

  Wire.beginTransmission(MPU_ADDRESS); // MPU address
  Wire.write(0x38);                    // Interrupt configuration register
  Wire.write(0x01);                    // enables Data Ready interrupt, which occurs each time a write operation to all of the sensor registers has been completed.
  Wire.endTransmission(true);
  delay(250);

  Serial.println("MPU6050 initialized\n");
};

void gyroData(struct GyroData *GyroData)
{
  // Reading the gyroscope values (for example)
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x43); // Starting register for gyroscope data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true); // Request 6 bytes (gyroscope data)

  // Reading the values
  int16_t gx_raw = Wire.read() << 8 | Wire.read();
  int16_t gy_raw = Wire.read() << 8 | Wire.read();
  int16_t gz_raw = Wire.read() << 8 | Wire.read();

#ifndef CONVERT
  // Print gyroscope values
  Serial2.print("raw Gyro X: ");
  Serial2.print(gx_raw);
  Serial2.print("\tY: ");
  Serial2.print(gy_raw);
  Serial2.print("\tZ: ");
  Serial2.println(gz_raw);
#endif

  GyroData->gx_raw = gx_raw;
  GyroData->gy_raw = gy_raw;
  GyroData->gz_raw = gz_raw;

#ifdef CONVERT
  // Convert the raw values to degrees per second using the sensitivity scale factor of 32.8 LSB/(degrees/s)
  float gyroX = gx_raw / 32.8;
  float gyroY = gy_raw / 32.8;
  float gyroZ = gz_raw / 32.8;

  // Print gyroscope values in degrees per second
  Serial2.print(hour());
  Serial2.print(":");
  Serial2.print(minute());
  Serial2.print(":");
  Serial2.print(second());
  Serial2.print("\tGyro X: ");
  Serial2.print(gyroX);
  Serial2.print(" deg/s");
  Serial2.print("\tY: ");
  Serial2.print(gyroY);
  Serial2.print(" deg/s");
  Serial2.print("\tZ: ");
  Serial2.print(gyroZ);
  Serial2.println(" deg/s");

  GyroData->gyroX = gyroX;
  GyroData->gyroY = gyroY;
  GyroData->gyroZ = gyroZ;
#endif
}

void accData(struct AccelData *AccelData)
{
  // Reading the accelerometer values (for example)
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true); // Request 6 bytes (acceleration data)

  // Reading the values
  int16_t ax_raw = Wire.read() << 8 | Wire.read();
  int16_t ay_raw = Wire.read() << 8 | Wire.read();
  int16_t az_raw = Wire.read() << 8 | Wire.read();

#ifndef CONVERT
  // Print acceleration values
  Serial2.print("raw Accel X: ");
  Serial2.print(ax_raw);
  Serial2.print("\tY: ");
  Serial2.print(ay_raw);
  Serial2.print("\tZ: ");
  Serial2.println(az_raw);

#endif

  AccelData->ax_raw = ax_raw;
  AccelData->ay_raw = ay_raw;
  AccelData->az_raw = az_raw;

#ifdef CONVERT
  // Convert the raw values to g using the sensitivity scale factor of 16384 LSB/g
  float accelX = ax_raw / 16384.0;
  float accelY = ay_raw / 16384.0;
  float accelZ = az_raw / 16384.0;

  // the g into m/s^2
  accelX = accelX * 9.82;
  accelY = (accelY * 9.82) * 0.9949341439;
  accelZ = (accelZ * 9.82) * 0.9722772277;

  // Print acceleration values in g
  Serial2.print(hour());
  Serial2.print(":");
  Serial2.print(minute());
  Serial2.print(":");
  Serial2.print(second());
  Serial2.print("\tAccel X: ");
  Serial2.print(accelX);
  Serial2.print(" m/s^2");
  Serial2.print("\tY: ");
  Serial2.print(accelY);
  Serial2.print(" m/s^2");
  Serial2.print("\tZ: ");
  Serial2.print(accelZ);
  Serial2.println(" m/s^2");

  AccelData->accelX = accelX;
  AccelData->accelY = accelY;
  AccelData->accelZ = accelZ;

#endif
}

void tempData(struct AltitudeData *AltitudeData, struct trimming_parameters *trimming_parameters)
{
  // Reading the altitude values (for example)
  Wire.beginTransmission(BMP_ADDRESS);
  Wire.write(0xFA); // Starting register for altitude data
  Wire.endTransmission(false);
  Wire.requestFrom(BMP_ADDRESS, 3, true); // Request 3 bytes (altitude data)

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

  Serial2.print(hour());
  Serial2.print(":");
  Serial2.print(minute());
  Serial2.print(":");
  Serial2.print(second());

  Serial2.print("\tTemperature: ");
  Serial2.print(temperature);
  Serial2.println("C");
}

void preasureData(struct AltitudeData *AltitudeData, struct trimming_parameters *trimming_parameters)
{
  // Reading the altitude values (for example)
  Wire.beginTransmission(BMP_ADDRESS);
  Wire.write(0xF7); // Starting register for altitude data
  Wire.endTransmission(false);
  Wire.requestFrom(BMP_ADDRESS, 3, true); // Request 3 bytes (altitude data)

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

  Serial2.print(hour());
  Serial2.print(":");
  Serial2.print(minute());
  Serial2.print(":");
  Serial2.print(second());
  Serial2.print("\tPressure: ");
  Serial2.print(AltitudeData->pressure);
  Serial2.println("Pa");
}

void readAltitude(float seaLevelhPa, struct AltitudeData *AltitudeData)
{
  float altitude;

  float pressure = AltitudeData->pressure; // in Si units for Pascal
  pressure /= 100;

  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  AltitudeData->altitude = altitude;

  Serial2.print(hour());
  Serial2.print(":");
  Serial2.print(minute());
  Serial2.print(":");
  Serial2.print(second());
  Serial2.print("\tAltitude: ");
  Serial2.print(AltitudeData->altitude);
  Serial2.println("m");
}

void readMagnetometer()
{
  uint8_t var1;
  uint8_t var2;
  // Reading the magnetometer values (for example)
  Wire.beginTransmission(HMC_ADDRESS);
  Wire.write(0x03); // Starting register for magnetometer data
  Wire.endTransmission(false);
  Wire.requestFrom(HMC_ADDRESS, 6, true); // Request 6 bytes (magnetometer data)

  // Reading the values
  var1 = Wire.read();
  var2 = Wire.read();
  int16_t m_x_raw = var1 << 8 | var2;
  var1 = Wire.read();
  var2 = Wire.read();
  int16_t m_z_raw = var1 << 8 | var2;
  var1 = Wire.read();
  var2 = Wire.read();
  int16_t m_y_raw = var1 << 8 | var2;

#ifndef CONVERT
  // Print magnetometer values
  Serial.print("raw magnometer X: ");
  Serial.print(m_x_raw);
  Serial.print("\tY: ");
  Serial.print(m_y_raw);
  Serial.print("\tZ: ");
  Serial.println(m_z_raw);
#endif
/*
 GyroData->gx_raw = gx_raw;
 GyroData->gy_raw = gy_raw;
 GyroData->gz_raw = gz_raw;
*/
#ifdef CONVERT
  // Convert the raw values to Gauss using the sensitivity scale factor of 4.35 milliGauss per LSB
  float magX = ((float)m_x_raw * (float)4.35) + (float)746.025;
  float magY = (float)m_y_raw * (float)4.35;
  float magZ = (float)m_z_raw * (float)4.35;

  // Print gyroscope values in degrees per second

  Serial2.print(hour());
  Serial2.print(":");
  Serial2.print(minute());
  Serial2.print(":");
  Serial2.print(second());
  Serial2.print(",");

  // Serial2.print("\tMagnetometer X: ");
  Serial2.print(magX);
  //Serial.print(magX);
  // Serial2.print(" Milli Gauss");
  Serial2.print(",");

  // Serial2.print("\tY: ");
  Serial2.print(magY);
  // Serial2.print(" Milli Gauss");
  Serial2.print(",");

  // Serial2.print("\tZ: ");
  Serial2.print(magZ);
  Serial2.print(",");

  Serial2.println(" Milli Gauss");
/*
 GyroData->gyroX = gyroX;
 GyroData->gyroY = gyroY;
 GyroData->gyroZ = gyroZ;
*/
#endif
}
