#include <I2C.h>

void scanForAdress(){
    Serial.println("Scanning for I2C devices...");
  
    // Scan I2C addresses from 1 to 127
    for (byte i = 1; i < 127; i++) {
      Wire.beginTransmission(i);
      byte error = Wire.endTransmission();
      
      if (error == 0) {
        // Print the I2C address if the device is found
        Serial.print("I2C device found at address 0x");
        if (i < 16) {
          Serial.print("0");
        }
        Serial.println(i, HEX);
      }
    }
    Serial.println("Scan complete.");
}

// MPU6050 initialization
void initMPU6050(){
   Wire.beginTransmission(0x68); // MPU6050 address
   Wire.write(0x6B); // Power management register
   Wire.write(0);    // Wake up the MPU6050
   Wire.endTransmission(true);
};

void accData(struct AccelData *AccelData){
// Reading the accelerometer values (for example)
Wire.beginTransmission(0x68);
Wire.write(0x3B);  // Starting register for accelerometer data
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