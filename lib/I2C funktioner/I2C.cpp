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
 int16_t ax = Wire.read() << 8 | Wire.read();
 int16_t ay = Wire.read() << 8 | Wire.read();
 int16_t az = Wire.read() << 8 | Wire.read();

 // Print acceleration values
 Serial.print("Accel X: ");
 Serial.print(ax);
 Serial.print("\tY: ");
 Serial.print(ay);
 Serial.print("\tZ: ");
 Serial.println(az);

AccelData->ax = ax;
AccelData->ay = ay;
AccelData->az = az;

}