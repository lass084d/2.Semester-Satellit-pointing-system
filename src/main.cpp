#include <I2C.h>
#include <Arduino.h>

/**
 * @link for the datasheet of the BMP280
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
 * @link for the datasheet of the MPU6050
 * https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 * @link for the I2C register map of the MPU6050
 * https://www.invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 */

struct AccelData myAccelData;
struct trimming_parameters myTrimmingParameters;
struct AltitudeData myAltitudeData;
struct GyroData myGyroData;
int IN3 = 5;
int IN4 = 6;

void setup()
{
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.begin(9600);
  Wire.begin();
  initBMP280(&myTrimmingParameters,
             MODE_NORMAL,   // Opperating mode
             SAMPLING_X1,   // Temp oversampling
             SAMPLING_X4,   // Pressure oversampling
             FILTER_X16,    //  IIR Filtering
             STANDBY_MS_1); // Standby time
  initMPU6050();
  delay(1000);
}
void rotateClockwiseMotor2(int speed) {
  analogWrite(IN3, speed);
  digitalWrite(IN4, LOW);
  Serial.println("Clockwise Rotation! " + String(speed));
  delay(200);
}

void rotateCounterClockwiseMotor2(int speed) {
  digitalWrite(IN3, LOW);
  analogWrite(IN4, speed);
  Serial.println("Counterclockwise Rotation! " + String(speed));
  delay(200);
}

void stopMotor2() {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  Serial.println("Motor Stopped!");
}

void loop()
{
  rotateClockwiseMotor2(255);
  accData(&myAccelData);
  gyroData(&myGyroData);
  tempData(&myAltitudeData, &myTrimmingParameters);
  preasureData(&myAltitudeData, &myTrimmingParameters);
  readAltitude(1020.5, &myAltitudeData);
  Serial.println();
  delay(800);

  stopMotor2();
  delay(500);

  rotateCounterClockwiseMotor2(255);
  accData(&myAccelData);
  gyroData(&myGyroData);
  tempData(&myAltitudeData, &myTrimmingParameters);
  preasureData(&myAltitudeData, &myTrimmingParameters);
  readAltitude(1026.6, &myAltitudeData);
  Serial.println();
  delay(800);
  stopMotor2();
  delay(2000);
}
