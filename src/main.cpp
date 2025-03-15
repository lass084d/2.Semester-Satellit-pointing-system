#include <I2C.h>
#include <Arduino.h>

void rotateClockwiseMotor(int speed);
void rotateCounterClockwiseMotor(int speed);
void stopMotor();

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
  rotateClockwiseMotor(255);
  delay(500);
  stopMotor();
  delay(500);
  rotateCounterClockwiseMotor(255);
  delay(500);
  stopMotor();
  delay(500);
  Serial.println("Setup done!");
  Serial.println("Bitrate, CW1, CW2, CW3, CW4, CW5, CCW1, CCW2, CCW3, CCW4, CCW5");
}
void rotateClockwiseMotor(int speed) {
  digitalWrite(IN4, LOW);
  analogWrite(IN3, speed);
  // Serial.println("Clockwise Rotation! " + String(speed));
}

void rotateCounterClockwiseMotor(int speed) {
  digitalWrite(IN3, LOW);
  analogWrite(IN4, speed);
  //Serial.println("Counterclockwise Rotation! " + String(speed));
}

void stopMotor() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
 // Serial.println("Motor Stopped!");
}

void circle(int speed) {
  float cw[5];  // Array til Clockwise målinger
  float ccw[5]; // Array til Counterclockwise målinger

  rotateClockwiseMotor(speed);
  delay(300);

  for (int x = 0; x < 5; x++) {
    gyroData(&myGyroData);
    cw[x] = myGyroData.gyroZ;  // Gemmer målingen i arrayet
    delay(200);
  }

  stopMotor();
  delay(200);

  rotateCounterClockwiseMotor(speed);
  delay(300);

  for (int x = 0; x < 5; x++) {
    gyroData(&myGyroData);
    ccw[x] = myGyroData.gyroZ;  // Gemmer målingen i arrayet
    delay(200);
  }
  
  stopMotor();
  delay(200);

  Serial.print(speed);
  Serial.print(",");
  for (int x = 0; x < 5; x++) {
    Serial.print(cw[x]);
    Serial.print(",");
  }
  for (int x = 0; x < 5; x++) {
    Serial.print(ccw[x]);
    if (x < 4) Serial.print(","); // Undgå ekstra komma i slutningen
  }
  Serial.println();
}

void loop(){
//Motor kører ikke ved mindre end 31 bits. Dejlig pivende lyd kan nydes her i starten :D
  int min = 120;
  int max = 255;

for (int n = min; n < max + 1; n++)
{
  //Serial.print("Bitrate: " + String(n));
  //Serial.print(" ||| ");
  circle(n);
}

  //Serial.println("Max bitrate reached");
      while(true) {
        //do nothing - programmet er finished
      }
}
