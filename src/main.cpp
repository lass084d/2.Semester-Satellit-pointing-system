#include <I2C.h>
#include "esp_timer.h"
#include <math.h>

// init struct for mag data
struct MagData myMagData;

// Set the desired angle (radians) and define desired X and Y koordinates
double desiredAngle = 0;
double desiredX;
double desiredY;

// PID-controller Constants and variables
const u_int8_t KP = 3, KI = 0, KD = 0;

double P, I = 0, D;
double sensorAngle;
double error;
double lastError;
double PIDOutput;
double PIDOutput2;

// Time
double dt;
double startTime = 0;
double currentTime;

// set pins
const int motorDirPin = 4;
const int PWMPin = 2;

// Pwm converting values
double Vmaks = 9;
double PWMmaks = 4095;
double PWMToMotor;

void SerialKomm()
{
  Serial2.print("tid");
  Serial2.print(";");
  Serial2.print("xakse");
  Serial2.print(";");
  Serial2.print("yakse");
  Serial2.print(";");
  Serial2.print("zakse");
  Serial2.print(";");
  Serial2.print("fejl");
  Serial2.print(";");
  Serial2.print("spendig");
  Serial2.print(";");
  Serial2.print("PWM");
  Serial2.print(";");
  Serial2.print("intigral");
}

void SerialKom()
{
  Serial2.print(millis());
  Serial2.print(";");
  Serial2.print(myMagData.magX);
  Serial2.print(";");
  Serial2.print(myMagData.magY);
  Serial2.print(";");
  Serial2.print(myMagData.magZ);
  Serial2.print(";");
  Serial2.print(error, 6);
  Serial2.print(";");
  Serial2.print(PIDOutput2, 6);
  Serial2.print(";");
  Serial2.print(PWMToMotor, 6);
  Serial2.print(";");
  Serial2.println(I,6);
}

double ErrorAngleAndDirection(double desiredx, double desiredy)
{

  // Read the magnetometer
  readMagnetometer(&myMagData);

  // find the angle between the vector and the current angle
  double sensorX = myMagData.magX; // Sensor data!!!
  double sensorY = myMagData.magY; // Sensor data!!!
  double dotproduct = desiredx * sensorX + desiredy * sensorY;

  double lenOfxy = sqrt(pow(sensorX, 2) + pow(sensorY, 2));
  double angle = acos(dotproduct / lenOfxy); // Formula acos((a ⋅ b)/(|a|*|b|)), but the length of the desired vector is 1

  // Use the crossproduct to find the direction that is the shortest
  double crossproduct = sensorX * desiredy - sensorY * desiredx;

  if (crossproduct <= 0)
  {
    digitalWrite(motorDirPin, HIGH);
    Serial.println("Move the motor Counterclockwise, so the cubesat moves clockwise");
    return angle;
  }
  else if (crossproduct > 0)
  {
    digitalWrite(motorDirPin, LOW);
    Serial.println("Move the motor clockwise, so the cubesat moves counterclockwise");
    return -angle;
  }

}

void setup()
{
  delay(10000); // Wait for the serial monitor to open
  // put your setup code here, to run once:
  lastError = 0;
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // Serial2 for the serial monitor
  Wire.begin(21, 22);                      // I2C pins SDA and SCL
  initMPU();                               // Initialize the MPU6050
  initHMC();                               // Initialize the HMC5883L
  pinMode(motorDirPin, OUTPUT);
  pinMode(PWMPin, OUTPUT);
  SerialKomm(); // Send the header to the serial monitor
}

void loop()
{
  // put your main code here, to run repeatedly:

  // Convert the desired angle to coordinates
  desiredX = cos(desiredAngle);
  desiredY = sin(desiredAngle);

  // Find the error and set the direction the motor need to spin in
  error = ErrorAngleAndDirection(desiredX, desiredY);
  Serial.println(error, 6);

  // The difference in time since last time the PID ran
  currentTime = esp_timer_get_time() * 1e-6;
  dt = currentTime - startTime;
  startTime = esp_timer_get_time() * 1e-6;

  // calculate the PID values based on the error (output in voltage):
  P = KP * error;
  I += KI * (error * dt);
  D = KD * ((error - lastError) / dt);

  // Convert voltage to pwm
  PIDOutput = P+I+D;



  if (PIDOutput > Vmaks || PIDOutput < -Vmaks)
  {
    PIDOutput2 = Vmaks;
  }
  else
  {
    PIDOutput2 = PIDOutput;
  }

  if (PIDOutput != PIDOutput2 && error * PIDOutput >= 0)
  {
    I -= KI * (error * dt);
  }

  PWMToMotor = (abs(PIDOutput2) / Vmaks) * PWMmaks;

  analogWrite(PWMPin, PWMToMotor);

  lastError = error;

  SerialKom(); // Send the data to the serial monitor

  delay(15); // Delay to make the serial monitor readable
  /*
  Serial.print("error\t");
  Serial.println(error,6);
  Serial.print("PWM\t");
  Serial.println(PWMToMotor,6);
  Serial.print("spænding\t");
  Serial.println(PIDOutput2,6);
  delay(500);
  */
}
