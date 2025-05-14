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
const double KP = 0.5, KI = 0.3, KD = 0;

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


double delayPeriod = 30*1e-3; //Time in microseconds (which is converted from milliseconds*/1000)


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
  Serial2.print(";");
  Serial2.println("Deltat");
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
  Serial2.print(I,6);
  Serial2.print(";");
  Serial2.println(dt,6);
}

double ErrorAngleAndDirection(double desiredx, double desiredy)
{

  // Read the magnetometer
  readMagnetometer(&myMagData);

  // find the angle between the vector and the current angle
  double sensorX = myMagData.magX; // Sensor dataX!!!
  double sensorY = myMagData.magY; // Sensor dataY!!!
  //double sensorX = 0;
  //double sensorY = 5600;

  double dotproduct = desiredx * sensorX + desiredy * sensorY;

  double lenOfxy = sqrt(pow(sensorX, 2) + pow(sensorY, 2));

  double angle = acos(dotproduct / lenOfxy); // Formula acos((a ⋅ b)/(|a|*|b|)), but the length of the desired vector is 1

  // Use the crossproduct to find the direction that is the shortest
  double crossproduct = sensorX * desiredy - sensorY * desiredx;


  if (crossproduct <= 0)
  {
    //digitalWrite(motorDirPin, HIGH);
    //Serial.println("Move the motor Counterclockwise, so the cubesat moves clockwise");
    return angle;
  }
  else
  {
    //digitalWrite(motorDirPin, LOW);
    //Serial.println("Move the motor clockwise, so the cubesat moves counterclockwise");
    return -angle;
  }
}

void setup()
{
  //delay(7000); // Wait for the serial monitor to open
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
  startTime = esp_timer_get_time() * 1e-6;
}

void loop()
{
  // put your main code here, to run repeatedly:

  // The difference in time since last time the PID ran
  currentTime = esp_timer_get_time() * 1e-6;
  dt = currentTime - startTime;
  startTime = esp_timer_get_time() * 1e-6;


  // Convert the desired angle to coordinates
  desiredX = cos(desiredAngle);
  desiredY = sin(desiredAngle);

  // Find the error and set the direction the motor need to spin in
  error = ErrorAngleAndDirection(desiredX, desiredY);
  //Serial.println(error, 6);

  // calculate the PID values based on the error (output in voltage):
  P = KP * error;
  I += KI * (error * dt);
  D = KD * ((error - lastError) / dt);

  // Convert voltage to pwm
  PIDOutput = P+I+D;



  if (PIDOutput > Vmaks)
  {
    PIDOutput2 = Vmaks;
  }
  else if(PIDOutput < -Vmaks){
    PIDOutput2 = -Vmaks;
  }
  else
  {
    PIDOutput2 = PIDOutput;
  }

  if (PIDOutput != PIDOutput2 && error * PIDOutput >= 0)
  {
    I -= KI * (error * dt);
  }
  

  if (PIDOutput2 > 0){
    digitalWrite(motorDirPin, HIGH);
  }
  else{
    digitalWrite(motorDirPin, LOW);
  }

  PWMToMotor = (abs(PIDOutput2) / Vmaks) * PWMmaks;
  if(PWMToMotor + 259 > 4095){
    PWMToMotor = 4095;
  }
  else{
    PWMToMotor += 259;
  }

  analogWrite(PWMPin, PWMToMotor);

  lastError = error;

  SerialKom(); // Send the data to the serial monitor

  while (startTime + delayPeriod >= esp_timer_get_time() * 1e-6)
  {
    
  }
}
