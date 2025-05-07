#include <I2C.h>
#include "esp_timer.h"
#include <math.h>

//init struct for mag data
struct MagData myMagData;

//Set the desired angle (radians) and define desired X and Y koordinates
double desiredAngle = 180*M_PI/180;
double desiredX;
double desiredY;

//PID-controller Constants and variables
const u_int8_t KP = 40, KI=18, KD=2;

double P, I = 0, D;
double sensorAngle;
double error;
double lastError;
double PIDOutput;
double PIDOutput2;

//Time
double dt;
double startTime = 0;
double currentTime;

//set pins
const int motorDirPin = 20;
const int PWMPin = 19;

//Pwm converting values
double Vmaks = 9;
double PWMmaks = 4095;
double PWMToMotor;

  
double ErrorAngleAndDirection (double desiredx, double desiredy){   

//Read the magnetometer
readMagnetometer(&myMagData); 

//find the angle between the vector and the current angle   
double sensorX = myMagData.magX; //Sensor data!!!
double sensorY = myMagData.magY; //Sensor data!!!
double dotproduct = desiredx*sensorX+desiredy*sensorY;
  
double lenOfxy = sqrt(pow(sensorX,2)+pow(sensorY,2));
double angle = acos(dotproduct/lenOfxy); //Formula acos((a ⋅ b)/(|a|*|b|)), but the length of the desired vector is 1 


//Use the crossproduct to find the direction that is the shortest
double crossproduct = sensorX*desiredy-sensorY*desiredx;

if (crossproduct<=0)
  {
    digitalWrite(motorDirPin,LOW);
    Serial.println("Move the motor Counterclockwise, so the cubesat moves clockwise");
  }
  else if (crossproduct>0)
  {
    digitalWrite(motorDirPin,HIGH);
    Serial.println("Move the motor clockwise, so the cubesat moves counterclockwise");
  }

return angle;
}


void setup() {
  // put your setup code here, to run once:
  lastError = 0;
  Serial.begin(115200);
  pinMode(motorDirPin,OUTPUT);
  pinMode(PWMPin, OUTPUT);
  
}


void loop() {
  // put your main code here, to run repeatedly:



  //Convert the desired angle to coordinates  
  desiredX = cos(desiredAngle);
  desiredY = sin(desiredAngle);   
  
//Find the error and set the direction the motor need to spin in
  error = ErrorAngleAndDirection(desiredX,desiredY);
  Serial.println(error,6);

  //The difference in time since last time the PID ran
  currentTime = esp_timer_get_time()*1e-6;
  dt = currentTime - startTime;
  startTime = esp_timer_get_time()*1e-6;

  //calculate the PID values based on the error (output in voltage):
  P = KP * error;
  I += KI*(error*dt);
  D = KD*((error-lastError)/dt);

  //Convert voltage to pwm
  PIDOutput = P+I+D;

  if(PIDOutput > Vmaks){
    PIDOutput2 = Vmaks;
  }
  else{
    PIDOutput2 = PIDOutput;
  }

  if(PIDOutput != PIDOutput2 && error*PIDOutput >=0){
    I -= KI * (error * dt);
  }
  
  PIDOutput2 = P+I+D;;

  PWMToMotor =(PIDOutput2/Vmaks)*PWMmaks;

  analogWrite(PWMPin,PWMToMotor);

  lastError = error;
}

