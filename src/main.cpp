#include "esp_timer.h"
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "BluetoothSerial.h"
#include <I2C.h>


#define serial Serial2.print
#define serialln Serial2.println

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

struct PIDData
{
  // Set the desired angle (radians) and define desired X and Y koordinates
  double desiredAngle = 0;
  double desiredX;
  double desiredY;

  // PID-controller Constants and variables
  const double KP = 5.73, KI = 2, KD = 0;

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

  double delayPeriod = 30 * 1e-3; // Time in microseconds (which is converted from milliseconds*/1000)

  // Pwm converting values
  double Vmaks = 9;
  double PWMToMotor;
  const double resolution = 12;
  const double PWMmaks = (int)(pow(2, resolution) - 1);
  const int8_t pwmChannel = 0;
};

// set pins
const int motorDirPin = 4;
const int PWMPin = 2;
long long lastTime = 0;

// actual angle
double actualAngle = 0;

// opret navm og bluetooth seriel kommunikation
String device_name = "ESD-213-esp32";
BluetoothSerial SerialBT;

// struct til kopi af data
struct BTSendData
{
  double rad;
  double angle;
  double error;
  bool hasReceivedAngle = false;
};

bool started = true;


void btReceiveTask(struct BTSendData *btSend, struct PIDData *pidData, bool *started)
{
  // while (1) {
  if (SerialBT.hasClient())
  {
    if (SerialBT.available())
    {
      // Læs op til newline og fjern mellemrum
      String angleCmd = SerialBT.readStringUntil('\n');
      angleCmd.trim();

      // Konverter til double
      double deg = angleCmd.toDouble();

      // Tjek for gyldigt tal (inkl. "0")
      if (!angleCmd.isEmpty() && (deg != 0 || angleCmd == "0"))
      {
        btSend->rad = deg * M_PI / 180.0;

        // gemmer de ønskede værdier
        if (true /*xSemaphoreTake(dataMutex, 10 / portTICK_PERIOD_MS)*/)
        {
          pidData->desiredAngle = btSend->rad;
          btSend->hasReceivedAngle = true;
          // xSemaphoreGive(dataMutex);
        }

        // Udskriv modtaget vinkel eller ugyldig vinkel
        SerialBT.println("Modtaget vinkel: " + String(deg) + "° (" + String(btSend->rad, 3) + " rad)");
        Serial.println("Modtaget vinkel: " + String(deg) + "° (" + String(btSend->rad, 3) + " rad)");
      }
      else if (angleCmd == "start")
      {
        *started = true;
        pidData->startTime = esp_timer_get_time() * 1e-6;
        pidData->I = 0;
      }
      else if (angleCmd == "stop")
      {
        *started = false;
        ledcWrite(pidData->pwmChannel, 0); //dette fungere som analog write;
      }
      else
      {
        SerialBT.println("Ugyldig kommando: '" + angleCmd + "'");
        Serial.println("Ugyldig kommando: '" + angleCmd + "'");
      }
    }
  }
  else
  {
    static bool once = true;
    if (once)
    {
      Serial.println("Venter på Bluetooth-forbindelse...");
      once = false;
    }
  }

  // vTaskDelay(500 / portTICK_PERIOD_MS);
  //}
}

void btSendTask(struct BTSendData btSend, double actualAngle, struct PIDData pidData)
{
  if (SerialBT.hasClient())
  {
    /*
    // læser aktuelle data
    if ( xSemaphoreTake(dataMutex, 10 / portTICK_PERIOD_MS))
    {
      if (hasReceivedAngle)
      {
        angleToSend = btSend.angle;
        errorToSend = btSend.error;
      }
      //xSemaphoreGive(dataMutex);
    }
    */
    // printer den aktuelle vinkel og fejlen

    SerialBT.println("aktuel vinkel: " + String(actualAngle, 2) + "°");
    SerialBT.println("aktuel fejl: " + String((btSend.error * (180 / M_PI)), 2) + "°");
    SerialBT.println("PWM: " + String(pidData.PWMToMotor));
    SerialBT.println(" ");
  }
}

void SerialKomm()
{
  serial("tid");
  serial(";");
  serial("xakse");
  serial(";");
  serial("yakse");
  serial(";");
  serial("zakse");
  serial(";");
  serial("fejl");
  serial(";");
  serial("spendig");
  serial(";");
  serial("PWM");
  serial(";");
  serial("intigral");
  serial(";");
  serialln("Deltat");
}

void SerialKom(struct PIDData pidData, struct MagData myMagData)
{
  serial(millis());
  serial(";");
  serial(myMagData.magX);
  serial(";");
  serial(myMagData.magY);
  serial(";");
  serial(myMagData.magZ);
  serial(";");
  serial(pidData.error, 6);
  serial(";");
  serial(pidData.PIDOutput2, 6);
  serial(";");
  serial(pidData.PWMToMotor, 6);
  serial(";");
  serial(pidData.I, 6);
  serial(";");
  serial(pidData.dt, 6);
  serialln("");
}

// init struct for mag data
struct MagData myMagData;
struct BTSendData btSend;
struct PIDData pidData;

double ErrorAngleAndDirection(struct PIDData pidData, double *actualAngle, struct BTSendData *btSend)
{

  // Read the magnetometer
  readMagnetometer(&myMagData);

  // find the angle between the vector and the current angle
  double sensorX = myMagData.magX; // Sensor dataX!!!
  double sensorY = myMagData.magY; // Sensor dataY!!!
  // double sensorX = 0;
  // double sensorY = 5600;

  double dotproduct = pidData.desiredX * sensorX + pidData.desiredY * sensorY;

  double lenOfxy = sqrt(pow(sensorX, 2) + pow(sensorY, 2));

  double angle = acos(dotproduct / lenOfxy); // Formula acos((a ⋅ b)/(|a|*|b|)), but the length of the desired vector is 1

  double AactualAngle = acos(sensorX / lenOfxy);

  // Use the crossproduct to find the direction that is the shortest
  double crossproduct = sensorX * pidData.desiredY - sensorY * pidData.desiredX;

  if (-sensorY > 0)
  {
    *actualAngle = ((2 * M_PI) - AactualAngle) * (180 / M_PI);
  }
  else
  {
    *actualAngle = (AactualAngle) * (180 / M_PI);
  }

  if (crossproduct <= 0)
  {
    // digitalWrite(motorDirPin, HIGH);
    // Serial.println("Move the motor Counterclockwise, so the cubesat moves clockwise");
    btSend->error = angle;
    return angle;
  }
  else
  {
    // digitalWrite(motorDirPin, LOW);
    // Serial.println("Move the motor clockwise, so the cubesat moves counterclockwise");
    btSend->error = -angle;
    return -angle;
  }
}
int abe = 0;
int bitch = 0;
void setup()
{
  // delay(7000); // Wait for the serial monitor to open
  pinMode(motorDirPin, OUTPUT);
  pinMode(PWMPin, OUTPUT);
  ledcSetup(pidData.pwmChannel, 20000, pidData.resolution);
  ledcAttachPin(PWMPin, pidData.pwmChannel);
  pidData.lastError = 0;
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // Serial2 for the serial monitor
  Wire.begin(21, 22);                      // SDA, SCL pins for I2C
  SerialBT.begin(device_name);
  initMPU(); // Initialize the MPU6050
  initHMC(); // Initialize the HMC5883L
  // SerialKomm(); // Send the header to the serial monitor
  pidData.startTime = esp_timer_get_time() * 1e-6;
}

void loop()
{

  btReceiveTask(&btSend, &pidData, &started);

  while (started)
  {
    // put your main code here, to run repeatedly:

    // The difference in time since last time the PID ran
    pidData.currentTime = esp_timer_get_time() * 1e-6;
    pidData.dt = pidData.currentTime - pidData.startTime;
    pidData.startTime = esp_timer_get_time() * 1e-6;

    // Convert the desired angle to coordinates
    pidData.desiredX = cos(pidData.desiredAngle);
    pidData.desiredY = sin(pidData.desiredAngle);

    // Find the error and set the direction the motor need to spin in
    pidData.error = ErrorAngleAndDirection(pidData, &actualAngle, &btSend); // Find the error and set the direction the motor need to spin in
    // Serial.println(error, 6);

    // calculate the PID values based on the error (output in voltage):
    pidData.P = pidData.KP * pidData.error;
    pidData.I += pidData.KI * (pidData.error * pidData.dt);
    pidData.D = pidData.KD * ((pidData.error - pidData.lastError) / pidData.dt);

    // Convert voltage to pwm
    pidData.PIDOutput = pidData.P + pidData.I + pidData.D;

    if (pidData.PIDOutput > pidData.Vmaks)
    {
      pidData.PIDOutput2 = pidData.Vmaks;
    }
    else if (pidData.PIDOutput < -pidData.Vmaks)
    {
      pidData.PIDOutput2 = -pidData.Vmaks;
    }
    else
    {
      pidData.PIDOutput2 = pidData.PIDOutput;
    }

    if (pidData.PIDOutput != pidData.PIDOutput2 && pidData.error * pidData.PIDOutput >= 0)
    {
      pidData.I -= pidData.KI * (pidData.error * pidData.dt);
    }

    if (pidData.PIDOutput2 > 0)
    {
      digitalWrite(motorDirPin, HIGH);
    }
    else
    {
      digitalWrite(motorDirPin, LOW);
    }

    pidData.PWMToMotor = (abs(pidData.PIDOutput2) / pidData.Vmaks) * pidData.PWMmaks;

    ledcWrite(pidData.pwmChannel, pidData.PWMToMotor); //dette fungere som analog write

    pidData.lastError = pidData.error;

    btReceiveTask(&btSend, &pidData, &started); // Read the bluetooth data

    if (lastTime + 2000 < millis())
    {
      lastTime = millis();
      btSendTask(btSend, actualAngle, pidData); // Send the data to the bluetooth monitor
    }
    // SerialKom(pidData, myMagData); // Send the data to the serial monitor

    while (pidData.startTime + pidData.delayPeriod >= esp_timer_get_time() * 1e-6)
    {
    }
  }
}
