#include "arduino.h"
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
  double desiredX = 0;
  double desiredY = 0;
  double sensorX = 0;
  double sensorY = 0;

  double dotproduct;

  double lenOfxy;

  double angle;

  double angleTo0;

  double crossproduct;

  // PID-controller Constants and variables
  const double KP = 5.73, KI = 2, KD = 0;

  double P, I = 0, D;
  double sensorAngle;
  double error = 0;
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

// opret navm og bluetooth seriel kommunikation
String device_name = "ESD-213-esp32";
BluetoothSerial SerialBT;

// struct til kopi af data
struct BTSendData
{
  double rad = 0;
  double angle = 0;
  double error = 0;
  int PWMfromMotor = 0;
  bool started = true;
};

// Boolean to check if the program is started
bool started = true;

// The mutexes are initialised.
SemaphoreHandle_t radBTMutex = NULL;
SemaphoreHandle_t angleBTMutex = NULL;
SemaphoreHandle_t errorBTMutex = NULL;
SemaphoreHandle_t PWMfromMotorBTMutex = NULL;
SemaphoreHandle_t startedBTMutex = NULL;

// init struct for mag data
struct MagData myMagData;
struct BTSendData btSend;
struct PIDData pidData;

//RecibeTaskVariables
double deg;
String angleCmd;

void btReceiveTask(void *pv0)
{
  while (1)
  {

    if (SerialBT.hasClient())
    {
      if (SerialBT.available())
      {
        // Læs op til newline og fjern mellemrum
        angleCmd = SerialBT.readStringUntil('\n');
        angleCmd.trim();

        // Konverter til double
        deg = angleCmd.toDouble();

        // Tjek for gyldigt tal (inkl. "0")
        if (!angleCmd.isEmpty() && (deg != 0 || angleCmd == "0"))
        {
          if (radBTMutex != NULL)
          {
            if (xSemaphoreTake(radBTMutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
              btSend.rad = deg * M_PI / 180.0;
              // Udskriv modtaget vinkel eller ugyldig vinkel
              SerialBT.println("Modtaget vinkel: " + String(deg) + "° (" + String(btSend.rad, 3) + " rad)");
              xSemaphoreGive(radBTMutex);
            }
          }
        }
        else if (angleCmd == "start")
        {
          if (startedBTMutex != NULL)
          {
            if (xSemaphoreTake(startedBTMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
              btSend.started = true;
              xSemaphoreGive(startedBTMutex);
            }
          }
        }
        else if (angleCmd == "stop")
        {
          if (startedBTMutex != NULL)
          {
            if (xSemaphoreTake(startedBTMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
              btSend.started = false;
              xSemaphoreGive(startedBTMutex);
            }
          }
        }
        else
        {
          SerialBT.println("Ugyldig kommando: '" + angleCmd + "'");
          SerialBT.println("Prøv at indtaste en vinkel i grader eller 'start' og 'stop'");
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

void btSendTask(void *pv1)
{
  while (1)
  {
    if (SerialBT.hasClient())
    {
      if (angleBTMutex != NULL)
      {
        if (xSemaphoreTake(angleBTMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
          SerialBT.println("aktuel vinkel: " + String(btSend.angle, 2) + "°");
          xSemaphoreGive(angleBTMutex);
        }
      }
      if (errorBTMutex != NULL)
      {
        if (xSemaphoreTake(errorBTMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
          SerialBT.println("aktuel fejl: " + String((btSend.error * (180 / M_PI)), 2) + "°");
          xSemaphoreGive(errorBTMutex);
        }
      }
      if (PWMfromMotorBTMutex != NULL)
      {
        if (xSemaphoreTake(PWMfromMotorBTMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
          SerialBT.println("PWM: " + String(btSend.PWMfromMotor));
          xSemaphoreGive(PWMfromMotorBTMutex);
        }
      }

      SerialBT.println(" ");
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void PIDMotorTask(void *pv2)
{
  while (1)
  {

    // Mutex for the btsend data and check if its started or not.
    if (startedBTMutex != NULL)
    {
      if (xSemaphoreTake(startedBTMutex, 0) == pdTRUE)
      {
        started = btSend.started;
        xSemaphoreGive(startedBTMutex);
      }
    }

    if (!started)
    {
      ledcWrite(pidData.pwmChannel, 0);
      pidData.I = 0;
      pidData.startTime = esp_timer_get_time() * 1e-6;
    }
    if (started)
    {
      // The difference in time since last time the PID ran
      pidData.currentTime = esp_timer_get_time() * 1e-6;
      pidData.dt = pidData.currentTime - pidData.startTime;
      pidData.startTime = esp_timer_get_time() * 1e-6;

      if (radBTMutex != NULL)
      {
        if (xSemaphoreTake(radBTMutex, 0) == pdTRUE)
        {
          pidData.desiredAngle = btSend.rad;
          xSemaphoreGive(radBTMutex);
        }
      }
      // Convert the desired angle to coordinates
      pidData.desiredX = cos(pidData.desiredAngle);
      pidData.desiredY = sin(pidData.desiredAngle);

      // Read the magnetometer
      readMagnetometer(&myMagData);

      // Find the angle between the vector and the current angle
      pidData.sensorX = myMagData.magX; // Data from magnetometer dataX!!!
      pidData.sensorY = myMagData.magY; // Data from magnetometer dataY!!!

      pidData.dotproduct = pidData.desiredX * pidData.sensorX + pidData.desiredY * pidData.sensorY;

      pidData.lenOfxy = sqrt(pow(pidData.sensorX, 2) + pow(pidData.sensorY, 2));

      pidData.angle = acos(pidData.dotproduct / pidData.lenOfxy); // Formula acos((a ⋅ b)/(|a|*|b|)), but the length of the desired vector is 1

      pidData.angleTo0 = acos(pidData.sensorX / pidData.lenOfxy);

      // Use the crossproduct to find the direction that is the shortest
      pidData.crossproduct = pidData.sensorX * pidData.desiredY - pidData.sensorY * pidData.desiredX;
      if (angleBTMutex != NULL)
      {
        if (xSemaphoreTake(angleBTMutex, 0) == pdTRUE)
        {
          if (-pidData.sensorY > 0)
          {
            btSend.angle = ((2 * M_PI) - pidData.angleTo0) * (180 / M_PI);
          }
          else
          {
            btSend.angle = (pidData.angleTo0) * (180 / M_PI);
          }
          xSemaphoreGive(angleBTMutex);
        }
      }

      if (pidData.crossproduct <= 0)
      {
        if (errorBTMutex != NULL)
        {
          if (xSemaphoreTake(errorBTMutex, 0) == pdTRUE)
          {
            btSend.error = pidData.angle;
            xSemaphoreGive(errorBTMutex);
          }
          pidData.error = pidData.angle;
        }
      }
      else
      {
        if (errorBTMutex != NULL)
        {
          if (xSemaphoreTake(errorBTMutex, 0) == pdTRUE)
          {
            btSend.error = -pidData.angle;
            xSemaphoreGive(errorBTMutex);
          }
          pidData.error = -pidData.angle;
        }
      }

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
      if (PWMfromMotorBTMutex != NULL)
      {
        if (xSemaphoreTake(PWMfromMotorBTMutex, 0) == pdTRUE)
        {
          btSend.PWMfromMotor = pidData.PWMToMotor;
          xSemaphoreGive(PWMfromMotorBTMutex);
        }
      }

      ledcWrite(pidData.pwmChannel, pidData.PWMToMotor); // dette fungere som analog write

      pidData.lastError = pidData.error;
    }
    vTaskDelay(pdMS_TO_TICKS(75));
  }
}

void SerialStartKom()
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

void setup()
{

  pinMode(motorDirPin, OUTPUT);
  pinMode(PWMPin, OUTPUT);
  ledcSetup(pidData.pwmChannel, 10000, pidData.resolution);
  ledcAttachPin(PWMPin, pidData.pwmChannel);
  pidData.lastError = 0;
  Serial.begin(115200);

  Serial2.begin(9600, SERIAL_8N1, 16, 17); // Serial2 for the serial monitor
  Wire.begin(21, 22);                      // SDA, SCL pins for I2C
  SerialBT.begin(device_name);
  initMPU(); // Initialize the MPU6050
  initHMC(); // Initialize the HMC5883L
             // SerialStartKom(); // Send the header to the serial monitor

  radBTMutex = xSemaphoreCreateMutex();
  angleBTMutex = xSemaphoreCreateMutex();
  errorBTMutex = xSemaphoreCreateMutex();
  PWMfromMotorBTMutex = xSemaphoreCreateMutex();
  startedBTMutex = xSemaphoreCreateMutex();

  if (startedBTMutex == NULL || PWMfromMotorBTMutex == NULL || errorBTMutex == NULL || angleBTMutex == NULL || radBTMutex == NULL)
  {
    Serial.println("En eller flere mutexes blev ikke oprettet korrekt.");
    while (1)
    {
    } // Stopper programmet, hvis der er fejl
  }

  xTaskCreatePinnedToCore(
      btSendTask,   // Task funktion
      "btSendTask", // Navn
      8192,         // Stack størrelse
      NULL,         // Parametre
      1,            // Prioritet
      NULL,         // Task handle
      0             // Core
  );

  xTaskCreatePinnedToCore(
      btReceiveTask,   // Task funktion
      "btReceiveTask", // Navn
      8192,            // Stack størrelse
      NULL,            // Parametre
      2,               // Prioritet
      NULL,            // Task handle
      0                // Core
  );

  xTaskCreatePinnedToCore(
      PIDMotorTask,   // Task funktion
      "PIDMotorTask", // Navn
      8192,           // Stack størrelse
      NULL,           // Parametre
      3,              // Prioritet
      NULL,           // Task handle
      0               // Core
  );

  // Start FreeRTOS Scheduler
  vTaskStartScheduler();
  Serial.printf("Free heap: %d\n", esp_get_free_heap_size());
}
void loop()
{
}
