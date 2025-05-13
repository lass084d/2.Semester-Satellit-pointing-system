#include "BluetoothSerial.h"
#include "esp_timer.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif


// opret navm og bluetooth seriel kommunikation
String device_name = "ESP32-BT";
BluetoothSerial SerialBT;

//pins
const int motorPin1 = 22;
const int motorPin2 = 23;
const int PWMPin = 21;

//PID constants 
const double KP = 1e-6;
const double KI = 1.3e-8;
const double KD = 9e-6;

double P, I = 0, D;
double sensorAngle;
double error;
double lastError;
double PIDOutput;

//Time
double dt;
double startTime = 0;
double currentTime;

//Set the desired angle (radians) and define desired X and Y koordinates
double desiredAngle;
double desiredX;
double desiredY;

//Pwm converting values
double Vmaks = 9;
double PWMmaks = 4095;
double PWMToMotor;

// gem IMU-data
struct Magdata myMagData;

// struct til kopi af data
struct BTSendData {
  double angle;
  double error;
  
};

BTSendData btSend;


bool hasReceivedAngle = false;

// global mutex
SemaphoreHandle_t dataMutex;

void btReceiveTask(void *pvParameters) {
  while (1) {
    if (SerialBT.hasClient()) {
      if (SerialBT.available()) {
        // Læs op til newline og fjern mellemrum
        String angleCmd = SerialBT.readStringUntil('\n');
        angleCmd.trim();

        // Konverter til double
        double deg = angleCmd.toDouble();

        // Tjek for gyldigt tal (inkl. "0")
        if (!angleCmd.isEmpty() && (deg != 0 || angleCmd == "0")) {
          double rad = deg * M_PI / 180.0;
          
          // gemmer de ønskede værdier
          if (xSemaphoreTake(dataMutex, 10 / portTICK_PERIOD_MS)) {
            desiredAngle = rad;
            desiredX = cos(desiredAngle);
            desiredY = sin(desiredAngle);
            hasReceivedAngle = true;
            xSemaphoreGive(dataMutex);
          }

          // Udskriv modtaget vinkel eller ugyldig vinkel
          SerialBT.println("Modtaget vinkel: " + String(deg) + "° (" + String(rad, 3) + " rad)");
          Serial.println("Modtaget vinkel: " + String(deg) + "° (" + String(rad, 3) + " rad)");
        } else {
          SerialBT.println("Ugyldig vinkel: '" + angleCmd + "'");
          Serial.println("Ugyldig vinkel: '" + angleCmd + "'");
        }
      }
    } else {
      static bool once = true;
      if (once) {
        Serial.println("Venter på Bluetooth-forbindelse...");
        once = false;
      }
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);  // mindre delay = hurtigere respons
  }
}

/* IKKE FÆRDIG
void controlLoopTask (void *pvParameters) {
  double sensorX;
  double sensorY;

  while (1) {
    if(xSemaphoreTake(dataMutex, 20/ portTICK_PERIOD_MS)) {
      readMagnetometer(&myMagData);
      sensorX = myMagData.x;
      sensorY = myMagData.y;

      double dotproduct = desiredx * sensorX + desiredy * sensorY;

      double lenOfxy = sqrt(pow(sensorX, 2) + pow(sensorY, 2));
     
      double angle = acos(dotproduct / lenOfxy); // Formula acos((a ⋅ b)/(|a|*|b|)), but the length of the desired vector is 1

      btSend.angle = angle;
      btSend.error = desiredAngle - angle;

      // Use the crossproduct to find the direction that is the shortest
      double crossproduct = sensorX * desiredy - sensorY * desiredx;

      xSemaphoreGive(dataMutex);
    }

  }
}
*/
void btSendTask(void *pvParameters) {
  
  double angleToSend;
  double errorToSend;

  while(1) {
    if (SerialBT.hasClient()) {
      // læser aktuelle data
      if(xSemaphoreTake(dataMutex, 10/portTICK_PERIOD_MS)) {
        if (hasReceivedAngle) {
          angleToSend = btSend.angle;
          errorToSend = btSend.error;
        }
          xSemaphoreGive(dataMutex);        
      }
       // printer den aktuelle vinkel og fejlen 
          SerialBT.println("aktuel vinkel:" + String(angleToSend,2) + "°");
          SerialBT.println("aktuel fejl:" + String(errorToSend,2) + "°");
    }

  vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
 } 


void setup() {
  Serial.begin(115200);
  delay(1000);
  SerialBT.begin(device_name);
  

  // opret mutex
  dataMutex = xSemaphoreCreateMutex();
  

  xTaskCreatePinnedToCore(
    btReceiveTask,
    "BT Receive",
    4096,
    NULL,
    2,
    NULL,
    0
  ); 

  xTaskCreatePinnedToCore(
    btSendTask,
    "BT Send",
    4096,
    NULL,
    1,
    NULL,
    0
  );
  
  /*
  xTaskCreatePinnedToCore(
    controlLoopTask,
    "ControlLoop",
    4096,
    NULL,
    2,
    NULL,
    1,
  );
*/

}

void loop() {
  // put your main code here, to run repeatedly:

}
