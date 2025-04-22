#include <I2C.h>

/**
 * @link for the datasheet of the BMP280
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
 * @link for the datasheet of the MPU6050
 * https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 * @link for the I2C register map of the MPU6050
 * https://www.invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 * @link for the datasheet of the HMC5883L
 * https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
 * @link for finding magnetic field strength at points on earth's surface
 * https://www.magnetic-declination.com/
 * @link for af konvertere enheder (tesla til gauss)
 * https://www.kjmagnetics.com/magnetic-unit-converter.asp?srsltid=AfmBOopKtH_5hIZi36xgF-83J_Lwkvn93AuMc-fwOuHIk1iahbT6r5oa
 */

struct AccelData myAccelData;
struct GyroData myGyroData;

long long abe = 0;

void setup()
{

  pinMode(13, INPUT_PULLUP);
  while (digitalRead(13) == HIGH)
  {
    // Wait for the button to be pressed
  }

  delay(3000);
  Serial.begin(115200); // USB Serial for PC monitoring
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Wire.begin(21, 22);

  Serial.println();

  scanForAdress();
  initMPU();
  initHMC();

  Serial2.print("Time");
  Serial2.print(",");
  Serial2.print("X-axis");
  Serial2.print(",");
  Serial2.print("Y-axis");
  Serial2.print(",");
  Serial2.println("Z-axis");

  readMagnetometer();

  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(2, HIGH);
  digitalWrite(4, LOW);
  abe = millis();
}

void loop()
{
  readMagnetometer();

  // accData(&myAccelData);
  // gyroData(&myGyroData);
  // Serial2.println();

  if (millis() > abe + 120000)
  {
    digitalWrite(2, HIGH);
    digitalWrite(4, HIGH);
    while (1)
    {
      readMagnetometer();
      if (millis() > abe + 180000)
      {
        while (1)
        {
        }
      }
    }
  }
}
