#include <I2C.h>

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

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  initBMP280(&myTrimmingParameters,
             MODE_NORMAL,   // Opperating mode
             SAMPLING_X1,   // Temp oversampling
             SAMPLING_X4,   // Pressure oversampling
             FILTER_X16,    //  IIR Filtering
             STANDBY_MS_1); // Standby time
  initMPU6050();
}

void loop()
{
  accData(&myAccelData);
  gyroData(&myGyroData);
  tempData(&myAltitudeData, &myTrimmingParameters);
  preasureData(&myAltitudeData, &myTrimmingParameters);
  readAltitude(1026.6, &myAltitudeData);
  Serial.println();
  delay(5000);
}
