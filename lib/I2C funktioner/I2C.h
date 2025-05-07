#include <Arduino.h>
#include <Wire.h>
#include <TimeLib.h>

/**
 * @brief enum used for setting the oversampling settings of the BMP280 (oversampeling sampeles multiple times and averages the result)
 * @param SAMPLING_NONE No over-sampling
 * @param SAMPLING_X1 1x over-sampling
 * @param SAMPLING_X2 2x over-sampling
 * @param SAMPLING_X4 4x over-sampling
 * @param SAMPLING_X8 8x over-sampling
 * @param SAMPLING_X16 16x over-sampling
 */
enum sensor_sampling
{
    /** No over-sampling. */
    SAMPLING_NONE = 0b00000000,
    /** 1x over-sampling. */
    SAMPLING_X1 = 0b00000001,
    /** 2x over-sampling. */
    SAMPLING_X2 = 0b00000010,
    /** 4x over-sampling. */
    SAMPLING_X4 = 0b00000011,
    /** 8x over-sampling. */
    SAMPLING_X8 = 0b00000100,
    /** 16x over-sampling. */
    SAMPLING_X16 = 0b00000101
};

/**
 * @brief enum used for setting the power mode of the BMP280
 * @param MODE_SLEEP Sleep mode
 * @param MODE_FORCED Forced mode
 * @param MODE_NORMAL Normal mode
 * @param MODE_SOFT_RESET_CODE Software reset
 */
enum sensor_mode
{
    /** Sleep mode. */
    MODE_SLEEP = 0b00000000,
    /** Forced mode. */
    MODE_FORCED = 0b00000001,
    /** Normal mode. */
    MODE_NORMAL = 0b00000011,
    /** Software reset. */
    MODE_SOFT_RESET_CODE = 0b10110110
};

/**
 * @brief enum used for setting the IIR filter settings of the BMP280
 * @param FILTER_OFF No filtering
 * @param FILTER_X2 2x filtering
 * @param FILTER_X4 4x filtering
 * @param FILTER_X8 8x filtering
 * @param FILTER_X16 16x filtering
 */
enum sensor_filter
{
    /** No filtering. */
    FILTER_OFF = 0b00000000,
    /** 2x filtering. */
    FILTER_X2 = 0b00000001,
    /** 4x filtering. */
    FILTER_X4 = 0b00000010,
    /** 8x filtering. */
    FILTER_X8 = 0b00000011,
    /** 16x filtering. */
    FILTER_X16 = 0b00000100
};

/**
 * @brief enum used for setting the standby settings of the BMP280
 * @param STANDBY_MS_1 0.5 ms standby
 * @param STANDBY_MS_63 62.5 ms standby
 * @param STANDBY_MS_125 125 ms standby
 * @param STANDBY_MS_250 250 ms standby
 * @param STANDBY_MS_500 500 ms standby
 * @param STANDBY_MS_1000 1000 ms standby
 * @param STANDBY_MS_2000 2000 ms standby
 * @param STANDBY_MS_4000 4000 ms standby
 */
enum standby_duration
{
    /** 0.5 ms standby. */
    STANDBY_MS_1 = 0b00000000,
    /** 62.5 ms standby. */
    STANDBY_MS_63 = 0b00000001,
    /** 125 ms standby. */
    STANDBY_MS_125 = 0b00000010,
    /** 250 ms standby. */
    STANDBY_MS_250 = 0b00000011,
    /** 500 ms standby. */
    STANDBY_MS_500 = 0b00000100,
    /** 1000 ms standby. */
    STANDBY_MS_1000 = 0b00000101,
    /** 2000 ms standby. */
    STANDBY_MS_2000 = 0b00000110,
    /** 4000 ms standby. */
    STANDBY_MS_4000 = 0b00000111,
};

/**
 * @brief Struct to store the trimming parameters for the BMP280
 * @param dig_T1 The first trimming parameter for the temperature
 * @param dig_T2 The second trimming parameter for the temperature
 * @param dig_T3 The third trimming parameter for the temperature
 * @param dig_P1 The first trimming parameter for the pressure
 * @param dig_P2 The second trimming parameter for the pressure
 * @param dig_P3 The third trimming parameter for the pressure
 * @param dig_P4 The fourth trimming parameter for the pressure
 * @param dig_P5 The fifth trimming parameter for the pressure
 * @param dig_P6 The sixth trimming parameter for the pressure
 * @param dig_P7 The seventh trimming parameter for the pressure
 * @param dig_P8 The eigth trimming parameter for the pressure
 * @param dig_P9 The ninth trimming parameter for the pressure
 * @param t_fine The fine temperature as a global value
 */
struct trimming_parameters
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    int32_t t_fine; // t_fine carries fine temperature as global value
};

/**
 * @brief Struct to store the acceleration data
 * @param ax_raw The raw x-axis acceleration data
 * @param ay_raw The raw y-axis acceleration data
 * @param az_raw The raw z-axis acceleration data
 * @param accelX The x-axis acceleration data converted to m/s^2
 * @param accelY The y-axis acceleration data converted to m/s^2
 * @param accelZ The z-axis acceleration data converted to m/s^2
 */
struct AccelData
{
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;

    float accelX;
    float accelY;
    float accelZ;
};

/**
 * @brief Struct to store the altitude data
 * @param temperature The temperature of the sensors
 * @param pressure The pressure at the sensors location
 * @param altitude The altitude above the sea at the sensors location
 */
struct AltitudeData
{
    float temperature;
    float pressure;
    float altitude;
};

/**
 * @brief Struct to store the gyroscope data
 * @param gx_raw The raw x-axis gyroscope data
 * @param gy_raw The raw y-axis gyroscope data
 * @param gz_raw The raw z-axis gyroscope data
 * @param gyroX The x-axis gyroscope data converted to degrees per second
 * @param gyroY The y-axis gyroscope data converted to degrees per second
 * @param gyroZ The z-axis gyroscope data converted to degrees per second
 */
struct GyroData
{
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;

    double gyroX;
    double gyroY;
    double gyroZ;
};

/**
 * @brief Struct to store the magnetometer data
 * @param mx_raw The raw x-axis magnetometer data
 * @param my_raw The raw y-axis magnetometer data
 * @param mz_raw The raw z-axis magnetometer data
 * @param magX The x-axis magnetometer data converted to Gauss
 * @param magY The y-axis magnetometer data converted to Gauss
 * @param magZ The z-axis magnetometer data converted to Gauss
 */
struct MagData
{
    int16_t mx_raw;
    int16_t my_raw;
    int16_t mz_raw;

    double magX;
    double magY;
    double magZ;
};

#define BMP_ADDRESS 0x77
#define MPU_ADDRESS 0x68
#define HMC_ADDRESS 0x1E
//#define TEST
#define CONVERT
//#define SETTINGS
#define RXD2 16  // RX2 pin
#define TXD2 17  // TX2 pin

/**
 * @brief Sets the sampling settings for the BMP280
 * @param mode The Power mode of the sensor
 * @param tempSampling The amount of sampels taken for calculating the average temperature reading (oversampling)
 * @param pressSampling The amount of sampels taken for calculating the average pressure reading (oversampling)
 * @param filter The IIR filtering mode to apply (if any)
 * @param duration The time btween each reading/measurement of pressure and temperature
 */
void setSamplingSettings(sensor_mode mode,
                         sensor_sampling tempSampling,
                         sensor_sampling pressSampling,
                         sensor_filter filter,
                         standby_duration duration
);

/**
 * @brief Initializes the BMP280 with the given settings
 * @param trimming_parameters The struct to store the trimming parameters
 * @param mode The Power mode of the sensor
 * @param tempSampling The amount of sampels taken for calculating the average temperature reading (oversampling)
 * @param pressSampling The amount of sampels taken for calculating the average pressure reading (oversampling)
 * @param filter The IIR filtering mode to apply (if any)
 * @param duration The time btween each reading/measurement of pressure and temperature
 */
void initBMP(struct trimming_parameters *trimming_parameters,
                sensor_mode mode,
                sensor_sampling tempSampling,
                sensor_sampling pressSampling,
                sensor_filter filter,
                standby_duration duration
);

/**
 * @brief Scans the I2C bus for devices and prints the addresses of the devices found
 */
void scanForAdress();

/**
 * @brief Initializes the MPU6050
 */
void initMPU();

void initHMC();

/**
 * @brief Reads the acceleration from the MPU6050 and stores it in the AccelData struct
 * @param AccelData The struct to store the acceleration data
 */
void accData(struct AccelData *AccelData);

/**
 * @brief Reads the gyroscope data from the MPU6050 and stores it in the GyroData struct
 * @param GyroData The struct to store the gyroscope data
 */
void gyroData(struct GyroData *GyroData);

/**
 * @brief Reads the temperature from the BMP280 and stores it in the AltitudeData struct
 * @param AltitudeData The struct to store data from the BMP280
 * @param trimming_parameters The struct containing the trimming parameters for the BMP280
 */
void tempData(struct AltitudeData *AltitudeData, struct trimming_parameters *trimming_parameters);

/**
 * @brief Reads the pressure from the BMP280 and stores it in the AltitudeData struct
 * @param AltitudeData The struct to store data from the BMP280
 * @param trimming_parameters The struct containing the trimming parameters for the BMP280
 */
void preasureData(struct AltitudeData *AltitudeData, struct trimming_parameters *trimming_parameters);

/**
 *@brief Calculates the altitude based on the pressure at the sensors curent location and the sea level pressure
 *@param SeaLevelhPa The pressure at sea level in hPa
 *@param AltitudeData The struct to store the altitude data
 */
void readAltitude(float seaLevelhPa, struct AltitudeData *AltitudeData);

/**
 *@brief measures the srength of the magnetic field in the x, y and z direction relativ to the sensor
 */
void readMagnetometer(struct MagData *MagData);