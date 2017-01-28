/*
 * @Author Michael Lenover (WARG)
 * @date   January 5, 2017
 * @brief  Magnetometer initialization and data retrieval functions
 * Uses Adafruit LSM303 Unified Acceleromter/Magnetometer for magnetometer data.
 * Connects to Arduino using I2C communication
 * Connect pin SDA on accelerometer to pin 20
 * Connect pin SCL on accelerometer to pin 21
 * Connect power to 3.3V, ground to ground
 */

#ifndef MAGNETOMETER
#define MAGNETOMETER


//Arbirtrary ID used to identify sensor if additional sensors are added
#define COMPASS_ID 12345

#define CALIBRATE_MAGNETOMETER false

//Magnetometer data struct
typedef struct {
  float x;
  float y;
  float z;
  float angle;
} MagnetometerData;

extern volatile MagnetometerData magnetometer_data;

bool initMagnetometer();

void getMagneticNorth();

#endif
