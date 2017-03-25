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

/**
Arbirtrary ID used to identify sensor if additional sensors are added
Required for Adafruit library
*/
#define MAGNETOMETER_ID 12345

#define CALIBRATE_MAGNETOMETER false

//Magnetometer data struct
typedef struct {
  float x;
  float y;
  float z;
  float heading;
} MagnetometerData;

extern MagnetometerData mag_data;

/**
 * Initializes magnetometer
 * Outputs debug messages if unable to connect or initialize
 */
bool initMagnetometer();

/**
 * Retrieves magnetic field value from magnetometer 
 * Used to determine direction of magnetic north
 * Values in micro Teslas for each direction (x,y,z)
 */
void getMagneticNorth();

#endif
