/**
 * @file   Accelerometer.hpp
 * @Author Michael Lenover (WARG)
 * @date   January 25, 2017
 * @brief  Accelerometer initialization and data aquisition functions.
 * Uses Adafruit LSM303 Unified Acceleromter/Magnetometer for accelerometer data.
 * Connects to Arduino using I2C communication
 * Connect pin SDA on accelerometer to pin 20
 * Connect pin SCL on accelerometer to pin 21
 * Connect power to 3.3V, ground to ground
 * @see https://www.arduino.cc/en/Reference/Wire
 * @see https://www.adafruit.com/product/1120
 */

#ifndef ACCELEROMETER
#define ACCELEROMETER

/**
  * Arbirtrary ID used to identify sensor if additional sensors are added
  * Required for Adafruit library
  * Returns true if successfully connected, false otherwise
  */
#define ACCELEROMETER_ID 12345

/**
 * Accelerometer data struct 
 * Contains x,y,z components of vector pointing in direction of gravity
 * Units in m/s^2
 */
typedef struct {
  float x;
  float y;
  float z;
} AccelerometerData;

extern AccelerometerData accel_data;

/**
 * Initializes accelerometer
 * Outputs debug messages if unable to connect or initialize
 */
bool initAccelerometer(void);//Comment


/**
 * Retrieves acceleration value from accelerometer 
 * Used to determine direction of gravity
 * Values in m/s^2 for each direction (x,y,z)
 */
void parseAcceleration();

#endif

