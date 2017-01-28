/**
 * @file   Magnetometer.hpp
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

//Arbirtrary ID used to identify sensor if additional sensors are added
#define COMPASS_ID 12345

/**
 * Accelerometer data struct 
 * Contains x,y,z components of vector pointing in direction of gravity
 */
typedef struct {
  float x;
  float y;
  float z;
} AccelerometerData;

void initAccelerometer();

void getGravity();

#endif

