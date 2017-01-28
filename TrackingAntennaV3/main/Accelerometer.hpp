/**
 * @file   Magnetometer.hpp
 * @Author Michael Lenover (WARG)
 * @date   January 25, 2017
 * @brief  Accelerometer initialization and data aquisition functions.
 * Uses Adafruit LSM303 Unified Acceleromter/Magnetometer for accelerometer data.
 * @see https://www.adafruit.com/product/1120
 */

#ifndef ACCELEROMETER
#define ACCELEROMETER


//Arbirtrary ID used to identify sensor if additional sensors are added
#define COMPASS_ID 12345

//Accelerometer data struct
typedef struct {
  float x;
  float y;
  float z;
} Accelerometer;

void initAccelerometer();

void getGravity();

#endif
