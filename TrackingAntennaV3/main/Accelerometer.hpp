/**
 * @file   Magnetometer.hpp
 * @Author Michael Lenover (WARG)
 * @date   January 25, 2017
 * @brief  
 * @see https://www.arduino.cc/en/Reference/Ethernet
 */

#ifndef ACCELEROMETER
#define ACCELEROMETER

#define COMPASS_ID 12345

//Accelerometer data struct
typedef struct {
  float x;
  float y;
  float z;
} Accelerometer;

void initAccelerometer();

#endif
