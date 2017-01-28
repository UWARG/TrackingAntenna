/*
 * @Author Michael Lenover (WARG)
 * @date   January 5, 2017
 * @brief  Magnetometer initialization functions
 */

#ifndef MAGNETOMETER
#define MAGNETOMETER

#define COMPASS_ID 12345

#define CALIBRATE_MAGNETOMETER false

//Magnetometer data struct
typedef struct {
  float x;
  float y;
  float z;
  float angle;
} Magnetometer;

void initMagnetometer();

#endif
