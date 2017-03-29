/**
 * @file   Accelerometer.hpp
 * @Author Michael Lenover (WARG)
 * @date   January 28, 2017
 * @brief  Tracking antenna initialization functions.
 * Uses accelerometer value to determine tilt angle offset
 * Uses magnetometer value to determine pan angle offset
 */

#ifndef INIT
#define INIT

typedef struct {
  float tilt_min, tilt_max;
  float heading_min, heading_max;
} Init;

extern Init initialize;

void calibrateTilt();

void calibratePan(float dec);


void worldPan(float heading);
void worldTilt(float pitch);

#endif

