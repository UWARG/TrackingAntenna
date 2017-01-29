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
  float tilt_offset;
} Init;

extern Init initialize;
#endif
