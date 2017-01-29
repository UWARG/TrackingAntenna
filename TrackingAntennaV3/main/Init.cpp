/**
 * @file   Accelerometer.cpp
 * @Author Michael Lenover (WARG)
 * @date   January 28, 2017
 */

#include "Init.hpp"
#include "Accelerometer.hpp"
#include "TrackingServos.hpp"
#include <Arduino.h>
#include <Math.h>

Init initialize;

void calibrateTilt(){
  //Set antenna to 0 degrees, as determined by servo encoder
  tilt(0);

  delay(2000);

  parseAcceleration();

  initialize.tilt_offset = atan2(accel_data.z, accel_data.x);
}

