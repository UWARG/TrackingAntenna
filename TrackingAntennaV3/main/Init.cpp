/**
 * @file   Accelerometer.cpp
 * @Author Michael Lenover (WARG)
 * @date   January 28, 2017
 */

#include "Init.hpp"
#include "Accelerometer.hpp"
#include "TrackingServos.hpp"
#include <Arduino.h>
#include <math.h>

Init initialize;

void calibrateTilt(){
  //Set antenna to 0 degrees, as determined by servo encoder
  tilt(0);

  delay(2000); // wait 

  float angle = 0;
  for (int i = 0; i < 5; i++) {
    parseAcceleration();
    angle += atan2(accel_data.x, accel_data.z) * (PI/180);
  }
  initialize.tilt_offset = -angle / 5.f;
}

