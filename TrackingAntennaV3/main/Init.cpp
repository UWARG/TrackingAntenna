/**
 * @file   Accelerometer.cpp
 * @Author Michael Lenover (WARG)
 * @date   January 28, 2017
 */

#include "Init.hpp"
#include "Accelerometer.hpp"
#include "Magnetometer.hpp"
#include "TrackingServos.hpp"
#include <Arduino.h>
#include <math.h>

#define SAMPLES 10

Init initialize;

void calibrateTilt(){
  //Set antenna to 0 degrees, as determined by servo encoder
  tilt(TILT_ANGLE_MIN_LIMIT / 10.f);

  delay(5000); // wait for antenna to stabilize

  // average accelerometer readings
  float angle = 0;
  for (int i = 0; i < SAMPLES; i++) {
    parseAcceleration();
    angle += accel_data.pitch;
    delay(50);
  }
  initialize.tilt_min = angle / SAMPLES;

  // tilt to max limit
  tilt(TILT_ANGLE_MAX_LIMIT / 10.f);

  delay(5000);

  angle = 0;
  for (int i = 0; i < SAMPLES; i++) {
    parseAcceleration();
    angle += accel_data.pitch;
    delay(50);
  }
  initialize.tilt_max = angle / SAMPLES;

}

void calibratePan() {
  pan(-120);

  delay(5000);

  // average compass readings
  float angle = 0;
  for (int i = 0; i < SAMPLES; i++) {
    getMagneticNorth();
    angle += mag_data.heading;
    delay(100);
  }
  initialize.heading_min = angle / SAMPLES;

  // tilt to max limit
  pan(120);

  delay(5000);

  angle = 0;
  for (int i = 0; i < SAMPLES; i++) {
    getMagneticNorth();
    angle += mag_data.heading;
    delay(100);
  }
  initialize.heading_max = angle / SAMPLES;

}

