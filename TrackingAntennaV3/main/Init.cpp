/**
 * @file   Accelerometer.cpp
 * @Author Michael Lenover (WARG)
 * @date   January 28, 2017
 */

#include "Init.hpp"
#include "Util.hpp"
#include "Accelerometer.hpp"
#include "Magnetometer.hpp"
#include "TrackingServos.hpp"
#include "Logger.hpp"
#include <Arduino.h>
#include <math.h>

#define SAMPLES 10
#define WAIT 4000

Init initialize;

void calibrateTilt(){
  debug("Calibrating tilt servo...");
  //Set antenna to 0 degrees, as determined by servo encoder
  tilt(TILT_SERVO_PWM_MAX); // max because reversed

  delay(WAIT); // wait for antenna to stabilize

  // average accelerometer readings
  float angle = 0;
  for (int i = 0; i < SAMPLES; i++) {
    parseAcceleration();
    angle += accel_data.pitch;
    delay(100);
  }
  initialize.tilt_min = angle / SAMPLES;

  // tilt to max limit
  tilt(TILT_SERVO_PWM_MIN);

  delay(WAIT);

  angle = 0;
  for (int i = 0; i < SAMPLES; i++) {
    parseAcceleration();
    angle += accel_data.pitch;
    delay(100);
  }
  initialize.tilt_max = angle / SAMPLES;
}

// calibrate pan using local magnetic declination angle
void calibratePan(float dec) {
  debug("Calibrating pan servo...");
  pan(PAN_SERVO_PWM_MIN);

  delay(WAIT);

  // average compass readings
  float angle = 0;
  for (int i = 0; i < SAMPLES; i++) {
    getMagneticNorth();
    angle += mag_data.heading;
    delay(100);
  }
  initialize.heading_min = (angle / SAMPLES) + dec;

  // tilt to max limit
  pan(PAN_SERVO_PWM_MIN);

  delay(WAIT);

  angle = 0;
  for (int i = 0; i < SAMPLES; i++) {
    getMagneticNorth();
    angle += mag_data.heading;
    delay(100);
  }
  initialize.heading_max = (angle / SAMPLES) + dec;
}

void worldPan(float heading) {
  int pwm = round(mapf(heading, initialize.heading_min, initialize.heading_max, PAN_SERVO_PWM_MIN, PAN_SERVO_PWM_MAX));
  pan(pwm);
}

void worldTilt(float pitch) {
  int pwm = round(mapf(pitch, initialize.tilt_min, initialize.tilt_max, TILT_SERVO_PWM_MAX, TILT_SERVO_PWM_MIN)); // reversed on purpose (gearing)
  tilt(pwm);
}


