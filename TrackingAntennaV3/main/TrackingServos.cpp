/**
 * @file   TrackingServos.cpp
 * @Author Serj Babayan (WARG)
 * @date   January 7, 2017
 */
#include "Arduino.h"
#include "TrackingServos.hpp"
#include <Servo.h>

static Servo pan_servo;
static Servo tilt_servo;

void initializeServos(void){
    pan_servo.attach(PAN_SERVO_PIN, PAN_SERVO_PWM_MIN, PAN_SERVO_PWM_MAX);
    tilt_servo.attach(TILT_SERVO_PIN, TILT_SERVO_PWM_MIN, TILT_SERVO_PWM_MAX);
}

void pan(int degrees){
    int output = map(degrees, -90, 90, PAN_SERVO_PWM_MIN, PAN_SERVO_PWM_MAX);
    pan_servo.writeMicroseconds(output);
}

void tilt(int degrees){
    degrees = constrain(degrees, TILT_ANGLE_MIN_LIMIT, TILT_ANGLE_MAX_LIMIT);

    int output = map(degrees, 0, 90, TILT_SERVO_PWM_MAX, TILT_SERVO_PWM_MIN); // reversed on purpose
    tilt_servo.writeMicroseconds(output);
}

