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

void pan(float deg){
    int tenth_deg = round(deg*10);
    int output = map(tenth_deg, -PAN_ANGLE_LIMIT, PAN_ANGLE_LIMIT, PAN_SERVO_PWM_MIN, PAN_SERVO_PWM_MAX);
    pan_servo.writeMicroseconds(output);
}

void tilt(float deg){
    int tenth_deg = round(deg*10);
    tenth_deg = constrain(tenth_deg, TILT_ANGLE_MIN_LIMIT, TILT_ANGLE_MAX_LIMIT);

    int output = map(tenth_deg, 0, 900, TILT_SERVO_PWM_MAX, TILT_SERVO_PWM_MIN); // reversed on purpose
    tilt_servo.writeMicroseconds(output);
}

