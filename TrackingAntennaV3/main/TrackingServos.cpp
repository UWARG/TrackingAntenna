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

void pan(int pwm){
    pwm = constrain(pwm, PAN_SERVO_PWM_MIN, PAN_SERVO_PWM_MAX);
    pan_servo.writeMicroseconds(pwm);
}

void tilt(int pwm){
    pwm = constrain(pwm, TILT_SERVO_PWM_MIN, TILT_SERVO_PWM_MAX);
    tilt_servo.writeMicroseconds(pwm);
}

