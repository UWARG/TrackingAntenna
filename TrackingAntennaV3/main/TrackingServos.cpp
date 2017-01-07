/**
 * @file   TrackingServos.cpp
 * @Author Serj Babayan (WARG)
 * @date   January 7, 2017
 */
#include "TrackingServos.hpp"
#include <Servo.h>

static Servo pan_servo;
static Servo tilt_servo;

void initializeServos(void){
    pan_servo.attach(PAN_SERVO_PIN, PAN_SERVO_PWM_MIN, PAN_SERVO_PWM_MAX);
    tilt_servo.attach(TILT_SERVO_PIN, TILT_SERVO_PWM_MIN, TILT_SERVO_PWM_MAX);
}

void pan(int degrees){
    pan_servo.write(normalizeAngle(degrees));
}

void tilt(int degrees){
    tilt_servo.write(normalizeAngle(degrees));
}

/**
 * Simple normalization of the degree angle (caps at 0 and 180 respectively)
 */
static int normalizeAngle(int degrees){
    if(degrees < 0){
        return 0;
    } else if (degrees > 180){
        return 180;
    }
    return degrees;
}
