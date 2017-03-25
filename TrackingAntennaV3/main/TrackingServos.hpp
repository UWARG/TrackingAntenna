/**
 * @file   TrackingServos.hpp
 * @Author Serj Babayan (WARG)
 * @date   January 7, 2017
 * @brief  Functions for servo intitialization fo the tracking antenna and provides general functions
 *         for controlling the pan and tilt of the tracking antenna
 * @see https://www.arduino.cc/en/Reference/Servo
 */
#ifndef TRACKING_SERVOS
#define TRACKING_SERVOS

/**
 * The PWM signals that correspond to the minimum and maxium (0-degree and 180-degree) angle on the servo
 * The closer these values are to the actual values of the servo, the more accuratly we can position the servos.
 * The arduino defaults are 544 and 2400 respectively
 */

#define PAN_SERVO_MID 1570
#define PAN_SERVO_RANGE 70
#define PAN_SERVO_PWM_MIN PAN_SERVO_MID - PAN_SERVO_RANGE
#define PAN_SERVO_PWM_MAX PAN_SERVO_MID + PAN_SERVO_RANGE
#define TILT_SERVO_PWM_MIN 1785
#define TILT_SERVO_PWM_MAX 2150

/**
 * The actual pins the servo's are connected to. Note that on the MEGA you can use pretty much
 * any pin, though its better to use pins that are PWM capable (pins 2 to 13 and 44 to 46). Also
 * note that pins 2 and 3 can also be used for interrupts, so its better to reserve those pins for
 * future use.
 */
#define PAN_SERVO_PIN 9
#define TILT_SERVO_PIN 8

/**
 * Because we'll never actually want the tracking antenna pointing all the
 * way down or up, this places a software limit on how low/high the tracking antenna can tilt
 * In tenth degrees.
 */
#define TILT_ANGLE_MIN_LIMIT 0
#define TILT_ANGLE_MAX_LIMIT 600

/**
 * Limit in either direction, in tenth degrees
 */
#define PAN_ANGLE_LIMIT 1200

/**
 * Initialize the servos. Need to be called before the pan and tilt functions are used
 */
void initializeServos(void);

/**
 * Control the pan of the tracking antenna
 * @param deg Desired angle in degrees. (0 is forward, CW is +ve)
 */
void pan(float deg);

/**
 * Control the tilt of the tracking antenna
 * @param deg Desired angle in degrees. (0 is flat, up is +ve)
 */
void tilt(float deg);

#endif
