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
#define PAN_SERVO_PWM_MIN 544
#define PAN_SERVO_PWM_MAX 2400
#define TILT_SERVO_PWM_MIN 544
#define TILT_SERVO_PWM_MAX 2400

/**
 * The actual pins the servo's are connected to. Note that on the MEGA you can use pretty much
 * any pin, though its better to use pins that are PWM capable (pins 2 to 13 and 44 to 46). Also
 * note that pins 2 and 3 can also be used for interrupts, so its better to reserve those pins for
 * future use.
 */
#define PAN_SERVO_PIN 4
#define TILT_SERVO_PIN 5

/**
 * Initialize the servos. Need to be called before the pan and tilt functions are used
 */
void initializeServos(void);

/**
 * Control the pan of the tracking antenna
 * @param degrees Any integer angle. Will cap the degree at 0 if given less than 0 and cap at 180 if given more than 180
 */
void pan(int degrees);

/**
 * Control the tilt of the tracking antenna
 * @param degrees Any integer angle in degrees. Will cap the degree at 0 if given less than 0 and cap at 180 if given more than 180
 */
void tilt(int degrees);

#endif
