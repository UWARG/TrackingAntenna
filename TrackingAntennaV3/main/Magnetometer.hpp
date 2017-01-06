/**
 * @file   Magnetometer.hpp
 * @Author Michael Lenover (WARG)
 * @date   January 5, 2017
 * @brief  Magnetometer initialization functions
 *         Connection based off of the Arduino ethernet library.
 *         Uses DHCP so no need to know the IP of the shield once connected.
 * @see https://www.arduino.cc/en/Reference/Ethernet
 */

#ifndef MAGNETOMETER
#define MAGNETOMETER

#define COMPASS_ID 12345

#define CALIBRATE_MAGNETOMETER true

void initMagnetometer();

#endif
