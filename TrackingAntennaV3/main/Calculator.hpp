/**
 * @file   Calculator.hpp
 * @Author Mark Dunk (WARG)
 * @date   March 15, 2017
 * @brief Calculates the angles required to orient the tracking anttena toward the plane. 
 */

#ifndef Calculator
#define Calculator

#include "Magnetometer.hpp"
#include "Network.hpp"

#define EARTH_RADIUS ((float)6371000)

void trackSpike(GPSLocation antenna_location, GPSLocation plane_location);

#endif
