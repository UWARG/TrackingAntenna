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

typedef struct {
    float alt;
    float lat;
    float lon;
} GPSLocation;

void trackSpike(GPSLocation antenna_location, NetworkData plane_location);

#endif
