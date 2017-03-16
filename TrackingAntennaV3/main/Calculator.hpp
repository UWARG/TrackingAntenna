/**
 * @file   Calculator.hpp
 * @Author Mark Dunk (WARG)
 * @date   March 15, 2017
 * @brief Calculates the angles required to orient the tracking anttena toward the plane. 
 */

#ifndef Calculator
#define Calculator

#define EARTH_RADIUS = 6371000;

typedef struct{
	int pan_angle;
	int tilt_angle;
}	TrackingAngles;

void trackSpike(GPSLocation* antenna_location, NetworkData* plane_location, MagnetometerData* orientation);