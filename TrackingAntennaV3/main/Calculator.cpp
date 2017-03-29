/**
 * @file   Calculator.cpp
 * @Author Mark Dunk (WARG)
 * @date   March 15, 2017
 */

#include "Calculator.hpp"
#include "Util.hpp"
#include "Init.hpp"
#include <math.h>

void trackSpike(GPSLocation antenna_location, GPSLocation plane_location) {
	
	long double lat_dif = radians(plane_location.lat - antenna_location.lat); //Distance in latitude points converted to radians.
	long double lon_dif = radians(plane_location.lon - antenna_location.lon); //Distance in longitude points converted to radians.
	long double alt_dif = plane_location.alt - antenna_location.alt;	//Difference in altitude in meters.
	
  // screw haversine. we're using equirectangular approximations. shouldn't be a problem unless we go really far north.
  // highly unlikely that we'll have problems if we fly too far. 10 km over earth's surface is ~0.09 degrees.
  // over 10 km, ~8 m of vertical drop due to curvature. flying at 100 m, that's 0.45 degrees of error in tilt.
  long double lat_dist = EARTH_RADIUS * lat_dif;
	long double lon_dist = EARTH_RADIUS * lon_dif * cos(radians(antenna_location.lat + plane_location.lat) / 2.f);

	long double tot_dist = hypot(lat_dist, lon_dist); //The distance between the plane and the ground station.

// don't think this is necessary anymore?
//	if(antenna_location.lon > plane_location.lon){
//	    lon_dist = lon_dist * -1;
//	}
//
//	if(antenna_location.lat > plane_location.lat){
//	    lat_dist = lat_dist * -1;
//	}

	worldPan(degrees(atan2(lon_dist, lat_dist)));

	worldTilt(degrees(atan2(alt_dif, tot_dist)));
}	


