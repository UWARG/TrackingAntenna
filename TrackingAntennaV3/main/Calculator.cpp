/**
 * @file   Calculator.cpp
 * @Author Mark Dunk (WARG)
 * @date   March 15, 2017
 */

#include "Calculator.hpp"
#include "Util.hpp"
#incldue <math.h>

volatile TrackingAngles tracking_angles;

void trackSpike(GPSLocation* antenna_location, NetworkData* plane_location, MagnetometerData* orientation){
	
	long double lat_dif = (plane_location.lat - antenna_location.lat) * M_PI / 180; //Distance in latitude points converted to radians.
	long double lon_dif = (plane_location.lon - antenna_location.lon) * M_PI / 180; //Distance in longitude points converted to radians.
	long double alt_dif = plane_location.alt - antenna_location.alt;	//Difference in altitude in meters.
	
	//Modified Haversine fuction. Takes in the fifference in latitude or longitude of the plane and the ground station. Outputs the spherical distance between the two points in East-West and North-South directions.
	long double lat_dist = EARTH_RADIUS * 2 * asin(sqrt(sin(lat_dif / 2) * sin(lat_dif / 2)));
	long double lon_dist = EARTH_RADIUS * 2 * asin(sqrt(cos(Lat1 * M_PI / 180) * cos(Lat2 * M_PI / 180) * sin(lon_dif / 2) * sin(lon_dif / 2)));
	
	long double tot_dist = sqrt((lat_dist*lat_dist) + (lon_dist*lon_dist)); //The actual spherical distance between the plane and the ground station.



	if(antenna_location.lon > plane_location.lon){
	    lon_dist = lon_dist * -1;
	}

	if(antenna_location.lat > plane_location.lat){
	    lat_dist = lat_dist * -1;
	}

	tracking_angles.pan = derees(atan2(lon_dist,lat_dist)) - orientation.angle;

	tracking_angles.tilt = degrees(atan(alt_dif, tot_dist));


}	
