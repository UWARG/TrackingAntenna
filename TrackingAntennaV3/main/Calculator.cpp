#include "Calculator.hpp"
#incldue <math.h>

void initialize(){
	//get compass reading, if facing north then when plane lon < antenna point left, if south reverse. If east when plane lat > antenna lat point left, reverse for west
	//1 is north, 2 is south, ect.
}
//This module is shit and a better solution should be created.
//East and North are positive
void trackSpike(GPSLocation* antenna_location, NetworkData* plane_location, char orientation){
	float antenna_vector [3];
	float plane_vector [3];

	antenna_location.alt += 6371000;
	plane_location.alt += 6371000;

	antenna_vector [0] = antenna_location.alt*sin(antenna_location.lat)*cos(antenna_location.lon);
	antenna_vector [1] = antenna_location.alt*sin(antenna_location.lat)*sin(antenna_location.lon);
	antenna_vector [2] = antenna_location.alt*cos(antenna_location.lat);

	plane_vector [0] = plane_location.alt*sin(plane_location.lat)*cos(plane_location.lon);
	plane_vector [1] = plane_location.alt*sin(plane_location.lat)*sin(plane_location.lon);
	plane_vector [2] = plane_location.alt*cos(plane_location.lat);

	float pan_angle = acos(dotProduct(antenna_vector[0], antenna_vector[1], plane_vector[0], plane_vector[1])/(magnitude(antenna_vector[0], antenna_vector[1])*magnitude(plane_vector[0], plane_vector[1])))*57.2958;
	float tilt_angle = acos(dotProduct(antenna_vector[1], antenna_vector[2], plane_vector[1], plane_vector[2])/(magnitude(antenna_vector[1], antenna_vector[2)*magnitude(plane_vector[1], plane_vector[2])))*57.2958;

	switch (orientation){
		case 1:
			if (antenna_location.lon < plane_location.lon){
				pan_angle += 90;
			}
			break;

		case 2:
			if (antenna_location.lon > plane_location.lon){
				pan_angle += 90;
			}
			break;

		case 3:
			if (antenna_location.lat > plane_location.lat){
				pan_angle += 90;
			}
			break;

		case 4:
			if (antenna_location.lat < plane_location.lat){
				pan_angle += 90;
			}
			break;
	}

	if (plane_location.alt > antenna_location.alt){
		tilt_angle += 90;
	}


}	
