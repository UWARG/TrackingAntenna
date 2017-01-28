#ifndef GPS
#define GPS

typedef struct {
	float alt;
	float lat;
	float lon;
} GPSLocation;

//Initializes the GPS module at the selected serial pins and baud rate.
void initGPS();

//Parses incoming GPS data and sends it to main
bool parseGPS();

#endif