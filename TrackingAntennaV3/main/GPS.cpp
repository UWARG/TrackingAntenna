#include "GPS.hpp"
#include "Logger.hpp"
#include <Adafruit_GPS.h>

//The current GPS location (latitude, longitude, and altitude)
GPSLocation gps_location;

Adafruit_GPS Adafruit_GPS(&Serial1);

//Initializes the GPS module at the selected serial pins and baud rate.
void initGPS(){
	Adafruit_GPS.begin(9600);
	Adafruit_GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	Adafruit_GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
}

//Parses incoming GPS data and sends it to main
bool parseGPS(void){
	//Waits for GPS message may result in program hanging. This should probably add a timer. :P

  bool debug_message_sent = false;
  while(!Adafruit_GPS.newNMEAreceived()){
    if(!debug_message_sent){
      debug("Waiting for GPS connection");
    }
    debug_message_sent = true;
	}
	Adafruit_GPS.parse(Adafruit_GPS.lastNMEA());

	gps_location.lat = Adafruit_GPS.latitudeDegrees;
	gps_location.lon = Adafruit_GPS.longitudeDegrees;
	gps_location.alt = Adafruit_GPS.altitude; //In meters

	if(!Adafruit_GPS.fix){
    error("Unable to connect to satellites");
		return false;
	}

  else{
    debug("GPS connected");
    return true;
  }
}
