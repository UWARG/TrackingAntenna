#include "Logger.hpp"
#include "Network.hpp"
#include "TrackingServos.hpp"
#include "Accelerometer.hpp"
#include "Magnetometer.hpp"
#include "TrackingServos.hpp"
#include "Calculator.hpp"
#include "Init.hpp"


long double dutch_lat = 43.530786;
long double dutch_lon = -80.576991;

long double sp_lat = 49.912919;
long double sp_lon = -98.268920;

GPSLocation here;
void setup(){
    initDebug();
    info("Starting up...");
    initializeServos();
    tilt(0);
    pan(0);

    initMagnetometer();
    initAccelerometer();
    
    calibrateTilt();
    tilt(0);
    calibratePan();
    pan(0);

    initNetwork();

    here.lat = dutch_lat;
    here.lon = dutch_lon;
    here.alt = 1;
}

void loop(){
    if (parsePacket()) {
      trackSpike(here, network_data);
    }
    
    renewNetwork(); 
    delay(100);
}
