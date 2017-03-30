#include "Logger.hpp"
#include "Network.hpp"
#include "TrackingServos.hpp"
#include "Accelerometer.hpp"
#include "Magnetometer.hpp"
#include "TrackingServos.hpp"
#include "Calculator.hpp"
#include "Init.hpp"

// alt, lat, lon
GPSLocation dutch = {1.0, 43.530786, -80.576991};
float dutch_dec = -9.63; // magnetic declination (angle between mag north & pole)

GPSLocation e5 = {1.0, 43.4725079, -80.5398190};
float e5_dec = -9.62;

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
    calibratePan(e5_dec);
    pan(0);

    initNetwork();

}

void loop(){
    if (parsePacket()) {
      trackSpike(e5, network_data);
    }
    
    renewNetwork(); 
}
