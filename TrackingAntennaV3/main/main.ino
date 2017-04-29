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

GPSLocation alma = {1.0, 48.5091985, -71.6465355};

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
    calibratePan(0);
    pan(0);

    initNetwork();

}

void loop(){
    if (parsePacket()) {
      trackSpike(alma, network_data);
    }
    
    renewNetwork(); 
}
