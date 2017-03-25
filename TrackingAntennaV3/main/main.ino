#include "Logger.hpp"
#include "TrackingServos.hpp"
#include "Accelerometer.hpp"
#include "Magnetometer.hpp"
#include "TrackingServos.hpp"
#include "Init.hpp"

void setup(){
    initDebug();
    info("Starting up...");
    initializeServos();
    initMagnetometer();
    initAccelerometer();

    tilt(0);
    pan(0);

    calibrateTilt();
    Serial.print(initialize.tilt_min); Serial.print(" "); Serial.println(initialize.tilt_max);

    delay(500);
    calibratePan();
    Serial.print(initialize.heading_min); Serial.print(" "); Serial.println(initialize.heading_max);
}

void loop(){
   
    tilt(0);
    pan(0);
//    parseAcceleration();
//    getMagneticNorth();
//    Serial.print("Pitch: ");  
//    Serial.print(accel_data.pitch);
//    Serial.print(" Heading: ");
//    Serial.println(mag_data.heading);
//    delay(200);
}
