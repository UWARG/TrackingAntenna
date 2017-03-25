#include "Logger.hpp"
#include "Network.hpp"
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
}
