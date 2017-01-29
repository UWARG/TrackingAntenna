#include "Logger.hpp"
#include "TrackingServos.hpp"
#include "Accelerometer.hpp"
#include "Magnetometer.hpp"
#include "TrackingServos.hpp"
#include "Math.h"

void setup(){
    initDebug();
    info("Starting up...");
    initializeServos();
    initMagnetometer();
    initAccelerometer();
}

void loop(){
    tilt(140);
    ///pan(0);
    parseAcceleration();
    Serial.println((360/(2*PI))*atan2(accel_data.x, accel_data.z));
    delay(100);
}
