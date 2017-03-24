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
    tilt(0);
    pan(0);
    parseAcceleration();
    getMagneticNorth();
    Serial.print("Pitch: ");  
    Serial.print((180/PI)*atan2(accel_data.x, accel_data.z));
    Serial.print(" Heading: ");
    Serial.println((180/PI)*mag_data.angle);
    delay(200);
}
