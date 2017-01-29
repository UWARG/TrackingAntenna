#include "Logger.hpp  "
#include "Network.hpp"
#include "Accelerometer.hpp"
#include "Magnetometer.hpp"

void setup(){
    initDebug();
    info("Starting up...");

    initNetwork();
    initDebug();
    initMagnetometer();
    initAccelerometer();
}

void loop(){
    if(parsePacket()){
      //using serial.println instead of debug() since debug takes a string as parameter
      Serial.println(network_data.alt);
      Serial.println(network_data.lon);
      Serial.println(network_data.lat);
    }
   
    renewNetwork(); 
    delay(100);
}
