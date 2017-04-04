/**
 * @file   Logger.cpp
 * @Author Serj Babayan (WARG)
 * @date   December 20, 2016
 */

#include "Logger.hpp"
#include <HardwareSerial.h>
#include <string.h> //needed for memcpy

void initDebug(void){
#if DEBUG_MESSAGES
    Serial.begin(BAUD_RATE);

    // wait for serial port to connect
    while (!Serial);
#endif
}

void debug(const char* message){
#if DEBUG_MESSAGES
    Serial.println(message);
#endif
}

void error(const char* message){
#if ERROR_MESSAGES
    Serial.println(message);
#endif
}

void info(const char* message){
#if INFO_MESSAGES
    Serial.println(message);
#endif
}

