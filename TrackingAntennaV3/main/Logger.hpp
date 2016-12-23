/**
 * @file   Logger.hpp
 * @Author Serj Babayan (WARG)
 * @date   December 20, 2016
 * @brief  Functions for debug printing. Can be turned off.
 */
#ifndef LOGGER
#define LOGGER

#include "Logger.hpp"
#include <HardwareSerial.h>
#include <string.h> //needed for memcpy

#define DEBUG_MESSAGES 1
#define BAUD_RATE 9600

/*
* Initializes serial interface for displaying debug messages
*/
void initDebug(void);

/**
* Display a message to through the serial line
* Only works if debug messages have been turned on
*/
void debug(const char* message);

#endif