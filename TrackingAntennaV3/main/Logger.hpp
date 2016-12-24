/**
 * @file   Logger.hpp
 * @Author Serj Babayan (WARG)
 * @date   December 20, 2016
 * @brief  Functions for debug printing. Can be turned off.
 */
#ifndef LOGGER
#define LOGGER

/**
 * Message levels to display
 */
#define DEBUG_MESSAGES 1
#define INFO_MESSAGES 1
#define ERROR_MESSAGES 1

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


/**
 * Higher level warning than debug level messages. For now they do
 * the same thing.
 */
void error(const char* message);

/**
 * Info level messages. For now does same thing as debug
 */
void info(const char* message);

#endif
