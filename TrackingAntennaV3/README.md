# WARG Tracking Antenna

3rd revision of the code base for the tracking antenna. The goal of this
revision is to modularize the existing tracking antenna code as well as unit
test it for optimal reliability.

A breakdown of the new code base has been planned, and a description of
all the different modules used is listed below. Each module will preferably
be implemented with a `ModuleName.hpp` and a `ModuleName.cpp` file.

Note that it was discussed before that the tracking antenna should have
a menu that would interact with the user. Before that is scoped out and
implemented however it's better to build the base for the tracking antenna,
before adding user interactivity to it.

## Installation
For everything to compile you need to copy everything that is in the `libraries` folder
into the library folder that the Arduino IDE will recognize. This is either the
`My Documents/Arduino/libraries` folder on window, or you can also put it in the
`libraries` folder in the arduino ide installation directory (for both Windows and Linux).

### Network Module
*Responsible for*:
- Connecting to the router through the ethernet shield
- Connecting to the data relay server
- Retrieving packets sent from the data relay server
- Parsing the received packets to obtain latitude, longitude, and latitude values
used within the application

*Features*:
- Can detect a disconnect between the data relay server. If so, will attempt
to reconnect without the need to restart the arduino

*Notes*:
- Cannot detect a disconnect through the ethernet port, so if the router is unplugged
or loses power, the application will have to be restarted to work again
- Has a globally defined variable called **network_data** that contains
the latest parsed longitude, latitude, and altitude values
- The ethernet shield connects via DHCP by default
- The IP and port of the data relay is hard-coded

### Acceloremeter/Magnetometer/GPS Modules
Three seperate modules with very similar functionalities.

*Responsible for*:
- Connecting to the respective sensors/shields
- Reading in values from the sensors
- Parsing the received values into a usable form

*Notes*:
- **Not implemented yet**
- Would be nice if there was detection of a sensor disconnect, and a way
  to attempt a reconnect (same as the network)
- Like the network, each module should provide a globally define variable/struct providing
  the latest values for the corresponding sensors
  
### Linear Algebra Module
Preferably if possible a library should be used for this instead. This
module is responsible for providing any functions necessary for manipulating
vectors and matrices that is required for the tracking antenna to calculate
the necessary angles.

If a library is implemented, it should provide structures for representing matrices,
vectors, etc. ie no raw array manipulations

Again, ideally this should be a library that we don't have to implement, unit test,
or maintain.

### Logger Module
*Responsible for*:
- Providing functions to abstract logging

*Features*:
- 3 different logging levels/functions available. Info, Error, and Debug,
  each of which can be turned off seperately
- In the future provides an easy way to, for example, redirect certain levels
  of logs to an SD card, or an LCD screen

*Notes*:
- Currently each function only accepts a string as parameter, so printing out raw numeric
  values is a bit tricky. For temporary debugging purposes, the Logger module also initializes
  the Serial interface, which you can use for development purpose for printing out raw numeric
  values and such (so Serial.println). Keep in mind Serial.println SHOULD NOT BE USED in production
  code, only when you're developing.
  
### Util Module
*Responsible for*:
- Providing any generic functions that don't really fit into any other module. This module should be relatively small

### Tracking Antenna Module
*Responsible for*:
- Servo intitialization for the tracking antenna
- Providing generic functions for controlling the servos of the tracking antenna

*Notes*:
- **Not implemented yet**
- Should really provide 2 functions, `pan(int angle)` and `tilt(int angle)` for controlling the tracking antenna

### Calculator Module
*Responsible for*:
- Provides functions calculating the required tilt and pan position of the tracking antenna given
  latitude, longtiude, altitude values from the plane and gps values of the tracking antenna.

*Notes*:
- **Not implemented yet**
- Should not use the global variables defined by the other modules. The functions for the calculation
  should take in the required values as function parameters.

### LCD Module
*Responsible for*:
- Providing generic functions for outputting text to an LCD display

*Notes*:
- **Not implemented yet**
- What text the LCD should display still has to be scoped out. An idea would be to display the 
  pan and tilt angles the tracking antenna is trying to get to.
  
### Main Module
Starting point of the application. Should be relatively short.
The only logic stored here should be for calling the intialization functions
for all the sensors, the network, calling the parsing functions for the modules,
and calling the calculation logic functions.

*Notes*:
- **This is the only module that is allowed to use the global variables defined in the other modules.**

## Naming Convention
`CAPITAL_SNAKE_CASE` for #define constants

`regular_snake_case` for variables

`camelCase()` for functions
