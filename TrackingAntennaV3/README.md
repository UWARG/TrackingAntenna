# WARG Tracking Antenna

3rd revision of the code base for the tracking antenna. The goal of this
revision is to modularize the existing tracking antenna code as well as unit
test it for optimal reliability.

A breakdown of the new code base has been planned, and a description of
all the different modules used is listed below. Each module will preferably
be implemented with a `ModuleName.hpp` and a `ModuleName.cpp` file.

#### Network Module
Responsible for:
- Connecting to the router through the ethernet shield
- Connecting to the data relay server
- Retrieving packets sent from the data relay server
- Parsing the received packets to obtain latitude, longitude, and latitude values
used within the application

Features:
- Can detect a disconnect between the data relay server. If so, will attempt
to reconnect without the need to restart the arduino

Notes:
- Cannot detect a disconnect through the ethernet port, so if the router is unplugged
or loses power, the application will have to be restarted to work again
- Has a globally defined variable called **network_data** that contains
the latest parsed longitude, latitude, and altitude values
- The ethernet shield connects via DHCP by default
- The IP and port of the data relay is hard-coded into the code


