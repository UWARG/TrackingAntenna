# TrackingAntenna
Code for controlling the tracking antenna. (Currently an arduino)

An arduino controls 2 servos for plane tracking (pan and tilt). 
Communication from the xbee if achieved with an ethernet shield. 
There is a simple debug interface through serial.

Quick setup guide:

set INITIALISATION to 0.
be careful when the code uploads as the antenna may swing unpredictably.
turn the base of the antenna to point eastish
at this point you can verify the antenna is recieving telemetry data. set DEBUG to 1 and open arduino serial monitor
hold the plane next to the antenna, and update the INITIAL_LATTITUDE, INITAL_LONGITUDE, INITIAL_ALTITUDE.
set INITIALISATION TO 1. again be careful about the unpredictabe swing.
put the plane on the runway, some distance away from the antenna is needed for fine tunning the angle.
rotate the base of the antenna until it point at the plane. it should already have been decently close. 
check the altitude (tilt) seems correct.
check there are no wire to snag as it rotates.

Issues:
if the plane flies from esat-north-east, to east-south-east, the antenna will go from 45 degrees to 315 degrees. this is wrong, it should be 45 to -45.
