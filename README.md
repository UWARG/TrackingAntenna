# TrackingAntenna
Code for controlling the tracking antenna. (Currently an arduino mega)

An arduino controls 2 servos for plane tracking (pan and tilt). 
Communication from the xbee if achieved with an ethernet shield. 
There is a simple debug interface through serial.


**Libaries**
[MatrixMath](https://github.com/codebendercc/MatrixMath)
[Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor)
Servo
SPI
Ethernet

**Installation**
1. Install the [Arduino development environment](https://www.arduino.cc/en/Main/Software)
2. Install the following libraries using the Arduino Library Repository
  * Servo
  * SPI
  * Ethernet
3. Install the following libraries by downloading and including the files:
  * Matrix Math
  * Adafruit Sensor

**Quick setup guide:**

1. Set INITIALISATION to 0.
2. Be careful when the code uploads as the antenna may swing unpredictably.
3. Turn the base of the antenna to point eastish
4. At this point you can verify the antenna is recieving telemetry data. set DEBUG to 1 and open arduino serial monitor
5. Hold the plane next to the antenna, and update the INITIAL_LATTITUDE, INITAL_LONGITUDE, INITIAL_ALTITUDE.
6. Set INITIALISATION TO 1. again be careful about the unpredictabe swing.
7. Put the plane on the runway, some distance away from the antenna is needed for fine tunning the angle.
8. Rotate the base of the antenna until it point at the plane. it should already have been decently close. 
9. Check the altitude (tilt) seems correct.
10. Check there are no wire to snag as it rotates.
