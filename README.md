# TrackingAntenna
Code for controlling the tracking antenna. (Currently an arduino mega)

An arduino controls 2 servos for plane tracking (pan and tilt). 
Communication from the xbee if achieved with an ethernet shield. 
There is a simple debug interface through serial.


**Libaries**

* [MatrixMath](https://github.com/codebendercc/MatrixMath)
* [Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor)
* Servo
* SPI
* Ethernet
* Adafruit LSM303DLHC

**Installation**

1. Install the [Arduino development environment](https://www.arduino.cc/en/Main/Software)
2. Install the following libraries using the Arduino Library Repository
  * Servo
  * SPI
  * Ethernet
  * Adafruit LSM303DLHC
  
3. Install the following libraries by downloading and including the files:
  * Matrix Math
  * Adafruit Sensor

**Connecting to the Network**

1. Plug in router and connect Arduino Ethernet Shield to ethernet port on router. (If using the wired router, also connect the computer otherwise the WIFI network should work).
2. ENSURE YOU ARE CONNECTED TO THE CORRECT NETWORK.
3. Using [data-relay-station](https://github.com/UWARG/data-relay-station) navigate to the install directory of the data-relay-station and launch a terminal or CMD window.
4. Type `py data_relay.py` to start the data-relay-station. Alternatively, use simulation file by typing `py data_relay.py --simfile <filename.csv>`
5. Determine the IP Address of the router. You can do this by typing `ipconfig /all` in a CMD window (on windows).
6. Choose an IP address for the Arduino. It must be within the valid subnet. Typically, the first few numbers of the IP address are shared with the router. If it isn't, you may be doing something wrong. Replace the line `IPAddress ip(192,168,1,107); //this is for the arduino` with the correct address. Note, this may be changed in the future, where the arduino will self-initialize its IP address.
7. Determine the IP address of the computer. Once again, you can do this by typing `ipconfig /all` in a CMD window. Replace the IP address in the line `IPAddress server(192,168,1,103);  //pi or computer that is hosting the data`. This once again may be dynamically allocated in the future, using a data-relay-station broadcasting feature.
8. You can detect a succesful connection by opening up the Arduino serial monitor, or by checking the data-relay-station terminal output for a familiar IP address.

A successful data-relay-station connection may look as such:
![data-relay-station connection](http://imgur.com/hdNWMdi.png)

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
