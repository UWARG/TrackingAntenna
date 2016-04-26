/*PWM does not seem to correspond to a particular position and degree --> PWM conversion may need to be adjusted
 * To calculate PWM/degree ratio:
 *    Adjust code below so that antenna sweeps out approximately a 180 degree semi-circle
 *    Divide PWM corresponding to 180 degrees by 180 (eg. if 120 PWM = 180 degrees, then degree --> PWM = 2/3)
 *    #define new value at top of antenna code
 *    
 * NOTES:
 *    1500 is midpoint for Hitec 785 motor
 *    90 degrees ususally approximates to between 60-63 PWM
*/
#include <Servo.h> 
 
int servoPin = 5;
 
Servo servo;  
 
int angle = 0;   // servo position in degrees 
int pos = 0;   // servo position in PWM 
 
void setup() 
{ 
  servo.attach(servoPin, 600, 2400); 
} 
 
void loop() 
{ 
  servo.writeMicroseconds(1500); 
  delay(3000);
  servo.writeMicroseconds(1440); 
//  servo.writeMicroseconds(610); //Far left position 
  delay(3000);
  servo.writeMicroseconds(1560); 
//  servo.writeMicroseconds(2390); //Far right position
  delay(3000);
} 
