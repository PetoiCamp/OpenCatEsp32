/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald

 modified for the ESP32 on March 2017
 by John Bennett

 see http://www.arduino.cc/en/Tutorial/Sweep for a description of the original code

 * Different servos require different pulse widths to vary servo angle, but the range is 
 * an approximately 500-2500 microsecond pulse every 20ms (50Hz). In general, hobbyist servos
 * sweep 180 degrees, so the lowest number in the published range for a particular servo
 * represents an angle of 0 degrees, the middle of the range represents 90 degrees, and the top
 * of the range represents 180 degrees. So for example, if the range is 1000us to 2000us,
 * 1000us would equal an angle of 0, 1500us would equal 90 degrees, and 2000us would equal 1800
 * degrees.
 * 
 * Circuit: (using an ESP32 Thing from Sparkfun)
 * Servo motors have three wires: power, ground, and signal. The power wire is typically red,
 * the ground wire is typically black or brown, and the signal wire is typically yellow,
 * orange or white. Since the ESP32 can supply limited current at only 3.3V, and servos draw
 * considerable power, we will connect servo power to the VBat pin of the ESP32 (located
 * near the USB connector). THIS IS ONLY APPROPRIATE FOR SMALL SERVOS. 
 * 
 * We could also connect servo power to a separate external
 * power source (as long as we connect all of the grounds (ESP32, servo, and external power).
 * In this example, we just connect ESP32 ground to servo ground. The servo signal pins
 * connect to any available GPIO pins on the ESP32 (in this example, we use pin 18.
 * 
 * In this example, we assume a Tower Pro MG995 large servo connected to an external power source.
 * The published min and max for this servo is 1000 and 2000, respectively, so the defaults are fine.
 * These values actually drive the servos a little past 0 and 180, so
 * if you are particular, adjust the min and max values to match your needs.
 */

#define BiBoard_V1_0
#include <ESP32Servo.h>
Servo myservo[12];  // create servo object to control a servo
// 16 servo objects can be created on the ESP32

int pos = 0;  // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
#if defined BiBoard_V0_1 || defined BiBoard_V0_2
byte servoPin[] = { 19, 4, 2, 27,   // head or shoulder roll
                    33, 5, 15, 14,  // shoulder pitch
                    32, 18, 13, 12 };
#elif defined BiBoard_V1_0
byte servoPin[] = {
  18, 5, 14, 27,   // head or shoulder roll
  23, 15, 12, 33,  // shoulder pitch
  19, 4, 13, 32    // knee
};
#endif

void setup() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  for (byte s = 0; s < 12; s++) {
    myservo[s].setPeriodHertz(200);
    myservo[s].attach(servoPin[s], 500, 2500);
    // using default min/max of 500us and 2500us
    // different servos may require different min/max settings
    // for an accurate 0 to 180 sweep
  }
}

void loop() {
  for (pos = 60; pos <= 120; pos += 1) {
    for (byte s = 0; s < 12; s++)
      myservo[s].write(pos);  // tell servo to go to position in variable 'pos'
    delay(4);                 // waits 15ms for the servo to reach the position
  }
  for (pos = 120; pos >= 60; pos -= 1) {
    for (byte s = 0; s < 12; s++)
      myservo[s].write(pos);  // tell servo to go to position in variable 'pos'
    delay(4);                 // waits 15ms for the servo to reach the position
  }
}
