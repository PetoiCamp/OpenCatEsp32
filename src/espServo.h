#include "PetoiESP32Servo/ESP32Servo.h"

ServoModel servoG41   (180,    SERVO_FREQ,      500,      2500);
ServoModel servoP1S   (290,    SERVO_FREQ,      500,      2500);//1s/4 = 250ms 250ms/2500us=100Hz
ServoModel servoP2K   (290,    SERVO_FREQ,      500,      2500);
#ifdef BiBoard2
#include "pcaServo.h"
#endif

#define P_STEP 32
#define P_BASE 3000 + 6 * P_STEP // 3000~3320
#define P_HARD (P_BASE + P_STEP * 2)
#define P_SOFT (P_BASE - P_STEP * 2)


Servo servo[PWM_NUM];  // create servo object to control a servo
// 16 servo objects can be created on the ESP32

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42

void servoSetup() {
  i2c_eeprom_read_buffer(EEPROM_CALIB, (byte *)servoCalib, DOF);
  //------------------angleRange  frequency  minPulse  maxPulse;
  ServoModel *model;
#ifdef BiBoard
  PTL("Setup ESP32 PWM servo driver...");
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  /*  * LEDC Chan to Group/Channel/Timer Mapping
  ** ledc: 0  => Group: 0, Channel: 0, Timer: 0
  ** ledc: 1  => Group: 0, Channel: 1, Timer: 0
  ** ledc: 8  => Group: 1, Channel: 0, Timer: 0
  ** ledc: 9  => Group: 1, Channel: 1, Timer: 0
  ** ledc: 2  => Group: 0, Channel: 2, Timer: 1
  ** ledc: 3  => Group: 0, Channel: 3, Timer: 1
  ** ledc: 10 => Group: 1, Channel: 2, Timer: 1
  ** ledc: 11 => Group: 1, Channel: 3, Timer: 1
  ** ledc: 4  => Group: 0, Channel: 4, Timer: 2
  ** ledc: 5  => Group: 0, Channel: 5, Timer: 2
  ** ledc: 12 => Group: 1, Channel: 4, Timer: 2
  ** ledc: 13 => Group: 1, Channel: 5, Timer: 2

  ** ledc: 6  => Group: 0, Channel: 6, Timer: 3
  ** ledc: 7  => Group: 0, Channel: 7, Timer: 3
  ** ledc: 14 => Group: 1, Channel: 6, Timer: 3
  ** ledc: 15 => Group: 1, Channel: 7, Timer: 3
  */

  for (int c = 0; c < PWM_NUM; c++) {
    byte s = c;//attachOrder[c];
    int joint;
    if (WALKING_DOF == 8)
      joint = ( s > 3) ? s + 4 : s;
    else// if (WALKING_DOF == 12)
      joint = s + 4;
    switch (servoModelList[joint]) {
      case G41:
        model = &servoG41;
        break;
      case P1S:
        model = &servoP1S;
        break;
      case P2K:
        model = &servoP2K;
        break;
    }
    servo[s].attach(PWM_pin[s], model);
    zeroPosition[joint] = model->getAngleRange() / 2 + float(middleShift[joint])  * rotationDirection[joint];
    calibratedZeroPosition[joint] = zeroPosition[joint] + float(servoCalib[joint])  * rotationDirection[joint];
  }
#else
  Serial.println("Set up PCA9685 PWM servo driver...");
  pwm.begin();
  pwm.setup(servoModelList);
#endif
}


void setServoP(unsigned int p) {
  for (byte s = 0; s < PWM_NUM; s++)
#ifdef BiBoard
    servo[s].writeMicroseconds(p);
#else
    pwm.writeMicroseconds(s, p);
#endif
  }


void allRotate() {
  for (int pos = -50; pos < 50; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    for (int s = 0; s < PWM_NUM; s++) {
#ifdef BiBoard
      servo[s].write(pos);    // tell servo to go to position in variable 'pos'
#else //BiBoard2
      pwm.writeAngle(s, pos);
#endif
      delay(1);             // waits 15ms for the servo to reach the position
      Serial.println(pos);
    }
  }
  for (int pos = 50; pos > -50; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    for (int s = 0; s < PWM_NUM; s++) {
#ifdef BiBoard
      servo[s].write(pos);    // tell servo to go to position in variable 'pos'
#else //BiBoard2
      pwm.writeAngle(s, pos); //may go out of range. check!
#endif
      delay(1);             // waits 15ms for the servo to reach the position
      Serial.println(pos);
    }
  }
}

#ifdef GYRO_PIN
void allRotateWithIMU() {
  for (int s = 0; s < PWM_NUM; s++) {
#ifdef BiBoard
    servo[s].write(90 + ypr[1]  + ypr[2] ); // tell servo to go to position in variable 'pos'
#else //BiBoard2
    pwm.writeAngle(s, 90 + ypr[1]  + ypr[2] );
#endif
    //    delay(1);             // waits 15ms for the servo to reach the position
    Serial.print(ypr[1]);
    Serial.print('\t');
    Serial.println(ypr[2]);
  }
}
#endif

//byte shutOrder[] = {4, 7, 8, 11, 0, 5, 6, 1, 2, 9, 10, 3};
//byte shutOrder[] = { 1, 2, 3, 5, 6, 8, 9, 10, 4, 7, 8, 11,};
//bool shutEsp32Servo = false;
void shutServos() {
  ServoModel *model;
  for (byte s = 0; s < PWM_NUM ; s++) {//PWM_NUM
#ifdef BiBoard
    /* the following method can shut down the servos.
       however, because a single Timer is divided to generate four PWMs, there's random noise when the PWM transits to zero.
       It will cause the servo to jump before shutdown.
    */
//    if (shutEsp32Servo)
      servo[s].writeMicroseconds(0);//the joints may randomly jump when the signal goes to zero. the source is in hardware
    //    servo[s].detach(); //another way to turn off the servo
    //    int joint;
    //    if (WALKING_DOF == 8)
    //      joint = ( s > 3) ? s + 4 : s;
    //    else// if (WALKING_DOF == 12)
    //      joint = s + 4;
    //    switch (servoModelList[joint]) {
    //      case G41:
    //        model = &servoG41;
    //        break;
    //      case P1S:
    //        model = &servoP1S;
    //        break;
    //      case P2K:
    //        model = &servoP2K;
    //        break;
    //    }
    //    servo[s].attach(PWM_pin[s], model);
#else //using PCA9685
    pwm.setPWM(s, 0, 4096);
#endif
  }
//  shutEsp32Servo = false;
}
