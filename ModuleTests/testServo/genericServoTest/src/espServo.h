
#ifdef ESP_PWM
#include "esp32-hal-adc.h"
#include "PetoiESP32Servo/ESP32Servo.h"
//------------------angleRange  frequency  minPulse  maxPulse;
ServoModel servoG41(180, SERVO_FREQ, 500, 2500);
ServoModel servoP1S(270, SERVO_FREQ, 500, 2500);  // 1s/4 = 250ms 250ms/2500us=100Hz
ServoModel servoP1L(270, SERVO_FREQ, 500, 2500);
Servo servo[PWM_NUM];  // create servo object to control a servo
// 16 servo objects can be created on the ESP32
#define EEPROM_CALIB 20
#else
//------------------angleRange  frequency  minPulse  maxPulse;
#include "pcaServo.h"
#endif



#define P_STEP 32
#define P_BASE 3000 + 3 * P_STEP  // 3000~3320
#define P_HARD (P_BASE + P_STEP * 2)
#define P_WORKING (P_BASE - P_STEP * 2)
#define P_SOFT (P_BASE - P_STEP * 3)
#define FEEDBACK_SIGNAL 3500
int feedbackSignal = FEEDBACK_SIGNAL;
int waitTimeForResponse = 10000;
int maxPulseWidth = 2600;

ServoModel *modelObj[PWM_NUM];
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
int8_t movedJoint[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int8_t movedCountDown = 3;  // allow the driving servo to pause in the middle rather than changing its state instantly
int measureServoPin = -1;
byte nPulse = 3;

#ifdef ESP_PWM
void attachAllESPServos() {
  PTLF("Calibrated Zero Position");
  for (int c = 0; c < PWM_NUM; c++) {
    byte s = c;  // attachOrder[c];
    int joint;
    if (WALKING_DOF == 8)
      joint = (s > 3) ? s + 4 : s;
    else  // if (WALKING_DOF == 12)
      joint = s + 4;
    switch (servoModelList[joint]) {
      case G41:
        modelObj[s] = &servoG41;
        break;
      case P1S:
        modelObj[s] = &servoP1S;
        break;
      case P2K:
        modelObj[s] = &servoP1L;
        break;
    }
    servo[s].attach(PWM_pin[s], modelObj[s]);
    zeroPosition[joint] = modelObj[s]->getAngleRange() / 2 + float(middleShift[joint]) * rotationDirection[joint];
    calibratedZeroPosition[joint] = zeroPosition[joint] + float(servoCalib[joint]) * rotationDirection[joint];
    PT(calibratedZeroPosition[joint]);
    PT('\t');
  }
  PTL();
}
void reAttachAllServos() {
  for (int c = 0; c < PWM_NUM; c++)
    if (!servo[c].attached()) {
      byte s = c < 4 ? c : c + 4;
      if (!movedJoint[s]) {
        servo[c].attach(PWM_pin[c], modelObj[c]);
      }
    }
  delay(12);
}
#endif

void servoSetup() {
#ifdef INVERSE_SERVO_DIRECTION
  for (byte s = 0; s < DOF; s++)
    rotationDirection[s] *= -1;
#endif

#ifdef ESP_PWM
  i2c_eeprom_read_buffer(EEPROM_CALIB, (byte *)servoCalib, DOF);
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
  attachAllESPServos();
#else
  Serial.println("Set up PCA9685 PWM servo driver...");
  pwm.begin();
  pwm.setup(servoModelList);
#endif
}

void shutServos(byte id = PWM_NUM) {
  ServoModel *model;
  if (id == PWM_NUM) {
    for (byte s = 0; s < PWM_NUM; s++) {  // PWM_NUM
#ifdef ESP_PWM
      /* the following method can shut down the servos.
       however, because a single Timer is divided to generate four PWMs, there's random noise when the PWM transits to zero.
       It will cause the servo to jump before shutdown.
    */
      //    if (shutEsp32Servo)
      servo[s].writeMicroseconds(0);  // the joints may randomly jump when the signal goes to zero. the source is in hardware
                                      //     servo[s].detach(); //another way to turn off the servo
                                      //     int joint;
                                      //     if (WALKING_DOF == 8)
                                      //       joint = ( s > 3) ? s + 4 : s;
                                      //     else// if (WALKING_DOF == 12)
                                      //       joint = s + 4;
                                      //     switch (servoModelList[joint]) {
                                      //       case G41:
                                      //         model = &servoG41;
                                      //         break;
                                      //       case P1S:
                                      //         model = &servoP1S;
                                      //         break;
                                      //       case P2K:
                                      //         model = &servoP1L;
                                      //         break;
                                      //     }
                                      //     servo[s].attach(PWM_pin[s], model);
#else                                 // using PCA9685
      pwm.setPWM(s, 0, 4096);
#endif
    }
  } else {
    id = (PWM_NUM == 12 && id > 3) ? id - 4 : id;
#ifdef ESP_PWM
    servo[id].writeMicroseconds(0);
#else  // using PCA9685
    pwm.setPWM(id, 0, 4096);
#endif
  }
  //  shutEsp32Servo = false;
}

void setServoP(unsigned int p) {
  for (byte s = 0; s < PWM_NUM; s++)
#ifdef ESP_PWM
    servo[s].writeMicroseconds(p);
#else
    pwm.writeMicroseconds(s, p);
#endif
}
void setJoint(byte i, float angle, float speedRatio = 0) {
  if (PWM_NUM == 12 && WALKING_DOF == 8 && i > 3 && i < 8) {  //there's no such joint in this configuration
    PTHL("invalid joint ", i);
    return;
  }
  int actualServoIndex = (PWM_NUM == 12 && i > 3) ? i - 4 : i;
  // angle = max(float(angleLimit[i][0]), min(float(angleLimit[i][1]), angle));
  int duty0 = calibratedZeroPosition[i] + currentAng[i] * rotationDirection[i];
  // previousAng[i] = currentAng[i];
  currentAng[i] = angle;
  int duty = calibratedZeroPosition[i] + angle * rotationDirection[i];
  int steps = speedRatio > 0 ? int(round(abs(duty - duty0) / 1.0 /*degreeStep*/ / speedRatio)) : 0;
  //if default speed is 0, no interpolation will be used
  //otherwise the speed ratio is compared to 1 degree per second.

  for (int s = 0; s <= steps; s++) {
    int degree = duty + (steps == 0 ? 0 : (1 + cos(M_PI * s / steps)) / 2 * (duty0 - duty));
#ifdef VOLTAGE
    if (!lowBatteryQ)
#endif
    {
#ifdef ESP_PWM
      servo[actualServoIndex].write(degree);
#else
      pwm.writeAngle(actualServoIndex, degree);
#endif
    }
    //    delayMicroseconds(1);
  }
}
void setAllJoint(int *dutyAng, byte offset = 0) {
  for (int8_t i = DOF - 1; i >= offset; i--) {
    setJoint(i, dutyAng[i]);
    delay(10);
  }
}

void allRotate() {
  for (int pos = -20; pos < 20; pos += 1) {  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    for (int s = 0; s < PWM_NUM; s++) {
      setJoint(s, pos);
      delay(1);  // waits 15ms for the servo to reach the position
      Serial.println(pos);
    }
  }
  for (int pos = 20; pos > -20; pos -= 1) {  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    for (int s = 0; s < PWM_NUM; s++) {
#ifdef ESP_PWM
      servo[s].write(pos);  // tell servo to go to position in variable 'pos'
#else                       // NyBoard
      pwm.writeAngle(s, pos);  // may go out of range. check!
#endif
      delay(1);  // waits 15ms for the servo to reach the position
      Serial.println(pos);
    }
  }
}

// byte shutOrder[] = {4, 7, 8, 11, 0, 5, 6, 1, 2, 9, 10, 3};
// byte shutOrder[] = { 1, 2, 3, 5, 6, 8, 9, 10, 4, 7, 8, 11,};
// bool shutEsp32Servo = false;
