#include "PetoiESP32Servo/ESP32Servo.h"
//------------------angleRange  frequency  minPulse  maxPulse;
ServoModel servoG41(180, SERVO_FREQ, 500, 2500);
ServoModel servoP1S(270, SERVO_FREQ, 500, 2500);  // 1s/4 = 250ms 250ms/2500us=100Hz
ServoModel servoP1L(270, SERVO_FREQ, 500, 2500);
#ifdef BiBoard2
#include "pcaServo.h"
#endif

#define P_STEP 32
#define P_BASE 3000 + 3 * P_STEP  // 3000~3320
#define P_HARD (P_BASE + P_STEP * 2)
#define P_SOFT (P_BASE - P_STEP * 2)
#define FEEDBACK_SIGNAL 3500
int feedbackSignal = FEEDBACK_SIGNAL;
int waitTimeForResponse = 10000;
int maxPulseWidth = 2600;

Servo servo[PWM_NUM];  // create servo object to control a servo
                       // 16 servo objects can be created on the ESP32
ServoModel *modelObj[PWM_NUM];
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
int8_t movedJoint[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int8_t movedCountDown = 3;  // allow the driving servo to pause in the middle rather than changing its state instantly
int measureServoPin = -1;
byte nPulse = 3;

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

void servoSetup() {
  i2c_eeprom_read_buffer(EEPROM_CALIB, (byte *)servoCalib, DOF);
#ifdef INVERSE_SERVO_DIRECTION
  for (byte s = 0; s < DOF; s++)
    rotationDirection[s] *= -1;
#endif

#ifdef ESP_PWM
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

void shutServos() {
  ServoModel *model;
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
#else                               // using PCA9685
    pwm.setPWM(s, 0, 4096);
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

int measurePulseWidth(uint8_t pwmReadPin) {
  long start = micros();
  while (!digitalRead(pwmReadPin)) {  // 等待高电平
    if (micros() - start > waitTimeForResponse)
      return -1;
  }
  long t1 = micros();
  while (digitalRead(pwmReadPin)) {
    if (micros() - t1 > maxPulseWidth)
      return -1;
  }
  return (micros() - t1);
}

float readFeedback(byte s)  // returns the pulse width in microseconds
{                           // s is not the joint index, but the pwm pin index that may shift by 4
  //if(!servo[s].attached())// adding this condition will cause servos to jig. why?
  servo[s].attach(PWM_pin[s], modelObj[s]);  // sometimes servo[s].attach() is true, but it still needs to be attached. why there's no conflict?
  delay(12);                                 // it takes time to attach. potentially it can be avoided using the attached() check. but it doesn't work for now.
  servo[s].writeMicroseconds(feedbackSignal);
  servo[s].detach();
  pinMode(PWM_pin[s], INPUT);
  float mean = 0;
  int n = nPulse;
  for (byte i = 0; i < nPulse; i++) {  // 测三次求平均值
    int temp = measurePulseWidth(PWM_pin[s]);
    if (temp < 0) {
      n--;
      if (n == 0)
        return -1;
    } else if (i > 0)
      mean += temp;
  }
  if (n > 1) {
    // PTT(n, ": ")
    return mean / (n - 1);
  } else
    return -1;
}

void servoFeedback(int8_t index = 16) {
  int readAngles[16];
  byte begin = 0, end = 15;
  if (index > -1 && index < 16)
    begin = end = index;  //
  for (byte jointIdx = begin; jointIdx <= end; jointIdx++) {
    if (jointIdx == 4)  // skip the shoulder roll joints
      jointIdx += 4;
    byte i = jointIdx < 4 ? jointIdx : jointIdx - 4;
    int feedback = readFeedback(i);
    if (feedback > -1) {
      float convertedAngle = (servo[i].pulseToAngle(feedback) - calibratedZeroPosition[jointIdx]) / rotationDirection[jointIdx];
      if (begin == end)
        PTT(jointIdx, '\t')
      PTD(convertedAngle, 1);
      if (begin != end)
        PT('\t');
      readAngles[jointIdx] = round(convertedAngle);
      if (fabs(currentAng[jointIdx] - convertedAngle) > (movedJoint[jointIdx] ? 1 : 2)) {  // allow smaller tolarance for driving joint
                                                                                           // allow larger tolerance for driven joint
        movedJoint[jointIdx] = movedCountDown;
      } else if (movedJoint[jointIdx])  //if it's not moved, its state will be decreased until set to false.
        movedJoint[jointIdx]--;
      currentAng[jointIdx] = readAngles[jointIdx];
    }
  }
  PTL();
}
bool servoFollow() {
  bool checkAll = true, moved = false;
  byte movedJointList[DOF];
  byte movedJointCount = 0;
  for (byte i = 0; i < PWM_NUM; i++) {  // decide if to check all servos.
    byte jointIdx = i < 4 ? i : i + 4;
    if (movedJoint[jointIdx]) {  // only the previouslly moved joints will be checked.
      servoFeedback(jointIdx);
      checkAll = false;
      movedJointList[movedJointCount++] = jointIdx;
    }
  }
  if (checkAll) {  // if no joint has been moved, all joints will be checked
    servoFeedback(16);
  }
  for (byte jointIdx = 0; jointIdx < 16; jointIdx++)
    newCmd[jointIdx] = currentAng[jointIdx];
  for (byte i = 0; i < movedJointCount; i++) {
    byte jointIdx = movedJointList[i];
    for (byte j = 0; j < 4; j++)
      if (j != jointIdx % 4) {
        newCmd[(jointIdx / 4) * 4 + j] = currentAng[jointIdx];
      } else {
        newCmd[jointIdx] = currentAng[jointIdx];
      }
    moved = true;
  }
  return moved;
}

void allRotate() {
  for (int pos = -50; pos < 50; pos += 1) {  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    for (int s = 0; s < PWM_NUM; s++) {
#ifdef ESP_PWM
      servo[s].write(pos);  // tell servo to go to position in variable 'pos'
#else                       // BiBoard2
      pwm.writeAngle(s, pos);
#endif
      delay(1);  // waits 15ms for the servo to reach the position
      Serial.println(pos);
    }
  }
  for (int pos = 50; pos > -50; pos -= 1) {  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    for (int s = 0; s < PWM_NUM; s++) {
#ifdef ESP_PWM
      servo[s].write(pos);  // tell servo to go to position in variable 'pos'
#else                       // BiBoard2
      pwm.writeAngle(s, pos);  // may go out of range. check!
#endif
      delay(1);  // waits 15ms for the servo to reach the position
      Serial.println(pos);
    }
  }
}

#ifdef GYRO_PIN

void allRotateWithIMU() {
  for (int s = 0; s < PWM_NUM; s++) {
#ifdef ESP_PWM
    servo[s].write(90 + ypr[1] + ypr[2]);  // tell servo to go to position in variable 'pos'
#else                                      // BiBoard2
    pwm.writeAngle(s, 90 + ypr[1] + ypr[2]);
#endif
    //    delay(1);             // waits 15ms for the servo to reach the position
    Serial.print(ypr[1]);
    Serial.print('\t');
    Serial.println(ypr[2]);
  }
}
#endif

// byte shutOrder[] = {4, 7, 8, 11, 0, 5, 6, 1, 2, 9, 10, 3};
// byte shutOrder[] = { 1, 2, 3, 5, 6, 8, 9, 10, 4, 7, 8, 11,};
// bool shutEsp32Servo = false;
