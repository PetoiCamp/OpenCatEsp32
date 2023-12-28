#define PT(s) Serial.print(s)  // abbreviate print commands
#define PTD(s, format) Serial.print(s, format)
#define PT_FMT(s, format) Serial.print(s, format)  // abbreviate print commands
#define PTL(s) Serial.println(s)
#define PTF(s) Serial.print(F(s))  //trade flash memory for dynamic memory with F() function
#define PTLF(s) Serial.println(F(s))
#define PTT(s, delimeter) \
  { \
    Serial.print(s); \
    Serial.print(delimeter); \
  }
#define PTH(head, value) \
  { \
    Serial.print(head); \
    Serial.print('\t'); \
    Serial.println(value); \
  }
#include "ESP32Servo.h"
#define SERVO_FREQ 200
//------------------angleRange  frequency  minPulse  maxPulse;
enum ServoModel_t {
  G41 = 0,
  P1S,
  P2K
};
ServoModel servoP1S(270, SERVO_FREQ, 500, 2500);  // 1s/4 = 250ms 250ms/2500us=

#define P_STEP 32
#define P_BASE 3000 + 3 * P_STEP  // 3000~3320
#define P_HARD (P_BASE + P_STEP * 2)
#define P_SOFT (P_BASE - P_STEP * 2)
#define FEEDBACK_SIGNAL 3500
#define PWM_NUM 12
#define DOF 16
#define WALKING_DOF 8
int feedbackSignal = FEEDBACK_SIGNAL;
int waitTimeForResponse = 10000;
int maxPulseWidth = 2600;

Servo servo[PWM_NUM];  // create servo object to control a servo
// 16 servo objects can be created on the ESP32

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42

int measureServoPin = -1;
byte nPulse = 3;

float degPerRad = 180.0 / M_PI;
float radPerDeg = M_PI / 180.0;
const uint8_t PWM_pin[PWM_NUM] = {
  19, 4, 2, 27,   // head or shoulder roll
  33, 5, 15, 14,  // shoulder pitch
  32, 18, 13, 12  // knee
};
int zeroPosition[DOF] = {};
int calibratedZeroPosition[DOF] = {};
int currentAng[DOF] = { -30, -80, -45, 0,
                        0, 0, 0, 0,
                        75, 75, 75, 75,
                        -55, -55, -55, -55 };
int8_t servoCalib[DOF] = { 0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 0 };
int8_t rotationDirection[] = { 1, -1, 1, 1,
                               1, -1, 1, -1,
                               1, -1, -1, 1,
                               -1, 1, 1, -1 };
int8_t middleShift[] = { 0, 15, 0, 0,
                         -45, -45, -45, -45,
                         55, 55, -55, -55,
                         -55, -55, -55, -55 };
int angleLimit[][2] = {
  { -120, 120 },
  { -30, 80 },
  { -120, 120 },
  { -120, 120 },
  { -90, 60 },
  { -90, 60 },
  { -90, 90 },
  { -90, 90 },
  { -200, 80 },
  { -200, 80 },
  { -80, 200 },
  { -80, 200 },
  { -80, 200 },
  { -80, 200 },
  { -70, 200 },
  { -80, 200 },
};
void attachAllESPServos() {
  PTLF("Calibrated Zero Position");
  ServoModel *model = &servoP1S;
  for (int c = 0; c < PWM_NUM; c++) {
    byte s = c;  // attachOrder[c];
    int joint;
    if (WALKING_DOF == 8)
      joint = (s > 3) ? s + 4 : s;
    else  // if (WALKING_DOF == 12)
      joint = s + 4;


    servo[s].attach(PWM_pin[s], model);
    zeroPosition[joint] = model->getAngleRange() / 2 + float(middleShift[joint]) * rotationDirection[joint];
    calibratedZeroPosition[joint] = zeroPosition[joint] + float(servoCalib[joint]) * rotationDirection[joint];
    PT(calibratedZeroPosition[joint]);
    PT('\t');
  }
  PTL();
}

void servoSetup() {
#ifdef INVERSE_SERVO_DIRECTION
  for (byte s = 0; s < DOF; s++)
    rotationDirection[s] *= -1;
#endif

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
}

void shutServos() {
  ServoModel *model;
  for (byte s = 0; s < PWM_NUM; s++) {  // PWM_NUM
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
  }
  //  shutEsp32Servo = false;
}

void setServoP(unsigned int p) {
  for (byte s = 0; s < PWM_NUM; s++)
    servo[s].writeMicroseconds(p);
}

int measurePulseWidth(uint8_t pwmReadPin) {
  long start = micros();
  while (!digitalRead(pwmReadPin)) {  //等待高电平
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

float readFeedback(byte s) {  //s is not the joint index, but the pwm pin index that may shift by 4
  ServoModel *model = &servoP1S;
  byte modelIndex = s < 4 ? s : s + 4;

  servo[s].attach(PWM_pin[s], model);
  delay(12);                                   //it takes time to attach
  servo[s].writeMicroseconds(feedbackSignal);  // set servo to mid-point
                                               // myservo.writeMicroseconds(feedbackSignal);  // set servo to mid-point
  servo[s].detach();
  pinMode(PWM_pin[s], INPUT);
  float mean = 0;
  int n = nPulse;
  for (byte i = 0; i < nPulse; i++) {  //测三次求平均值
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

void servoFeedback(int8_t index = 16) { //only apply to specific models
  if (index > -1 && index < 16) {
    byte i = index < 4 ? index : index - 4;
    int feedback = readFeedback(i);
    if (feedback > -1) {
      PTT(index, '\t')
      PTD((servo[i].pulseToAngle(feedback) - calibratedZeroPosition[index]) / rotationDirection[index], 1);
      PTL();
    }
  } else {
    for (byte i = 0; i < 12; i++) {
      byte jointIdx = i < 4 ? i : i + 4;
      int feedback = readFeedback(i);
      if (feedback > -1) {  //duty[i] = calibratedZeroPosition[i] + angle * rotationDirection[i];
                            //angle = (duty[i] - calibratedZeroPosition[i])/rotationDirection[i];
        PTD((servo[i].pulseToAngle(feedback) - calibratedZeroPosition[jointIdx]) / rotationDirection[jointIdx], 1);
        PT('\t');
      }
    }
    PTL();
  }
}
void calibratedPWM(byte i, float angle, float speedRatio = 0) {
  int actualServoIndex = (PWM_NUM == 12 && i > 3) ? i - 4 : i;
  angle = max(float(angleLimit[i][0]), min(float(angleLimit[i][1]), angle));
  int duty0 = calibratedZeroPosition[i] + currentAng[i] * rotationDirection[i];
  currentAng[i] = angle;
  int duty = calibratedZeroPosition[i] + angle * rotationDirection[i];
  int steps = speedRatio > 0 ? int(round(abs(duty - duty0) / 1.0 /*degreeStep*/ / speedRatio)) : 0;
  //if default speed is 0, no interpolation will be used
  //otherwise the speed ratio is compared to 1 degree per second.

  for (int s = 0; s <= steps; s++) {
    int degree = duty + (steps == 0 ? 0 : (1 + cos(M_PI * s / steps)) / 2 * (duty0 - duty));
    servo[actualServoIndex].write(degree);
  }
}
void allRotate() {
  for (int pos = -50; pos < 50; pos += 1) {
    // in steps of 1 degree
    for (int s = 0; s < 16; s++) {
      if (s > 3 && s < 8)  //there's no pin for joint 4,5,6,7 on BiBoard
        continue;
      calibratedPWM(s, s > 11 ? pos : -pos, 1);  // tell servo to go to position in variable 'pos'
      Serial.println(pos);
    }
    delay(5);
  }
  for (int pos = 50; pos > -50; pos -= 1) {
    for (int s = 0; s < 16; s++) {
      if (s > 3 && s < 8)  //there's no pin for joint 4,5,6,7 on BiBoard
        continue;
      calibratedPWM(s, s > 11 ? pos : -pos, 1);  // tell servo to go to position in variable 'pos'
      Serial.println(pos);
    }
    delay(5);
  }
}

void setup() {
  setServoP(P_SOFT);
  servoSetup();
}
void loop() {
  allRotate();
}
