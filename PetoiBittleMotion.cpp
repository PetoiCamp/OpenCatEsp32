#include "PetoiBittleMotion.h"
#include "PetoiESP32PWMServo.h"
#include "BiboardPinDef.h"
#include "Instincts.h"
#include "PetoiMPU6050DMP.h"    // Simplified MPU6050 DMP library, outputs Euler angles in degrees

#define DOF 12
#define WALKING_DOF 8
#define SKILL wkF
//
//int angleArray1[DOF] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//int angleArray2[DOF] = {145, 145, 145, 145, 145, 145, 145, 145, 145, 145, 145, 145};
//
//int midAngleShift[DOF] = {0, 100, 100, 190, 235, 0, 0, 100, 190, 100, 280, 0};

int caliAngle[DOF] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

ESP32PWMServo *servo = new ESP32PWMServo[DOF];
int middleAngle[] = {145, 145, 145, 145, 145, 145, 145, 145, 145, 145, 145, 145};

int middleShifts[] = {0, 15, 0, 0,
                      //-45, -45, -45, -45,
                      45, 45, -45, -45,
                      -75, -75, -75, -75
                     };
int servoCalibs[DOF] = {0, 0, 0, 0,
                        6, -1, -7, 0,
                        -5, 7, -6, -5
                       };
int rotationDirections[] = {1, -1, 1, 1,
                            //1, -1, 1, -1,
                            1, -1, -1, 1,
                            -1, 1, 1, -1
                           };

// balancing parameters
#define ROLL_LEVEL_TOLERANCE 3//the body is still considered as level, no angle adjustment
#define PITCH_LEVEL_TOLERANCE 2
#define NUM_ADAPT_PARAM  2    // number of parameters for adaption
float levelTolerance[2] = {ROLL_LEVEL_TOLERANCE, PITCH_LEVEL_TOLERANCE}; //the body is still considered as level, no angle adjustment

#define LARGE_ROLL 90
#define LARGE_PITCH 75

//the following coefficients will be divided by radPerDeg in the adjust() function. so (float) 0.1 can be saved as (int8_t) 1
//this trick allows using int8_t array insead of float array, saving 96 bytes and allows storage on EEPROM
#define panF 60
#define tiltF 60
#define sRF 50    //shoulder roll factor
#define sPF 12    //shoulder pitch factor
#define uRF 60    //upper leg roll factor
#define uPF 30    //upper leg pitch factor
#define lRF (-1.5*uRF)  //lower leg roll factor 
#define lPF (-1.5*uPF)  //lower leg pitch factor
#define LEFT_RIGHT_FACTOR 1.2
#define FRONT_BACK_FACTOR 1.2
#define POSTURE_WALKING_FACTOR 0.7
#ifdef POSTURE_WALKING_FACTOR
float postureOrWalkingFactor;
#endif

#ifdef X_LEG  // >< leg
int adaptiveParameterArray[16][NUM_ADAPT_PARAM] = {
  { -panF, 0}, { -panF, -tiltF}, { -2 * panF, 0}, {0, 0},
  // {sRF, -sPF}, { -sRF, -sPF}, { -sRF, sPF}, {sRF, sPF},
  {uRF, uPF}, {uRF, uPF}, { -uRF, uPF}, { -uRF, uPF},
  {lRF, lPF}, {lRF, lPF}, { -lRF, lPF}, { -lRF, lPF}
};
#else         // >> leg
int adaptiveParameterArray[16][NUM_ADAPT_PARAM] = {
  { -panF, 0}, { -panF / 2, -tiltF}, { -2 * panF, 0}, {0, 0},
  //{sRF, -sPF}, { -sRF, -sPF}, { -sRF, sPF}, {sRF, sPF},
  {uRF, uPF}, {uRF, uPF}, {uRF, uPF}, {uRF, uPF},
  {lRF, -0.5 * lPF}, {lRF, -0.5 * lPF}, {lRF, 0.5 * lPF}, {lRF, 0.5 * lPF}
};
#endif
float degPerRad = 180 / M_PI;
float radPerDeg = M_PI / 180;
float RollPitchDeviation[2];
float currentAdjust[DOF] = {};
int ramp = 1;
inline int adaptiveCoefficient(byte idx, byte para) {
  return adaptiveParameterArray[idx][para];

}

float adjust(byte i) {
  float rollAdj, pitchAdj, adj;
  if (i == 1 || i > 3)  {//check idx = 1
    bool leftQ = (i - 1 ) % 4 > 1 ? true : false;
    //bool frontQ = i % 4 < 2 ? true : false;
    //bool upperQ = i / 4 < 3 ? true : false;
    float leftRightFactor = 1;
    if ((leftQ && ramp * RollPitchDeviation[0] > 0 )// operator * is higher than &&
        || ( !leftQ && ramp * RollPitchDeviation[0] < 0))
      leftRightFactor = LEFT_RIGHT_FACTOR * abs(ramp);
    rollAdj = fabs(RollPitchDeviation[0]) * adaptiveCoefficient(i, 0) * leftRightFactor;

  }
  else
    rollAdj = RollPitchDeviation[0] * adaptiveCoefficient(i, 0) ;
  currentAdjust[i] = radPerDeg * (
#ifdef POSTURE_WALKING_FACTOR
                       (i > 3 ? postureOrWalkingFactor : 1) *
#endif
                       rollAdj - ramp * adaptiveCoefficient(i, 1) * ((i % 4 < 2) ? ( RollPitchDeviation[1]) : RollPitchDeviation[1]));
  return currentAdjust[i];
}

void servoInit() {
  for (byte s = 0; s < 12; s++) {
    servo[s] = ESP32PWMServo(PETOI_P1S);
    servo[s].attach(PWM_pin[s], s);
    servo[s].write(0 );
    servo[s].write(middleAngle[s] + (middleShifts[s] + servoCalibs[s]) * rotationDirections[s] );
    Serial.print(middleAngle[s]);
    Serial.print("\t");
  }
  Serial.println();

//  while (!Serial.available());
//  char resetJointCalibrationQ = Serial.read();
}

float angle = 0;
float aStep = 0.5;
int joint = 16;
float expectedRollPitch[2];
template <typename T> int8_t sign(T val) {
  return (T(0) < val) - (val < T(0));
}
void testRun() {
  int motionArray[DOF];
  float ypr[3];                   // Yaw Pitch Raw data in degrees from MPU6050 DMP
  servoInit();
  while (0) {
    if (Serial.available() > 0) {
      String inBuffer = Serial.readStringUntil('\n');
      joint = atoi(inBuffer.c_str());

    }
    if (joint < 12) {
      //servo[joint].write(( angle));
      servo[joint].write(middleAngle[joint] + rotationDirections[joint] * (middleShifts[joint] + angle));
      Serial.println(angle);
    }
    else {
      for (byte s = 0; s < 12; s++) {
        servo[s].write(middleAngle[s] + (s > 7 ? -1 : 1)*rotationDirections[s] * (middleShifts[s] + angle));
      }
      //Serial.println();
    }

    delay(5);
    angle += aStep;
    if (angle > 100 || angle < -100)
      aStep *= -1;

  }
  int period = SKILL[0];
  for (byte g = 0; g < 2; g++)
    expectedRollPitch[g] = SKILL[1 + g];
  int t = 0;

  while (1) {
    getDMPRawResult();
    getDMPReadableYawPitchRaw(ypr);
    for (byte i = 0; i < 2; i++) {
      ypr[2 - i] = -ypr[2 - i];
      RollPitchDeviation[i] = ypr[2 - i] * 180.0 / M_PI - expectedRollPitch[i]; //all in degrees
      //PTL(RollPitchDeviation[i]);
      RollPitchDeviation[i] = sign(ypr[2 - i]) * max(fabs(RollPitchDeviation[i]) - levelTolerance[i], 0.0);//filter out small angles
    }
    for (int j = 0; j < 8; j++) {
      servo[j + 4].write(middleAngle[j + 4] + rotationDirections[j + 4] * (middleShifts[j + 4] + servoCalibs[j + 4] + SKILL[4 + t * 8 + j] +
                         adjust(j + 4)));
//      Serial.print(adjust(j + 4));
//      Serial.print("\t");
    }
    //Serial.println();
     delay(8);
    t++;
    if (t == period)t = 0;
  }
}
