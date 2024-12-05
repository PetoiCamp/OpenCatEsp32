#ifdef ROBOT_ARM
bool pincerClosedQ = true;
#endif

void calibratedPWM(byte i, float angle, float speedRatio = 0)
{
  if (PWM_NUM == 12 && WALKING_DOF == 8 && i > 3 && i < 8) // there's no such joint in this configuration
    return;
  int actualServoIndex = (PWM_NUM == 12 && i > 3) ? i - 4 : i;
  angle = max(float(angleLimit[i][0]), min(float(angleLimit[i][1]), angle));
  int duty0 = calibratedZeroPosition[i] + currentAng[i] * rotationDirection[i];
  previousAng[i] = currentAng[i];
  currentAng[i] = angle;
  // #ifdef ROBOT_ARM
  //   if (actualServoIndex == 2 && currentAng[2] == 0 && pincerClosedQ)
  //     return;
  // #endif
  int duty = calibratedZeroPosition[i] + angle * rotationDirection[i];
  int steps = speedRatio > 0 ? int(round(abs(duty - duty0) / 1.0 /*degreeStep*/ / speedRatio)) : 0;
  // if default speed is 0, no interpolation will be used
  // otherwise the speed ratio is compared to 1 degree per second.

  for (int s = 0; s <= steps; s++)
  {
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
  // #ifdef ROBOT_ARM
  //   if (actualServoIndex == 2 && currentAng[2] == 0 && !pincerClosedQ) {
  //     shutServos(2);  //release the power on the pincer to avoid stuck
  //     pincerClosedQ = true;
  //   }
  // #endif
}

void allCalibratedPWM(int *dutyAng, byte offset = 0)
{
  for (int8_t i = DOF - 1; i >= offset; i--)
  {
    calibratedPWM(i, dutyAng[i]);
  }
}

template <typename T>
void transform(T *target, byte angleDataRatio = 1, float speedRatio = 1, byte offset = 0, int period = 0, int runDelay = 8)
{
  if (0)
  {                           //(offset != 0)) {
    T *target_[DOF - offset]; // target_ ： nearest frame in target gait
    for (int j = 0; j < DOF - offset; j++)
    {
      target_[j] = target;
    }
    int min_pose_dis[8] = {1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000};
    int min_pose_idx[8] = {};
    int gait_len = abs(period);

    int cur_dis[8] = {};
    for (int i = 0; i < gait_len; i++)
    {
      for (int j = offset; j < DOF; j++)
      {
        int currrent_vel = currentAng[j] - previousAng[j];
        int pose_difference = currentAng[j] - target[i * (DOF - offset) + j - offset];
        int vel_bonus = pose_difference * currrent_vel < 0 ? 20 : 0; // if they have different sign, then add 20 degree penalty  need tune here
        cur_dis[j - offset] = pow(currentAng[j] - target[i * (DOF - offset) + j - offset], 2) - vel_bonus;
        if (cur_dis[j - offset] < min_pose_dis[j - offset])
        {
          min_pose_dis[j - offset] = cur_dis[j - offset];
          min_pose_idx[j - offset] = i;
        }
      }
    }

    for (int j = 0; j < DOF - offset; j++)
    {
      target_[j] = target + min_pose_idx[j] * (DOF - offset);
    }

    float *svel = new float[DOF - offset];
    float curr_max_abs_svel = 0.0;
    float *evel = new float[DOF - offset];
    int *cAng_cp = new int[DOF];
    int maxDiff = 0;
    arrayNCPY(cAng_cp, currentAng, DOF);
    for (int i = offset; i < DOF; i++)
    {
      maxDiff = max(maxDiff, abs(currentAng[i] - target_[i - offset][i - offset] * angleDataRatio));
      svel[i - offset] = (currentAng[i] - previousAng[i]);
      if (curr_max_abs_svel < abs(currentAng[i] - previousAng[i]))
      {
        curr_max_abs_svel = abs(currentAng[i] - previousAng[i]);
      }
      evel[i - offset] = (target + ((min_pose_idx[i - offset] + 1 >= gait_len) ? 0 : (min_pose_idx[i - offset] + 1)) * (DOF - offset))[i - offset] - (target + min_pose_idx[i - offset] * (DOF - offset))[i - offset];
    }
    if (curr_max_abs_svel < 1)
    {
      curr_max_abs_svel = 1;
    }

    float target_max_abs_svel = 0;
    for (int i = 0; i < DOF - offset; i++)
    {
      if (target_max_abs_svel < abs(target[i] - target[DOF - offset + i]))
      {
        target_max_abs_svel = abs(target[i] - target[DOF - offset + i]);
      };
    }
    if (target_max_abs_svel < 1)
    {
      target_max_abs_svel = 1;
    }

    Serial.print(" target_max_abs_svel: ");
    Serial.print(target_max_abs_svel);
    Serial.print("\n curr_max_abs_svel: ");
    Serial.print(curr_max_abs_svel);

    float max_abs_svel_diff = target_max_abs_svel - curr_max_abs_svel;

    int max_remain_steps = 0;
    for (int j = 0; j < DOF - offset; j++)
    {
      max_remain_steps = max(max_remain_steps, gait_len - min_pose_idx[j]);
    }

    int steps = max(int(maxDiff), 5);

    int Transit_cycle = 1;
    float svel_change_per_dt = max_abs_svel_diff / (max_remain_steps + gait_len * Transit_cycle + steps);
    int rundelay = runDelay * 1000; // this delay can be changed by remote controller.

    for (int i = 0; i < steps; i++)
    {
      for (int j = 0; j < DOF - offset; j++)
      {
        // interpolation
        float A = (float)(svel[j] + evel[j]) / pow(steps, 2) - 2 * (target_[j][j] * angleDataRatio - cAng_cp[j + offset]) / pow(steps, 3);
        float B = (float)(-2 * svel[j] - evel[j]) / steps + 3 * (target_[j][j] * angleDataRatio - cAng_cp[j + offset]) / pow(steps, 2);
        calibratedPWM(j + offset, A * pow(i, 3) + B * pow(i, 2) + svel[j] * i + cAng_cp[j + offset]);
      }
      curr_max_abs_svel += svel_change_per_dt;
      delayMicroseconds(int(rundelay * target_max_abs_svel / curr_max_abs_svel));
    }

    // sync joint to the end of gait cycle

    float sync_speed[DOF - offset];
    for (int j = 0; j < DOF - offset; j++)
    {
      sync_speed[j] = (float)(gait_len - min_pose_idx[j]) / max_remain_steps;
    }

    float step_counter[DOF - offset] = {0, 0, 0, 0, 0, 0, 0, 0};
    float step_counter_pre[DOF - offset] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (int k = 0; k < max_remain_steps - 1; k++)
    {
      for (int j = 0; j < DOF - offset; j++)
      {
        step_counter[j] += sync_speed[j];
        step_counter_pre[j] = step_counter[j];
        calibratedPWM(j + offset, (target + (min_pose_idx[j] + int(step_counter[j])) * (DOF - offset))[j] * angleDataRatio);
      }
      curr_max_abs_svel += svel_change_per_dt;
      delayMicroseconds(int(rundelay * target_max_abs_svel / curr_max_abs_svel));
    }

    for (int Trs = 0; Trs < Transit_cycle; Trs++)
    {
      for (int k = 0; k < gait_len; k++)
      {
        for (int j = 0; j < DOF - offset; j++)
        {
          calibratedPWM(j + offset, *(target + k * (DOF - offset) + j) * angleDataRatio);
        }
        curr_max_abs_svel += svel_change_per_dt;
        delayMicroseconds(int(rundelay * target_max_abs_svel / curr_max_abs_svel));
      }
    }

    delete[] svel;
    delete[] evel;
    delete[] cAng_cp;
  }

  else
  {
    int *diff = new int[DOF - offset], maxDiff = 0;
    for (byte i = offset; i < DOF; i++)
    {
      if (manualHeadQ && i < HEAD_GROUP_LEN && token == T_SKILL) // the head motion will be handled by skill.perform()
        continue;
      diff[i - offset] = currentAng[i] - target[i - offset] * angleDataRatio;
      maxDiff = max(maxDiff, abs(diff[i - offset]));
    }
    int steps = speedRatio > 0 ? int(round(maxDiff / 1.0 /*degreeStep*/ / speedRatio)) : 0; // default speed is 1 degree per step
    // if (maxDiff == 0) {
    //   delete[] diff;
    //   return;
    // }
    for (int s = 0; s <= steps; s++)
    {
      if (updateGyroQ && printGyroQ)
      {
        print6Axis();
      }
      for (byte i = offset; i < DOF; i++)
      {
#ifdef ESP_PWM
        if (movedJoint[i]) // don't drive the servo if it's being moved by hand in the follow function.
          continue;
        if (manualHeadQ && i < HEAD_GROUP_LEN && token == T_SKILL) // the head motion will be handled by skill.perform()
          continue;
        if (WALKING_DOF == 8 && i > 3 && i < 8)
          continue;
        if (WALKING_DOF == 12 && i < 4)
          continue;
#endif
        float dutyAng = (target[i - offset] * angleDataRatio + (steps == 0 ? 0 : (1 + cos(M_PI * s / steps)) / 2 * diff[i - offset]));
        calibratedPWM(i, dutyAng);
      }
      delay((DOF - offset) / 2);
    }
    delete[] diff;
  }
}

// #define WEIGHT 2
// template <typename T> void transform( T * target, byte angleDataRatio = 1, float speedRatio = 2, byte offset = 0) {  // transformCubic
//   {

//     int maxDiff = 0;
//     T *nextFrame = target + DOF - offset;
//     //svel: vel at the starting point of the interpolation.   evel: vel at the ending point.
//     int *svel = new int [DOF - offset];
//     int *evel = new int [DOF - offset];
//     int *cAng_cp = new int [DOF];
//     arrayNCPY(cAng_cp, currentAng, DOF);
//     for (byte i = offset; i < DOF; i++) {
//         if (WALKING_DOF == 8 && i > 3 && i < 8)
//         continue;
//         if (WALKING_DOF == 12 && i < 4)
//         continue;
//       maxDiff = max(maxDiff, abs( currentAng[i] - target[i - offset] * angleDataRatio));
//       svel[i - offset] = (currentAng[i] - previousAng[i])/WEIGHT;
//       evel[i - offset] = ((offset != 0) ? nextFrame[i - offset] * angleDataRatio - target[i - offset] * angleDataRatio : 0)/WEIGHT;
//     }
// //    printList(currentAng);
// //    PTL();
//     int steps = int(round(maxDiff / speedRatio )); //default speed is 1 degree per step
//     //int steps = (offset!=0) ? 10:20;// interpolation points

//     for (int s = 0; s < steps; s++) {
//       for (int j = offset; j < DOF; j++) {

//             if (WALKING_DOF == 8 && j > 3 && j < 8)
//             continue;
//             if (WALKING_DOF == 12 && j < 4)
//             continue;

//         ///////////////interpolation///////////////
//         float A = (float)(svel[j - offset] + evel[j - offset])  / pow(steps, 2) - 2 * (target[j - offset] * angleDataRatio - cAng_cp[j]) / pow(steps, 3);
//         float B = (float)(-2 * svel[j - offset]  - evel[j - offset] ) / steps + 3 * (target[j - offset] * angleDataRatio - cAng_cp[j]) / pow(steps, 2);
//         float dutyAng = A * pow(s, 3) + B * pow(s, 2) + svel[j - offset]  * s + cAng_cp[j];

//         calibratedPWM (j, dutyAng);
//         delayMicroseconds(500);
//       }
// //      PTL();
//     }
// //    PTL();
// //    printList(target);
//     delete [] svel;
//     delete [] evel;
//     delete [] cAng_cp;
//   }
// }

// balancing parameters
#define ROLL_LEVEL_TOLERANCE 5 // the body is still considered as level, no angle adjustment
#define PITCH_LEVEL_TOLERANCE 3
#define NUM_ADAPT_PARAM 2                                                // number of parameters for adaption
float levelTolerance[2] = {ROLL_LEVEL_TOLERANCE, PITCH_LEVEL_TOLERANCE}; // the body is still considered as level, no angle adjustment

#define LARGE_ROLL 90
#define LARGE_PITCH 75

// the following coefficients will be divided by radPerDeg in the adjust() function. so (float) 0.1 can be saved as (int8_t) 1
// this trick allows using int8_t array insead of float array, saving 96 bytes and allows storage on EEPROM
#define panF 60
#define tiltF 60
#define sRF 50           // shoulder roll factor
#define sPF 12           // shoulder pitch factor
#define uRF 50           // upper leg roll factor
#define uPF 50           // upper leg pitch factor
#define lRF (-1.5 * uRF) // lower leg roll factor
#define lPF (-1.5 * uPF) // lower leg pitch factor
#define LEFT_RIGHT_FACTOR 2
#define FRONT_BACK_FACTOR 1.2
#define POSTURE_WALKING_FACTOR 0.5
#define ADJUSTMENT_DAMPER 5
// #ifdef POSTURE_WALKING_FACTOR
// float postureOrWalkingFactor = 1;
// #endif

#ifdef X_LEG // >< leg
float adaptiveParameterArray[][NUM_ADAPT_PARAM] = {
    {-panF / 2, 0}, {-panF / 2, -tiltF}, {-2 * panF, 0}, {0, -1 * tiltF}, {sRF, -sPF}, {-sRF, -sPF}, {-sRF, sPF}, {sRF, sPF}, {uRF, uPF}, {uRF, uPF}, {-uRF, uPF}, {-uRF, uPF}, {lRF, lPF}, {lRF, lPF}, {-lRF, lPF}, {-lRF, lPF}};
#else // >> leg
float adaptiveParameterArray[][NUM_ADAPT_PARAM] = {
    {-panF / 2, 0}, {panF / 8, -tiltF / 3}, {0, 0}, {-1 * panF, 0}, {sRF, -sPF}, {-sRF, -sPF}, {-sRF, sPF}, {sRF, sPF}, {uRF, uPF}, {uRF, uPF}, {uRF, uPF}, {uRF, uPF}, {lRF, -0.5 * lPF}, {lRF, -0.5 * lPF}, {lRF, 0.5 * lPF}, {lRF, 0.5 * lPF}};
#endif

float adjust(byte i)
{
  float rollAdj, pitchAdj, adj;
  if (i == 1 || i > 3)
  { // check idx = 1
    bool leftQ = (i - 1) % 4 > 1 ? true : false;
    // bool frontQ = i % 4 < 2 ? true : false;
    // bool upperQ = i / 4 < 3 ? true : false;
    float leftRightFactor = 1;
    if ((leftQ && balanceSlope[0] * RollPitchDeviation[0] > 0) // operator * is higher than &&
        || (!leftQ && balanceSlope[0] * RollPitchDeviation[0] < 0))
      leftRightFactor = LEFT_RIGHT_FACTOR * abs(balanceSlope[0]);
    rollAdj = (i == 1 || i > 7 ? fabs(RollPitchDeviation[0]) : RollPitchDeviation[0]) * adaptiveParameterArray[i][0] * leftRightFactor;
    //    rollAdj = fabs(RollPitchDeviation[0]) * adaptiveParameterArray[i][0] * leftRightFactor;
  }
  else
    rollAdj = RollPitchDeviation[0] * adaptiveParameterArray[i][0];

  float idealAdjust = radPerDeg * (
#ifdef POSTURE_WALKING_FACTOR
                                      (i > 3 ? POSTURE_WALKING_FACTOR : 1) *
#endif
                                          balanceSlope[0] * rollAdj -
                                      balanceSlope[1] * adaptiveParameterArray[i][1] * ((i % 4 < 2) ? (RollPitchDeviation[1]) : RollPitchDeviation[1]));
#ifdef ADJUSTMENT_DAMPER
  currentAdjust[i] += max(min(idealAdjust - currentAdjust[i], float(ADJUSTMENT_DAMPER)), -float(ADJUSTMENT_DAMPER));
#else
  currentAdjust[i] = idealAdjust;
#endif
  currentAdjust[i] = max(float(-45), min(float(45), currentAdjust[i]));
  return currentAdjust[i];
}

int calibratePincerByVibration(int start, int end, int step, int threshold = 10000 * gFactor)
{
  PTT("Try ", start);
  PTT(" ~ ", end);
  PTTL(" by ", step);
  float angLag0 = xyzReal[0];
  float angLag1 = xyzReal[1];
  calibratedPWM(2, 20);
  delay(300);
  for (int a = start; a < end; a += step)
  {
    calibratedPWM(2, -120);
    delay(300);
    angLag0 = xyzReal[0];
    calibratedPWM(2, a);
    long startTime = millis();
    long after;
    // int maxVibration = 0;
    // int correspondingAng;
    do
    {
      print6Axis();
      after = millis() - startTime;
      // readIMU();
      // delay(50);
      float diff0 = angLag0 - xyzReal[0];
      if (diff0)
      {
        // if (abs(diff0) > abs(maxVibration)) {
        //   maxVibration = diff0;
        //   correspondingAng = a;
        // }
        if (abs(diff0) > threshold)
        {
          PTHL(a, abs(diff0));
          return a;
        }
        angLag0 = xyzReal[0];
      }
    } while (after <= 200);
    // PTHL(a, maxVibration);
  }
  return end;
}

int calibrationReference[] = {
// the angle difference between P1L and P1S is significant for auto calibration.
#ifdef NYBBLE // with plastic servo P1L
    0, 42, 0, 0, 0, 0, 0, 0,
    72, 72, -68, -68, -63, -63, 63, 63
#elif defined ROBOT_ARM //  Bittle R with metal servo P1S
    0, 55, 0, 0, 0, 0, 0, 0,
    65, 65, 77, 77, -63, -63, -63, -63
#else                   // Bittle with plastic servo P1L
    0, 0, 0, 0, 0, 0, 0, 0,
    73, 73, 76, 76, -66, -66, -66, -66
#endif
};

// int calibrationReference2[] = {
// #ifdef NYBBLE  //with plastic servo P1L
//   0, 42, 0, 0, 0, 0, 0, 0,
//   72, 72, -68, -68, -63, -63, 63, 63
// #elif defined ROBOT_ARM  //  Bittle R with metal servo P1S
//   0, 50, 0, 0, 0, 0, 0, 0,
//   -85, -85, -70, -70,83, 83, 83, 83
// #else                    //Bittle with plastic servo P1L
//   0, 0, 0, 0, 0, 0, 0, 0,
//   73, 73, 76, 76, -66, -66, -66, -66
// #endif
// };
void autoCalibrate()
{
  // PTLF("Auto calibration reference:");
  // printList(calibrationReference);
  PTLF("Push the robot tightly to the ground. Enter any character when ready.");
  while (!Serial.available()) // wait for user input
    ;
  while (Serial.available())
    Serial.read();
  for (byte t = 0; t < connectedCountDown; t++)
  {
    servoFeedback();
    // printList(movedJoint);
    // printList(connectedFeedbackServo);
  }
  for (byte i = 0; i < DOF; i++)
  {
    if (connectedFeedbackServo[i] > 0)
    {
      int diff = currentAng[i] - calibrationReference[i];
      servoCalib[i] += diff;
    }
  }
  saveCalib(servoCalib);
  tQueue->addTask(T_REST, "");
  tQueue->addTask(T_CALIBRATE, "");
  measureServoPin = 16; // reattach the servos in the next reaction loop
}

void signalGenerator(int8_t resolution, int8_t speed, int8_t *pars, int8_t len, bool move = 1, char curveMethod = 't')
{ // trigonometric
  // circular
  int targetFrame[DOF + 1];
  // arrayNCPY(targetFrame, currentAng, DOF);
  for (int i = 0; i < DOF; i++)
    targetFrame[i] = currentAng[i];
  for (int t = 0; t < 360; t += resolution)
  {
    for (int8_t i = 0; i < len / 5; i++)
    {
      int8_t jointIdx = pars[i * 5];
      int8_t midpoint = pars[i * 5 + 1];
      int8_t amp = pars[i * 5 + 2];
      int8_t freq = pars[i * 5 + 3];
      int8_t phase = pars[i * 5 + 4];
      // PTHL("len", (len + 1) / 5);
      // PTHL("jointIdx", jointIdx);
      // PTHL("amp", amp);
      // PTHL("freq", freq);
      // PTHL("phase", phase);
      // PTHL("midpoint", midpoint);
      int angle;
      if (curveMethod == 't')
        // angle = round(amp * sin(M_PI * 2.0 * freq * (t + phase*3/freq) / 360.0)) + midpoint;//phase: 120 => 1 full period
        // else if(curveMethod == 'c')
        angle = midpoint + round(amp * sin(2.0 * M_PI * ((t + phase * 3 / freq) / (360.0 / freq))));
      PTT(angle, ',');
      targetFrame[jointIdx] = angle;
    }
    if (move)
      transform(targetFrame, 1, speed);
    PTL();
  }
}
#define MAX_FRAME 125
#define IDLE_LEARN 2000
#define SMALL_DIFF 3
#define READY_COUNTDOWN 10
int totalFrame = 0;
int8_t learnData[11 * MAX_FRAME];
int8_t learnDataPrev[11];
void learnByDrag()
{
  totalFrame = 0;
  bool skip = false;
  int getReady = 0;

  while (getReady < READY_COUNTDOWN)
  {
    PTHL("ready", READY_COUNTDOWN - getReady);
    readAllFeedbackFast();
    int diff = 0;
    for (int i = 0; i < 11; i++)
    {
      int j = (i > 2) ? i + 5 : i;
      learnData[totalFrame * 11 + i] = currentAng[j];
      diff += (currentAng[j] - learnDataPrev[i]) * (currentAng[j] - learnDataPrev[i]);
      learnDataPrev[i] = currentAng[j];
    }
    PTHL("diff", diff);
    if (diff <= SMALL_DIFF)
      getReady++;
    else
      getReady = 0;
  }
  beep(30, 300);
  PTL("Start to record motion");
  long idleLearnTimer = millis();
  while (totalFrame < MAX_FRAME                     // not exceed the max frame
         && !Serial.available()                     // not ended by user
         && millis() - idleLearnTimer < IDLE_LEARN) // not idle for a long time
  {
    if (!(totalFrame % 10))
      PTT(totalFrame, '\t');
    readAllFeedbackFast();
    int diff = 0;
    for (int i = 0; i < 11; i++)
    {
      int j = (i > 2) ? i + 5 : i;
      learnData[totalFrame * 11 + i] = currentAng[j];
      diff += (currentAng[j] - learnDataPrev[i]) * (currentAng[j] - learnDataPrev[i]);
    }
    // PTHL("diff", diff);
    if (diff > SMALL_DIFF)
    { // won't record if the joints are not moved
      for (int i = 0; i < 11; i++)
        learnDataPrev[i] = learnData[totalFrame * 11 + i];
      idleLearnTimer = millis();
      totalFrame++;
    }
  }
  while (Serial.available())
    Serial.read();
  beep(30, 300);
  tQueue->addTask('k', "up");
  measureServoPin = 16; // reattach the servos in the next reaction loop
}
void performLearn()
{
  int target[DOF];
  for (int i = 0; i < DOF; i++)
    target[i] = currentAng[i];
  PTL('{');
  PTTL(-totalFrame, ",0,0,1,\n0,0,0,");
  for (int f = 0; f < totalFrame; f++)
  {
    for (int i = 0; i < 11; i++)
    {
      int j = (i > 2) ? i + 5 : i;
      target[j] = learnData[f * 11 + i];
      PTT(target[j], ",\t");
      if (i == 2)
        PT("0,0,0,0,0,\t");
    }
    PTL("8,0,0,0,");
    transform(target, 1, 2);
  }
  PTL("};");
}
