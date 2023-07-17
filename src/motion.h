void calibratedPWM(byte i, float angle, float speedRatio = 0) {
  int actualServoIndex = (PWM_NUM == 12 && i > 3) ? i - 4 : i;
  angle = max(float(angleLimit[i][0]), min(float(angleLimit[i][1]), angle));
  int duty0 = calibratedZeroPosition[i] + currentAng[i] * rotationDirection[i];
  previousAng[i] = currentAng[i];
  currentAng[i] = angle;
  int duty = calibratedZeroPosition[i] + angle * rotationDirection[i];
  int steps = speedRatio > 0 ? int(round(abs(duty - duty0) / 1.0 /*degreeStep*/ / speedRatio)) : 0;
  //if default speed is 0, no interpolation will be used
  //otherwise the speed ratio is compared to 1 degree per second.

  for (int s = 0; s <= steps; s++) {
    int degree = duty + (steps == 0 ? 0 : (1 + cos(M_PI * s / steps)) / 2 * (duty0 - duty));
#ifdef ESP_PWM
    servo[actualServoIndex].write(degree);
#else
    pwm.writeAngle(actualServoIndex, degree);
#endif
    //    delayMicroseconds(1);
  }
}

void allCalibratedPWM(int *dutyAng, byte offset = 0) {
  for (int8_t i = DOF - 1; i >= offset; i--) {
    calibratedPWM(i, dutyAng[i]);
  }
}

template<typename T> void transform(T *target, byte angleDataRatio = 1, float speedRatio = 1, byte offset = 0, int period = 0, int runDelay = 8) {
  if (0) {                     //(offset != 0)) {
    T *target_[DOF - offset];  // target_ ： nearest frame in target gait
    for (int j = 0; j < DOF - offset; j++) { target_[j] = target; }
    int min_pose_dis[8] = { 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000 };
    int min_pose_idx[8] = {};
    int gait_len = abs(period);

    int cur_dis[8] = {};
    for (int i = 0; i < gait_len; i++) {
      for (int j = offset; j < DOF; j++) {
        int currrent_vel = currentAng[j] - previousAng[j];
        int pose_difference = currentAng[j] - target[i * (DOF - offset) + j - offset];
        int vel_bonus = pose_difference * currrent_vel < 0 ? 20 : 0;  // if they have different sign, then add 20 degree penalty  need tune here
        cur_dis[j - offset] = pow(currentAng[j] - target[i * (DOF - offset) + j - offset], 2) - vel_bonus;
        if (cur_dis[j - offset] < min_pose_dis[j - offset]) {
          min_pose_dis[j - offset] = cur_dis[j - offset];
          min_pose_idx[j - offset] = i;
        }
      }
    }

    for (int j = 0; j < DOF - offset; j++) {
      target_[j] = target + min_pose_idx[j] * (DOF - offset);
    }

    float *svel = new float[DOF - offset];
    float curr_max_abs_svel = 0.0;
    float *evel = new float[DOF - offset];
    int *cAng_cp = new int[DOF];
    int maxDiff = 0;
    arrayNCPY(cAng_cp, currentAng, DOF);
    for (int i = offset; i < DOF; i++) {
      maxDiff = max(maxDiff, abs(currentAng[i] - target_[i - offset][i - offset] * angleDataRatio));
      svel[i - offset] = (currentAng[i] - previousAng[i]);
      if (curr_max_abs_svel < abs(currentAng[i] - previousAng[i])) { curr_max_abs_svel = abs(currentAng[i] - previousAng[i]); }
      evel[i - offset] = (target + ((min_pose_idx[i - offset] + 1 >= gait_len) ? 0 : (min_pose_idx[i - offset] + 1)) * (DOF - offset))[i - offset] - (target + min_pose_idx[i - offset] * (DOF - offset))[i - offset];
    }
    if (curr_max_abs_svel < 1) { curr_max_abs_svel = 1; }

    float target_max_abs_svel = 0;
    for (int i = 0; i < DOF - offset; i++) {
      if (target_max_abs_svel < abs(target[i] - target[DOF - offset + i])) { target_max_abs_svel = abs(target[i] - target[DOF - offset + i]); };
    }
    if (target_max_abs_svel < 1) { target_max_abs_svel = 1; }

    Serial.print(" target_max_abs_svel: ");
    Serial.print(target_max_abs_svel);
    Serial.print("\n curr_max_abs_svel: ");
    Serial.print(curr_max_abs_svel);

    float max_abs_svel_diff = target_max_abs_svel - curr_max_abs_svel;

    int max_remain_steps = 0;
    for (int j = 0; j < DOF - offset; j++) {
      max_remain_steps = max(max_remain_steps, gait_len - min_pose_idx[j]);
    }

    int steps = max(int(maxDiff), 5);

    int Transit_cycle = 1;
    float svel_change_per_dt = max_abs_svel_diff / (max_remain_steps + gait_len * Transit_cycle + steps);
    int rundelay = runDelay * 1000;  // this delay can be changed by remote controller.

    for (int i = 0; i < steps; i++) {
      for (int j = 0; j < DOF - offset; j++) {
        //interpolation
        float A = (float)(svel[j] + evel[j]) / pow(steps, 2) - 2 * (target_[j][j] * angleDataRatio - cAng_cp[j + offset]) / pow(steps, 3);
        float B = (float)(-2 * svel[j] - evel[j]) / steps + 3 * (target_[j][j] * angleDataRatio - cAng_cp[j + offset]) / pow(steps, 2);
        calibratedPWM(j + offset, A * pow(i, 3) + B * pow(i, 2) + svel[j] * i + cAng_cp[j + offset]);
      }
      curr_max_abs_svel += svel_change_per_dt;
      delayMicroseconds(int(rundelay * target_max_abs_svel / curr_max_abs_svel));
    }

    // sync joint to the end of gait cycle

    float sync_speed[DOF - offset];
    for (int j = 0; j < DOF - offset; j++) {
      sync_speed[j] = (float)(gait_len - min_pose_idx[j]) / max_remain_steps;
    }

    float step_counter[DOF - offset] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    float step_counter_pre[DOF - offset] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    for (int k = 0; k < max_remain_steps - 1; k++) {
      for (int j = 0; j < DOF - offset; j++) {
        step_counter[j] += sync_speed[j];
        step_counter_pre[j] = step_counter[j];
        calibratedPWM(j + offset, (target + (min_pose_idx[j] + int(step_counter[j])) * (DOF - offset))[j] * angleDataRatio);
      }
      curr_max_abs_svel += svel_change_per_dt;
      delayMicroseconds(int(rundelay * target_max_abs_svel / curr_max_abs_svel));
    }

    for (int Trs = 0; Trs < Transit_cycle; Trs++) {
      for (int k = 0; k < gait_len; k++) {
        for (int j = 0; j < DOF - offset; j++) {
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

  else {
    int *diff = new int[DOF - offset], maxDiff = 0;
    for (byte i = offset; i < DOF; i++) {
      if (manualHeadQ && i < HEAD_GROUP_LEN && token == T_SKILL)  // the head motion will be handled by skill.perform()
        continue;
      diff[i - offset] = currentAng[i] - target[i - offset] * angleDataRatio;
      maxDiff = max(maxDiff, abs(diff[i - offset]));
    }
    int steps = speedRatio > 0 ? int(round(maxDiff / 1.0 /*degreeStep*/ / speedRatio)) : 0;  //default speed is 1 degree per step
    // if (maxDiff == 0) {
    //   delete[] diff;
    //   return;
    // }
    for (int s = 0; s <= steps; s++) {
      for (byte i = offset; i < DOF; i++) {
#ifdef ESP_PWM
        if (manualHeadQ && i < HEAD_GROUP_LEN && token == T_SKILL)  // the head motion will be handled by skill.perform()
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
#define ROLL_LEVEL_TOLERANCE 5  //the body is still considered as level, no angle adjustment
#define PITCH_LEVEL_TOLERANCE 3
#define NUM_ADAPT_PARAM 2                                                   // number of parameters for adaption
float levelTolerance[2] = { ROLL_LEVEL_TOLERANCE, PITCH_LEVEL_TOLERANCE };  //the body is still considered as level, no angle adjustment

#define LARGE_ROLL 90
#define LARGE_PITCH 75

//the following coefficients will be divided by radPerDeg in the adjust() function. so (float) 0.1 can be saved as (int8_t) 1
//this trick allows using int8_t array insead of float array, saving 96 bytes and allows storage on EEPROM
#define panF 60
#define tiltF 60
#define sRF 50            //shoulder roll factor
#define sPF 12            //shoulder pitch factor
#define uRF 50            //upper leg roll factor
#define uPF 50            //upper leg pitch factor
#define lRF (-1.5 * uRF)  //lower leg roll factor
#define lPF (-1.5 * uPF)  //lower leg pitch factor
#define LEFT_RIGHT_FACTOR 2
#define FRONT_BACK_FACTOR 1.2
#define POSTURE_WALKING_FACTOR 0.5
#define ADJUSTMENT_DAMPER 5
//#ifdef POSTURE_WALKING_FACTOR
//float postureOrWalkingFactor = 1;
//#endif

#ifdef X_LEG  // >< leg
float adaptiveParameterArray[][NUM_ADAPT_PARAM] = {
  { -panF, 0 }, { -panF / 2, -tiltF }, { -2 * panF, 0 }, { 0, 0 }, { sRF, -sPF }, { -sRF, -sPF }, { -sRF, sPF }, { sRF, sPF }, { uRF, uPF }, { uRF, uPF }, { -uRF, uPF }, { -uRF, uPF }, { lRF, lPF }, { lRF, lPF }, { -lRF, lPF }, { -lRF, lPF }
};
#else  // >> leg
float adaptiveParameterArray[][NUM_ADAPT_PARAM] = {
  { -panF, 0 }, { -panF / 2, -tiltF }, { -2 * panF, 0 }, { 0, 0 }, { sRF, -sPF }, { -sRF, -sPF }, { -sRF, sPF }, { sRF, sPF }, { uRF, uPF }, { uRF, uPF }, { uRF, uPF }, { uRF, uPF }, { lRF, -0.5 * lPF }, { lRF, -0.5 * lPF }, { lRF, 0.5 * lPF }, { lRF, 0.5 * lPF }
};
#endif

float adjust(byte i) {
  float rollAdj, pitchAdj, adj;
  if (i == 1 || i > 3) {  //check idx = 1
    bool leftQ = (i - 1) % 4 > 1 ? true : false;
    //bool frontQ = i % 4 < 2 ? true : false;
    //bool upperQ = i / 4 < 3 ? true : false;
    float leftRightFactor = 1;
    if ((leftQ && slope * RollPitchDeviation[0] > 0)  // operator * is higher than &&
        || (!leftQ && slope * RollPitchDeviation[0] < 0))
      leftRightFactor = LEFT_RIGHT_FACTOR * abs(slope);
    rollAdj = (i == 1 || i > 7 ? fabs(RollPitchDeviation[0]) : RollPitchDeviation[0]) * adaptiveParameterArray[i][0] * leftRightFactor;
    //    rollAdj = fabs(RollPitchDeviation[0]) * adaptiveParameterArray[i][0] * leftRightFactor;

  } else
    rollAdj = RollPitchDeviation[0] * adaptiveParameterArray[i][0];

  float idealAdjust = radPerDeg * (
#ifdef POSTURE_WALKING_FACTOR
                        (i > 3 ? POSTURE_WALKING_FACTOR : 1) *
#endif
                          rollAdj
                        - slope * adaptiveParameterArray[i][1] * ((i % 4 < 2) ? (RollPitchDeviation[1]) : RollPitchDeviation[1]));
#ifdef ADJUSTMENT_DAMPER
  currentAdjust[i] += max(min(idealAdjust - currentAdjust[i], float(ADJUSTMENT_DAMPER)), -float(ADJUSTMENT_DAMPER));
#else
  currentAdjust[i] = idealAdjust;
#endif
  return currentAdjust[i];
}
