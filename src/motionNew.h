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
#ifdef BiBoard
    servo[actualServoIndex].write(duty + (steps == 0 ? 0 : (1 + cos(M_PI * s / steps)) / 2 * (duty0 - duty)));
#else
    pwm.writeAngle(actualServoIndex, duty + (steps == 0 ? 0 : (1 + cos(M_PI * s / steps)) / 2 * (duty0 - duty)));
#endif
    //    delayMicroseconds(1);
  }
}

void allCalibratedPWM(char *dutyAng, byte offset = 0) {
  for (int8_t i = DOF - 1; i >= offset; i--) {
    calibratedPWM(i, dutyAng[i]);
  }
}



// template <typename T> void transform( T * target, byte angleDataRatio = 1, float speedRatio = 1, byte offset = 0) {
//   //  PTL("transform");
//   int *diff = new int [DOF - offset], maxDiff = 0;
//   for (byte i = offset; i < DOF; i++) {
//     diff[i - offset] =   currentAng[i] - target[i - offset] * angleDataRatio;
//     maxDiff = max(maxDiff, abs( diff[i - offset]));
//   }
//   int steps = speedRatio > 0 ? int(round(maxDiff / 1.0/*degreeStep*/ / speedRatio)) : 0 ; //default speed is 1 degree per step

//   for (int s = 0; s <= steps; s++) {
//     for (byte i = offset; i < DOF; i++) {
// #ifdef BiBoard
//       if (WALKING_DOF == 8 && i > 3 && i < 8)
//         continue;
//       if (WALKING_DOF == 12 && i < 4)
//         continue;
// #endif
//       float dutyAng = (target[i - offset] * angleDataRatio + (steps == 0 ? 0 : (1 + cos(M_PI * s / steps)) / 2 * diff[i - offset]));
//       calibratedPWM(i,  dutyAng);
//     }
//     delay((DOF - offset) / 2);
//   }
//   delete [] diff;
//   //  printList(currentAng);
//   //PTL();
// }




template<typename T> void transform(T *target, byte angleDataRatio = 1, float speedRatio = 1, byte offset = 0, int period = 0) {


  if ((offset != 0)) {

    T *target_[DOF - offset];  // target_ ： nearest frame in target gait
    for (int j = 0; j < DOF - offset; j++) { target_[j] = target; }
    int min_pose_dis[8] = { 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000 };
    int min_pose_idx[8] = {};
    // int8_t left5 = (*(target-4));
    // int8_t right3 = (*(target-4));
    // int8_t mask_r3=7;
    // int8_t mask_r5=31;
    int gait_len = abs(period);  //((left5>>3)&mask_r5)|((right3&mask_r3)<<5);
    // Serial.print(" header:\n ");
    // Serial.print(" gait_len: ");Serial.print( gait_len );
    // Serial.print(" h1: ");Serial.print( (*(target-3)) );
    // Serial.print(" h2: ");Serial.print( (*(target-2)) );
    // Serial.print(" h3: ");Serial.print( (*(target-1)) );
    // Serial.print(" \n ");
    // Serial.print(" frame 0:\n ");
    // Serial.print(" j0: ");Serial.print(*(target));
    // Serial.print(" j1: ");Serial.print(*(target+1));
    // Serial.print(" j2: ");Serial.print(*(target+2));
    // Serial.print(" j3: ");Serial.print(*(target+3));
    // Serial.print(" j4: ");Serial.print(*(target+4));
    // Serial.print(" j5: ");Serial.print(*(target+5));
    // Serial.print(" j6: ");Serial.print(*(target+6));
    // Serial.print(" j7: ");Serial.print(*(target+7));
    // Serial.print(" \n\n ");
    int cur_dis[8] = {};
    for (int i = 0; i < gait_len; i++) {
      for (int j = offset; j < DOF; j++) {
        int currrent_vel = currentAng[j] - previousAng[j];
        int pose_difference = currentAng[j] - target[i * (DOF - offset) + j - offset];
        int vel_bonus = pose_difference * currrent_vel < 0 ? 10000 : 0;  // if they have different sign, then add bonus
        cur_dis[j - offset] = pow(currentAng[j] - target [i * (DOF - offset) + j - offset], 2) - vel_bonus;
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
    float *evel = new float[DOF - offset];
    int *cAng_cp = new int[DOF];
    int maxDiff = 0;
    arrayNCPY(cAng_cp, currentAng, DOF);
    for (int i = offset; i < DOF; i++) {
      maxDiff = max(maxDiff, abs(currentAng[i] - target_[i - offset][i - offset] * angleDataRatio));
      svel[i - offset] = currentAng[i] - previousAng[i];
      evel[i - offset] = 0;  //((offset!=0)?nextFrame[i - offset]*angleDataRatio-target_[i - offset]*angleDataRatio:0);
    }

    int steps = speedRatio > 0 ? int(round(maxDiff / 1.0 /*degreeStep*/ / speedRatio)) : 0;//max(int(round(maxDiff / speedRatio)), 10);
    // interpolation for each joint
    for (int i = 0; i <= steps; i++) {
      for (int j = 0; j < DOF - offset; j++) {



        ///////////////interpolation///////////////
        float A = (float)(svel[j] + evel[j]) / pow(steps, 2) - 2 * (target_[j][j] * angleDataRatio - cAng_cp[j + offset]) / pow(steps, 3);
        float B = (float)(-2 * svel[j] - evel[j]) / steps + 3 * (target_[j][j] * angleDataRatio - cAng_cp[j + offset]) / pow(steps, 2);
        calibratedPWM(j + offset, A * pow(i, 3) + B * pow(i, 2) + svel[j] * i + cAng_cp[j + offset]);
        // delayMicroseconds(700);
      }
    }
    // sync joint
    float sync_speed[DOF - offset];
    int max_remain_steps = 0;
    for (int j = 0; j < DOF - offset; j++) {
      max_remain_steps = max(max_remain_steps, gait_len - min_pose_idx[j]);
    }
    max_remain_steps *= 5;
    for (int j = 0; j < DOF - offset; j++) {
      sync_speed[j] = (float)(gait_len - min_pose_idx[j]) / max_remain_steps;
    }
    float step_counter[DOF - offset] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    float step_counter_pre[DOF - offset] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    for (int k = 0; k < max_remain_steps; k++) {
      for (int j = 0; j < DOF - offset; j++) {
        step_counter[j] += sync_speed[j];
        if (step_counter[j] - step_counter_pre[j] >= 1) {
          step_counter_pre[j] = step_counter[j];
          calibratedPWM(j + offset, (target + (min_pose_idx[j] + int(step_counter[j])) * (DOF - offset))[j] * angleDataRatio);
        }
        // delayMicroseconds(700);
      }
    }

    delete[] svel;
    delete[] evel;
    delete[] cAng_cp;
  }

  else {
    float *svel = new float[DOF - offset];
    int *cAng_cp = new int[DOF];
    int maxDiff = 0;
    arrayNCPY(cAng_cp, currentAng, DOF);
    for (int i = offset; i < DOF; i++) {
      maxDiff = max(maxDiff, abs(currentAng[i] - target[i - offset] * angleDataRatio));
      svel[i - offset] = currentAng[i] - previousAng[i];
    }
    int steps = speedRatio > 0 ? int(round(maxDiff / 1.0 /*degreeStep*/ / speedRatio)) : 0;//max(int(round(maxDiff / speedRatio)), 5);
    for (int i = offset; i < DOF; i++) {
      svel[i - offset] /= max(steps * 0.06, 1.0);
    }
    for (int i = 0; i <= steps; i++) {
      for (int j = 0; j < DOF - offset; j++) {
#ifdef BiBoard
        if (WALKING_DOF == 8 && j - offset > 3 && j < 8 - offset)
          continue;
        if (WALKING_DOF == 12 && j - offset < 4)
          continue;
#endif
        ///////////////interpolation///////////////
        float A = (float)(svel[j]) / pow(steps, 2) - 2 * (target[j] * angleDataRatio - cAng_cp[j + offset]) / pow(steps, 3);
        float B = (float)(-2 * svel[j]) / steps + 3 * (target[j] * angleDataRatio - cAng_cp[j + offset]) / pow(steps, 2);
        calibratedPWM(j + offset, A * pow(i, 3) + B * pow(i, 2) + svel[j] * i + cAng_cp[j + offset]);
        delayMicroseconds(500);
      }
    }
    delete[] svel;
    delete[] cAng_cp;
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
//     //int steps = (offset!=0)?10:20;// interpolation points

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
#define ADJUSTMENT_GAP_DAMPER 5
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

float expectedRollPitch[2];
float RollPitchDeviation[2];
float currentAdjust[DOF] = {};
int ramp = 1;

float adjust(byte i) {
  float rollAdj, pitchAdj, adj;
  if (i == 1 || i > 3) {  //check idx = 1
    bool leftQ = (i - 1) % 4 > 1 ? true : false;
    //bool frontQ = i % 4 < 2 ? true : false;
    //bool upperQ = i / 4 < 3 ? true : false;
    float leftRightFactor = 1;
    if ((leftQ && ramp * RollPitchDeviation[0] > 0)  // operator * is higher than &&
        || (!leftQ && ramp * RollPitchDeviation[0] < 0))
      leftRightFactor = LEFT_RIGHT_FACTOR * abs(ramp);
    rollAdj = (i == 1 || i > 7 ? fabs(RollPitchDeviation[0]) : RollPitchDeviation[0]) * adaptiveParameterArray[i][0] * leftRightFactor;
    //    rollAdj = fabs(RollPitchDeviation[0]) * adaptiveParameterArray[i][0] * leftRightFactor;

  } else
    rollAdj = RollPitchDeviation[0] * adaptiveParameterArray[i][0];

  float idealAdjust = radPerDeg * (
#ifdef POSTURE_WALKING_FACTOR
                        (i > 3 ? POSTURE_WALKING_FACTOR : 1) *
#endif
                          rollAdj
                        + ramp * adaptiveParameterArray[i][1] * ((i % 4 < 2) ? (RollPitchDeviation[1]) : RollPitchDeviation[1]));
  currentAdjust[i] += sign(idealAdjust - currentAdjust[i]) * (min(fabs(idealAdjust - currentAdjust[i]), float(ADJUSTMENT_GAP_DAMPER)));

  return currentAdjust[i];
}