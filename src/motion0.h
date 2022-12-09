void calibratedPWM(byte i, float angle, float speedRatio = 0) {
  int actualServoIndex = (PWM_NUM == 12 && i > 3) ? i - 4 : i;
  angle = max(float(angleLimit[i][0]), min(float(angleLimit[i][1]), angle));
  int duty0 = calibratedZeroPosition[i] + currentAng[i] * rotationDirection[i];
  previousAng[i] = currentAng[i];
  currentAng[i] = angle;
  int duty = calibratedZeroPosition[i] + angle * rotationDirection[i];
  int steps = speedRatio > 0 ? int(round(abs(duty - duty0) / 1.0/*degreeStep*/ / speedRatio)) : 0;
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

void allCalibratedPWM(char * dutyAng, byte offset = 0) {
  for (int8_t i = DOF - 1; i >= offset; i--) {
    calibratedPWM(i, dutyAng[i]);
  }
}

template <typename T> void transform( T * target, byte angleDataRatio = 1, float speedRatio = 1, byte offset = 0, int period = 0) {
  //  PTL("transform");
  int *diff = new int [DOF - offset], maxDiff = 0;
  for (byte i = offset; i < DOF; i++) {
    diff[i - offset] =   currentAng[i] - target[i - offset] * angleDataRatio;
    maxDiff = max(maxDiff, abs( diff[i - offset]));
  }
  int steps = speedRatio > 0 ? int(round(maxDiff / 1.0/*degreeStep*/ / speedRatio)) : 0 ; //default speed is 1 degree per step

  for (int s = 0; s <= steps; s++) {
    for (byte i = offset; i < DOF; i++) {
#ifdef BiBoard
      if (WALKING_DOF == 8 && i > 3 && i < 8)
        continue;
      if (WALKING_DOF == 12 && i < 4)
        continue;
#endif
      float dutyAng = (target[i - offset] * angleDataRatio + (steps == 0 ? 0 : (1 + cos(M_PI * s / steps)) / 2 * diff[i - offset]));
      calibratedPWM(i,  dutyAng);
    }
    delay((DOF - offset) / 2);
  }
  delete [] diff;
  //  printList(currentAng);
  //PTL();
}

#define WEIGHT 2
template <typename T> void transformCubic( T * target, byte angleDataRatio = 1, float speedRatio = 2, byte offset = 0) {
  {

    int maxDiff = 0;
    T *nextFrame = target + DOF - offset;
    //svel: vel at the starting point of the interpolation.   evel: vel at the ending point.
    int *svel = new int [DOF - offset];
    int *evel = new int [DOF - offset];
    int *cAng_cp = new int [DOF];
    arrayNCPY(cAng_cp, currentAng, DOF);
    for (byte i = offset; i < DOF; i++) {
      maxDiff = max(maxDiff, abs( currentAng[i] - target[i - offset] * angleDataRatio));
      svel[i - offset] = (currentAng[i] - previousAng[i])/WEIGHT;
      evel[i - offset] = ((offset != 0) ? nextFrame[i - offset] * angleDataRatio - target[i - offset] * angleDataRatio : 0)/WEIGHT;
    }
//    printList(currentAng);
//    PTL();
    int steps = int(round(maxDiff / speedRatio )); //default speed is 1 degree per step
    //int steps = (offset!=0)?10:20;// interpolation points

    for (int s = 0; s < steps; s++) {
      for (int j = offset; j < DOF; j++) {
        ///////////////interpolation///////////////
        float A = (float)(svel[j - offset] + evel[j - offset])  / pow(steps, 2) - 2 * (target[j - offset] * angleDataRatio - cAng_cp[j]) / pow(steps, 3);
        float B = (float)(-2 * svel[j - offset]  - evel[j - offset] ) / steps + 3 * (target[j - offset] * angleDataRatio - cAng_cp[j]) / pow(steps, 2);
        float dutyAng = A * pow(s, 3) + B * pow(s, 2) + svel[j - offset]  * s + cAng_cp[j];
//        PT(dutyAng);
//        PT(",\t");
        calibratedPWM (j, dutyAng);
      }
//      PTL();
    }
//    PTL();
//    printList(target);
    delete [] svel;
    delete [] evel;
    delete [] cAng_cp;
  }
}

// balancing parameters
#define ROLL_LEVEL_TOLERANCE 5//the body is still considered as level, no angle adjustment
#define PITCH_LEVEL_TOLERANCE 3
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
#define uRF 50    //upper leg roll factor
#define uPF 50    //upper leg pitch factor
#define lRF (-1.5*uRF)  //lower leg roll factor 
#define lPF (-1.5*uPF)  //lower leg pitch factor
#define LEFT_RIGHT_FACTOR 2
#define FRONT_BACK_FACTOR 1.2
#define POSTURE_WALKING_FACTOR 0.5
#define ADJUSTMENT_GAP_DAMPER 5
//#ifdef POSTURE_WALKING_FACTOR
//float postureOrWalkingFactor = 1;
//#endif

#ifdef X_LEG  // >< leg
float adaptiveParameterArray[][NUM_ADAPT_PARAM] = {
  { -panF, 0}, { -panF / 2, -tiltF}, { -2 * panF, 0}, {0, 0},
  {sRF, -sPF}, { -sRF, -sPF}, { -sRF, sPF}, {sRF, sPF},
  {uRF, uPF}, {uRF, uPF}, { -uRF, uPF}, { -uRF, uPF},
  {lRF, lPF}, {lRF, lPF}, { -lRF, lPF}, { -lRF, lPF}
};
#else         // >> leg
float adaptiveParameterArray[][NUM_ADAPT_PARAM] = {
  { -panF, 0}, { -panF / 2, -tiltF}, { -2 * panF, 0}, {0, 0},
  {sRF, -sPF}, { -sRF, -sPF}, { -sRF, sPF}, {sRF, sPF},
  {uRF, uPF}, {uRF, uPF}, {uRF, uPF}, {uRF, uPF},
  {lRF, -0.5 * lPF}, {lRF, -0.5 * lPF}, {lRF, 0.5 * lPF}, {lRF, 0.5 * lPF}
};
#endif

float expectedRollPitch[2];
float RollPitchDeviation[2];
float currentAdjust[DOF] = {};
int ramp = 1;

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
    rollAdj =  (i == 1 || i > 7  ? fabs(RollPitchDeviation[0]) : RollPitchDeviation[0]) * adaptiveParameterArray[i][0] * leftRightFactor;
    //    rollAdj = fabs(RollPitchDeviation[0]) * adaptiveParameterArray[i][0] * leftRightFactor;

  }
  else
    rollAdj = RollPitchDeviation[0] * adaptiveParameterArray[i][0] ;

  float idealAdjust = radPerDeg * (
#ifdef POSTURE_WALKING_FACTOR
                        (i > 3 ? POSTURE_WALKING_FACTOR : 1) *
#endif
                        rollAdj + ramp * adaptiveParameterArray[i][1] * ((i % 4 < 2) ? ( RollPitchDeviation[1]) : RollPitchDeviation[1]));
  currentAdjust[i] += sign(idealAdjust - currentAdjust[i]) * (min(fabs(idealAdjust - currentAdjust[i]), float(ADJUSTMENT_GAP_DAMPER)));

  return currentAdjust[i];
}
