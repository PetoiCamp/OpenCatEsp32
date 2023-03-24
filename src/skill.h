#define MEMORY_ADDRESS_SIZE 4

class SkillPreview {
public:
  char* skillName;  //use char array instead of String to save memory
  int period;       //the period of a skill. 1 for posture, >1 for gait, <-1 for behavior
  int index;
  SkillPreview(int s) {
    skillName = new char[strlen(skillNameWithType[s])];
    strcpy(skillName, skillNameWithType[s]);
    skillName[strlen(skillNameWithType[s]) - 1] = '\0';  //drop the last charactor of skill type
    unsigned int pgmAddress = (unsigned int)progmemPointer[s];
    period = (int8_t)pgm_read_byte(pgmAddress);  //automatically cast to char*
    index = s;
  }
};

class SkillList : public QList<SkillPreview*> {
public:
  SkillList() {
    PT("Build skill list...");
    //  PT(sizeof(progmemPointer) / MEMORY_ADDRESS_SIZE);
    PTL(sizeof(skillNameWithType) / MEMORY_ADDRESS_SIZE);
    for (int s = 0; s < sizeof(progmemPointer) / MEMORY_ADDRESS_SIZE; s++) {
      SkillPreview* tempAddress = new SkillPreview(s);
      this->push_back(tempAddress);
    }
    //  PTF("free memory: ");//before building the skill list
    //  PTL(ESP.getFreeHeap());
    for (randomMindListLength = 0; randomMindList[randomMindListLength] != NULL; randomMindListLength++)
      ;
  }
  int lookUp(const char* key) {
    byte nSkills = sizeof(progmemPointer) / MEMORY_ADDRESS_SIZE;
    byte randSkillIdx = strcmp(key, "x") ? nSkills : random(nSkills);
    byte keyLen = strlen(key);
    char lr = key[keyLen - 1];
    for (int s = 0; s < nSkills; s++) {
      char readName[CMD_LEN + 1];
      strcpy(readName, this->get(s)->skillName);
      if (s == randSkillIdx && (!strcmp(readName, "bf") || !strcmp(readName, "ff") || !strcmp(readName, "rc") || !strcmp(readName, "rl") || !strcmp(readName, "jmp"))) {  //forbid violent motions in random mode
        randSkillIdx++;
        continue;
      }
      byte nameLen = strlen(readName);
      if (s == randSkillIdx          //random skill
          || !strcmp(readName, key)  //exact match: gait type + F or L, behavior
          // || readName[nameLen - 1] == 'L' && !strncmp(readName, key, nameLen - 1)
          || readName[nameLen - 1] != 'F' && strcmp(readName, "bk") && !strncmp(readName, key, keyLen - 1) && (lr == 'L' || lr == 'R' || lr == 'X')  // L, R or X
      ) {
        return s;
      }
    }
    PT('?');  //key not found
    PTL(key);
    return -1;
  }
};
SkillList* skillList;

class Skill {
public:
  char skillName[20];  //use char array instead of String to save memory
  int8_t offsetLR;
  int period;  //the period of a skill. 1 for posture, >1 for gait, <-1 for behavior
  float transformSpeed;
  byte skillHeader;
  byte frameSize;
  int expectedRollPitch[2];  //expected body orientation (roll, pitch)
  byte angleDataRatio;       //divide large angles by 1 or 2. if the max angle of a skill is >128, all the angls will be divided by 2
  byte loopCycle[3];         //the looping section of a behavior (starting row, ending row, repeating cycles)
  byte firstMotionJoint;
  int8_t* dutyAngles;  //the data array for skill angles and parameters

  Skill() {
    skillName[0] = '\0';  //use char array instead of String to save memory
    offsetLR = 0;
    period = 0;
    transformSpeed = 1;
    frameSize = 0;
    expectedRollPitch[0] = expectedRollPitch[1] = 0;
    angleDataRatio = 1;
    loopCycle[0] = loopCycle[1] = loopCycle[2] = 0;
    firstMotionJoint = 0;
    dutyAngles = NULL;
  }
  void buildSkill() {  // K token
    strcpy(skillName, "tmp");
    offsetLR = 0;
    period = (int8_t)newCmd[0];  //automatically cast to char*
    dataLen(period);
    formatSkill();
  }

  void buildSkill(int s) {
    strcpy(skillName, newCmd);
    unsigned int pgmAddress = (unsigned int)progmemPointer[s];
    period = (int8_t)pgm_read_byte(pgmAddress);  //automatically cast to char*
    for (int i = 0; i < dataLen(period); i++) {
      newCmd[i] = pgm_read_byte(pgmAddress++);
    }
    newCmd[dataLen(period)] = '~';
    formatSkill();
  }
  ~Skill() {
  }
  int dataLen(int8_t p) {
    skillHeader = p > 0 ? 4 : 7;
    frameSize = p > 1 ? WALKING_DOF :  //gait
                  p == 1 ? DOF
                         :  //posture
                  DOF + 4;  //behavior
    int len = skillHeader + abs(p) * frameSize;
    return len;
  }

  void inplaceShift() {
    int angleLen = abs(period) * frameSize;                 // need one extra byte for terminator '~'
    int shiftRequiredByNewCmd = CMD_LEN - skillHeader + 1;  // required shift to store CMD_LEN + 1 chars. it can hold a command with CMD_LEN chars. the additioanl byte is required by '\0'.
    spaceAfterStoringData = BUFF_LEN - angleLen - 1;        // the bytes before the dutyAngles. The allowed command's bytes needs to -1
    // PTH("request", shiftRequiredByNewCmd);
    // PTH("aloShft", BUFF_LEN - (skillHeader + angleLen));
    if (CMD_LEN > spaceAfterStoringData) {
      PTF("LMT ");
      PTL(spaceAfterStoringData);
    }
    for (int i = 0; i <= angleLen; i++)
      newCmd[BUFF_LEN - i] = newCmd[skillHeader + angleLen - i];
    dutyAngles = (int8_t*)newCmd + BUFF_LEN - angleLen;
  }

  void formatSkill() {
    transformSpeed = 1;  //period > 1 ? 1 : 0.5;
    firstMotionJoint = (period <= 1) ? 0 : DOF - WALKING_DOF;

    for (int i = 0; i < 2; i++) {
      expectedRollPitch[i] = (int8_t)newCmd[1 + i];
#ifdef GYRO_PIN
      yprTilt[2 - i] = 0;
#endif
    }
    angleDataRatio = (int8_t)newCmd[3];
    byte baseHeader = 4;
    if (period < 0) {
      for (byte i = 0; i < 3; i++)
        loopCycle[i] = (int8_t)newCmd[baseHeader++];
    }
    inplaceShift();
    // int len = abs(period) * frameSize;
    // dutyAngles = new int8_t[len];
    // for (int k = 0; k < abs(period); k++) {
    //   for (int col = 0; col < frameSize; col++) {
    //     if (WALKING_DOF == 12 && GAIT_ARRAY_DOF == 8 && period > 1)
    //       if (col < 4)
    //         dutyAngles[k * frameSize + col] = 0;
    //       else
    //         dutyAngles[k * frameSize + col] = int8_t(newCmd[skillHeader + k * GAIT_ARRAY_DOF + col - 4]);
    //     else
    //       dutyAngles[k * frameSize + col] = int8_t(newCmd[skillHeader + k * frameSize + col]);
    //   }
    // }
  }
#define PRINT_SKILL_DATA
  void info() {
    PT("Skill Name: ");
    PTL(skillName);
    PTF("period: ");
    PT(period);
    PT(",\texpected(pitch,roll): (");
    PT(expectedRollPitch[0]);
    PT(",");
    PT(expectedRollPitch[1]);
    PT(")\t");
    PTF("angleRatio: ");
    PTL(angleDataRatio);
    if (period < 0) {
      PT("loop frame: ");
      for (byte i = 0; i < 3; i++)
        PT(String((byte)loopCycle[i]) + ", ");
      PTL();
    }
#ifdef PRINT_SKILL_DATA
    int showRows = 1;
    for (int k = 0; k < abs(period); k++) {
      if (abs(period) <= showRows + 2 || k < showRows || k == abs(period) - 1) {
        for (int col = 0; col < frameSize; col++) {
          PT((int8_t)dutyAngles[k * frameSize + col]);
          PT(",\t");
        }
        PTL();
      } else {
        if (k == showRows) {
          PTF(" skipping ");
          PT(abs(period) - 1 - showRows);
          PTF(" frames");
        }
        PT('.');
        if (k == abs(period) - 2)
          PTL();
      }
    }
#endif
    PTL();
  }
  void mirror() {  //Create a mirror function to allow the robot to pick random directions of behaviors.
    //It makes the robot more unpredictable and helps it get rid of an infinite loop,
    //such as failed fall-recovering against a wall.
    expectedRollPitch[0] = -expectedRollPitch[0];
    for (int k = 0; k < abs(period); k++) {
      if (period <= 1) {
        dutyAngles[k * frameSize] = -dutyAngles[k * frameSize];
        dutyAngles[k * frameSize + 2] = -dutyAngles[k * frameSize + 2];
      }
      for (byte col = (period > 1) ? 0 : 2; col < ((period > 1) ? WALKING_DOF : DOF) / 2; col++) {
        int8_t temp = dutyAngles[k * frameSize + 2 * col];
        dutyAngles[k * frameSize + 2 * col] = dutyAngles[k * frameSize + 2 * col + 1];
        dutyAngles[k * frameSize + 2 * col + 1] = temp;
      }
    }
  }
  int nearestFrame() {
    if (period == 1)
      frame = 0;
    else  //find the nearest frame using certain algorithm
      frame = 0;
    return frame;
  }
  void transformToSkill(int frame = 0) {
    //      info();
    transform(dutyAngles + frame * frameSize, angleDataRatio, transformSpeed, firstMotionJoint, period, runDelay);
  }
  void convertTargetToPosture(int* targetFrame) {
    int extreme[2];
    getExtreme(targetFrame, extreme);
    if (extreme[0] < -125 || extreme[1] > 125) {
      angleDataRatio = 2;
      for (int i = 0; i < DOF; i++)
        targetFrame[i] /= 2;
    } else
      angleDataRatio = 1;
    arrayNCPY(dutyAngles, targetFrame, DOF);
    period = 1;
    firstMotionJoint = 0;
    frameSize = DOF;
    frame = 0;
  }
  void perform() {
    if (period < 0) {  //behaviors
      int8_t repeat = loopCycle[2] >= 0 && loopCycle[2] < 2 ? 0 : loopCycle[2] - 1;
      for (byte c = 0; c < abs(period); c++) {  //the last two in the row are transition speed and delay
        //  PT("step "); PTL(c);
        //  printList(dutyAngles + c * frameSize);
        transform(dutyAngles + c * frameSize, angleDataRatio, dutyAngles[DOF + c * frameSize] / 8.0);

#ifdef GYRO_PIN  //if opt out the gyro, the calculation can be really fast
        if (dutyAngles[DOF + 2 + c * frameSize]) {
          int triggerAxis = dutyAngles[DOF + 2 + c * frameSize];
          int triggerAngle = dutyAngles[DOF + 3 + c * frameSize];

          float currentYpr = ypr[abs(triggerAxis)];
          float previousYpr = currentYpr;
          //            long triggerTimer = millis();
          while (1) {
            read_IMU();
            print6Axis();
            currentYpr = ypr[abs(triggerAxis)];
            PT(currentYpr);
            PTF("\t");
            PTL(triggerAngle);
            if ((180 - fabs(currentYpr) > 2)                                                                                           //skip the angle when the reading jumps from 180 to -180
                && (triggerAxis * currentYpr > triggerAxis * triggerAngle && triggerAxis * previousYpr < triggerAxis * triggerAngle))  //the sign of triggerAxis will deterine whether the current angle should be larger or smaller than the trigger angle
              break;
            previousYpr = currentYpr;
          }
        }
#endif
        delay(abs(dutyAngles[DOF + 1 + c * frameSize] * 50));

        if (repeat != 0 && c != 0 && c == loopCycle[1]) {
          c = loopCycle[0] - 1;
          if (repeat > 0)  //if repeat <0, infinite loop. only reset button will break the loop
            repeat--;
        }
      }
      printToken();
    } else {  //postures and gaits
#ifdef GYRO_PIN
      if (imuUpdated && gyroBalanceQ && !(frame % imuSkip)) {
        //          PT(ypr[2]); PT('\t');
        //          PT(RollPitchDeviation[0]); PT('\t');
        //          printList(currentAdjust);
        for (byte i = 0; i < 2; i++) {
          RollPitchDeviation[i] = ypr[2 - i] - expectedRollPitch[i];                                                                          //all in degrees
          RollPitchDeviation[i] = sign(ypr[2 - i]) * max(float(fabs(RollPitchDeviation[i]) - levelTolerance[i]), float(0)) + yprTilt[2 - i];  //filter out small angles
        }
      }
#endif

      for (int jointIndex = 0; jointIndex < DOF; jointIndex++) {
        //          PT(jointIndex); PT('\t');
#ifndef HEAD
        if (jointIndex == 0)
          jointIndex = 2;
#endif
#ifndef TAIL
        if (jointIndex == 2)
          jointIndex = DOF - WALKING_DOF;
#endif
#if WALKING_DOF == 8
        if (jointIndex == 4)
          jointIndex = 8;
#endif
        //          PT(jointIndex); PT('\t');
        float duty;
        if (abs(period) > 1 && jointIndex < firstMotionJoint || abs(period) == 1 && jointIndex < 4 && manualHeadQ) {
          if (!manualHeadQ && jointIndex < 4) {
            duty = (jointIndex != 1 ? offsetLR : 0)  //look left or right
                   + 10 * sin(frame * (jointIndex + 2) * M_PI / abs(period));
          } else
            duty = currentAng[jointIndex] + max(-10, min(10, (targetHead[jointIndex] - currentAng[jointIndex])))
                   - gyroBalanceQ * currentAdjust[jointIndex];
        } else {
          duty = dutyAngles[frame * frameSize + jointIndex - firstMotionJoint] * angleDataRatio;
        }
        //          PT(duty); PT('\t');
        calibratedPWM(jointIndex, duty
#ifdef GYRO_PIN
                                    + gyroBalanceQ * (!exceptions ? (!(frame % imuSkip) ? adjust(jointIndex) : currentAdjust[jointIndex]) : 0)
#endif
        );
      }
      //        PTL();
      frame += tStep;
      if (frame >= abs(period))
        frame = 0;
    }
  }
};
Skill* skill;

void loadBySkillName(const char* skillName) {  //get lookup information from on-board EEPROM and read the data array from storage
  int skillIndex = skillList->lookUp(skillName);
  if (skillIndex != -1) {
    // if (skill != NULL)
    //   delete[] skill;
    char lr = skillName[strlen(skillName) - 1];
    skill->offsetLR = (lr == 'L' ? 30 : (lr == 'R' ? -30 : 0));
    skill->buildSkill(skillList->get(skillIndex)->index);
    strcpy(newCmd, skill->skillName);
    if (strcmp(newCmd, "calib") && skill->period == 1)
      protectiveShift = esp_random() % 100 / 10.0 - 5;
    else
      protectiveShift = 0;
#ifdef GYRO_PIN
    // keepDirectionQ = (skill->period > 1) ? false : true;
    thresX = (skill->period > 1) ? 12000 : 8000;
    thresY = (skill->period > 1) ? 10000 : 6000;
// thresZ = (skill->period > 1) ? -8000 : -10000;
#endif
    for (byte i = 0; i < DOF; i++)
      skill->dutyAngles[i] += protectiveShift;
    // skill->info();
    if (lr == 'R' || (lr == 'X' || lr != 'L') && random(100) % 2)
      skill->mirror();  //randomly mirror the direction of a behavior
    skill->transformToSkill(skill->nearestFrame());
#ifdef NYBBLE
    for (byte i = 0; i < HEAD_GROUP_LEN; i++)
      targetHead[i] = currentAng[i] - currentAdjust[i];
#endif
    //    runDelay = delayMid + 2;
    // skill->info();
  }
}
