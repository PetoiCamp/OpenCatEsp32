#define POWER_SAVER 120 //make the robot rest after a certain period, unit is seconds
#define IDLE_SHORT 15
#define IDLE_LONG 30
#define EVERY_X_SECONDS 5
int idleThreshold = IDLE_SHORT;
#define RANDOM_MIND true     //let the robot do random stuffs. use token 'z' to activate/deactivate
int randomInterval = 1000;
const char *randomMindList[] = {"iRand", "ksit", "u",
#ifdef CUB
                                "kfd", "krt",
#else
                                "kck", "m0 -45 0 45 0 -45 0 45",
#endif
                                NULL
                               };
byte choiceWeight[] = {100, 10, 5,
#ifdef CUB
                       1, 1,
#else
                       2, 1,
#endif
                      };
                      
int randomMindListLength;
int randomBase = 0;
long idleTimer;
long randTimer;

void allRandom() {
  char tokenSet[] = {T_INDEXED_SIMULTANEOUS_BIN, T_MOVE_BIN};
  int8_t jointSet[] = {0, 1, 2, 8, 9,  12, 13, 14, 15};
  byte rangeSet[] = {90, 45, 180, 5, 5, 10, 10, 5, 5};
  //  byte rangeSet[] = {90, 90, 180, 50, 50, 100, 100, 50, 50};

  token = tokenSet[random() % 2];
  cmdLen = rand() % 4 + 4;
  for (byte r = 0; r < cmdLen; r++) {
    byte j = rand() % sizeof(jointSet);
    dataBuffer[r * 2] = jointSet[j];
    dataBuffer[r * 2 + 1] = (int8_t)min(max(currentAng[jointSet[j]] + rand() % rangeSet[j] - rangeSet[j] / 2, -90), 90);
    //    PT(jointSet[j]); PT('\t'); PTL(int(dataBuffer[r * 2 + 1]));
  }
  cmdLen *= 2;
  dataBuffer[cmdLen] = '\0';
}

void randomMind() {
  if (token != T_CALIBRATE && token != T_REST && idleTimer && (millis() - idleTimer) > idleThreshold * 1000) { //in idle state
    if (millis() - randTimer > randomInterval) {//every second throw a dice
      randTimer = millis();
      int randomNum = esp_random() % randomBase;
      byte randomChoice = -1;
      while (randomNum >= 0) {
        randomChoice++;
        randomNum -= choiceWeight[randomChoice];
      }
      if (randomChoice == 0)
        allRandom();
      else {
        token = randomMindList[randomChoice][0];
        if (token == T_SKILL)
          strcpy(newCmd, randomMindList[randomChoice] + 1);// this is duable only because newCmd+1 is after newCmd!
        else
          strcpy((char*)dataBuffer, randomMindList[randomChoice] + 1);
      }
      newCmdIdx = 100;
    }

  }
}

void powerSaver(int idleThreshold = 10) { //unit is second
  if (idleTimer && (millis() - idleTimer) > POWER_SAVER * 1000) {
    if (token != T_REST) {
      token = T_REST;
      newCmdIdx = 4;
      idleTimer = 0;
//      shutEsp32Servo = true;
    }
  }
}
