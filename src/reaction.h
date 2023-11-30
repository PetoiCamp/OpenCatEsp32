void dealWithExceptions() {
#ifdef GYRO_PIN
  if (gyroBalanceQ && exceptions) {  // the gyro reaction switch can be toggled on/off by the 'g' token
    switch (exceptions) {
      case -1:
        {
          PTL("EXCEPTION 1");
          strcpy(newCmd, "lnd");
          loadBySkillName(newCmd);
          shutServos();  // does not shut the P1S servo.while token p in serial monitor can.? ? ?
          delay(1000);
          token = 'k';
          strcpy(newCmd, "up");
          newCmdIdx = -1;
          break;
        }
      case -2:
        {
          PTL("EXCEPTION 2");
          soundFallOver();
          //  for (int m = 0; m < 2; m++)
          //    meow(30 - m * 12, 42 - m * 12, 20);
          token = 'k';
          manualHeadQ = false;
          strcpy(newCmd, "rc");
          newCmdIdx = -2;
          // tQueue->addTaskToFront('k', "rc");
          break;
        }
      case -3:
        {
          PTL("EXCEPTION 3");
          if (  // skill->period == 1 &&
            strncmp(lastCmd, "vt", 2)) {
            char xSymbol[] = { '^', 'v' };
            char ySymbol[] = { '<', '>' };
            char xDirection = xSymbol[sign(ARX) > 0];
            char yDirection = ySymbol[sign(ARY) > 0];
            float forceAngle = atan(float(abs(ARX)) / ARY) * degPerRad;
            if (tQueue->cleared()) {
              if (abs(forceAngle) < 70) {
                tQueue->addTask('k', yDirection == '<' ? "wkL" : "wkR");
                tQueue->addTask('i', yDirection == '<' ? "0 -45" : "0 45", 1000);
                tQueue->addTask('i', "");
              } else {
                if (xDirection == '^') {
                  tQueue->addTask('k', "wkF", 300);
                  tQueue->addTask('i', yDirection == '<' ? "0 75" : "0 -75", 700);
                  tQueue->addTask('i', "");
                } else {
                  tQueue->addTask('k', yDirection == '<' ? "bkR" : "bkL", 1000);
                  tQueue->addTask('k', yDirection == '<' ? "wkL" : "wkR", 1000);
                }
              }
              // tQueue->addTask('k', "up", 100);
              delayPrevious = runDelay;
              runDelay = 3;
            }
            PT(abs(ARX) > abs(ARY) ? xDirection : yDirection);
            PTL();
          }
          break;
        }
      case -4:
        {
          PTL("EXCEPTION 4");
          char *currentGait = skill->skillName;  // it may not be gait
          char gaitDirection = currentGait[strlen(currentGait) - 1];
          float yawDiff = int(ypr[0] - previous_ypr[0]) % 180;
          if (tQueue->cleared()) {
            if (skill->period <= 1 || !strcmp(skill->skillName, "vtF")) {  // not gait or stepping
              tQueue->addTask('k', yawDiff > 0 ? "vtR" : "vtL", round(abs(yawDiff) * 15));
              // tQueue->addTask('k', "up", 100);
              delayPrevious = runDelay;
              runDelay = 3;
            } else {
              // if (gaitDirection == 'L' || gaitDirection == 'R')   //turning gait
              previous_ypr[0] = ypr[0];
            }
            // else {
            //   if (gaitDirection == 'L' || gaitDirection == 'R') {  //turning gait
            //     previous_ypr[0] = ypr[0];
            //   } else {
            //     currentGait[strlen(currentGait) - 1] = yawDiff > 0 ? 'R' : 'L';
            //     PTL(currentGait);
            //     tQueue->addTask('k', currentGait, round(abs(yawDiff) * 15));
            //     currentGait[strlen(currentGait) - 1] = 'F';
            //     PTL(currentGait);
            //     tQueue->addTask('k', currentGait);
            //   }
            // }
          }
          break;
        }
      default:
        {
          break;
        }
    }

    if (exceptions != -4)
      print6Axis();
    read_IMU();  // flush the IMU to avoid static readings and infinite loop

    if (tQueue->lastTask == NULL) {
      if (strcmp(lastCmd, "") && strcmp(lastCmd, "lnd") && *strGet(newCmd, -1) != 'L' && *strGet(lastCmd, -1) != 'R') {
        PTH("save last task ", lastCmd);
        tQueue->lastTask = new Task('k', lastCmd);
      }
    }
  }
  // if (tQueue->cleared() && runDelay <= delayException)
  //   runDelay = delayPrevious;
#endif
}

#ifdef VOLTAGE
float vFactor = 4096 / 3.3 / 3;
float low_voltage = LOW_VOLTAGE * vFactor;
bool lowBattery() {
  long currentTime = millis() / CHECK_BATTERY_PERIOD;
  if (currentTime > uptime) {
    uptime = currentTime;
    float voltage = analogRead(VOLTAGE);
    if (voltage == 0 || voltage < low_voltage && abs(voltage - lastVoltage) > 10) {  // if battery voltage < threshold, it needs to be recharged
      // give the robot a break when voltage drops after sprint
      // adjust the thresholds according to your batteries' voltage
      // if set too high, the robot will stop working when the battery still has power.
      // If too low, the robot may not alarm before the battery shuts off
      if (!safeRest) {
        strcpy(lastCmd, "rest");
        loadBySkillName(lastCmd);
        shutServos();
        safeRest = true;
      }
      PT("Low power: ");
      PT(voltage / vFactor);
      PTL("V");
      if (i2c_eeprom_read_byte(EEPROM_BOOTUP_SOUND_STATE))
        playMelody(melodyLowBattery, sizeof(melodyLowBattery) / 2);
      //    strip.show();
      int8_t bStep = 1;
      for (byte brightness = 1; brightness > 0; brightness += bStep) {
#ifdef NEOPIXEL_PIN
        strip.setPixelColor(0, strip.Color(brightness, 0, 0));
        strip.show();
#endif
#ifdef PWM_LED_PIN
        analogWrite(PWM_LED_PIN, 255 - brightness);
#endif
        if (brightness == 255)
          bStep = -1;
        delay(5);
      }
      lastVoltage = voltage;
      return true;
    }
    if (safeRest) {
      strcpy(lastCmd, "rest");
      loadBySkillName(lastCmd);
      shutServos();
      safeRest = false;
    }
    lastVoltage = voltage;
  }
  return false;
}
#endif

void reaction() {
  if (newCmdIdx) {
    // PTLF("-----");
    lowerToken = tolower(token);
    if (initialBoot) {  //-1 for marking the bootup calibration state
      fineAdjust = true;
      gyroBalanceQ = true;
      autoSwitch = RANDOM_MIND;
      initialBoot = false;
    }
    if (token != T_REST && newCmdIdx < 5)
      idleTimer = millis();
    if (newCmdIdx < 5 && lowerToken != T_BEEP && token != T_MEOW && token != T_LISTED_BIN && token != T_INDEXED_SIMULTANEOUS_BIN && token != T_TILT && token != T_READ && token != T_WRITE)
      beep(15 + newCmdIdx, 5);  // ToDo: check the muted sound when newCmdIdx = -1
    if (hardServoQ && (lowerToken == T_SKILL || lowerToken == T_INDEXED_SEQUENTIAL_ASC || lowerToken == T_INDEXED_SIMULTANEOUS_ASC)) {
#ifdef T_SERVO_MICROSECOND
      setServoP(P_SOFT);
      hardServoQ = false;
#endif
    }
    if ((lastToken == T_CALIBRATE || lastToken == T_REST || !strcmp(lastCmd, "fd")) && token != T_CALIBRATE) {
      gyroBalanceQ = true;
      printToAllPorts('G');
    }
    if (token != T_PAUSE && !tStep) {
      tStep = 1;
      printToAllPorts('p');
    }
#ifdef ESP_PWM
    if (token != T_SERVO_FEEDBACK && measureServoPin != -1) {
      attachAllESPServos();
      measureServoPin = -1;
    }
#endif

    switch (token) {
      case T_QUERY:
        {
          printToAllPorts(MODEL);
          printToAllPorts(SoftwareVersion);
          break;
        }
      case T_GYRO_FINENESS:
      case T_GYRO_BALANCE:
      case T_PRINT_GYRO:
      case T_VERBOSELY_PRINT_GYRO:
      case T_RANDOM_MIND:
      case T_SLOPE:
        {
          if (token == T_RANDOM_MIND) {
            autoSwitch = !autoSwitch;
            token = autoSwitch ? 'Z' : 'z';  // G for activated gyro
          }
#ifdef GYRO_PIN
          else if (token == T_GYRO_FINENESS) {
            fineAdjust = !fineAdjust;
            // imuSkip = fineAdjust ? IMU_SKIP : IMU_SKIP_MORE;
            runDelay = fineAdjust ? delayMid : delayShort;
            token = fineAdjust ? 'G' : 'g';  // G for activated gyro
          } else if (token == T_GYRO_BALANCE) {
            gyroBalanceQ = !gyroBalanceQ;
            token = gyroBalanceQ ? 'G' : 'g';  // G for activated gyro
          } else if (token == T_PRINT_GYRO) {
            print6Axis();
          } else if (token == T_VERBOSELY_PRINT_GYRO) {
            printGyro = !printGyro;
            token = printGyro ? 'V' : 'v';  // V for verbosely print gyro data
          } else if (token == T_SLOPE) {
            slope = -slope;
            token = slope > 0 ? 'R' : 'r';  // G for activated gyro
          }
#endif
          break;
        }
      case T_PAUSE:
        {
          tStep = !tStep;             // tStep can be -1
          token = tStep ? 'p' : 'P';  // P for pause activated
          if (tStep)
            token = T_SKILL;
          else
            shutServos();
          break;
        }
      case T_ACCELERATE:
        {
          runDelay = max(0, runDelay - 1);
          break;
        }
      case T_DECELERATE:
        {
          runDelay = min(delayLong, runDelay + 1);
          break;
        }
      case T_REST:
        {
          strcpy(newCmd, "rest");
          if (strcmp(newCmd, lastCmd)) {
            loadBySkillName(newCmd);
          }
          shutServos();
          gyroBalanceQ = false;
          manualHeadQ = false;
          printToAllPorts('g');
          break;
        }
      case T_JOINTS:
        {  // show the list of current joint anles
          //          printRange(DOF);
          //          printList(currentAng);
          printToAllPorts('=');
          if (cmdLen)
            printToAllPorts(currentAng[atoi(newCmd)]);
          else {
            printToAllPorts(range2String(DOF));
            printToAllPorts(list2String(currentAng));
          }
          break;
        }
      case T_MELODY:
        {
          playMelody(melody1, sizeof(melody1) / 2);
          break;
        }
#ifdef ULTRASONIC
      case T_COLOR:
        {
          if (cmdLen < 2)  // a single 'C' will turn off the manual color mode
            manualEyeColorQ = false;
          else {  // turn on the manual color mode
            manualEyeColorQ = true;
            long color = ((long)(uint8_t(newCmd[0])) << 16) + ((long)(uint8_t(newCmd[1])) << 8) + (long)(uint8_t(newCmd[2]));
            mRUS04.SetRgbEffect(E_RGB_INDEX(uint8_t(newCmd[3])), color, uint8_t(newCmd[4]));
          }
          break;
        }
#endif
      case ';':
        {
          setServoP(P_SOFT);
          break;
        }
      case ':':
        {
          setServoP(P_HARD);
          break;
        }
      case T_SAVE:
        {
          PTLF("save offset");
          saveCalib(servoCalib);
#ifdef VOICE
          if (newCmdIdx == 2)
            Serial2.println("XAc");
#endif
          break;
        }
      case T_ABORT:
        {
          PTLF("aborted");
          i2c_eeprom_read_buffer(EEPROM_CALIB, (byte *)servoCalib, DOF);
#ifdef VOICE
          if (newCmdIdx == 2)
            Serial2.println("XAc");
#endif
          break;
        }
      case T_RESET:
        {
          i2c_eeprom_write_byte(EEPROM_BIRTHMARK_ADDRESS, 'R');  // esp_random() % 128); //mark the board as uninitialized
          PTL("Alter the birthmark for reset!");
          delay(5);
          ESP.restart();
          break;
        }
      case T_CALIBRATE:                 // calibration
      case T_INDEXED_SEQUENTIAL_ASC:    // move multiple indexed joints to angles once at a time (ASCII format entered in the serial monitor)
      case T_INDEXED_SIMULTANEOUS_ASC:  // move multiple indexed joints to angles simultaneously (ASCII format entered in the serial monitor)
#ifdef T_SERVO_MICROSECOND
      case T_SERVO_MICROSECOND:  // send pulse with unit of microsecond to a servo
#endif
#ifdef T_SERVO_FEEDBACK
      case T_SERVO_FEEDBACK:
#endif
      case T_TILT:  //tilt the robot, format: t axis angle. 0:yaw, 1:pitch, 2:roll
      case T_MEOW:  //meow
      case T_BEEP:  //beep(tone, duration): tone 0 is pause, duration range is 0~255
#ifdef T_TUNER
      case T_TUNER:
#endif
        {
          if (token == T_INDEXED_SIMULTANEOUS_ASC && cmdLen == 0)
            manualHeadQ = false;
          else {
            int targetFrame[DOF + 1];
            // arrayNCPY(targetFrame, currentAng, DOF);
            for (int i = 0; i < DOF; i++) {
              targetFrame[i] = currentAng[i] - (gyroBalanceQ ? currentAdjust[i] : 0);
            }
            targetFrame[DOF] = '~';
            char *pch;
            pch = strtok(newCmd, " ,");
            nonHeadJointQ = false;
            do {  // it supports combining multiple commands at one time
              // for example: "m8 40 m8 -35 m 0 50" can be written as "m8 40 8 -35 0 50"
              // the combined commands should be less than four. string len <=30 to be exact.
              int target[2] = {};
              int inLen = 0;
              for (byte b = 0; b < 2 && pch != NULL; b++) {
                target[b] = atoi(pch);  //@@@ cast
                pch = strtok(NULL, " ,\t");
                inLen++;
              }
              if ((token == T_INDEXED_SEQUENTIAL_ASC || token == T_INDEXED_SIMULTANEOUS_ASC) && target[0] >= 0 && target[0] < DOF) {
                targetFrame[target[0]] = target[1];
                if (target[0] < 4) {
                  targetHead[target[0]] = target[1];
                  manualHeadQ = true;
                } else
                  nonHeadJointQ = true;
              }
              if (token == T_CALIBRATE) {
                gyroBalanceQ = false;
                if (lastToken != T_CALIBRATE) {
#ifdef T_SERVO_MICROSECOND
                  setServoP(P_HARD);
                  hardServoQ = true;
#endif
#ifdef VOICE
                  if (newCmdIdx == 2)
                    Serial2.println("XAd");
#endif
                  strcpy(newCmd, "calib");
                  loadBySkillName(newCmd);
                }
                if (inLen == 2) {
                  if (target[1] >= 1001) {  // Using 1001 for incremental calibration. 1001 is adding 1 degree, 1002 is adding 2 and 1009 is adding 9 degrees
                    target[1] = servoCalib[target[0]] + target[1] - 1000;
                  } else if (target[1] <= -1001) {  // Using -1001 for incremental calibration. -1001 is removing 1 degree, 1002 is removing 2 and 1009 is removing 9 degrees
                    target[1] = servoCalib[target[0]] + target[1] + 1000;
                  }
                  servoCalib[target[0]] = target[1];
                }

                int duty = zeroPosition[target[0]] + float(servoCalib[target[0]]) * rotationDirection[target[0]];
                int actualServoIndex = (PWM_NUM == 12 && target[0] > 3) ? target[0] - 4 : target[0];
#ifdef ESP_PWM
                servo[actualServoIndex].write(duty);
#else
                pwm.writeAngle(actualServoIndex, duty);
#endif
                printToAllPorts(range2String(DOF));
                printToAllPorts(list2String(servoCalib));
                printToAllPorts(token);
                // printToAllPorts(list2String(target, 2));
              } else if (token == T_INDEXED_SEQUENTIAL_ASC) {
                transform(targetFrame, 1, 1);
                delay(10);
              }
#ifdef T_SERVO_MICROSECOND
              else if (token == T_SERVO_MICROSECOND) {
#ifdef ESP_PWM
                servo[PWM_pin[target[0]]].writeMicroseconds(target[1]);
#else
                pwm.writeMicroseconds(PWM_pin[target[0]], target[1]);
#endif
              }
#endif
#ifdef T_SERVO_FEEDBACK
              else if (token == T_SERVO_FEEDBACK) {
                measureServoPin = (inLen == 1) ? target[0] : 16;
              }
#endif
#ifdef GYRO_PIN
              else if (token == T_TILT) {
                yprTilt[target[0]] = target[1];
              }
#endif
              else if (token == T_MEOW) {
                meow(random() % 2 + 1, (random() % 4 + 2) * 10);
              } else if (token == T_BEEP) {
                if (target[0] > 0 && target[1] > 0)
                  beep(target[0], 1000 / target[1]);
              }
#ifdef T_TUNER
              else if (token == T_TUNER) {
                *par[target[0]] = target[1];
                PT(target[0]);
                PT('\t');
                PTL(target[1]);
              }
#endif
              // delay(5);
            } while (pch != NULL);
#ifdef T_TUNER
            if (token == T_TUNER) {
              for (byte p = 0; p < 6; p++) {
                PT(*par[p]);
                PT('\t');
              }
              PTL();
            }
#endif
            if ((token == T_INDEXED_SEQUENTIAL_ASC || token == T_INDEXED_SIMULTANEOUS_ASC) && (nonHeadJointQ || lastToken != T_SKILL)) {
              // printToAllPorts(token);
              transform(targetFrame, 1, transformSpeed);  // if (token == T_INDEXED_SEQUENTIAL_ASC) it will be useless
              skill->convertTargetToPosture(targetFrame);
            }
            // if (token == T_INDEXED_SEQUENTIAL_ASC)
            //   skill->convertTargetToPosture();
            // if (token == T_INDEXED_SIMULTANEOUS_ASC) {
            //   PTL(token);  //make real-time motion instructions more timely
            //   if (nonHeadJointQ || lastToken != T_SKILL) {
            //     transform(targetFrame, 1, 4);
            //     skill->convertTargetToPosture();
            //   }
            // }
            delete[] pch;
          }
          break;
        }

      // this block handles array like arguments
      case T_INDEXED_SEQUENTIAL_BIN:
      case T_INDEXED_SIMULTANEOUS_BIN:
      case T_READ:
      case T_WRITE:
        {  // indexed joint motions: joint0, angle0, joint1, angle1, ... (binary encoding)
          if (cmdLen < 2)
            manualHeadQ = false;
          else {
            int targetFrame[DOF + 1];
            for (int i = 0; i < DOF; i++) {
              targetFrame[i] = currentAng[i] - (gyroBalanceQ ? currentAdjust[i] : 0);
            }
            targetFrame[DOF] = '~';
            byte group = token == T_WRITE ? 3 : 2;
            for (int i = 0; i < cmdLen; i += group) {
              if (newCmd[i] >= 0 && newCmd[i] < DOF) {
                targetFrame[newCmd[i]] = (int8_t)newCmd[i + 1];
                if (newCmd[i] < 4) {
                  targetHead[newCmd[i]] = (int8_t)newCmd[i + 1];
                  manualHeadQ = true;
                } else
                  nonHeadJointQ = true;
              }
              if (token == T_INDEXED_SEQUENTIAL_BIN) {
                transform(targetFrame, 1, transformSpeed);
                delay(10);
              } else if (token == T_WRITE) {
                pinMode(newCmd[i + 1], OUTPUT);
                if (newCmd[i] == TYPE_ANALOG) {
                  analogWrite(newCmd[i + 1], uint8_t(newCmd[i + 2]));  // analog value can go up to 255.
                                                                       // the value was packed as unsigned byte by ardSerial
                                                                       // but casted by readSerial() as signed char and saved into newCmd.
                } else if (newCmd[i] == TYPE_DIGITAL)
                  digitalWrite(newCmd[i + 1], newCmd[i + 2]);
              } else if (token == T_READ) {
                printToAllPorts('=');
                pinMode(newCmd[i + 1], INPUT);
                if (newCmd[i] == TYPE_ANALOG)  // Arduino Uno: A2->16, A3->17
                  printToAllPorts(analogRead(newCmd[i + 1]));
                else if (newCmd[i] == TYPE_DIGITAL)
                  printToAllPorts(digitalRead(newCmd[i + 1]));
              }
            }
            if (nonHeadJointQ || lastToken != T_SKILL) {
              // printToAllPorts(token);
              transform(targetFrame, 1, transformSpeed);  // if (token == T_INDEXED_SEQUENTIAL_BIN) it will be useless
              skill->convertTargetToPosture(targetFrame);
            }
            // if (token == T_INDEXED_SEQUENTIAL_BIN)
            //   skill->convertTargetToPosture();
            // if (token == T_INDEXED_SIMULTANEOUS_BIN) {
            //   PTL(token);  //make real-time motion instructions more timely
            //                // if (lastToken != T_SKILL)
            //   if (nonHeadJointQ || lastToken != T_SKILL) {
            //     transform(targetFrame, 1, 4);
            //     skill->convertTargetToPosture();
            //   }
            // }
          }
          break;
        }
      case T_LISTED_BIN:  // list of all 16 joint: angle0, angle2,... angle15 (binary encoding)
        {
          transform((int8_t *)newCmd, 1, transformSpeed);  // need to add angleDataRatio if the angles are large
          break;
        }
      case T_BEEP_BIN:
        {
          if (cmdLen < 2) {  // toggle on/off the bootup melody
            bool soundState = !i2c_eeprom_read_byte(EEPROM_BOOTUP_SOUND_STATE);
            printToAllPorts(soundState ? "Unmute" : "Muted");
            i2c_eeprom_write_byte(EEPROM_BOOTUP_SOUND_STATE, soundState);
          } else {
            for (byte b = 0; b < cmdLen / 2; b++) {
              if ((int8_t)newCmd[2 * b] > 0 && (int8_t)newCmd[2 * b + 1] > 0)
                beep((int8_t)newCmd[2 * b], 1000 / (int8_t)newCmd[2 * b + 1]);
            }
          }
          break;
        }
      case T_TEMP:
        {  // call the last skill data received from the serial port
          loadDataFromI2cEeprom((unsigned int)i2c_eeprom_read_int16(SERIAL_BUFF));
          skill->buildSkill();
          skill->transformToSkill(skill->nearestFrame());
          printToAllPorts(token);
          token = T_SKILL;
          strcpy(newCmd, "tmp");
          break;
        }
      case T_SKILL_DATA:  // takes in the skill array from the serial port, load it as a regular skill object and run it locally without continuous communication with the master
        {
          unsigned int i2cEepromAddress = SERIAL_BUFF + 2;        // + esp_random() % (EEPROM_SIZE - SERIAL_BUFF - 2 - 2550);  //save to random position to protect the EEPROM
          i2c_eeprom_write_int16(SERIAL_BUFF, i2cEepromAddress);  // the address takes 2 bytes to store
          copydataFromBufferToI2cEeprom(i2cEepromAddress, (int8_t *)newCmd);
          skill->buildSkill();
          skill->transformToSkill(skill->nearestFrame());
          // newCmdIdx = 0;
          strcpy(newCmd, "tmp");
          if (skill->period > 0)
            printToAllPorts(token);
          token = T_SKILL;
          break;
        }
      case T_SKILL:
        {
          if (!strcmp("x", newCmd)        // x for random skill
              || strcmp(lastCmd, newCmd)  // won't transform for the same gait.
              || skill->period <= 1) {    // skill->period can be NULL!
            // it's better to compare skill->skillName and newCmd.
            // but need more logics for non skill cmd in between
            loadBySkillName(newCmd);  // newCmd will be overwritten as dutyAngles then recovered from skill->skillName
            if (skill->period > 0)
              printToAllPorts(token);
            // skill->info();
          }
          break;
        }
      case T_TASK_QUEUE:
        {
          tQueue->createTask();
          break;
        }
      default:
        {
          printToAllPorts("Undefined token!");
          break;
        }
    }

    if (token == T_SKILL && newCmd[0] != '\0') {
      if (skill->period > 0)
        strcpy(lastCmd, newCmd);
      else
        strcpy(lastCmd, "up");
    }

    if (token != T_SKILL || skill->period > 0) {  // it will change the token and affect strcpy(lastCmd, newCmd)
      printToAllPorts(token);                        // postures, gaits and other tokens can confirm completion by sending the token back
      if (lastToken == T_SKILL && (lowerToken == T_GYRO_FINENESS || lowerToken == T_PRINT_GYRO || lowerToken == T_INDEXED_SIMULTANEOUS_ASC || lowerToken == T_INDEXED_SEQUENTIAL_ASC || token == T_JOINTS || token == T_RANDOM_MIND || token == T_SLOPE || token == T_ACCELERATE || token == T_DECELERATE || token == T_PAUSE || token == T_TILT))
        token = T_SKILL;
    }
    resetCmd();
  }

  if (token == T_SKILL) {
    skill->perform();
    if (skill->period > 1)
      delay(delayShort + max(0, int(runDelay
#ifdef GYRO_PIN
                                    - (max(abs(ypr[1]), abs(ypr[2])) / 10)  // accelerate gait when tilted
#endif
                                    )));
    if (skill->period < 0) {
      if (!strcmp(skill->skillName, "fd")) {  // need to optimize logic to combine "rest" and "fold"
        shutServos();
        gyroBalanceQ = false;
        printToAllPorts('g');
        idleTimer = 0;
        token = '\0';
      } else {
        // newCmd[0] = '\0';
        // arrayNCPY(skill->dutyAngles, skill->dutyAngles + (abs(skill->period) - 1) * skill->frameSize, DOF);
        // skill->period = 1;
        // frame = 0;
        skill->convertTargetToPosture(currentAng);
      }
      for (int i = 0; i < DOF; i++)
        currentAdjust[i] = 0;
      printToAllPorts(token);  // behavior can confirm completion by sending the token back
    }
    // if (exceptions && lastCmd[strlen(lastCmd) - 1] < 'L' && skillList->lookUp(lastCmd) > 0) {  //can be simplified here.
    //   if (lastCmd[0] != '\0')
    //     loadBySkillName(lastCmd);
    if (tQueue->cleared() && tQueue->lastTask != NULL) {
      PT("Use last task ");
      tQueue->loadTaskInfo(tQueue->lastTask);
      delete tQueue->lastTask;
      tQueue->lastTask = NULL;
      PTL(newCmd);
    }
  } else if (token == T_SERVO_FEEDBACK)
    servoFeedback(measureServoPin);
}
