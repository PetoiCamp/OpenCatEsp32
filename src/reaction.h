#include "esp32-hal.h"
void dealWithExceptions() {
#ifdef GYRO_PIN
  if (gyroBalanceQ && imuException) {  // the gyro reaction switch can be toggled on/off by the 'g' token
    switch (imuException) {
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
          PTL("EXCEPTION: Fall over");
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
          if (tQueue->cleared() && skill->period == 1) {
            PTL("EXCEPTION: Knocked");
            tQueue->addTask('k', "knock");
#if defined NYBBLE && defined ULTRASONIC
            if (!moduleActivatedQ[0]) {  // serial2)
              int8_t clrRed[] = { 125, 0, 0, 0, 0, 126 };
              int8_t clrBlue[] = { 0, 0, 125, 0, 0, 126 };
              tQueue->addTask('C', clrRed, 1);
              tQueue->addTask('C', clrBlue);
            }
#endif
            tQueue->addTask('k', "up");
          }
          break;
        }
      case -4:
        {
          PTL("EXCEPTION: Pushed");
          // Acceleration Real
          //      ^ head
          //        ^ x+
          //        |
          //  y+ <------ y-
          //        |
          //        | x-
          if (skill->period == 1 && strncmp(lastCmd, "vt", 2)) {
            char xSymbol[] = { '^', 'v' };
            char ySymbol[] = { '<', '>' };
            char xDirection = xSymbol[sign(ARX) > 0];
            char yDirection = ySymbol[sign(ARY) > 0];
            float forceAngle = atan(float(abs(ARX)) / ARY) * degPerRad;
            PT(abs(ARX) > abs(ARY) ? xDirection : yDirection);
            PTHL(" ForceAngle:", forceAngle);
            if (tQueue->cleared()) {
              if (xDirection == '^') {
                // tQueue->addTask('i', yDirection == '<' ? "0 -75" : "0 75");
                if (abs(forceAngle) < 75)
                  // tQueue->addTask('i', yDirection == '<' ? "0 45" : "0 -45");
                  tQueue->addTask('k', yDirection == '<' ? "wkL" : "wkR", 700);
                // tQueue->addTask('i', "");
                else {
                  tQueue->addTask('k', "wkF", 700);
                  // tQueue->addTask('i', "");
                  tQueue->addTask('k', "bkF", 500);
                }
              } else {
                // tQueue->addTask('k', yDirection == '<' ? "bkR" : "bkL", 1000);
                if (abs(forceAngle) < 75)
                  tQueue->addTask('k', yDirection == '<' ? "wkR" : "wkL", 700);
                else {
                  tQueue->addTask('k', "bkF", 500);
                  tQueue->addTask('k', "wkF", 700);
                }
              }
            }
            tQueue->addTask('k', "up");
            delayPrevious = runDelay;
            runDelay = 3;
            PTL();
          }
          break;
        }
      case -5:
        {
          PTL("EXCEPTION: Turned");
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
    // if (imuException != -4)
    print6Axis();
    // readIMU();  // flush the IMU to avoid static readings and infinite loop

    // if (tQueue->lastTask == NULL) {
    //   if (strcmp(lastCmd, "") && strcmp(lastCmd, "lnd") && *strGet(newCmd, -1) != 'L' && *strGet(lastCmd, -1) != 'R') {
    //     PTH("save last task ", lastCmd);
    //     tQueue->lastTask = new Task('k', lastCmd);
    //   }
    // }
  }
// if (tQueue->cleared() && runDelay <= delayException)
//   runDelay = delayPrevious;
#endif
}

// V_read / 4096 * 3.3 = V_real / ratio
// V_real = V_read / 4096 * 3.3 * ratio
// V_real = V_read / vFactor, vFactor = 4096 / 3.3 / ratio
// a more accurate fitting for V1_0 is V_real = V_read / 515 + 1.95

#ifdef VOLTAGE
bool lowBattery() {
  long currentTime = millis() / CHECK_BATTERY_PERIOD;
  if (currentTime > uptime) {
    uptime = currentTime;
    float voltage = analogRead(VOLTAGE);
#ifdef BiBoard_V1_0
    voltage = voltage / 515 + 1.9;
#else
    voltage = voltage / 414;
#endif
    if (voltage < NO_BATTERY_VOLTAGE2 || (voltage < LOW_VOLTAGE2                                     // powered by 6V, voltage >= NO_BATTERY && voltage < LOW_VOLTAGE2
                                          || voltage > NO_BATTERY_VOLTAGE && voltage < LOW_VOLTAGE)  // powered by 7.4V
                                           && abs(voltage - lastVoltage) < 0.2                       // not caused by power fluctuation during movements
    ) {                                                                                              // if battery voltage is low, it needs to be recharged
      // give the robot a break when voltage drops after sprint
      // adjust the thresholds according to your batteries' voltage
      // if set too high, the robot will stop working when the battery still has power.
      // If too low, the robot may not alarm before the battery shuts off
      lowBatteryQ = true;
      if (!safeRest) {
        // shutServos();
        // delay(2000);
        strcpy(lastCmd, "rest");
        loadBySkillName(lastCmd);
        shutServos();
        safeRest = true;
      }
      if (!batteryWarningCounter) {
        PTF("Low power: ");
        PT(voltage);
        PTL("V");
        PTLF("Long-press the battery's button to turn it on!");
#ifdef I2C_EEPROM_ADDRESS
        if (i2c_eeprom_read_byte(EEPROM_BOOTUP_SOUND_STATE))
#else
        if (config.getBool("bootSndState", 1))
#endif
          playMelody(melodyLowBattery, sizeof(melodyLowBattery) / 2);
      }
      batteryWarningCounter = (batteryWarningCounter + 1) % BATTERY_WARNING_FREQ;
      //    strip.show();
      //       int8_t bStep = 1;
      //       for (byte brightness = 1; brightness > 0; brightness += bStep) {
      // #ifdef NEOPIXEL_PIN
      //         strip.setPixelColor(0, strip.Color(brightness, 0, 0));
      //         strip.show();
      // #endif
      // #ifdef PWM_LED_PIN
      //         analogWrite(PWM_LED_PIN, 255 - brightness);
      // #endif
      //         if (brightness == 255)
      //           bStep = -1;
      //         delay(5);
      //       }
      lastVoltage = voltage;
      return true;
    }
    if (safeRest) {
      // strcpy(lastCmd, "rest");
      // loadBySkillName(lastCmd);
      // shutServos();
      safeRest = false;
    }
    lastVoltage = voltage;
    if ((voltage > LOW_VOLTAGE + 0.2                                         // powered by 7.4V
         || (voltage > LOW_VOLTAGE2 + 0.2 && voltage < NO_BATTERY_VOLTAGE))  // powered by 6V, voltage >= NO_BATTERY && voltage < LOW_VOLTAGE2
        && lowBatteryQ)                                                      // +0.1 to avoid fluctuation around the threshold
    {
      if (voltage > LOW_VOLTAGE + 0.2)
        PTL("Got 7.4 V power");
      else
        PTL("Got 6.0 V power");
      playMelody(melodyOnBattery, sizeof(melodyOnBattery) / 2);
      lowBatteryQ = false;
      batteryWarningCounter = 0;
    }
  }
  return false;
}
#endif

void reaction() {
  if (newCmdIdx) {
    // PTLF("-----");
    lowerToken = tolower(token);
    if (initialBoot) {  //-1 for marking the bootup calibration state
      fineAdjustQ = true;
      // updateGyroQ = true;
      gyroBalanceQ = true;
      autoSwitch = RANDOM_MIND;
      initialBoot = false;
    }
#ifdef PWM_LED_PIN
    digitalWrite(PWM_LED_PIN, HIGH);
#endif
    if (token != T_REST && newCmdIdx < 5)
      idleTimer = millis();
    if (newCmdIdx < 5 && lowerToken != T_BEEP && token != T_MEOW && token != T_LISTED_BIN && token != T_INDEXED_SIMULTANEOUS_BIN && token != T_TILT && token != T_READ && token != T_WRITE)
      beep(15 + newCmdIdx, 5);  // ToDo: check the muted sound when newCmdIdx = -1
    if (!workingStiffness && (lowerToken == T_SKILL || lowerToken == T_INDEXED_SEQUENTIAL_ASC || lowerToken == T_INDEXED_SIMULTANEOUS_ASC)) {
#ifdef T_SERVO_MICROSECOND
      setServoP(P_WORKING);
      workingStiffness = true;
#endif
    }
    if ((lastToken == T_CALIBRATE || lastToken == T_REST || lastToken == T_SERVO_FOLLOW || !strcmp(lastCmd, "fd")) && token != T_CALIBRATE) {
      // updateGyroQ = true;
      gyroBalanceQ = true;
      printToAllPorts('G');
    }
    if (token != T_PAUSE && !tStep) {
      tStep = 1;
      printToAllPorts('p');
    }
#ifdef ESP_PWM
    if (token != T_SERVO_FEEDBACK && token != T_SERVO_FOLLOW && measureServoPin != -1) {
      for (byte i = 0; i < DOF; i++)
        movedJoint[i] = 0;
      reAttachAllServos();
      measureServoPin = -1;
    }
#endif

    switch (token) {
      case T_HELP_INFO:
        {
          PTLF("* Please refer to docs.petoi.com.\nEnter any character to continue.");
          while (!Serial.available())
            ;
          break;
        }
      case T_QUERY:
        {
          printToAllPorts(MODEL);
          printToAllPorts(SoftwareVersion);
          break;
        }
      case T_NAME:
        {
          if (cmdLen > 16)
            printToAllPorts("ERROR! The name should be within 16 characters!");
          else if (cmdLen)
            customBleID(newCmd, cmdLen);  // customize the Bluetooth device's broadcast name. e.g. nMyDog will name the device as "MyDog"
                                          // it takes effect the next time the board boosup. it won't interrupt the current connecton.
          printToAllPorts(
#ifdef I2C_EEPROM_ADDRESS
            readLongByBytes(EEPROM_BLE_NAME)
#else
            config.getString("ID")
#endif
          );
          break;
        }
      case T_GYRO:
      case T_RANDOM_MIND:
        {
          if (token == T_RANDOM_MIND) {
            autoSwitch = !autoSwitch;
            token = autoSwitch ? 'Z' : 'z';  // G for activated gyro
          }
#ifdef GYRO_PIN
          else if (token == T_GYRO) {
            if (cmdLen == 0) {
              gyroBalanceQ = !gyroBalanceQ;
              token = gyroBalanceQ ? 'G' : 'g';  // G for activated gyro
            } else {
              byte i = 0;
              while (newCmd[i] != '\0') {
                if (toupper(newCmd[i]) == T_GYRO_FINENESS) {
                  fineAdjustQ = newCmd[i] == T_GYRO_FINENESS;
                  token = fineAdjustQ ? 'G' : 'g';  // G for activated gyro
                } else if (toupper(newCmd[i]) == T_GYRO_BALANCE)
                  gyroBalanceQ = newCmd[i] == T_GYRO_BALANCE;
                else if (toupper(newCmd[i]) == T_GYRO_PRINT) {
                  printGyroQ = newCmd[i] == T_GYRO_PRINT;
                  print6Axis();
                } else if (newCmd[i] == '?') {
                  PTF("Gyro state:");
                  PTT(" Balance-", gyroBalanceQ);
                  PTT(" Print-", printGyroQ);
                  PTTL(" Frequency-", fineAdjustQ);
                }
                i++;
              }
              imuSkip = fineAdjustQ ? IMU_SKIP : IMU_SKIP_MORE;
              runDelay = fineAdjustQ ? delayMid : delayShort;
            }
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
#ifdef VOLTAGE
      case T_POWER:
        {
          float voltage = analogRead(VOLTAGE);
#ifdef BiBoard_V1_0
          voltage = voltage / 515 + 1.9;
#else
          voltage = voltage / 414;
#endif
          String message = "Voltage: ";
          printToAllPorts(message + voltage + " V");
          break;
        }
#endif
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
          gyroBalanceQ = false;
          printToAllPorts('g');
          if (cmdLen == 0) {
            strcpy(newCmd, "rest");
            if (strcmp(newCmd, lastCmd)) {
              loadBySkillName(newCmd);
            }
            shutServos();
            manualHeadQ = false;
          } else if (cmdLen == 1) {  // allow turning off a single joint
            shutServos(atoi(newCmd));
          }
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
        // case T_MELODY:
        //   {
        //     playMelody(melody1, sizeof(melody1) / 2);
        //     break;
        //   }
#ifdef ULTRASONIC
      case T_COLOR:
        {
          if (!ultrasonicLEDinitializedQ)
            rgbUltrasonicSetup();
          if (cmdLen < 2)  // a single 'C' will turn off the manual color mode
            manualEyeColorQ = false;
          else {  // turn on the manual color mode
            manualEyeColorQ = true;
            ultrasonic.SetRgbEffect(E_RGB_INDEX(uint8_t(newCmd[3])), ultrasonic.color(newCmd[0], newCmd[1], newCmd[2]), uint8_t(newCmd[4]));
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
            SERIAL_VOICE.println("XAc");
#endif
          break;
        }
      case T_ABORT:
        {
          PTLF("aborted");
#ifdef I2C_EEPROM_ADDRESS
          i2c_eeprom_read_buffer(EEPROM_CALIB, (byte *)servoCalib, DOF);
#else
          config.getBytes("calib", servoCalib, DOF);
#endif
#ifdef VOICE
          if (newCmdIdx == 2)
            SERIAL_VOICE.println("XAc");
#endif
          break;
        }
      case T_RESET:
        {
          resetAsNewBoard('R');
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
      case T_SERVO_FOLLOW:
#endif
      case T_TILT:  // tilt the robot, format: t axis angle. 0:yaw, 1:pitch, 2:roll
      case T_MEOW:  // meow
      case T_BEEP:  // beep(tone, duration): tone 0 is pause, duration range is 0~255
#ifdef T_TUNER
      case T_TUNER:
#endif
      case T_BALANCE_SLOPE:
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
                if (target[0] == DOF) {  // auto calibrate all body joints using servos' angle feedback
                  strcpy(newCmd, "rest");
                  loadBySkillName(newCmd);
                  shutServos();
                  autoCalibrate();
                  break;
                }
                if (lastToken != T_CALIBRATE) {
#ifdef T_SERVO_MICROSECOND
                  setServoP(P_HARD);
                  workingStiffness = false;
#endif
#ifdef VOICE
                  if (newCmdIdx == 2)  // only deactivate the voice module via serial port
                    SERIAL_VOICE.println("XAd");
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
#ifdef ROBOT_ARM
                if (target[0] == -2)  // auto calibrate the robot arm's pincer
                {
                  // loadBySkillName("triStand");
                  // shutServos();
                  calibratedPWM(1, 90);
                  delay(500);
                  int criticalAngle = calibratePincerByVibration(-25, 25, 4);
                  criticalAngle = calibratePincerByVibration(criticalAngle - 4, criticalAngle + 4, 1);
                  servoCalib[2] = servoCalib[2] + criticalAngle + 16;
                  PTHL("Pincer calibrate angle: ", servoCalib[2]);
#ifdef I2C_EEPROM_ADDRESS
                  i2c_eeprom_write_byte(EEPROM_CALIB + 2, servoCalib[2]);
#else
                  config.putBytes("calib", servoCalib, DOF);
#endif
                  calibratedZeroPosition[2] = zeroPosition[2] + float(servoCalib[2]) * rotationDirection[2];
                  loadBySkillName("calib");
                } else
#endif
                  if (target[0] < DOF && target[0] >= 0) {
                  int duty = zeroPosition[target[0]] + float(servoCalib[target[0]]) * rotationDirection[target[0]];
                  if (PWM_NUM == 12 && WALKING_DOF == 8 && target[0] > 3 && target[0] < 8)  // there's no such joint in this configuration
                    continue;
                  int actualServoIndex = (PWM_NUM == 12 && target[0] > 3) ? target[0] - 4 : target[0];
#ifdef ESP_PWM
                  servo[actualServoIndex].write(duty);
#else
                  pwm.writeAngle(actualServoIndex, duty);
#endif
                }
                printToAllPorts(range2String(DOF));
                printToAllPorts(list2String(servoCalib));
                printToAllPorts(token);
                // printToAllPorts(list2String(target, 2));
              } else if (token == T_INDEXED_SEQUENTIAL_ASC) {
                transform(targetFrame, 1, 1);
                delay(10);
              }
#ifdef T_SERVO_MICROSECOND
              else if (token == T_SERVO_MICROSECOND) {  // there might be some problems.
#ifdef ESP_PWM
                servo[PWM_pin[target[0]]].writeMicroseconds(target[1]);
#else
                pwm.writeMicroseconds(PWM_pin[target[0]], target[1]);
#endif
              }
#endif
#ifdef T_SERVO_FEEDBACK
              else if (token == T_SERVO_FEEDBACK) {
                setServoP(P_SOFT);
                workingStiffness = false;
                gyroBalanceQ = false;
                // measureServoPin = (inLen == 1) ? target[0] : 16;
                if (inLen == 0)
                  measureServoPin = 16;
                else if (inLen == 1 && target[0] > 2500 && target[0] < 4000) {
                  feedbackSignal = target[0];
                  PTF("Change feedback signal to ");
                  PTL(feedbackSignal);
                } else
                  measureServoPin = target[0];
              } else if (token == T_SERVO_FOLLOW) {
                setServoP(P_SOFT);
                workingStiffness = false;
                gyroBalanceQ = false;
                measureServoPin = 16;
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
                if (inLen == 0) {  // toggle on/off the bootup melody

#ifdef I2C_EEPROM_ADDRESS
                  soundState = !i2c_eeprom_read_byte(EEPROM_BOOTUP_SOUND_STATE);
                  i2c_eeprom_write_byte(EEPROM_BOOTUP_SOUND_STATE, soundState);
#else
                  soundState = !config.getBool("bootSndState");
                  config.putBool("bootSndState", soundState);
#endif
                  printToAllPorts(soundState ? "Unmute" : "Muted");
                  if (soundState && !buzzerVolume) {  // if i want to unmute but the volume was set to 0
                    buzzerVolume = 5;                 // set the volume to 5/10
#ifdef I2C_EEPROM_ADDRESS
                    i2c_eeprom_write_byte(EEPROM_BUZZER_VOLUME, buzzerVolume);
#else
                    config.putChar("buzzerVolume", buzzerVolume);
#endif
                    playMelody(volumeTest, sizeof(volumeTest) / 2);
                  }
                } else if (inLen == 1) {                      // change the buzzer's volume
                  buzzerVolume = max(0, min(10, target[0]));  // in scale of 0~10
                  if (soundState ^ (buzzerVolume > 0))
                    printToAllPorts(buzzerVolume ? "Unmute" : "Muted");  // only print if the soundState changes
                  soundState = buzzerVolume;
#ifdef I2C_EEPROM_ADDRESS
                  i2c_eeprom_write_byte(EEPROM_BOOTUP_SOUND_STATE, soundState);
                  i2c_eeprom_write_byte(EEPROM_BUZZER_VOLUME, buzzerVolume);
#else
                  config.putBool("bootSndState", soundState);
                  config.putChar("buzzerVolume", buzzerVolume);
#endif
                  PTF("Changing volume to ");
                  PT(buzzerVolume);
                  PTL("/10");
                  playMelody(volumeTest, sizeof(volumeTest) / 2);
                } else if (target[1] > 0) {
                  beep(target[0], 1000 / target[1]);
                }
              }
#ifdef T_TUNER
              else if (token == T_TUNER) {
                if (inLen > 1) {
                  *par[target[0]] = target[1];
                  PT(target[0]);
                  PT('\t');
                  PTL(target[1]);
                }
              }
#endif
              else if (token == T_BALANCE_SLOPE) {
                if (inLen == 2) {
                  balanceSlope[0] = max(-2, min(2, target[0]));
                  balanceSlope[1] = max(-2, min(2, target[1]));
                }
              }
              // delay(5);
            } while (pch != NULL);
#ifdef T_TUNER
            if (token == T_TUNER) {
              for (byte p = 0; p < sizeof(initPars) / sizeof(int8_t); p++) {
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
              } else if (token == T_WRITE) {  // Write a/d pin value
                pinMode(newCmd[i + 1], OUTPUT);
                if (newCmd[i] == TYPE_ANALOG) {
                  analogWrite(newCmd[i + 1], uint8_t(newCmd[i + 2]));  // analog value can go up to 255.
                                                                       // the value was packed as unsigned byte by ardSerial
                                                                       // but casted by readSerial() as signed char and saved into newCmd.
                } else if (newCmd[i] == TYPE_DIGITAL)
                  digitalWrite(newCmd[i + 1], newCmd[i + 2]);
              } else if (token == T_READ) {  // Read a/d pin
                // 34 35 36 37 38 39 97 100
                // "  #  $  %  &  '  a  d
                // e.g. analogRead(35) = Ra# in the Serial Monitor
                //                     = [R,a,35] in the Python API
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
      case EXTENSION:
        {
          // PTH("cmdLen = ", cmdLen);
          if (newCmd[0] != 'U' || (newCmd[0] == 'U' && cmdLen == 1)) {  // when reading the distance from ultrasonic sensor, the cmdLen is 3.
            // and we don't want to change the activation status of the ultrasonic sensor behavior
            reconfigureTheActiveModule(newCmd);
          }

          // deal with the following command
          switch (newCmd[0]) {
#ifdef VOICE
            case EXTENSION_VOICE:
              {
                set_voice();
                break;
              }
#endif
#ifdef ULTRASONIC
            case EXTENSION_ULTRASONIC:
              {
                if (cmdLen >= 3) {
                  printToAllPorts('=');
                  printToAllPorts(readUltrasonic((int8_t)newCmd[1], (int8_t)newCmd[2]));
                }
                break;
              }
#endif
#ifdef CAMERA
            case EXTENSION_CAMERA:
              {
                char *option = newCmd;
                while (*(++option) != '~') {
                  if (*option == 'P')
                    cameraPrintQ = 2;
                  else if (*option == 'p')
                    cameraPrintQ = 1;
                  else if (*option == 'R')
                    cameraReactionQ = true;
                  else if (*option == 'r')
                    cameraReactionQ = false;
                }
                break;
              }
#endif
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
          if (cmdLen == 0) {  // toggle on/off the bootup melody
#ifdef I2C_EEPROM_ADDRESS
            soundState = !i2c_eeprom_read_byte(EEPROM_BOOTUP_SOUND_STATE);
            i2c_eeprom_write_byte(EEPROM_BOOTUP_SOUND_STATE, soundState);
#else
            soundState = !config.getBool("bootSndState");
            config.putBool("bootSndState", soundState);
#endif
            printToAllPorts(soundState ? "Unmute" : "Muted");
          } else {
            for (byte b = 0; b < cmdLen / 2; b++) {
              if ((int8_t)newCmd[2 * b + 1] > 0)
                beep((int8_t)newCmd[2 * b], 1000 / (int8_t)newCmd[2 * b + 1]);
            }
          }
          break;
        }
      case T_SIGNAL_GEN:  // resolution, speed, jointIdx, midpoint, amp, freq,phase
        {
          char *pch = strtok(newCmd, " ,");
          int inLen = 0;
          int8_t pars[60];  // allows 12 joints 5*12 = 60
          while (pch != NULL) {
            pars[inLen++] = atoi(pch);  //@@@ cast
            pch = strtok(NULL, " ,\t");
          }
          // for (int i = 0; i < inLen; i++)
          //   PTT(pars[i], ' ');
          // PTL();
          int8_t resolution = pars[0];
          int8_t speed = pars[1];
          signalGenerator(resolution, speed, pars + 2, inLen, 1);
          break;
        }
      case T_LEARN:
        {
          if (newCmd[0] == 'l') {  // learn
            gyroBalanceQ = false;
            loadBySkillName("up");
            delay(500);
            learnByDrag();
          } else if (newCmd[0] = 'p') {  // perform
            loadBySkillName("up");
            performLearn();
            delay(1000);
            loadBySkillName("up");
          }
          break;
        }
      case T_TEMP:
        {  // call the last skill data received from the serial port
#ifdef I2C_EEPROM_ADDRESS
          loadDataFromI2cEeprom((unsigned int)i2c_eeprom_read_int16(SERIAL_BUFF));
#else
          config.getBytes("tmp", newCmd, config.getBytesLength("tmp"));
#endif
          skill->buildSkill();
          skill->transformToSkill(skill->nearestFrame());
          printToAllPorts(token);
          token = T_SKILL;
          strcpy(newCmd, "tmp");
          break;
        }
      case T_SKILL_DATA:  // takes in the skill array from the serial port, load it as a regular skill object and run it locally without continuous communication with the master
        {
#ifdef I2C_EEPROM_ADDRESS
          unsigned int i2cEepromAddress = SERIAL_BUFF + 2;        // + esp_random() % (EEPROM_SIZE - SERIAL_BUFF - 2 - 2550);  //save to random position to protect the EEPROM
          i2c_eeprom_write_int16(SERIAL_BUFF, i2cEepromAddress);  // the address takes 2 bytes to store
          copydataFromBufferToI2cEeprom(i2cEepromAddress, (int8_t *)newCmd);
#else
          int bufferLen = dataLen(newCmd[0]);
          config.putBytes("tmp", newCmd, bufferLen);
#endif
          skill->buildSkill();
          skill->transformToSkill(skill->nearestFrame());
          manualHeadQ = false;
          strcpy(newCmd, "tmp");
          break;
        }
      case T_SKILL:
        {
          if (!strcmp("x", newCmd)        // x for random skill
              || strcmp(lastCmd, newCmd)  // won't transform for the same gait.
              || skill->period <= 1) {    // skill->period can be NULL!
            // it's better to compare skill->skillName and newCmd.
            // but need more logics for non skill cmd in between
            if (!strcmp(newCmd, "bk"))
              strcpy(newCmd, "bkF");
            loadBySkillName(newCmd);  // newCmd will be overwritten as dutyAngles then recovered from skill->skillName
            manualHeadQ = false;
            // if (skill->period > 0)
            //   printToAllPorts(token);
            // skill->info();
          }
          break;
        }
      case T_TASK_QUEUE:
        {
          tQueue->createTask();  // use 'q' to start the sequence.
                                 // add subToken followed by the subCommand
                                 // use ':' to add the delay time (mandatory)
                                 // add '>' to end the sub command
                                 // example: qk sit:1000>m 8 0 8 -30 8 0:500>
                                 // Nybble wash face: qksit:100>o 1 0, 0 40 -20 4 0, 1 -30 20 4 30, 8 -70 10 4 60, 12 -10 10 4 0, 15 10 0 4 0:100>
          break;
        }
      default:
        {
          printToAllPorts("Undefined token!");
          break;
        }
    }

    if (token == T_SKILL && newCmd[0] != '\0') {
      // if (skill->period > 0)
      strcpy(lastCmd, newCmd);
      // else
      //   strcpy(lastCmd, "up");
    }

    if (token != T_SKILL || skill->period > 0) {  // it will change the token and affect strcpy(lastCmd, newCmd)
      printToAllPorts(token);                     // postures, gaits and other tokens can confirm completion by sending the token back
      if (lastToken == T_SKILL && (lowerToken == T_GYRO || lowerToken == T_INDEXED_SIMULTANEOUS_ASC || lowerToken == T_INDEXED_SEQUENTIAL_ASC || lowerToken == T_PAUSE || token == T_JOINTS || token == T_RANDOM_MIND || token == T_BALANCE_SLOPE || token == T_ACCELERATE || token == T_DECELERATE || token == T_TILT))
        token = T_SKILL;
    }
    resetCmd();
#ifdef PWM_LED_PIN
    digitalWrite(PWM_LED_PIN, LOW);
#endif
  }

  if (tolower(token) == T_SKILL) {
#ifdef PWM_LED_PIN
    analogWrite(PWM_LED_PIN, abs(currentAng[8]));
#endif
    skill->perform();
    if (skill->period > 1)
      delay(delayShort + max(0, int(runDelay
#ifdef GYRO_PIN
                                    - gyroBalanceQ * (max(abs(ypr[1]), abs(ypr[2])) / 10)  // accelerate gait when tilted
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
        if (interruptedDuringBehavior) {
          loadBySkillName("up");
        } else
          skill->convertTargetToPosture(currentAng);
      }
      for (int i = 0; i < DOF; i++)
        currentAdjust[i] = 0;
      printToAllPorts(token);  // behavior can confirm completion by sending the token back
    }
    // if (imuException && lastCmd[strlen(lastCmd) - 1] < 'L' && skillList->lookUp(lastCmd) > 0) {  //can be simplified here.
    //   if (lastCmd[0] != '\0')
    //     loadBySkillName(lastCmd);

    // if (tQueue->cleared() && tQueue->lastTask != NULL) {
    //   PT("Use last task ");
    //   tQueue->loadTaskInfo(tQueue->lastTask);
    //   delete tQueue->lastTask;
    //   tQueue->lastTask = NULL;
    //   PTL(newCmd);
    // }
  } else if (token == T_SERVO_FEEDBACK)
    servoFeedback(measureServoPin);
  else if (token == T_SERVO_FOLLOW) {
    if (servoFollow()) {  // don't move the joints if no manual movement is detected
      reAttachAllServos();
      setServoP(P_SOFT);
      workingStiffness = false;
      transform((int8_t *)newCmd, 1, 2);
    }
  } else {
    // wdtTimer = millis();
    delay(1);  // avoid triggering WDT
  }
}
