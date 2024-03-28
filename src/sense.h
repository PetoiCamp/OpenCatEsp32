void read_serial() {
  Stream *serialPort = NULL;
// String source;
#ifdef BT_SPP
  if (SerialBT.available()) {  //give BT a higher priority over wired serial
    serialPort = &SerialBT;
    // source = "BT";
  } else
#endif
    if (Serial.available()) {
    serialPort = &Serial;
    // source = "SER";
  }
  if (serialPort) {
    token = serialPort->read();
    lowerToken = tolower(token);
    newCmdIdx = 2;
    delay(1);                                                  //leave enough time for serial read
    terminator = (token >= 'A' && token <= 'Z') ? '~' : '\n';  //capitalized tokens use binary encoding for long data commands
                                                               //'~' ASCII code = 126; may introduce bug when the angle is 126 so only use angles <= 125
    serialTimeout = (token == T_SKILL_DATA || lowerToken == T_BEEP) ? SERIAL_TIMEOUT_LONG : SERIAL_TIMEOUT;
    lastSerialTime = millis();
    do {
      if (serialPort->available()) {
        // long current = millis();
        // PTH(source, current - lastSerialTime);
        do {
          if ((token == T_SKILL || lowerToken == T_INDEXED_SIMULTANEOUS_ASC || lowerToken == T_INDEXED_SEQUENTIAL_ASC) && cmdLen >= spaceAfterStoringData
              || cmdLen >= BUFF_LEN) {
            PTF("OVF");
            beep(5, 100, 50, 5);
            do { serialPort->read(); } while (serialPort->available());
            printToAllPorts(token);
            token = T_SKILL;
            strcpy(newCmd, "up");
            cmdLen = 2;
            return;
          }
          newCmd[cmdLen++] = serialPort->read();
        } while (serialPort->available());
        lastSerialTime = millis();
      }
    } while (newCmd[cmdLen - 1] != terminator && long(millis() - lastSerialTime) < serialTimeout);  //the lower case tokens are encoded in ASCII and can be entered in Arduino IDE's serial monitor
                                                                                                    //if the terminator of the command is set to "no line ending" or "new line", parsing can be different
                                                                                                    //so it needs a timeout for the no line ending case
    // PTH("* " + source, long(millis() - lastSerialTime));
    cmdLen = (newCmd[cmdLen - 1] == terminator) ? cmdLen - 1 : cmdLen;
    newCmd[cmdLen] = (token >= 'A' && token <= 'Z') ? '~' : '\0';
    newCmdIdx = 2;
    // PTL(cmdLen);
    // printCmdByType(token, newCmd, cmdLen);
  }
}

void readSignal() {
#ifdef IR_PIN
#ifdef BT_BLE
  if (!deviceConnected)  //bluetooth controller will disable the IR receiver
#endif
    read_infrared();  //  newCmdIdx = 1
#endif
  read_serial();  //  newCmdIdx = 2
#ifdef BT_BLE
  detectBle();  //  newCmdIdx = 3;
  readBle();
#endif

#ifdef VOICE
  read_voice();
#endif

  long current = millis();
  if (newCmdIdx)
    idleTimer = millis() +
#ifdef DOUBLE_INFRARED_DISTANCE
                0
#else
                IDLE_TIME
#endif
      ;
  else if (token != T_CALIBRATE && token != T_SERVO_FOLLOW && token != T_SERVO_FEEDBACK && current - idleTimer > 0) {

#ifdef CAMERA
    read_camera();
#endif
#ifdef ULTRASONIC
    readRGBultrasonic();
#endif
#ifdef GESTURE
    read_gesture();
#endif
#ifdef PIR
    read_PIR();
#endif
#ifdef DOUBLE_TOUCH
    read_doubleTouch();
#endif
#ifdef DOUBLE_LIGHT
    read_doubleLight();
#endif
#ifdef DOUBLE_INFRARED_DISTANCE
    // read_doubleInfraredDistance();//has some bugs
#endif
#ifdef TOUCH0
    read_touch();
#endif
    // powerSaver -> 4
    // other -> 5
    // randomMind -> 100

    if (autoSwitch) {
      randomMind();             //make the robot do random demos
      powerSaver(POWER_SAVER);  //make the robot rest after a certain period, the unit is seconds
    }
  }
}

//— read human sensors (top level) —
void readHuman() {
#ifdef TOUCH0
  read_touch();
#endif
}
//— generate behavior by fusing all sensors and instruction
String decision() {
  return "";
}
