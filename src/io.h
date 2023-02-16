void read_sound() {
}
int read_light() {
  return 0;
}
int read_distance() {
  return 0;
}
void read_GPS() {
}
#ifdef TOUCH0
void read_touch() {
  byte touchPin[] = {
    TOUCH0,
    TOUCH1,
    TOUCH2,
    TOUCH3,
  };
  for (byte t = 0; t < 4; t++) {
    int touchValue = touchRead(touchPin[t]);  //do something with the touch?
    //    PT(touchValue);
    //    PT('\t');
  }
  //  PTL();
}
#endif
void readEnvironment() {
#ifdef GYRO_PIN
  if (checkGyro)
    imuUpdated = read_IMU();
#endif
  read_sound();
  read_light();
  read_distance();
  read_GPS();
}

template<typename T> void arrayNCPY(T *destination, const T *source, int len) {  //deep copy regardless of '\0'
  for (int i = 0; i < len; i++)
    destination[i] = source[i];
}

//— read master computer’s signals (middle level) —

//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Richard Li - 2020
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP with authentication)
//and also demonstrate that SerialBT has the same functionalities as a normal Serial

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
boolean confirmRequestPending = true;

void BTConfirmRequestCallback(uint32_t numVal) {
  confirmRequestPending = true;
  Serial.println(numVal);
}

void BTAuthCompleteCallback(boolean success) {
  confirmRequestPending = false;
  if (success) {
    Serial.println("Pairing success!!");
  } else {
    Serial.println("Pairing failed, rejected by user!!");
  }
}

void blueSspSetup() {
  SerialBT.enableSSP();
  SerialBT.onConfirmRequest(BTConfirmRequestCallback);
  SerialBT.onAuthComplete(BTAuthCompleteCallback);
  SerialBT.begin(strcat(readBleID(), "_SSP"));  //Bluetooth device name
  Serial.println("The device is started, now you can pair it with bluetooth!");
}

//void readBlueSSP() {
//  if (confirmRequestPending)
//  {
//    if (Serial.available())
//    {
//      int dat = Serial.read();
//      if (dat == 'Y' || dat == 'y')
//      {
//        SerialBT.confirmReply(true);
//      }
//      else
//      {
//        SerialBT.confirmReply(false);
//      }
//    }
//  }
//  else
//  {
//    if (Serial.available())
//    {
//      SerialBT.write(Serial.read());
//    }
//    if (SerialBT.available())
//    {
//      Serial.write(SerialBT.read());
//    }
//    delay(20);
//  }
//}

//end of Richard Li's code

void resetCmd() {
  //  PTL("lastT: " + String(lastToken) + "\tT: " + String(token) + "\tLastCmd: " + String(lastCmd) + "\tCmd: " + String(newCmd));
  if (lastToken == T_SKILL)
    idleThreshold = IDLE_SHORT;
  else
    idleThreshold = IDLE_LONG;
  if (strcmp(newCmd, "rc")) {
    strcpy(lastCmd, newCmd);
  }
  newCmdIdx = 0;
  lastToken = token;
  if (token != T_SKILL && token != T_CALIBRATE)
    token = '\0';
  newCmd[0] = '\0';
  cmdLen = 0;
}


void read_serial() {
  if (Serial.available() > 0) {
    Stream *stream = Serial;
    token = Serial.read();
    delay(1);  //leave enough time for serial read

    bufferPtr = (token == T_SKILL || token == T_INDEXED_SIMULTANEOUS_BIN) ? (int8_t *)newCmd : dataBuffer;                         // save in a independent memory to avoid breaking the current running skill
    char terminator = (token < 'a') ? '~' : '\n';                                                                                  //capitalized tokens use binary encoding for long data commands
                                                                                                                                   //'~' ASCII code = 126; may introduce bug when the angle is 126 so only use angles <= 125
    int timeout = (token == T_SKILL_DATA || token == T_BEEP_BIN || token == T_BEEP) ? SERIAL_TIMEOUT_LONG : SERIAL_TIMEOUT_SHORT;  //the lower case tokens are encoded in ASCII and can be entered in Arduino IDE's serial monitor
                                                                                                                                   //if the terminator of the command is set to "no line ending" or "new line", parsing can be different
                                                                                                                                   //so it needs a timeout for the no line ending case
    long lastTime = 0;
    do {
      if (Serial.available()) {
        if (cmdLen > CMD_LEN && bufferPtr == (int8_t *)newCmd || cmdLen > BUFF_LEN && bufferPtr == dataBuffer) {  //} || token == T_INDEXED_SIMULTANEOUS_ASC)) {
          PTLF("OVF");                                                                                            //when it overflows, the head value of dataBuffer will be changed. why???
          do { Serial.read(); } while (Serial.available());
          PTL(token);
          token = T_SKILL;
          strcpy(newCmd, "vtF");
          cmdLen = 7;
          break;
        }
        bufferPtr[cmdLen++] = Serial.read();
        lastTime = millis();
      }
    } while ((char)bufferPtr[cmdLen - 1] != terminator && long(millis() - lastTime) < timeout);
    cmdLen = (bufferPtr[cmdLen - 1] == terminator) ? cmdLen - 1 : cmdLen;
    bufferPtr[cmdLen] = '\0';
    newCmdIdx = 2;
    PTL(cmdLen);
  } else if (SerialBT.available() > 0) {
    Stream *stream = SerialBT;
    token = SerialBT.read();
    delay(1);  //leave enough time for serial read

    bufferPtr = (token == T_SKILL || token == T_INDEXED_SIMULTANEOUS_BIN) ? (int8_t *)newCmd : dataBuffer;                         // save in a independent memory to avoid breaking the current running skill
    char terminator = (token < 'a') ? '~' : '\n';                                                                                  //capitalized tokens use binary encoding for long data commands
                                                                                                                                   //'~' ASCII code = 126; may introduce bug when the angle is 126 so only use angles <= 125
    int timeout = (token == T_SKILL_DATA || token == T_BEEP_BIN || token == T_BEEP) ? SERIAL_TIMEOUT_LONG : SERIAL_TIMEOUT_SHORT;  //the lower case tokens are encoded in ASCII and can be entered in Arduino IDE's serial monitor
                                                                                                                                   //if the terminator of the command is set to "no line ending" or "new line", parsing can be different
                                                                                                                                   //so it needs a timeout for the no line ending case
    long lastTime = 0;
    do {
      if (SerialBT.available()) {
        if (cmdLen > CMD_LEN && bufferPtr == (int8_t *)newCmd || cmdLen > BUFF_LEN && bufferPtr == dataBuffer) {  //} || token == T_INDEXED_SIMULTANEOUS_ASC)) {
          PTLF("OVF");                                                                                            //when it overflows, the head value of dataBuffer will be changed. why???
          do { SerialBT.read(); } while (SerialBT.available());
          PTL(token);
          token = T_SKILL;
          strcpy(newCmd, "vtF");
          cmdLen = 7;
          break;
        }
        bufferPtr[cmdLen++] = SerialBT.read();
        lastTime = millis();
      }
    } while ((char)bufferPtr[cmdLen - 1] != terminator && long(millis() - lastTime) < timeout);
    cmdLen = (bufferPtr[cmdLen - 1] == terminator) ? cmdLen - 1 : cmdLen;
    bufferPtr[cmdLen] = '\0';
    newCmdIdx = 2;
    PTL(cmdLen);
  }
}

void readSignal() {
#ifdef IR_PIN
  if (!deviceConnected)  //bluetooth controller will disable the IR receiver
    read_infrared();     //  newCmdIdx = 1
#endif
  read_serial();  //  newCmdIdx = 2
  detectBle();    //  newCmdIdx = 3;
  readBle();


#ifdef VOICE
  read_voice();
#endif

  long current = millis();
  if (newCmdIdx)
    idleTimer = millis() + IDLE_LONG * 1000;
  else if (token != T_CALIBRATE && current - idleTimer > 0) {


#ifdef CAMERA
    read_camera();
#endif
#ifdef TOUCH0
    read_touch();
#endif
    // powerSaver -> 4
    // other -> 5
    // randomMind -> 100

    if (autoSwitch && newCmdIdx == 0)
      randomMind();  //make the robot do random demos
  }
}

void printToken(char t = token) {
  if (deviceConnected)
    bleWrite(String(t));
  if (!confirmRequestPending)
    SerialBT.println(t);
  PTL(t);
}
//— read human sensors (top level) —

String read_gesture() {
  return "";
}

void readHuman() {
  read_gesture();
#ifdef TOUCH0
  read_touch();
#endif
}
//— generate behavior by fusing all sensors and instruction
String decision() {
  return "";
}
