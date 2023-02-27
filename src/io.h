#include "soc/gpio_sig_map.h"
void read_sound() {
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
  read_GPS();
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
  // printCmd();
  // if (lastToken == T_SKILL)
  //   idleThreshold = IDLE_SHORT;
  // else
  //   idleThreshold = IDLE_LONG;
  // if (token == T_SKILL && strcmp(dataBuffer, "rc")) {
  //   strcpy(lastCmd, dataBuffer);
  // }
  newCmdIdx = 0;
  lastToken = token;
  if (token != T_SKILL && token != T_CALIBRATE)
    token = '\0';
  dataBuffer[0] = '\0';
  cmdLen = 0;
  // printCmd();
}

void printCmd() {
  PTF("lastT:");
  PT(lastToken);
  PTF("\tT:");
  PT(token);
  PTF("\tLastCmd:");
  PT(lastCmd);
  PT('\t');
  PT(cmdLen);
  PTF("\tCmd:");
  PTL(dataBuffer);
}

void read_serial() {
  Stream *serialPort = NULL;
  if (Serial.available()) {
    serialPort = &Serial;
  } else if (SerialBT.available()) {
    serialPort = &SerialBT;
  }
  if (serialPort) {
    token = serialPort->read();
    lowerToken = tolower(token);
    delay(1);  //leave enough time for serial read
      // save in a independent memory to avoid breaking the current running skill
    terminator = (token < 'a') ? '~' : '\n';                                                                       //capitalized tokens use binary encoding for long data commands
                                                                                                                   //'~' ASCII code = 126; may introduce bug when the angle is 126 so only use angles <= 125
    serialTimeout = (token == T_SKILL_DATA || lowerToken == T_BEEP) ? SERIAL_TIMEOUT_LONG : SERIAL_TIMEOUT_SHORT;  //the lower case tokens are encoded in ASCII and can be entered in Arduino IDE's serial monitor
                                                                                                                   //if the terminator of the command is set to "no line ending" or "new line", parsing can be different
                                                                                                                   //so it needs a timeout for the no line ending case
    lastSerialTime = 0;
    do {
      if (serialPort->available()) {
        if ((token == T_SKILL || lowerToken == T_INDEXED_SIMULTANEOUS_ASC || lowerToken == T_INDEXED_SEQUENTIAL_ASC) && cmdLen > spaceAfterStoringData || cmdLen > BUFF_LEN) {  //} || token == T_INDEXED_SIMULTANEOUS_ASC)) {
          PTLF("OVF");                                                                                                                                                          //when it overflows, the head value of dataBuffer will be changed. why???
          do { serialPort->read(); } while (serialPort->available());
          PTL(token);
          token = T_SKILL;
          strcpy(dataBuffer, "up");
          break;
        }
        dataBuffer[cmdLen++] = serialPort->read();
        lastSerialTime = millis();
      }
    } while ((char)dataBuffer[cmdLen - 1] != terminator && long(millis() - lastSerialTime) < serialTimeout);
    cmdLen = (dataBuffer[cmdLen - 1] == terminator) ? cmdLen - 1 : cmdLen;
    dataBuffer[cmdLen] = token < 'a' ? '~' : '\0';
    newCmdIdx = 2;
   printCmdByType(token, dataBuffer, cmdLen);
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


void readHuman() {
#ifdef TOUCH0
  read_touch();
#endif
}
//— generate behavior by fusing all sensors and instruction
String decision() {
  return "";
}
