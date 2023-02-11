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

int readSerialUntil(int8_t *destination, char terminator) {
  int c = 0;
  do {
    if (Serial.available())
      destination[c++] = Serial.read();
  } while ((char)destination[c - 1] != terminator);
  destination[c - 1] = '\0';
  return c - 1;
}

int readSerialBTUntil(int8_t *destination, char terminator) {
  int c = 0;
  do {
    if (SerialBT.available())
      destination[c++] = SerialBT.read();
  } while ((char)destination[c - 1] != terminator);
  destination[c - 1] = '\0';
  return c - 1;
}

void read_serial() {
  if (Serial.available() > 0) {
    newCmdIdx = 2;
    token = Serial.read();
    if (token == T_SKILL_DATA)
      readSerialUntil(dataBuffer, '~');
    else if (Serial.available() > 0) {
      String cmdBuffer;  //may overflow after many iterations. I use this method only to support "no line ending" in the serial monitor
      if (token == T_INDEXED_SIMULTANEOUS_BIN || token == T_LISTED_BIN || token == T_MOVE_BIN || token == T_BEEP_BIN || token == T_COLOR) {
        delay(5);                                 // allow long melody to pass over
        cmdBuffer = Serial.readStringUntil('~');  //'~' ASCII code = 126; may introduce bug when the angle is 126
      } else
        cmdBuffer = Serial.readStringUntil('\n');
      cmdLen = cmdBuffer.length();
      char *destination = (token == T_SKILL || token == T_TILT) ? newCmd : (char *)dataBuffer;
      //      for (int i = 0; i < cmdLen; i++)
      //        destination[i] = cmdBuffer[i];
      arrayNCPY(destination, cmdBuffer.c_str(), cmdLen);
      destination[cmdLen] = '\0';
      //      PTL("lastT: " + String(lastToken) + "\tT: " + String(token) + "\tLastCmd: " + String(lastCmd) + "\tCmd: " + String(newCmd));
#ifdef DEVELOPER
      PTF(" memory ");
      PTL(freeMemory());
#endif
    }
  } else if (SerialBT.available() > 0) {
    newCmdIdx = 2;
    token = SerialBT.read();
    if (token == T_SKILL_DATA)
      readSerialBTUntil(dataBuffer, '~');
    else if (SerialBT.available() > 0) {
      String cmdBuffer;  //may overflow after many iterations. I use this method only to support "no line ending" in the serial monitor
      if (token == T_INDEXED_SIMULTANEOUS_BIN || token == T_LISTED_BIN || token == T_MOVE_BIN || token == T_BEEP_BIN || token == T_COLOR) {
        delay(5);                                   // allow long melody to pass over
        cmdBuffer = SerialBT.readStringUntil('~');  //'~' ASCII code = 126; may introduce bug when the angle is 126
      } else
        cmdBuffer = SerialBT.readStringUntil('\n');
      cmdLen = cmdBuffer.length();
      char *destination = (token == T_SKILL || token == T_TILT) ? newCmd : (char *)dataBuffer;
      //      for (int i = 0; i < cmdLen; i++)
      //        destination[i] = cmdBuffer[i];
      arrayNCPY(destination, cmdBuffer.c_str(), cmdLen);
      destination[cmdLen] = '\0';
      //      PTL("lastT: " + String(lastToken) + "\tT: " + String(token) + "\tLastCmd: " + String(lastCmd) + "\tCmd: " + String(newCmd));
#ifdef DEVELOPER
      PTF(" memory ");
      PTL(freeMemory());
#endif
    }
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
