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
  if (gyroBalanceQ && !(frame % imuSkip))
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

#ifdef BT_SPP
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
boolean confirmRequestPending = true;
boolean BTconnected = false;

void BTConfirmRequestCallback(uint32_t numVal) {
  confirmRequestPending = true;
  Serial.println(numVal);
}

void BTAuthCompleteCallback(boolean success) {
  confirmRequestPending = false;
  if (success) {
    BTconnected = true;
    Serial.println("Pairing success!!");
  } else {
    BTconnected = false;
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

#endif

void printToken(char t = token) {
#ifdef BT_BLE
  if (deviceConnected)
    bleWrite(String(t));
#endif
#ifdef BT_SPP
  if (BTconnected)
    SerialBT.println(t);
#endif
  PTL(t);
}

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
            printToken(token);
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
