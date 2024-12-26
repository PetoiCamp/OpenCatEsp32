#ifdef VOICE
#include "voice.h"
#endif

#ifdef VOICE_LD3320
#include "voiceLD3320.h"
#endif

#ifdef CAMERA
#include "camera.h"
#endif

#ifdef ULTRASONIC
#include "ultrasonic.h"
#endif

#ifdef GESTURE
#include "gesture.h"
#endif

#ifdef PIR
#include "pir.h"
#endif

#ifdef DOUBLE_TOUCH
#include "doubleTouch.h"
#endif

#ifdef DOUBLE_LIGHT
#include "doubleLight.h"
#endif

#ifdef DOUBLE_INFRARED_DISTANCE
#include "doubleInfraredDistance.h"
#endif

#ifdef BACKTOUCH_PIN
#include "backTouch.h"
#endif

#ifdef ROBOT_ARM
#include "robotArm.h"
#endif

#ifdef OTHER_MODULES
#endif

#ifdef ALL_RANDOM
#else
// #define GYRO_PIN 0
#endif

void initModule(char moduleCode) {
  bool successQ = true;
  int8_t index = indexOfModule(moduleCode);
  switch (moduleCode) {
    case EXTENSION_GROVE_SERIAL:
      {
#ifdef BiBoard_V1_0
        Serial2.begin(115200, SERIAL_8N1, 9, 10);
#else
        Serial2.begin(115200, SERIAL_8N1, UART_RX2, UART_TX2);
#endif
        Serial2.setTimeout(SERIAL_TIMEOUT);
        PTL("Start Serial 2");
        break;
      }
#ifdef VOICE
    case EXTENSION_VOICE:
      {
        voiceSetup();
        break;
      }
#endif
#ifdef ULTRASONIC
    case EXTENSION_ULTRASONIC:
      {
        loadBySkillName("sit");
#ifdef NYBBLE
        rgbUltrasonicSetup();
#endif
        break;
      }
#endif
#ifdef DOUBLE_TOUCH
    case EXTENSION_DOUBLE_TOUCH:
      {
        loadBySkillName("sit");
        touchSetup();
        break;
      }
#endif
#ifdef DOUBLE_LIGHT
    case EXTENSION_DOUBLE_LIGHT:
      {
        loadBySkillName("sit");
        doubleLightSetup();
        break;
      }
#endif
#ifdef DOUBLE_IR_DISTANCE
    case EXTENSION_DOUBLE_IR_DISTANCE:
      {
        loadBySkillName("sit");
        doubleInfraredDistanceSetup();
        break;
      }
#endif
#ifdef PIR
    case EXTENSION_PIR:
      {
        pirSetup();
        break;
      }
#endif
#ifdef GESTURE
    case EXTENSION_GESTURE:
      {
        loadBySkillName("sit");
        gestureSetup();
        break;
      }
#endif
#ifdef BACKTOUCH_PIN
    case EXTENSION_BACKTOUCH:
      {
        backTouchSetup();
        break;
      }
#endif
#ifdef CAMERA
    case EXTENSION_CAMERA:
      {
        // loadBySkillName("sit");
        gyroBalanceQ = false;
        if (!cameraSetup()) {
          int i = indexOfModule(moduleCode);
          PTHL("*** Fail to start ", moduleNames[i]);
          successQ = false;
        }
        break;
      }
#endif
#ifdef QUICK_DEMO
    case EXTENSION_QUICK_DEMO:
      {
        break;
      }
#endif
  }
  moduleActivatedQ[index] = successQ;
#ifdef I2C_EEPROM_ADDRESS
  i2c_eeprom_write_byte(EEPROM_MODULE_ENABLED_LIST + index, successQ);
#else
  config.putBytes("moduleState", moduleActivatedQ, sizeof(moduleList) / sizeof(char));
#endif
}

void stopModule(char moduleCode) {
  switch (moduleCode) {
    case EXTENSION_GROVE_SERIAL:
      {
        Serial2.end();
        PTL("Stop Serial 2");
        break;
      }
#ifdef VOICE
    case EXTENSION_VOICE:
      {
        voiceStop();
        break;
      }
#endif
#ifdef ULTRASONIC
    case EXTENSION_ULTRASONIC:
      {
        // ultrasonicStop();   // Todo
        ultrasonicLEDinitializedQ = false;
        break;
      }
#endif
#ifdef DOUBLE_TOUCH
    case EXTENSION_DOUBLE_TOUCH:
      {
        break;
      }
#endif
#ifdef DOUBLE_LIGHT
    case EXTENSION_DOUBLE_LIGHT:
      {
        break;
      }
#endif
#ifdef DOUBLE_IR_DISTANCE
    case EXTENSION_DOUBLE_IR_DISTANCE:
      {
        manualHeadQ = false;
        break;
      }
#endif
#ifdef PIR
    case EXTENSION_PIR:
      {
        break;
      }
#endif
#ifdef GESTURE
    case EXTENSION_GESTURE:
      {
        gestureStop();
        break;
      }
#endif
#ifdef CAMERA
    case EXTENSION_CAMERA:
      {
        // cameraStop();   // Todo
        cameraSetupSuccessful = false;
        cameraTaskActiveQ = 0;
        gyroBalanceQ = true;
        break;
      }
#endif
#ifdef BACKTOUCH_PIN
    case EXTENSION_BACKTOUCH:
      {
        break;
      }
#endif
#ifdef QUICK_DEMO
    case EXTENSION_QUICK_DEMO:
      {
        break;
      }
#endif
  }
}
void showModuleStatus() {
  byte moduleCount = sizeof(moduleList) / sizeof(char);
  printListWithoutString((char *)moduleList, moduleCount);
  printListWithoutString(moduleActivatedQ, moduleCount);
  moduleDemoQ = (moduleActivatedQfunction(EXTENSION_DOUBLE_LIGHT) || moduleActivatedQfunction(EXTENSION_DOUBLE_TOUCH) || moduleActivatedQfunction(EXTENSION_GESTURE) || moduleActivatedQfunction(EXTENSION_DOUBLE_IR_DISTANCE) || moduleActivatedQfunction(EXTENSION_CAMERA) || moduleActivatedQfunction(EXTENSION_PIR) || moduleActivatedQfunction(EXTENSION_BACKTOUCH) || moduleActivatedQfunction(EXTENSION_ULTRASONIC) || moduleActivatedQfunction(EXTENSION_QUICK_DEMO));
}

void reconfigureTheActiveModule(char *moduleCode) {
  if (moduleCode[0] == '?') {
    showModuleStatus();
    return;
  }
  bool statusChangedQ = false;
  // PTHL("mode", moduleCode);                                          // negative number will deactivate all the modules
  for (byte i = 0; i < sizeof(moduleList) / sizeof(char); i++) {     // disable unneeded modules
    if (moduleActivatedQ[i] && moduleList[i] != moduleCode[0]) {     // if the modules is active and different from the new module
      if (moduleList[i] == EXTENSION_VOICE && moduleCode[0] != '~')  // it won't disable the voice
        continue;
      PTHL("- disable", moduleNames[i]);
      stopModule(moduleList[i]);
      moduleActivatedQ[i] = false;
      statusChangedQ = true;
#ifdef I2C_EEPROM_ADDRESS
      i2c_eeprom_write_byte(EEPROM_MODULE_ENABLED_LIST + i, false);
#endif
    }
  }
#ifndef I2C_EEPROM_ADDRESS
  config.putBytes("moduleState", moduleActivatedQ, sizeof(moduleList) / sizeof(char));
#endif
  for (byte i = 0; i < sizeof(moduleList) / sizeof(char); i++) {
    if (moduleList[i] == moduleCode[0] && !moduleActivatedQ[i]) {
      PTHL("+  enable", moduleNames[i]);
      initModule(moduleList[i]);
      statusChangedQ = true;
    }
  }
  if (statusChangedQ)  // if the status of the modules has changed, show the new status
    showModuleStatus();
}

void initModuleManager() {
  byte moduleCount = sizeof(moduleList) / sizeof(char);
  PTL(moduleCount);
  for (byte i = 0; i < moduleCount; i++) {
    if (moduleActivatedQ[i]) {
      initModule(moduleList[i]);
    }
#ifdef VOICE
    else if (moduleList[i] == EXTENSION_VOICE) {
      voiceStop();
    }
#endif
#ifdef ULTRASONIC
    else if (moduleList[i] == EXTENSION_ULTRASONIC) {
      rgbUltrasonicSetup();
    }
#endif
  }
  showModuleStatus();
}

void read_serial() {
  Stream *serialPort = NULL;
// String source;
#ifdef BT_SSP
  if (SerialBT.available()) {  // give BT a higher priority over wired serial
    serialPort = &SerialBT;
    // source = "BT";
  } else
#endif
    if (moduleActivatedQ[0] && Serial2.available()) {
    serialPort = &Serial2;
  } else if (Serial.available()) {
    serialPort = &Serial;
    // source = "SER";
  }
  if (serialPort) {
    token = serialPort->read();
    lowerToken = tolower(token);
    newCmdIdx = 2;
    delay(1);                                                  // leave enough time for serial read
    terminator = (token >= 'A' && token <= 'Z') ? '~' : '\n';  // capitalized tokens use binary encoding for long data commands
                                                               //'~' ASCII code = 126; may introduce bug when the angle is 126 so only use angles <= 125
    serialTimeout = (token == T_SKILL_DATA || lowerToken == T_BEEP) ? SERIAL_TIMEOUT_LONG : SERIAL_TIMEOUT;
    lastSerialTime = millis();
    do {
      if (serialPort->available()) {
        // long current = millis();
        // PTH(source, current - lastSerialTime);
        do {
          if ((token == T_SKILL || lowerToken == T_INDEXED_SIMULTANEOUS_ASC || lowerToken == T_INDEXED_SEQUENTIAL_ASC) && cmdLen >= spaceAfterStoringData || cmdLen > BUFF_LEN) {
            PTH("Cmd Length: ", cmdLen);
            PTF("OVF");
            beep(5, 100, 50, 5);
            do {
              serialPort->read();
            } while (serialPort->available());
            printToAllPorts(token);
            token = T_SKILL;
            strcpy(newCmd, "up");
            cmdLen = 2;
            return;
          }
          newCmd[cmdLen++] = serialPort->read();
          // PTHL(newCmd[cmdLen - 1], int8_t(newCmd[cmdLen - 1]));
        } while (serialPort->available());
        lastSerialTime = millis();
      }
    } while (newCmd[cmdLen - 1] != terminator && long(millis() - lastSerialTime) < serialTimeout);  // the lower case tokens are encoded in ASCII and can be entered in Arduino IDE's serial monitor
                                                                                                    // if the terminator of the command is set to "no line ending" or "new line", parsing can be different
                                                                                                    // so it needs a timeout for the no line ending case
    // PTH("* " + source, long(millis() - lastSerialTime));
    if (!(token >= 'A' && token <= 'Z') || token == 'X' || token == 'R' || token == 'W') {  // serial monitor is used to send lower cased tokens by users
                                                                                            // delete the unexpected '\r' '\n' if the serial monitor sends line ending symbols
      leftTrimSpaces(newCmd, &cmdLen);                                                      // allow space between token and parameters, such as "k sit"
      for (int i = cmdLen - 1; i >= 0; i--) {                                               // delete the '/r' and '/n' if the serial monitor is configured to send terminators
        if ((newCmd[i] == '\n') || (newCmd[i] == '\r')) {
          newCmd[i] = '\0';
          cmdLen--;
        }
      }
    }
    cmdLen = (newCmd[cmdLen - 1] == terminator) ? cmdLen - 1 : cmdLen;
    newCmd[cmdLen] = (token >= 'A' && token <= 'Z') ? '~' : '\0';
    if (token >= 'A' && token <= 'Z')
      newCmd[cmdLen + 1] = '\0';
    newCmdIdx = 2;
    // PTH("read_serial, cmdLen = ", cmdLen);
    // printCmdByType(token, newCmd);
  }
}

void readSignal() {
  moduleIndex = activeModuleIdx();
#ifdef IR_PIN
  read_infrared();  //  newCmdIdx = 1
#endif
  read_serial();  //  newCmdIdx = 2
#ifdef BT_BLE
  detectBle();  //  newCmdIdx = 3;
  readBle();
#endif
#ifdef BT_CLIENT
  readBleClient();
#endif
#ifdef VOICE
  if (moduleActivatedQ[indexOfModule(EXTENSION_VOICE)])
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
    if (moduleIndex == -1)  // no active module
      return;
#ifdef CAMERA
    if (moduleActivatedQ[indexOfModule(EXTENSION_CAMERA)])
      read_camera();
#endif
#ifdef ULTRASONIC
    if (moduleActivatedQ[indexOfModule(EXTENSION_ULTRASONIC)])
      read_RGBultrasonic();
#endif
#ifdef GESTURE
    if (moduleActivatedQ[indexOfModule(EXTENSION_GESTURE)])
      read_gesture();
#endif
#ifdef PIR
    if (moduleActivatedQ[indexOfModule(EXTENSION_PIR)])
      read_PIR();
#endif
#ifdef DOUBLE_TOUCH
    if (moduleActivatedQ[indexOfModule(EXTENSION_DOUBLE_TOUCH)])
      read_doubleTouch();
#endif
#ifdef DOUBLE_LIGHT
    if (moduleActivatedQ[indexOfModule(EXTENSION_DOUBLE_LIGHT)])
      read_doubleLight();
#endif
#ifdef DOUBLE_INFRARED_DISTANCE
    if (moduleActivatedQ[indexOfModule(EXTENSION_DOUBLE_IR_DISTANCE)])
      read_doubleInfraredDistance();  // has some bugs
#endif
#ifdef BACKTOUCH_PIN
    if (moduleActivatedQ[indexOfModule(EXTENSION_BACKTOUCH)])
      read_backTouch();
#endif
    // powerSaver -> 4
    // other -> 5
    // randomMind -> 100
    if (autoSwitch) {
      randomMind();             // make the robot do random demos
      powerSaver(POWER_SAVER);  // make the robot rest after a certain period, the unit is seconds
    }
  }
}

// — read human sensors (top level) —
void readHuman() {
#ifdef TOUCH0
  read_touch();
#endif
}
// — generate behavior by fusing all sensors and instruction
String decision() {
  return "";
}

void i2cDetect(TwoWire &wirePort) {
  if (&wirePort == &Wire1)
    wirePort.begin(UART_TX2, UART_RX2, 400000);
  byte error, address;
  int nDevices;
  int8_t i2cAddress[] = {
    0x50, 0x54, 0x60, 0x62, 0x68, 0x69
  };
  String i2cAddressName[] = { "Mu3 CameraP", "EEPROM", "Mu3 Camera", "AI Vision", "MPU6050", "ICM42670" };
  Serial.println("Scanning I2C network...");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    wirePort.beginTransmission(address);
    error = wirePort.endTransmission();
    if (error == 0) {
      Serial.print("- I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(":\t");
      for (byte i = 0; i < sizeof(i2cAddress) / sizeof(int8_t); i++) {
        if (address == i2cAddress[i]) {
          PT(i2cAddressName[i]);
          if (i == 0)
            MuQ = true;
          else if (i == 1)
            eepromQ = true;
          else if (i == 2)
            MuQ = true;  // The older Mu3 Camera and Sentry share the same address. Sentry is not supported yet.
          else if (i == 3)
            GroveVisionQ = true;
          else if (i == 4)
            mpuQ = true;
          else if (i == 5)
            icmQ = true;
          nDevices++;
          break;
        }
        if (i == sizeof(i2cAddress) / sizeof(int8_t) - 1) {
          PT("Misc.");
        }
      }
      PTL();
    } else if (error == 4) {
      Serial.print("- Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (!icmQ && !mpuQ) {
    updateGyroQ = false;
    PTL("No IMU detected!");
  }
  if (nDevices == 0)
    Serial.println("- No I2C devices found");
  else
    Serial.println("- done");
  if (&wirePort == &Wire1)
    wirePort.end();
}

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
    int touchValue = touchRead(touchPin[t]);  // do something with the touch?
    //    PT(touchValue);
    //    PT('\t');
  }
  //  PTL();
}
#endif
void readEnvironment() {
#ifdef GYRO_PIN
  // if (updateGyroQ && !(frame % imuSkip))
  //   imuUpdated = readIMU();
  if (updateGyroQ)
    if (imuUpdated & printGyroQ)
      print6Axis();
#endif
  read_sound();
  read_GPS();
}
