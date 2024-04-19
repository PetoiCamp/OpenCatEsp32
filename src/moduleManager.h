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

#ifdef GROVE_SERIAL_PASS_THROUGH
#define ULTRASONIC
#include "ultrasonic.h"
#endif

#ifdef OTHER_MODULES
#endif

#ifdef ALL_RANDOM
#else
// #define GYRO_PIN 0
#endif


int8_t moduleList[] = {
  EXTENSION_DOUBLE_TOUCH,
  EXTENSION_DOUBLE_LIGHT,
  EXTENSION_DOUBLE_IR_DISTANCE,
  EXTENSION_PIR,
  EXTENSION_ULTRASONIC,
  EXTENSION_GESTURE,
  EXTENSION_CAMERA_MU3,
  EXTENSION_VOICE
};
bool *moduleActivatedQ;
int8_t indexOfModule(char moduleName) {
  for (byte i = 0; i < sizeof(moduleList) / sizeof(char); i++)
    if (moduleName == moduleList[i])
      return i;
  return -1;
}
int8_t activeModuleIdx() {
  for (byte i = 0; i < sizeof(moduleList) / sizeof(char); i++)
    if (moduleActivatedQ[i])
      return i;
  return -1;
}
void initModule(byte idx) {
  switch (moduleList[idx]) {
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
        rgbUltrasonicSetup();
        break;
      }
#endif
#ifdef DOUBLE_TOUCH
    case EXTENSION_DOUBLE_TOUCH:
      {
        touchSetup();
        break;
      }
#endif
#ifdef DOUBLE_LIGHT
    case EXTENSION_DOUBLE_LIGHT:
      {
        doubleLightSetup();
        break;
      }
#endif
#ifdef DOUBLE_IR_DISTANCE
    case EXTENSION_DOUBLE_IR_DISTANCE:
      {
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
        gestureSetup();
        break;
      }
#endif
#ifdef CAMERA
    case EXTENSION_CAMERA_MU3:
      {
        cameraSetup();
        break;
      }
#endif
  }
}

void stopModule(byte idx) {
  switch (moduleList[idx]) {
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
        //ultrasonicStop();   // Todo
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
    case EXTENSION_CAMERA_MU3:
      {
        //cameraStop();   // Todo
        break;
      }
#endif
  }
}
void reconfigureTheActiveModule(byte idx) {  // negative number will deactivate all the modules
  for (byte i = 0; i < sizeof(moduleList) / sizeof(char); i++)
    if (i == idx) {
      if (!moduleActivatedQ[i])
        initModule(idx);
      moduleActivatedQ[i] = true;
    } else {
      if (moduleActivatedQ[i]) {
        stopModule(i);  //no need for now
        moduleActivatedQ[i] = false;
      }
    }
}

void showModuleStatus() {
  byte moduleCount = sizeof(moduleList) / sizeof(char);
  printListWithoutString((char *)moduleList, moduleCount);
  printListWithoutString(moduleActivatedQ, moduleCount);
}
void initModuleManager() {
  byte moduleCount = sizeof(moduleList) / sizeof(char);
  PTL(moduleCount);
  moduleActivatedQ = new bool[moduleCount];
  for (byte i = 0; i < moduleCount; i++)
    moduleActivatedQ[i] = false;
  showModuleStatus();
}