// #include <Wire.h>

#define MU_CAMERA
#define SENTRY1_CAMERA
#define GROVE_VISION_AI_V2
// #define TALL_TARGET

#ifdef BiBoard_V1_0
#define USE_WIRE1  // use the Grove UART as the Wire1, which is independent of Wire used by the main devices, such as the gyroscope and EEPROM.
#endif

#ifdef USE_WIRE1
#define CAMERA_WIRE Wire1
#else
#define CAMERA_WIRE Wire
#endif

#ifdef MU_CAMERA
// You need to install https://github.com/mu-opensource/MuVisionSensor3 as a zip library in Arduino IDE.
// Set the four dial switches on the camera as **v ^ v v** (the second switch dialed up to I2C) and connect the camera module to the I2C grove on NyBoard.
// The battery should be turned on to drive the servos.

// You can use these 3D printed structures to attach the camera module.
// https://github.com/PetoiCamp/NonCodeFiles/blob/master/stl/MuIntelligentCamera_mount.stl
// https://github.com/PetoiCamp/NonCodeFiles/blob/master/stl/bone.stl
// After uploading the code, you may need to press the reset buttons on the module and then the NyBoard.
/*
   Choose communication mode define here:
      I2C_MODE    : I2C mode, default pin: MU_SDA <==> ARDUINO_SDA, MU_SCL <==> ARDUINO_SCL
      SERIAL_MODE : Serial mode, default pin: MU_TX <==> ARDUINO_PIN3, MU_RX <==> ARDUINO_PIN2
*/
#define I2C_MODE
// #define SERIAL_MODE
/*
   Choose MU address here: 0x60, 0x61, 0x62, 0x63
          default address: 0x60
*/
#define MU_ADDRESS 0x50  // in later versions we set the I2C device to 0x50, 0x51, 0x52, 0x53
#define ALT_MU_ADDRESS 0x60

#include <MuVisionSensor.h>  //you need to download the library https://github.com/mu-opensource/MuVisionSensor3 into your ~/Documents/Arduino/libraries/
#ifdef I2C_MODE
#include <Wire.h>
#endif
#ifdef SERIAL_MODE
#include <SoftwareSerial.h>
#define TX_PIN 6
#define RX_PIN 7
SoftwareSerial mySerial(RX_PIN, TX_PIN);
#endif
#endif

#ifdef GROVE_VISION_AI_V2
#include "Seeed_Arduino_SSCMA/src/Seeed_Arduino_SSCMA.h"
// You need to install Seeed_Arduino_SSCMA via Arduino's library manager
// or download the library as a zip from https://github.com/Seeed-Studio/Seeed_Arduino_SSCMA
#endif

#define T_TUNER '>'
bool cameraSetupSuccessful = false;
int xCoord, yCoord, width, widthCounter;  // the x y returned by the sensor
float xError, xIntegral, xDerivative, xLastError = 0;
float yError, yIntegral, yDerivative, yLastError = 0;  // the scaled distance from the center of the frame
int outputX = 0, outputY = 0;                          // the current x y of the camera's direction in the world coordinate
int imgRangeX = 100;                                   // the frame size 0~100 on X and Y direction
int imgRangeY = 100;

int8_t Kp, Ki, Kd, tranSpeed, pan, tilt, frontUpX, backUpX, frontDownX, backDownX, frontUpY, backUpY, frontDownY, backDownY, tiltBase, frontUp, backUp, frontDown, backDown;
int8_t sizePars;
#ifdef ROBOT_ARM
float adjustmentFactor = 1.5;
#else
float adjustmentFactor = 1;
#endif

#define K_INT_TO_FLOAT 100.0
#define JOINT_INT_TO_FLOAT 10.0
#define CAM_COOL_DOWN 1000

#ifdef NYBBLE
int8_t nybblePars[] = {
  30, 11, 2, 20, 10, 15,
  60, -50, 31, -50,
  45, -40, 40, -36,
  0, 25, -60, 60, 16
};
#else  // BITTLE or CUB
#ifdef MU_CAMERA
int8_t bittleMuPars[] = {
  30, 10, 2, 20, 15, 15,
  60, 80, 30, 80,
  60, 30, 30, 70,
  40, 40, 60, 40, -30
};
#endif
#if defined GROVE_VISION_AI_V2
int8_t bittleGroveVisionPars[] = {
  10, 1, 2, 20, 10, 10,
  int8_t(60 * adjustmentFactor), int8_t(75 * adjustmentFactor), int8_t(30 * adjustmentFactor), int8_t(75 * adjustmentFactor),
  20, 25, 10, 25,
  0, 30, 60, 40, -10
};
#endif
#endif

int8_t *par[] = { &Kp, &Ki, &Kd, &tranSpeed, &pan, &tilt,
                  &frontUpX, &backUpX, &frontDownX, &backDownX,
                  &frontUpY, &backUpY, &frontDownY, &backDownY,
                  &tiltBase, &frontUp, &backUp, &frontDown, &backDown };

#ifdef MU_CAMERA
void muCameraSetup();
void read_MuCamera();
#endif

#ifdef SENTRY1_CAMERA
void sentry1CameraSetup();
void read_Sentry1Camera();
#endif

#ifdef GROVE_VISION_AI_V2
void groveVisionSetup();
void read_GroveVision();
#endif

int8_t *initPars;
bool cameraSetup() {
  if (!MuQ && !GroveVisionQ && !SentryQ) {
    return false;
  }
#ifdef NYBBLE
  initPars = nybblePars;
  sizePars = sizeof(nybblePars) / sizeof(int8_t);
#else  // BITTLE or CUB
#ifdef MU_CAMERA
  sizePars = sizeof(bittleMuPars) / sizeof(int8_t);
  if (MuQ)
    initPars = bittleMuPars;
#endif
#ifdef GROVE_VISION_AI_V2
  sizePars = sizeof(bittleGroveVisionPars) / sizeof(int8_t);
  if (GroveVisionQ) {
    initPars = bittleGroveVisionPars;
    printList(initPars, sizePars);
    imgRangeX = 240;  // the frame size 0~240 on X and Y direction
    imgRangeY = 240;
  }
#endif
#endif

#ifdef USE_WIRE1
  CAMERA_WIRE.begin(UART_TX2, UART_RX2, 400000);
#endif
  for (byte i = 0; i < sizePars; i++) {
    *par[i] = initPars[i];
  }
  widthCounter = 0;
#ifdef MU_CAMERA
  if (MuQ)
    muCameraSetup();
#endif
#ifdef SENTRY1_CAMERA
  if (SentryQ) {
    sentry1CameraSetup();
  }
#endif
#ifdef GROVE_VISION_AI_V2
  if (GroveVisionQ) {
    groveVisionSetup();
  }
#endif
  fps = 0;
  loopTimer = millis();
  return cameraSetupSuccessful;
}
void showRecognitionResult(int xCoord, int yCoord, int width, int height = -1) {
  PT(xCoord - imgRangeX / 2.0);  // get vision result: x axes value
  PT('\t');
  PT(yCoord - imgRangeY / 2.0);  // get vision result: y axes value
  PT('\t');
  PT("size = ");
  PT(width);
  if (height >= 0) {
    PT('\t');
    PT(height);
  }
  PT('\t');
}

// #define WALK  //let the robot move its body to follow people rather than sitting at the original position \
              // it works the best on the table so the robot doesn't need to loop upward.
// #define ROTATE

TaskHandle_t TASK_HandleCamera = NULL;
bool cameraTaskActiveQ = 0;
void cameraBehavior(int xCoord, int yCoord, int width) {
  if (cameraReactionQ && detectedObjectQ) {
#ifdef WALK
    if (width > 45 && width != 52)  // 52 maybe a noise signal
      widthCounter++;
    else
      widthCounter = 0;
    if (width < 25 && width != 16) {                                                                    // 16 maybe a noise signal
      tQueue->addTask('k', outputX < -15 ? "wkR" : (outputX > 15 ? "wkL" : "wkF"), (50 - width) * 50);  // walk towards you
      tQueue->addTask('k', "sit");
      tQueue->addTask('i', "");
      outputX = 0;
    } else if (widthCounter > 2) {
      tQueue->addTask('k', "bk", 1000);  // the robot will walk backward if you get too close!
      tQueue->addTask('k', "sit");
      tQueue->addTask('i', "");
      widthCounter = 0;
      outputX = 0;
    } else
#endif
    {
      xError = (imgRangeX / 2.0 - xCoord);                                                                        // atan((xCoord - imgRangeX / 2.0) / (imgRangeX / 2.0)) * degPerRad;//almost the same
      yError = (imgRangeY / 2.0 - yCoord);                                                                        // atan((yCoord - imgRangeY / 2.0) / (imgRangeX / 2.0)) * degPerRad;
      if ((abs(xError) > 1 || abs(yError) > 1) && (abs(xError - xLastError) > 0 || abs(yError - yLastError) > 0)  // the camera is not stable
                                                                                                                  // && (abs(xError - xLastError) < 50) && (abs(xError - xLastError) < 50)                                  // the camera is not moving too fast
          )
      // {
      //   detectedObjectQ = true;
      //   cameraCoolDown = CAM_COOL_DOWN;
      // }
      // else
      // {
      //   detectedObjectQ = false;
      //   cameraCoolDown = max(0, cameraCoolDown - 1);
      // }
      {
        if (detectedObjectQ) {
          xIntegral += xError;
          yIntegral += yError;
          xIntegral = max(min(xIntegral, float(5000)), float(-5000));
          yIntegral = max(min(yIntegral, float(5000)), float(-5000));
          xDerivative = xError - xLastError;
          yDerivative = yError - yLastError;
          xLastError = xError;
          yLastError = yError;
          outputX = max(min(xError * Kp / K_INT_TO_FLOAT + xIntegral * Ki / K_INT_TO_FLOAT + xDerivative * Kd / K_INT_TO_FLOAT, double(85)), double(-85));
          outputY = max(min(yError * Kp / K_INT_TO_FLOAT + yIntegral * Ki / K_INT_TO_FLOAT + yDerivative * Kd / K_INT_TO_FLOAT, double(85)), double(-85));
        } else if (!detectedObjectQ && cameraCoolDown > 0 && cameraCoolDown < CAM_COOL_DOWN - 100) {
          outputX = 0.95 * outputX;
          outputY = 0.95 * outputY;
        }
        PTT("\tcoord:", xCoord);
        PTT("\terror: ", xError);
        PTT("\tintegral: ", xIntegral);
        PTT("\tderivative: ", xDerivative);
        if (outputX || outputY)
          PTT("\toutput: ", outputX);
        PTL();

        // else
        // {
        //   PTLF("Camera centered...");
        //   outputX /= 2;
        //   outputY /= 2;
        //   xError = 0;
        //   yError = 0;
        //   xIntegral = 0;
        //   yIntegral = 0;
        //   delay(50);
        // }
        // PT('\t');
        // PT(outputX);
        // PT('\t');
        // PTL(outputY);
        // if (abs(outputX) < 60) {
        int8_t base[] = { 0, tiltBase, 0, 0,
                          0, 0, 0, 0,
                          frontUp, frontUp, backUp, backUp,
                          frontDown, frontDown, backDown, backDown };
        int8_t feedBackArray[][2] = {
          { pan, 0 },
          { 0, tilt },
          { 0, 0 },
          { 0, 0 },
          { 0, 0 },
          { 0, 0 },
          { 0, 0 },
          { 0, 0 },
          { frontUpX, (int8_t)-frontUpY },  // explicitly convert the calculation result to int8_t
          { (int8_t)-frontUpX, (int8_t)-frontUpY },
          { (int8_t)-backUpX, backUpY },
          { backUpX, backUpY },
          { (int8_t)-frontDownX, frontDownY },
          { frontDownX, frontDownY },
          { backDownX, (int8_t)-backDownY },
          { (int8_t)-backDownX, (int8_t)-backDownY },
        };
        transformSpeed = tranSpeed / JOINT_INT_TO_FLOAT / 5.0;
        for (int i = 0; i < DOF; i++) {
          float adj = float(base[i]) + (feedBackArray[i][0] ? outputX * JOINT_INT_TO_FLOAT / feedBackArray[i][0] : 0) + (feedBackArray[i][1] ? outputY * JOINT_INT_TO_FLOAT / feedBackArray[i][1] : 0);
          newCmd[i] = min(125, max(-125, int(adj)));
          // if (i == 0)//print adjustment of head pan joint
          // {
          //   PT(i);
          //   PT('\t');
          //   PT(adj);
          //   PT('\t');
          //   PT(int8_t(newCmd[i]));
          //   PTF(",\t");
          // }
        }
        // PTL();
        // newCmd[16] = '~';
        // printList((int8_t *)newCmd);
        detectedObjectQ = false;
        transform((int8_t *)newCmd, 1, transformSpeed);
        // }
#ifdef ROTATE
        else {
          tQueue->addTask('k', (outputX < 0 ? "vtR" : "vtL"), abs(outputX) * 40);  // spin its body to follow you
          tQueue->addTask('k', "sit");
          tQueue->addTask('i', "");
          outputX = 0;
        }
#endif
      }
    }
  }
}
int coords[3];

void taskReadCamera(void *par) {
  while (cameraTaskActiveQ) {
#ifdef MU_CAMERA
    if (MuQ)
      read_MuCamera();
#endif
#ifdef SENTRY1_CAMERA
    if (SentryQ)
      read_Sentry1Camera();
#endif
#ifdef GROVE_VISION_AI_V2
    if (GroveVisionQ)
      read_GroveVision();
#endif
    // vTaskDelay(1);
  }
  vTaskDelete(NULL);
}

void read_camera() {
  if (!cameraTaskActiveQ) {
    PTLF("Create Camera Task...");
    xTaskCreatePinnedToCore(
      taskReadCamera,      // task function
      "TaskReadCamera",    // name
      10000,               // task stack size​​
      NULL,                // parameters
      0,                   // priority
      &TASK_HandleCamera,  // handle
      0);                  // core
    cameraTaskActiveQ = 1;
    PTLF("Camera task activated.");
  }
  // long waitingTime = millis();
  // while (!detectedObjectQ && millis() - waitingTime < 20)
  //   delay(1); // wait for the camera to detect an object in another core
  if (detectedObjectQ) {
    cameraCoolDown = CAM_COOL_DOWN;
    if (cameraPrintQ) {
      showRecognitionResult(xCoord, yCoord, width);
      PTL();
      if (cameraPrintQ == 1)
        cameraPrintQ = 0;  // if the command is XCp, the camera will print the result only once
      else
        FPS();
    }
  }
  cameraBehavior(xCoord, yCoord, width);
  if (!cameraCoolDown) {
    if (!(cameraCoolDown % 20))
      PTL(cameraCoolDown);
    cameraCoolDown--;
  }
}
#ifdef MU_CAMERA
MuVisionSensor *Mu;
int skip = 1;  //, counter; //an efforts to reduce motion frequency without using delay. set skip >1 to take effect
int i2cdelay = 3;
long noResultTime = 0;
MuVisionType object[] = { VISION_BODY_DETECT, VISION_BALL_DETECT };
String objectName[] = { "body", "ball" };
int objectIdx = 0;
int lastBallType;
void muCameraSetup() {
  PTL("Setup Mu3");
  uint8_t err;
  // initialized MU on the I2C port
  byte trial = 0;
  do {
    MuVisionSensor *Mu0 = new MuVisionSensor(MU_ADDRESS);
    err = Mu0->begin(&CAMERA_WIRE);
    if (err == MU_OK) {
      PTLF("MU initialized at 0x50");
      Mu = Mu0;
    } else {
      PTHL("Trial ", trial);
      if (!trial++) {  // only print once for the first time
        PTLF("Set the four dial switches on the camera as v ^ v v (the second switch dialed up to I2C)");
        PTLF("Then connect the camera to the I2C Grove socket with SDA and SCL pins!");
        PTLF("The battery should be turned on to drive the servos");
      }
      delete Mu0;
      MuVisionSensor *Mu1 = new MuVisionSensor(ALT_MU_ADDRESS);
      err = Mu1->begin(&CAMERA_WIRE);
      if (err == MU_OK) {
        PTLF("MU initialized at 0x60");
        Mu = Mu1;
      } else {
        delete Mu1;
      }
    }
  } while (err != MU_OK && trial < 1);
  //  shutServos();
  //  counter = 0;
  //  motion.loadBySkillName("rest");
  //  transform(motion.dutyAngles);
  cameraSetupSuccessful = (err == MU_OK);
  if (cameraSetupSuccessful)
    (*Mu).VisionBegin(object[objectIdx]);
  else
    PTL("Failed to initialize the camera!");
  noResultTime = millis();
}

void read_MuCamera() {
  if (cameraSetupSuccessful) {
    if ((*Mu).GetValue(object[objectIdx], kStatus)) {  // update vision result and get status, 0: undetected, other:
      // PTL(objectName[objectIdx]);
      noResultTime = millis();  // update the timer
      updateCoordinateLock = true;
      xCoord = (int)(*Mu).GetValue(object[objectIdx], kXValue);
      yCoord = (int)(*Mu).GetValue(object[objectIdx], kYValue);
      width = (int)(*Mu).GetValue(object[objectIdx], kWidthValue);
      // height = (int)(*Mu).GetValue(VISION_BODY_DETECT, kHeightValue);
      updateCoordinateLock = false;
      // vvvvvvvvvvvv ball vvvvvvvvvvvvv
      detectedObjectQ = true;
      if (objectIdx == 1) {
        int ballType = (*Mu).GetValue(object[objectIdx], kLabel);
        if (lastBallType != ballType) {
          switch ((*Mu).GetValue(object[objectIdx], kLabel)) {  // get vision result: label value
            case MU_BALL_TABLE_TENNIS:
              PTLF("table tennis");
              break;
            case MU_BALL_TENNIS:
              PTLF("tennis");
              break;
            default:
              PTLF("unknow ball type");
              break;
          }
          lastBallType = ballType;
        }
      }
      //^^^^^^^^^^^^^ ball ^^^^^^^^^^^^^^
    } else if (millis() - noResultTime > 2000) {  // if no object is detected for 2 seconds, switch object
      (*Mu).VisionEnd(object[objectIdx]);
      objectIdx = (objectIdx + 1) % (sizeof(object) / 2);
      (*Mu).VisionBegin(object[objectIdx]);
      PTL(objectName[objectIdx]);
      noResultTime = millis();
      beep(25, 50, 50, objectIdx + 1);
    }
  }
}
#endif

#ifdef SENTRY1_CAMERA

#define SENTRY_ADDR 0x60  // 0x61 0x62 0x63

void writeRegData(char reg_addr, char reg_data) {
  CAMERA_WIRE.beginTransmission(SENTRY_ADDR);
  CAMERA_WIRE.write(reg_addr);
  CAMERA_WIRE.write(reg_data);
  CAMERA_WIRE.endTransmission();
  delay(1);
}

char readRegData(char reg_addr) {
  CAMERA_WIRE.beginTransmission(SENTRY_ADDR);
  CAMERA_WIRE.write(reg_addr);  // read label
  CAMERA_WIRE.endTransmission();

  CAMERA_WIRE.requestFrom(SENTRY_ADDR, 1);  // request 1 byte from slave device
  char ret = 0;
  while (CAMERA_WIRE.available()) {
    ret = CAMERA_WIRE.read();  // receive a byte
  }
  return ret;
  delay(1);
}

void sentry1CameraSetup() {
  // CAMERA_WIRE.begin();  // join i2c bus (address optional for master)
  delay(2000);  // wait for sentry1 startup, not necessary
  PTLF("Setup Sentry1");
  writeRegData(0x20, 0x07);  // set vision id: 7 (body for Sentry1)
  writeRegData(0x21, 0x01);  // enable vision
  // writeRegData(0x22, 0x10);  // set vision level: 0x10=Sensitive/Speed 0x20=balance(default if not set) 0x30=accurate ..........[UPDATE]
  delay(1000);
  PTLF("Sentry1 ready");
  tranSpeed = 1;
  cameraSetupSuccessful = true;
}

char frame_cnt = 0;

void read_Sentry1Camera() {
  //  Serial.println("loop...");
  if (cameraSetupSuccessful) {
    char frame_new = readRegData(0x1F);  // update frame count ..........[UPDATE]
    if (frame_new == frame_cnt) {        // if frame is unchanged, delay some time and return ..........[UPDATE]
      delay(10);                         // ..........[UPDATE]
      return;                            //  ..........[UPDATE]
    }                                    //  ..........[UPDATE]
    frame_cnt = frame_new;               // update frame_cnt ..........[UPDATE]

    char label = readRegData(0x89);  // read label/ value Low Byte, 1=detected, 0=undetected
    //  Serial.print("label: ");
    //  Serial.println(label);//
    if (label == 1) {  // have detected
      updateCoordinateLock = true;
      xCoord = readRegData(0x81);  // read x value Low Byte, 0~100, (LB is enought for sentry 1)
      yCoord = readRegData(0x83);  // read y value Low Byte, 0~100
      width = readRegData(0x85);   // read width value Low Byte, 0~100
                                   //  char height = readRegData(0x87);  // read height value Low Byte, 0~100, not necessary if already have read width
                                   // do something ......
      updateCoordinateLock = false;
      detectedObjectQ = true;
      delay(10);
    }
  }
  // do something or delay some time
}
#endif

#ifdef GROVE_VISION_AI_V2
SSCMA AI;
int height;
// OPT_ANGLE values
enum OptAngle : uint16_t {
  OPT_ANGLE_0 = 0,
  OPT_ANGLE_90 = 0xffff & (1 << 12),
  OPT_ANGLE_180 = 0xffff & (1 << 13),
  OPT_ANGLE_270 = 0xffff & (1 << 14),
};

// OPT_DETAIL values
enum OptResolution : uint8_t {
  OPT_DETAIL_240 = 0,  // 240*240 Auto
  OPT_DETAIL_480,      // 480*480 Auto
  OPT_DETAIL_640,      // 640*480 Auto
};
void groveVisionSetup() {
  PTLF("Setup Grove Vision AI Module");
  // CAMERA_WIRE.begin(10, 9, 400000);
  AI.begin(&CAMERA_WIRE);

  uint8_t count = 0;
  bool sensorEnable = true;
  uint16_t sensorVal = OPT_DETAIL_240 + (strcmp(MODEL, "Bittle R") ? OPT_ANGLE_90 : OPT_ANGLE_0);  // 240*240, rotate 90 degrees if Bittle R

  Serial.println("Set sensor angle and resolution...");
  do {
    delay(10);
    if (CMD_OK == AI.setSensor(sensorEnable, sensorVal)) {
      cameraSetupSuccessful = true;
      break;
    }
    count++;
    PTHL("count:", count);
  } while (count < 3);
}

void read_GroveVision() {
  if (cameraSetupSuccessful && !AI.invoke()) {
    if (AI.boxes().size() >= 1) {
      updateCoordinateLock = true;
      xCoord = AI.boxes()[0].x;  // read x value
      yCoord = AI.boxes()[0].y;  // read y value
      width = AI.boxes()[0].w;   // read width value
      height = AI.boxes()[0].h;  // read height value
      updateCoordinateLock = false;
      detectedObjectQ = true;
      // if (cameraPrintQ)
      // {
      //   for (int i = 0; i < AI.boxes().size(); i++)
      //   {
      //     Serial.print("Box[");
      //     Serial.print(i);
      //     Serial.print("] target=");
      //     Serial.print(AI.boxes()[i].target);
      //     Serial.print(", score=");
      //     Serial.print(AI.boxes()[i].score);
      //     Serial.print(", x=");
      //     Serial.print(AI.boxes()[i].x);
      //     Serial.print(", y=");
      //     Serial.print(AI.boxes()[i].y);
      //     Serial.print(", w=");
      //     Serial.print(AI.boxes()[i].w);
      //     Serial.print(", h=");
      //     Serial.println(AI.boxes()[i].h);
      //   }
      // }
    }
  }
}
#endif
