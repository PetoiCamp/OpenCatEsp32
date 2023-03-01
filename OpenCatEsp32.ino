//modify the model and board definitions
//***********************
#define BITTLE  //Petoi 9 DOF robot dog: 1 on head + 8 on leg
//#define NYBBLE  //Petoi 11 DOF robot cat: 2 on head + 1 on tail + 8 on leg
//#define CUB

#define BiBoard  //ESP32 Board with 12 channels of built-in PWM for joints
//#define BiBoard2  //ESP32 Board with 16 channels of PCA9685 PWM for joints
//***********************

//Send 'R' token to reset the birthmark in the EEPROM so that the robot will restart to reset
//#define AUTO_INIT  //activate it to automatically reset joint and imu calibration without prompts

//you can also activate the following modes (they will diable the gyro to save programming space)
//allowed combinations: RANDOM_MIND + ULTRASONIC, RANDOM_MIND, ULTRASONIC, VOICE, CAMERA
// #define ULTRASONIC    //for Nybble's ultrasonic sensor
#define VOICE         //Petoi Grove voice module
// #define PIR           //for PIR (Passive Infrared) sensor
// #define DOUBLE_TOUCH  //for double touch sensor
// #define DOUBLE_LIGHT  //for double light sensor
// #define GESTURE       //for Gesture module
// #define CAMERA        //for Mu Vision camera
//You need to install https://github.com/mu-opensource/MuVisionSensor3 as a zip library in Arduino IDE.
//Set the four dial switches on the camera as **v ^ v v** (the second switch dialed up to I2C) and connect the camera module to the I2C grove on NyBoard.
//The battery should be turned on to drive the servos.
//
//You can use these 3D printed structures to attach the camera module.
//https://github.com/PetoiCamp/NonCodeFiles/blob/master/stl/MuIntelligentCamera_mount.stl
//https://github.com/PetoiCamp/NonCodeFiles/blob/master/stl/bone.stl
//After uploading the code, you may need to press the reset buttons on the module and then the NyBoard.
//The tracking demo works the best with a yellow tennis ball or some other round objects. Demo: https://www.youtube.com/watch?v=CxGI-MzCGWM

#define MUTED
#include "src/OpenCat.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  //USB serial
  Serial.setTimeout(SERIAL_TIMEOUT);
  //  Serial1.begin(115200); //second serial port
  while (Serial.available() && Serial.read()) {
    delay(1);
  };  // empty buffer
  initRobot();
}

void loop() {
#ifdef VOLTAGE
  lowBattery();
#endif
  //  //—self-initiative
  //  if (autoSwitch) { //the switch can be toggled on/off by the 'z' token
  //    randomMind();//allow the robot to auto rest and do random stuff in randomMind.h
  //    powerSaver(POWER_SAVER);//make the robot rest after a certain period, the unit is seconds
  //
  //  }
  //  //— read environment sensors (low level)
  readEnvironment();
  //  //— special behaviors based on sensor events
  dealWithExceptions();  //low battery, fall over, lifted, etc.

  if (tQueue->size() > 0) {
    tQueue->popTask();
  } else {
    taskInterval = -1;
    readSignal();
    //  readHuman();
  }

  //  //— generate behavior by fusing all sensors and instruction
  //  decision();

  //  //— action
  //  //playSound();
#ifdef NEOPIXEL_PIN
  playLight();
#endif
  reaction();
}
