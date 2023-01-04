//modify the model and board definitions
//***********************
#define BITTLE    //Petoi 9 DOF robot dog: 1 on head + 8 on leg
//#define NYBBLE  //Petoi 11 DOF robot cat: 2 on head + 1 on tail + 8 on leg
//#define CUB

#define BiBoard     //ESP32 Board with 12 channels of built-in PWM for joints
//#define BiBoard2  //ESP32 Board with 16 channels of PCA9685 PWM for joints
//***********************

//Send 'R' token to reset the birthmark in the EEPROM so that the robot will restart to reset
//#define AUTO_INIT  //activate it to automatically reset joint and imu calibration without prompts

//you can also activate the following modes (they will diable the gyro to save programming space)
//allowed combinations: RANDOM_MIND + ULTRASONIC, RANDOM_MIND, ULTRASONIC, VOICE, CAMERA
//#define ULTRASONIC      //for Nybble's ultrasonic sensor
//#define VOICE           //for LD3320 module
//#define CAMERA          //for Mu Vision camera

#include "src/OpenCat.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);//USB serial
  Serial.setTimeout(5);
  //  Serial1.begin(115200); //second serial port
  while (Serial.available() && Serial.read()) {
    delay(1);
  }; // empty buffer
  PTLF("Flush the serial buffer...");
  PTLF("\n* Start *");
#ifdef BITTLE
  PTLF("Bittle");
#elif defined NYBBLE
  PTLF("Nybble");
#elif defined CUB
  PTLF("Cub");
#endif
  while (Serial.available() && Serial.read()); // empty buffer
  i2cDetect();
  i2cEepromSetup();
#ifdef GYRO_PIN
  imuSetup();
#endif
  bleSetup();
  blueSspSetup();
  servoSetup();
  skillList = new SkillList();
  for (byte i = 0; i < randomMindListLength; i++) {
    randomBase += choiceWeight[i];
  }

#ifdef NEOPIXEL_PIN
  ledSetup();
#endif
#ifdef PWM_LED_PIN
  pinMode(PWM_LED_PIN, OUTPUT);
#endif
#ifdef VOLTAGE
  while (lowBattery());
#endif

#ifdef IR_PIN
  irrecv.enableIRIn();
#endif

  QA();
  i2c_eeprom_write_byte( EEPROM_BIRTHMARK_ADDRESS, BIRTHMARK);//finish the test and mark the board as initialized

#ifdef VOICE
  voiceSetup();
#endif

#ifdef CAMERA
  cameraSetup();
#endif

  lastCmd[0] = '\0';
  newCmd[0] = '\0';

  //  if (exceptions) {// Make the robot enter joint calibration state (different from initialization) if it is upside down.
  //    strcpy(newCmd, "calib");
  //    exceptions = 0;
  //  }
  //  else {// Otherwise start up normally
  //    strcpy(newCmd, "rest");
  //    token = 'd';
  //    newCmdIdx = 6;
  //  }
  //  loadBySkillName(newCmd);
  //
  allCalibratedPWM(currentAng);  //soft boot for servos
  delay(500);
  playMelody(melodyNormalBoot, sizeof(melodyNormalBoot) / 2);
#ifdef GYRO_PIN
  read_IMU();  //ypr is slow when starting up. leave enough time between IMU initialization and this reading
  token = (exceptions) ? T_CALIBRATE : T_REST; //put the robot's side on the table to enter calibration posture for attaching legs
  newCmdIdx = 2;
#endif

  PTLF("k");
  PTL("Ready!");
  idleTimer = millis();
  beep(24, 50);

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
  readSignal();
  //  readHuman();

  //  //— special behaviors based on sensor events
  dealWithExceptions(); //low battery, fall over, lifted, etc.

  //  //— generate behavior by fusing all sensors and instruction
  //  decision();

  //  //— action
  //  //playSound();
#ifdef NEOPIXEL_PIN
  playLight();
#endif
  reaction();
}
