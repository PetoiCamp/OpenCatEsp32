/* BiBoard
    PWM:
                        |--------------------------------
                        |    PWM[0]           PWM[6]    |
                        |    PWM[1]           PWM[7]    |
                        |    PWM[2]           PWM[8]    |
                        |-----------                    |
                        |           |                   |
                        |   ESP32   |  IMU         USB-C|~~~~Tail~~~~
                        |           |                   |
                        |-----------                    |
                        |    PWM[3]           PWM[9]    |
                        |    PWM[4]           PWM[10]   |
                        |    PWM[5]           PWM[11]   |
                        |-------------------------------|

    Pin Name    |   ESP32 Pin   |   Arduino Pin Name    |   Alternative Function
    PWM[0]          GPIO4               4                   GPIO / Ain / Touch
    PWM[1]          GPIO5               5                   GPIO / VSPI SS
    PWM[2]          GPIO18              18                  GPIO / VSPI SCK
    -----------------------------------------------------------------------------
    PWM[3]          GPIO32              32                  GPIO / Ain / Touch
    PWM[4]          GPIO33              33                  GPIO / Ain / Touch
    PWM[5]          GPIO19              19                  GPIO / VSPI MISO
    -----------------------------------------------------------------------------
    PWM[6]          GPIO2               2                   boot pin, DO NOT PUT HIGH WHEN BOOT!
    PWM[7]          GPIO15              15                  GPIO / HSPI SS / Ain Touch
    PWM[8]          GPIO13              13                  built-in LED / GPIO / HSPI MOSI / Ain / Touch
    -----------------------------------------------------------------------------
    PWM[9]          GPIO12              12                  GPIO / HSPI MISO / Ain / Touch
    PWM[10]         GPIO14              14                  GPIO / HSPI SCK / Ain / Touch
    PWM[11]         GPIO27              27                  GPIO / Ain / Touch

    I2C:

    Pin Name    |   ESP32 Pin   |   Arduino Pin Name    |   Alternative Function
    I2C-SCL         GPIO22              22                  Fixed - ICM20600 - Pulled
    I2C-SDA         GPIO21              21                  Fixed = ICM20600 - Pulled

    System default, nothing to declaration!

    Other Peripherals:

    Pin Name    |   ESP32 Pin   |   Arduino Pin Name    |   Alternative Function
    IR_Remote       GPIO23              23                  Fixed - VS1838B IR
    DAC_Out         GPIO25              25                  Fixed - PAM8302
    IMU_Int         GPIO26              26                  Fixed - MPU6050 Interrupt

    System default, nothing to declare!
*/

/* BiBoard2
  IMU_Int     27
  BUZZER      14
  VOLTAGE     4
  RGB LED     15
  GREEN-LED   5
*/

/*  Total DOF            Walking DOF
                   Nybble    Bittle    Cub
   BiBoard  (12)  skip 0~4  skip 0~4    12
   BiBoard2 (16)  skip 0~8  skip 0~8  skip0~4
*/

// #define RevB
#define RevDE
#define SERIAL_TIMEOUT 10  // 5 may cut off the message
#define SERIAL_TIMEOUT_LONG 150
#ifdef BiBoard_V0_1
#define BOARD "B01"
#elif defined BiBoard_V0_2
#define BOARD "B02"
#elif defined BiBoard_V1_0
#define BOARD "B10"
#else
#define BOARD "B"
#endif
#define DATE "241104"  // YYMMDD
String SoftwareVersion = "";

#define BIRTHMARK '@'  // Send '!' token to reset the birthmark in the EEPROM so that the robot will know to restart and reset

#define BT_BLE  // toggle Bluetooth Low Energy (BLE）
#define BT_SSP  // toggle Bluetooth Secure Simple Pairing (BT_SSP)
// #define BT_CLIENT    // toggle Bluetooth client (BLE） for Micro:Bit
#define GYRO_PIN      // toggle the Inertia Measurement Unit (IMU), i.e. the gyroscope
#define IMU_MPU6050   // toggle the MPU6050 gyroscope
#define IMU_ICM42670  // toggle the ICM42670 gyroscope
#define SERVO_FREQ 240

// Tutorial: https://bittle.petoi.com/11-tutorial-on-creating-new-skills
#ifdef NYBBLE
#define MODEL "Nybble"
#define HEAD
#define TAIL
#define X_LEG
#define REGULAR P1S  // G41
#define KNEE P1S     // G41
#include "InstinctNybbleESP.h"

#elif defined BITTLE
#ifdef ROBOT_ARM
#define MODEL "Bittle R"
#else
#define MODEL "Bittle X"
#endif

#define HEAD
#define TAIL  // the robot arm's clip is assigned to the tail joint
#define LL_LEG

#ifndef MINI
#define REGULAR P1S
#define KNEE P1S
#else
#define REGULAR P50
#define KNEE P50
#endif
#ifdef ROBOT_ARM
#include "InstinctBittleESP_arm.h"
#else
#include "InstinctBittleESP.h"
#endif

#elif defined CUB
#define MODEL "DoF16"
#ifdef BiBoard2
#define HEAD
#define TAIL
#endif
#define LL_LEG
#define REGULAR P1S
#define KNEE P2K
#include "InstinctCubESP.h"
// #define MPU_YAW180
#endif

#if defined BiBoard_V0_1 || defined BiBoard_V0_2
#define ESP_PWM
#define PWM_NUM 12
#define INTERRUPT_PIN 26  // use pin 2 on Arduino Uno & most boards
#define BUZZER 25
#define IR_PIN 23
#define ANALOG1 34
#define ANALOG2 35
#define ANALOG3 36
#define ANALOG4 39
#define UART_RX2 16
#define UART_TX2 17
#define SERIAL_VOICE Serial2
#define IMU_MPU6050
#define I2C_EEPROM_ADDRESS 0x54  // Address of i2c eeprom chip

// L:Left-R:Right-F:Front-B:Back---LF, RF, RB, LB
const uint8_t PWM_pin[PWM_NUM] = {
  19, 4, 2, 27,   // head or shoulder roll
  33, 5, 15, 14,  // shoulder pitch
  32, 18, 13, 12  // knee
};

#elif defined BiBoard_V1_0
#define ESP_PWM
#define PWM_NUM 12
// #define INTERRUPT_PIN 26  // use pin 2 on Arduino Uno & most boards
#define BUZZER 2
// #define IR_PIN 23

#define LOW_VOLTAGE 7.0  // for 2S 7.4V power
#define NO_BATTERY_VOLTAGE 6.8
#define LOW_VOLTAGE2 5.0  // for 6V power
#define NO_BATTERY_VOLTAGE2 4.8
#ifdef RevB
#define VOLTAGE 35  // rev B
#define ANALOG2 32  // rev B
#elif defined RevDE
#define VOLTAGE 37  // rev D
#define ANALOG2 35  // rev D
#endif
#define ANALOG1 34
#define ANALOG3 36
#define ANALOG4 39
#define BACKTOUCH_PIN 38
#define VOICE_RX 26
#define VOICE_TX 25
#define UART_RX2 9
#define UART_TX2 10
#define SERIAL_VOICE Serial1
#define IMU_MPU6050
// #define IMU_ICM42670
// #define I2C_EEPROM_ADDRESS 0x54  //Address of i2c eeprom chip

// #ifdef ROBOT_ARM
// const uint8_t PWM_pin[PWM_NUM] = {
//   // swap the front left knee servo spot for better accessibility of the clip servo's cable
//   18, 5, 4, 23,    // head or shoulder roll
//   19, 15, 12, 33,  // shoulder pitch
//   32, 13, 14, 27   // knee
// };
// #else
#define PWM_LED_PIN 27
// L:Left-R:Right-F:Front-B:Back---LF, RF, RB, LB
const uint8_t PWM_pin[PWM_NUM] = {
  18, 5, 14, 27,  // head or shoulder roll
  23, 4, 12, 33,  // shoulder pitch
  19, 15, 13, 32  // knee
};
// #endif

#elif defined BiBoard2
#define PWM_NUM 16
#define INTERRUPT_PIN 27  // use pin 2 on Arduino Uno & most boards
#define BUZZER 14
#define VOLTAGE 4
#define LOW_VOLTAGE 7.0
#define NEOPIXEL_PIN 15
#define PWM_LED_PIN 5
#define IR_PIN 23
#define TOUCH0 12
#define TOUCH1 13
#define TOUCH2 32
#define TOUCH3 33
#define IMU_MPU6050
#define I2C_EEPROM_ADDRESS 0x54  // Address of i2c eeprom chip
// L:Left R:Right F:Front B:Back   LF,        RF,    RB,   LB

const uint8_t PWM_pin[PWM_NUM] = {
  12, 11, 4, 3,  //                                headPan, tilt, tailPan, NA
  13, 10, 5, 2,  // shoulder roll
  14, 9, 6, 1,   // shoulder pitch
  //                                  13,       10,     6,    2,     //shoulder roll
  //                                  14,        9,     5,    1,     //shoulder pitch
  15, 8, 7, 0  // knee
};

#endif

#define MAX_READING 4096.0  // to compensate the different voltage level of boards
#define BASE_RANGE 1024.0
double rate = 1.0 * MAX_READING / BASE_RANGE;

#define DOF 16
#if defined NYBBLE || defined BITTLE
#define WALKING_DOF 8
#define GAIT_ARRAY_DOF 8
#else  // CUB
#define WALKING_DOF 12
#define GAIT_ARRAY_DOF 8
#endif

enum ServoModel_t {
  G41 = 0,
  P1S,
  P2K,
  P50
};

ServoModel_t servoModelList[] = {
  REGULAR, REGULAR, REGULAR, REGULAR,
  REGULAR, REGULAR, REGULAR, REGULAR,
  REGULAR, REGULAR, REGULAR, REGULAR,
  KNEE, KNEE, KNEE, KNEE
};

bool newBoard = false;

#include <math.h>
// token list
#define T_ABORT 'a'              // abort the calibration values
#define T_BEEP 'b'               //b note1 duration1 note2 duration2 ... e.g. b12 8 14 8 16 8 17 8 19 4 \
                         //bVolume will change the volume of the sound, in scale of 0~10. 0 will mute all sound effect. e.g. b3. \
                         //a single 'b' will toggle all sound on/off
#define T_BEEP_BIN 'B'           //B note1 duration1 note2 duration2 ... e.g. B12 8 14 8 16 8 17 8 19 4 \
                         //a single 'B' will toggle all sound on/off
#define T_CALIBRATE 'c'          //send the robot to calibration posture for attaching legs and fine-tuning the joint offsets. \
                         //c jointIndex1 offset1 jointIndex2 offset2 ... e.g. c0 7 1 -4 2 3 8 5
#define T_COLOR 'C'              //change the eye colors of the RGB ultrasonic sensor \
                         //a single 'C' will cancel the manual eye colors
#define T_REST 'd'               //set the robot to rest posture and shut down all the servos \
                         //"d index" can turn off a single servo
#define T_SERVO_FEEDBACK 'f'     //return the servo's position info if the chip supports feedback. \
                                        //e.g. f8 returns the 8th joint's position. A single 'f' returns all the joints' position
#define T_SERVO_FOLLOW 'F'       // make the other legs follow the moved legs
#define T_GYRO 'g'               // gyro-related commands
#define T_GYRO_FINENESS 'F'      // increase the frequency of gyroscope sampling
#define T_GYRO_FINENESS_OFF 'f'  // reduce the frequency of gyroscope sampling to accelerate motion
#define T_GYRO_BALANCE 'B'       // turn on the gyro balancing
#define T_GYRO_BALANCE_OFF 'b'   // turn off the gyro balancing
#define T_GYRO_PRINT 'P'         // always print gyro data
#define T_GYRO_PRINT_OFF 'p'     // print gyro data once then stop

#define T_HELP_INFO 'h'                 // print some help information
#define T_INDEXED_SIMULTANEOUS_ASC 'i'  //i jointIndex1 jointAngle1 jointIndex2 jointAngle2 ... e.g. i0 70 8 -20 9 -20 \
                                        //a single 'i' will free the head joints if it were previously manually controlled.
#define T_INDEXED_SIMULTANEOUS_BIN 'I'  // I jointIndex1 jointAngle1 jointIndex2 jointAngle2 ... e.g. I0 70 8 -20 9 -20
#define T_JOINTS 'j'                    // A single "j" returns all angles. "j Index" prints the joint's angle. e.g. "j 8" or "j11".
#define T_SKILL 'k'
#define T_SKILL_DATA 'K'
#define T_BALANCE_SLOPE 'l'           // change the slope of the balancing adjustment in roll and pitch directions. \
                                      // default "l 1 1". the numbers allows [-2,-1,0,1,2]
#define T_LISTED_BIN 'L'              // a list of the DOFx joint angles: angle0 angle1 angle2 ... angle15
#define T_INDEXED_SEQUENTIAL_ASC 'm'  // m jointIndex1 jointAngle1 jointIndex2 jointAngle2 ... e.g. m0 70 0 -70 8 -20 9 -20
#define T_INDEXED_SEQUENTIAL_BIN 'M'  // M jointIndex1 jointAngle1 jointIndex2 jointAngle2 ... e.g. M0 70 0 -70 8 -20 9 -20
#define T_NAME 'n'                    // customize the Bluetooth device's broadcast name. e.g. nMyDog will name the device as "MyDog" \
                                      // it takes effect the next time the board boosup. it won't interrupt the current connecton.
#define T_MELODY 'o'
#define T_CPG 'r'    // Oscillator for Central Pattern Generator
#define T_PAUSE 'p'  // pause
#define T_POWER 'P'  // power, print the voltage
#define T_TASK_QUEUE 'q'
#define T_ROBOT_ARM 'R'
#define T_SAVE 's'
#define T_TILT 't'
#define T_TEMP 'T'  // call the last skill data received from the serial port
#define T_MEOW 'u'
#define T_SERVO_MICROSECOND 'w'  // PWM width modulation
#define T_XLEG 'x'
#define T_RANDOM_MIND 'z'  // toggle random behaviors

#define T_READ 'R'        // read pin     R
#define T_WRITE 'W'       // write pin                      W
#define TYPE_ANALOG 'a'   //            Ra(analog read)   Wa(analog write)
#define TYPE_DIGITAL 'd'  //            Rd(digital read)  Wd(digital write)

#define T_RESET '!'
#define T_QUERY '?'
#define T_ACCELERATE '.'
#define T_DECELERATE ','

#define EXTENSION 'X'
#define EXTENSION_GROVE_SERIAL 'S'        // connect to Grove UART2
#define EXTENSION_VOICE 'A'               // connect to Grove UART2 (on V0_*: a slide switch can choose the voice or the Grove), or UART1 (on V1). Hidden on board.
#define EXTENSION_DOUBLE_TOUCH 'T'        // connect to ANALOG1, ANALOG2
#define EXTENSION_DOUBLE_LIGHT 'L'        // connect to ANALOG1, ANALOG2
#define EXTENSION_DOUBLE_IR_DISTANCE 'D'  // connect to ANALOG3, ANALOG4
#define EXTENSION_PIR 'I'                 // connect to ANALOG3
#define EXTENSION_BACKTOUCH 'B'           // connect to BACKTOUCH_PIN
#define EXTENSION_ULTRASONIC 'U'          // connect to Grove UART2
#define EXTENSION_GESTURE 'G'             // connect to Grove I2C
#define EXTENSION_CAMERA 'C'              // connect to Grove I2C
#define EXTENSION_QUICK_DEMO 'Q'          // activate the quick demo at the end of OpenCatEsp32.ino

// bool updated[10];
float degPerRad = 180 / M_PI;
float radPerDeg = M_PI / 180;

// control related variables
#define IDLE_TIME 3000
long idleTimer = 0;
#define CHECK_BATTERY_PERIOD 1000  // every 1 seconds. 60 mins -> 3600 seconds
#define BATTERY_WARNING_FREQ 10    // every 10 seconds
byte batteryWarningCounter = 0;
float lastVoltage;
int uptime = -1;
int frame = 0;
int tStep = 1;
long loopTimer;
byte fps = 0;

char token;
char lastToken;
char lowerToken;
#define CMD_LEN 10
char *lastCmd = new char[CMD_LEN + 1];  // the last char must be '\0' for safe so CMD_LEN+1 elements are required
int cmdLen = 0;
byte newCmdIdx = 0;
int8_t periodGlobal = 0;
#define BUFF_LEN 2507  // 1524 =125*20+7=2507
char *newCmd = new char[BUFF_LEN + 1];
int spaceAfterStoringData = BUFF_LEN;
int serialTimeout;
char terminator;
// int serialTimeout;
long lastSerialTime = 0;

bool interruptedDuringBehavior = false;
bool lowBatteryQ = false;
bool gyroUpdateQ = true;
bool fineAdjustQ = true;
bool gyroBalanceQ = true;
bool printGyroQ = false;
bool autoSwitch = false;
bool walkingQ = false;
bool manualHeadQ = false;
bool nonHeadJointQ = false;
bool workingStiffness = true;
bool manualEyeColorQ = false;
// bool keepDirectionQ = true;
#define HEAD_GROUP_LEN 4  // used for controlling head pan, tilt, tail, and other joints independent from walking
int targetHead[HEAD_GROUP_LEN];

bool imuUpdated;
int imuException = 0;
byte transformSpeed = 2;
float protectiveShift;  // reduce the wearing of the potentiometer

int8_t moduleList[] = {
  EXTENSION_GROVE_SERIAL,
  EXTENSION_VOICE,
  EXTENSION_DOUBLE_TOUCH,
  EXTENSION_DOUBLE_LIGHT,
  EXTENSION_DOUBLE_IR_DISTANCE,
  EXTENSION_PIR,
  EXTENSION_BACKTOUCH,
  EXTENSION_ULTRASONIC,
  EXTENSION_GESTURE,
  EXTENSION_CAMERA,
  EXTENSION_QUICK_DEMO,
};

String moduleNames[] = { "Grove_Serial", "Voice", "Double_Touch", "Double_Light ", "Double_Ir_Distance ", "Pir", "BackTouch", "Ultrasonic", "Gesture", "Camera", "Quick_Demo" };
bool moduleActivatedQ[] = { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
bool moduleDemoQ = false;
byte moduleIndex;
bool initialBoot = true;
bool coinFace = true;
bool safeRest = true;
bool soundState;
byte buzzerVolume;
float amplifierFactor = 100.0;  // to fit the actual amplifier range of BiBoard

int delayLong = 20;
int delayMid = 5;
int delayException = 5;
int delayShort = 3;
int delayStep = 1;
int delayPrevious;
int runDelay = delayMid;

#ifdef NYBBLE
int8_t middleShift[] = { 0, 15, 0, 0,
                         -45, -45, -45, -45,
                         10, 10, -10, -10,
                         -30, -30, 30, 30 };
#elif defined BITTLE
#ifndef MINI
int8_t middleShift[] = { 0, -90, 0, 0,
                         -45, -45, -45, -45,
                         55, 55, -55, -55,
                         -55, -55, -55, -55 };
#else
int8_t middleShift[] = { 0, 0, 0, 0,
                         0, 0, 0, 0,
                         0, 0, 0, 0,
                         -15, -15, -15, -15 };
#endif

#else  // CUB
int8_t middleShift[] = { 0, 15, 0, 0,
                         -45, -45, -45, -45,
                         55, 55, -55, -55,
                         -45, -45, -45, -45 };
#endif

// #define INVERSE_SERVO_DIRECTION
#ifdef CUB
int8_t rotationDirection[] = { 1, -1, 1, 1,
                               1, -1, 1, -1,
                               1, -1, -1, 1,
                               1, -1, -1, 1 };
int angleLimit[][2] = {
  { -120, 120 },
  { -30, 80 },
  { -120, 120 },
  { -120, 120 },
  { -90, 60 },
  { -90, 60 },
  { -90, 90 },
  { -90, 90 },
  { -180, 120 },
  { -180, 120 },
  { -80, 200 },
  { -80, 200 },
  { -66, 100 },
  { -66, 100 },
  { -66, 100 },
  { -66, 100 },
};
#else
int8_t rotationDirection[] = { 1, -1, -1, 1,
                               1, -1, 1, -1,
                               1, -1, -1, 1,
                               -1, 1, 1, -1 };
#ifdef BITTLE
int angleLimit[][2] = {
  { -120, 120 },
#ifdef ROBOT_ARM
  { -10, 180 },
#else
  { -85, 85 },
#endif
  { -120, 120 },
  { -120, 120 },

  { -90, 60 },
  { -90, 60 },
  { -90, 90 },
  { -90, 90 },

  { -200, 80 },
  { -200, 80 },
  { -80, 200 },
  { -80, 200 },
  { -80, 200 },
  { -80, 200 },
  { -80, 200 },
  { -80, 200 },
};
#else  // Nybble
int angleLimit[][2] = {
  { -120, 120 },
  { -75, 35 },
  { -120, 120 },
  { -120, 120 },
  { -90, 60 },
  { -90, 60 },
  { -90, 90 },
  { -90, 90 },
  { -200, 80 },
  { -200, 80 },
  { -80, 200 },
  { -80, 200 },
  { -80, 80 },
  { -80, 80 },
  { -80, 80 },
  { -80, 80 },
};
#endif
#endif

#ifdef X_LEG
int currentAng[DOF] = { 0, 0, 0, 0,
                        0, 0, 0, 0,
                        75, 75, -75, -75,
                        -55, -55, 55, 55 };
int previousAng[DOF] = { 0, 0, 0, 0,
                         0, 0, 0, 0,
                         75, 75, -75, -75,
                         -55, -55, 55, 55 };
#else
int currentAng[DOF] = { 0, 0, 0, 0,
                        0, 0, 0, 0,
                        75, 75, 75, 75,
                        -55, -55, -55, -55 };
int previousAng[DOF] = { 0, 0, 0, 0,
                         0, 0, 0, 0,
                         75, 75, 75, 75,
                         -55, -55, -55, -55 };
#endif
int zeroPosition[DOF] = {};
int calibratedZeroPosition[DOF] = {};

int8_t servoCalib[DOF] = { 0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 0 };

int16_t mpuOffset[9] = { 0, 0, 0,
                         0, 0, 0,
                         0, 0, 0 };

float expectedRollPitch[2];
float RollPitchDeviation[2];
float currentAdjust[DOF] = {};
int balanceSlope[2] = { 1, 1 };  // roll, pitch

#include "tools.h"
#include "QList/QList.h"
#include "taskQueue.h"

#include "sound.h"
#include <Wire.h>
#include "configConstants.h"
#ifdef BT_BLE
#include "bleUart.h"
#endif
#ifdef BT_CLIENT
#include "bleClient.h"
#endif
#include "io.h"
#ifdef GYRO_PIN
#include "imu.h"
#endif
#ifdef IR_PIN
#include "infrared.h"
#endif
#include "espServo.h"
#include "motion.h"
#include "randomMind.h"

#include "skill.h"
#include "moduleManager.h"
#ifdef NEOPIXEL_PIN
#include "led.h"
#endif
#include "reaction.h"
#include "qualityAssurance.h"

void initRobot() {
  beep(20);
#ifdef VOLTAGE
  lowBattery();
#endif
  // #ifdef BiBoard_V1_0
  //   Wire.begin(22, 21);
  // #else
  Wire.begin();
  // #endif
  SoftwareVersion = SoftwareVersion + BOARD + "_" + DATE;
  PTL('k');
  PTLF("Flush the serial buffer...");
  PTL("\n* Start *");
  printToAllPorts(MODEL);
  PTF("Software version: ");
  printToAllPorts(SoftwareVersion);
  i2cDetect();
#ifdef I2C_EEPROM_ADDRESS
  soundState = i2c_eeprom_read_byte(EEPROM_BOOTUP_SOUND_STATE);
  buzzerVolume = max(byte(0), min(byte(10), i2c_eeprom_read_byte(EEPROM_BUZZER_VOLUME)));
#else
  config.begin("config", false);  // false: read/write mode. true: read-only mode.
  soundState = config.getBool("bootSndState");
  buzzerVolume = config.getChar("buzzerVolume");
#endif
  configSetup();
  PTF("Buzzer volume: ");
  PT(buzzerVolume);
  PTL("/10");

#ifdef GYRO_PIN
  imuSetup();
#endif
#ifdef BT_BLE
  bleSetup();
#endif
#ifdef BT_CLIENT
  bleClientSetup();
#endif
#ifdef BT_SSP
  blueSspSetup();
#endif
  servoSetup();
  lastCmd[0] = '\0';
  newCmd[0] = '\0';
  skill = new Skill();
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
  // #ifdef VOLTAGE
  //   do {
  //     PTL("Check battery. You can skip by entering any characters in the Serial Monitor.");
  //     if (Serial.available()) {
  //       Serial.read();  // allow breaking the loop with any serial input
  //       break;
  //     }
  //   } while (lowBattery());  //if the battery is low
  // #endif

#ifdef IR_PIN
  irrecv.enableIRIn();
#endif

  QA();

  tQueue = new TaskQueue();
  loadBySkillName("rest");  // must have to avoid memory crash. need to check why.
                            // allCalibratedPWM(currentAng); alone will lead to crash
  delay(500);

  initModuleManager();
#ifdef GYRO_PIN
  read_mpu6050();  // ypr is slow when starting up. leave enough time between IMU initialization and this reading
  if (!moduleDemoQ)
    tQueue->addTask((imuException) ? T_CALIBRATE : T_REST, "");
#endif
  PTL("Ready!");
  beep(24, 50);
  idleTimer = millis();
}
