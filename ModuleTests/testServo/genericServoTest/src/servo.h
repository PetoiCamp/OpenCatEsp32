#define PT(s) Serial.print(s)  // abbreviate print commands
#define PTD(s, format) Serial.print(s, format)
#define PT_FMT(s, format) Serial.print(s, format)  // abbreviate print commands
#define PTL(s) Serial.println(s)
#define PTF(s) Serial.print(F(s))  //trade flash memory for dynamic memory with F() function
#define PTLF(s) Serial.println(F(s))
#define PTT(s, delimeter) \
  { \
    Serial.print(s); \
    Serial.print(delimeter); \
  }
#define PTH(head, value) \
  { \
    Serial.print(head); \
    Serial.print('\t'); \
    Serial.print(value); \
  }
#define PTHL(head, value) \
  { \
    Serial.print(head); \
    Serial.print('\t'); \
    Serial.println(value); \
  }

#ifdef ARDUINO_AVR_UNO
#define PWM_NUM 16
#else
#define BiBoard_V0_1
// #define BiBoard_V1_0
#define PWM_NUM 12
#define ESP_PWM
#endif

#define SERVO_FREQ 240
enum ServoModel_t {
  G41 = 0,
  P1S,
  P2K
};
ServoModel_t servoModelList[] = {
  P1S, P1S, P1S, P1S,
  P1S, P1S, P1S, P1S,
  P1S, P1S, P1S, P1S,
  P1S, P1S, P1S, P1S
};
#define DOF 16
#define WALKING_DOF 8

#if defined BiBoard_V0_1 || defined BiBoard_V0_2
const uint8_t PWM_pin[PWM_NUM] = {
  19, 4, 2, 27,   // head or shoulder roll
  33, 5, 15, 14,  // shoulder pitch
  32, 18, 13, 12  // P1S
};
#elif defined BiBoard_V1_0
const uint8_t PWM_pin[PWM_NUM] = {
  18, 5, 14, 27,  // head or shoulder roll
  23, 4, 12, 33,  // shoulder pitch
  19, 15, 13, 32  // knee
};
#else
const uint8_t PWM_pin[PWM_NUM] = {
  12, 11, 4, 3,  //                                headPan, tilt, tailPan, NA
  13, 10, 5, 2,  // shoulder roll
  14, 9, 6, 1,   // shoulder pitch
  //                                  13,       10,     6,    2,     //shoulder roll
  //                                  14,        9,     5,    1,     //shoulder pitch
  15, 8, 7, 0  // P1S
};
#endif
int currentAng[DOF] = { 0, 0, 0, 0,
                        0, 0, 0, 0,
                        75, 75, 75, 75,
                        -55, -55, -55, -55 };
int8_t middleShift[] = { 0, 0, 0, 0,
                         -45, -45, -45, -45,
                         55, 55, -55, -55,
                         -55, -55, -55, -55 };
int8_t rotationDirection[] = { 1, -1, 1, 1,
                               1, -1, 1, -1,
                               1, -1, -1, 1,
                               -1, 1, 1, -1 };
int zeroPosition[DOF] = {};
int calibratedZeroPosition[DOF] = {};

int servoCalib[DOF] = { 0, 0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0 };

#define DEVICE_ADDRESS 0x54  //Address of eeprom chip
#include <Wire.h>

void i2c_eeprom_read_buffer(unsigned int eeaddress, byte *buffer, int length) {
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write((int)(eeaddress >> 8));    // MSB
  Wire.write((int)(eeaddress & 0xFF));  // LSB
  Wire.endTransmission();
  Wire.requestFrom(DEVICE_ADDRESS, length);
  int c = 0;
  for (c = 0; c < length; c++) {
    if (Wire.available()) buffer[c] = Wire.read();
    PTH((int8_t)buffer[c], ' ');
  }
}

#include "espServo.h"
