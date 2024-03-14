//code provided by the manufacturer
//modified by Rongzhong Li for better demonstration.
//Feb.16, 2021

#include "RgbUltrasonic.h"

RgbUltrasonic ultrasonic(16, 17);  //(signal, RGB) use the Grove Tx Rx
//RgbUltrasonic ultrasonic(27,23);  //(signal, RGB) use the infrared reciever's pin 23 and pwm pin 27
//The RGB LED ultrasonic module should be plugged in the fourth grove socket with D6, D7

int interval = 3;
#define ULTRASONIC_IMU_SKIP 10
long colors[] = { RGB_RED, RGB_PURPLE, RGB_GREEN, RGB_BLUE, RGB_YELLOW, RGB_WHITE };
long ultraTimer;
int ultraInterval = 1000;
int distance;
void rgbUltrasonicSetup() {
  ultrasonic.SetupLED();
  ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_RED, E_EFFECT_FLASH);
  ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_BLUE, E_EFFECT_BREATHING);
  ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_YELLOW, E_EFFECT_ROTATE);
}

void readRGBultrasonic() {
  if (millis() - ultraTimer > ultraInterval) {  //|| token == 'k' && millis() - ultraTimer > 3000) {
    ultraTimer = millis();
    ultraInterval = 0;
    randomInterval = 1000;
    distance = ultrasonic.GetUltrasonicDistance();
    if (distance == 640) {
      return;
    }

    if (distance > 60) {
      if (!manualEyeColorQ)
        ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_WHITE, E_EFFECT_BREATHING);
      ultraInterval = 1000;
      //      autoSwitch = true;
      randomInterval = 1000;
    } else if (distance > 40) {
      if (!manualEyeColorQ)
        ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_YELLOW, E_EFFECT_ROTATE);
    } else if (distance < 2) {
      token = T_SKILL;
      tQueue->addTask('k', "bk", 1500);
      randomInterval = 5000;
    } else if (distance < 4) {
      if (!manualEyeColorQ)
        ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_RED, E_EFFECT_FLASH);
      meow(rand() % 3 + 1, distance * 2);
      token = T_INDEXED_SIMULTANEOUS_BIN;
      int amplitude = 5;
      int allRand[] = { 0, currentAng[0] + rand() % 20 - 10, 1, currentAng[1] + rand() % (2 * amplitude) - amplitude, 2, currentAng[2] + rand() % (amplitude * 4) - amplitude * 2 };
      cmdLen = 6;
      for (byte i = 0; i < cmdLen; i++)
        newCmd[i] = allRand[i];
      newCmd[cmdLen] = '~';
      newCmdIdx = 6;
    } else if (distance < 6) {
      if (!manualEyeColorQ)
        ultrasonic.SetRgbColor(E_RGB_ALL, RGB_RED);
      tQueue->addTask('k', "sit", 2000);
    }

    else {  //6~40
      distance -= 6;
      if (!manualEyeColorQ)
        ultrasonic.SetRgbColor(E_RGB_ALL, colors[max(min(distance / 7, 5), 0)]);
      token = T_LISTED_BIN;
      int mid[] = {
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        30,
        30,
        -30,
        -30,
        30,
        30,
        -30,
        -30,
      };
      int allParameter[] = { currentAng[0] * 2 / 3 - distance / 2, int(-10 + currentAng[1] * 2 / 3 + distance / 1.5), (distance * 3 - 50) * (rand() % 50 < 1 ? rand() % 2 - 1 : 1), 0,
                             0, 0, 0, 0,
                             mid[8] - 15 + distance / 2, mid[9] - 15 + distance / 2, mid[10] - 30 + distance, mid[11] - 30 + distance,
                             mid[12] + 35 - distance, mid[13] + 35 - distance, mid[14] + 40 - distance, mid[15] + 40 - distance };
      //      printList(allParameter);
      cmdLen = 16;
      for (byte i = 0; i < cmdLen; i++)
        newCmd[i] = (int8_t)min(max(allParameter[i], -128), 127);
      newCmd[cmdLen] = '~';
      newCmdIdx = 6;
      randomInterval = 5000;
    }
  }
}

float readUltrasonic(int trigger, int echo = -1) {  //give two parameters for the traditional ultrasonic sensor
                                                    //give one parameter for the one pin ultrasonic sensor that shares the trigger and echo pins
  if (echo == -1)
    echo = trigger;
  int longestDistance = 200;  // 200 cm = 2 meters
  float farTime = longestDistance * 2 / 0.034;
  pinMode(trigger, OUTPUT);

  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  pinMode(echo, INPUT);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echo, HIGH, farTime);
  // Calculating the distance
  return duration * 0.034 / 2;  // 10^-6 * 34000 cm/s
}
