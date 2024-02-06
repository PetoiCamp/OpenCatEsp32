#ifndef _RGBULTRASONIC_H_
#define _RGBULTRASONIC_H_

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
// #include "RGBLed.h"
#include <Adafruit_NeoPixel.h>
#define UL_LIMIT_MIN 5
#define UL_LIMIT_MID 10
#define UL_LIMIT_MAX 400

#define RGB_RED 0x220000
#define RGB_GREEN 0x002200
#define RGB_BLUE 0x000022
#define RGB_YELLOW 0x222200
#define RGB_PURPLE 0x220022
#define RGB_WHITE 0x222222

//#define RGB_RED     0xFF0000
//#define RGB_GREEN   0x00FF00
//#define RGB_BLUE    0x0000FF
//#define RGB_YELLOW  0xFFFF00
//#define RGB_PURPLE  0xFF00FF
//#define RGB_WHITE   0xFFFFFF

typedef enum {
  E_RGB_ALL = 0,
  E_RGB_RIGHT = 1,
  E_RGB_LEFT = 2
} E_RGB_INDEX;

typedef enum {
  E_EFFECT_BREATHING = 0,
  E_EFFECT_ROTATE = 1,
  E_EFFECT_FLASH = 2
} E_RGB_EFFECT;

class RgbUltrasonic {
private:
  byte SignalPin, RgbPin;
  Adafruit_NeoPixel *mRgb;
public:
  float FrontDistance;
  int measurementInterval;
  long lastMeasurementTime;
  RgbUltrasonic(byte signal_pin, byte rgb_pin) {
    SignalPin = signal_pin;
    RgbPin = rgb_pin;
    lastMeasurementTime = millis();
    measurementInterval = 20;
    // mRgb = new RGBLed(RgbPin, 6);

    Serial.println("set up ultrasonic sensor");
    delay(2000);
  }
  float GetUltrasonicDistance(void) {  //in cm
    if (millis() - lastMeasurementTime >= measurementInterval) {
      unsigned long Time_Echo_us = 0;
      pinMode(SignalPin, OUTPUT);
      digitalWrite(SignalPin, LOW);
      delayMicroseconds(2);
      digitalWrite(SignalPin, HIGH);
      delayMicroseconds(20);
      digitalWrite(SignalPin, LOW);
      pinMode(SignalPin, INPUT);
      Time_Echo_us = pulseIn(SignalPin, HIGH);
      if ((Time_Echo_us < measurementInterval * 1000) && (Time_Echo_us > 1)) {  //max 172cm
        FrontDistance = Time_Echo_us / 58.00;
      }
      lastMeasurementTime = millis();
    }
    return FrontDistance;
  }
  void SetupLED() {
    mRgb = new Adafruit_NeoPixel(6, RgbPin, NEO_GRB + NEO_KHZ800);
    mRgb->begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
    mRgb->show();             // Turn OFF all pixels ASAP
    mRgb->setBrightness(50);  // Set BRIGHTNESS to about 1/5 (max = 255)
  }
  void SetRgbColor(E_RGB_INDEX index, long Color) {
    if (index == E_RGB_ALL)
      for (byte i = 0; i < 6; i++)
        mRgb->setPixelColor(i, Color);
    else if (index == E_RGB_RIGHT) {
      mRgb->setPixelColor(0, Color);
      mRgb->setPixelColor(1, Color);
      mRgb->setPixelColor(2, Color);

    } else if (index == E_RGB_LEFT) {
      mRgb->setPixelColor(3, Color);
      mRgb->setPixelColor(4, Color);
      mRgb->setPixelColor(5, Color);
    }
    mRgb->show();
  }
  void SetRgbEffect(E_RGB_INDEX index, long Color, uint8_t effect) {
    switch ((E_RGB_EFFECT)effect) {
      case E_EFFECT_BREATHING:
        for (long i = 5; i < 50; i++) {
          SetRgbColor(index, (i << 16) | (i << 8) | i);
          delay((i < 18) ? 18 : (256 / i));
        }
        for (long i = 50; i >= 5; i--) {
          SetRgbColor(index, (i << 16) | (i << 8) | i);
          delay((i < 18) ? 18 : (256 / i));
        }
        break;
      case E_EFFECT_ROTATE:
        SetRgbColor(E_RGB_ALL, 0);
        mRgb->setPixelColor(0, Color);
        mRgb->setPixelColor(3, Color);
        mRgb->show();
        delay(200);
        mRgb->setPixelColor(0, 0);
        mRgb->setPixelColor(3, 0);
        mRgb->setPixelColor(1, Color);
        mRgb->setPixelColor(4, Color);
        mRgb->show();
        delay(200);
        mRgb->setPixelColor(1, 0);
        mRgb->setPixelColor(4, 0);
        mRgb->setPixelColor(2, Color);
        mRgb->setPixelColor(5, Color);
        mRgb->show();
        delay(200);
        mRgb->setPixelColor(2, 0);
        mRgb->setPixelColor(5, 0);
        mRgb->show();
        break;
      case E_EFFECT_FLASH:
        for (byte i = 0; i < 6; i++) {
          SetRgbColor(E_RGB_ALL, Color);
          delay(100);
          SetRgbColor(E_RGB_ALL, 0);
          delay(100);
        }
        break;
    }
  }
};
#endif
