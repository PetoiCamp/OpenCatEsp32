#include <ratio>
#include "esp32-hal.h"
#include "RgbUltrasonic.h"

RgbUltrasonic::RgbUltrasonic(byte signal_pin, byte rgb_pin) {
  SignalPin = signal_pin;
  RgbPin = rgb_pin;
  lastMeasurementTime = millis();
  measurementInterval = 20;
  // mRgb = new RGBLed(RgbPin, 6);

  Serial.println("set up ultrasonic sensor");
  delay(2000);
}

void RgbUltrasonic::SetupLED() {
  mRgb = new Adafruit_NeoPixel(6, RgbPin, NEO_GRB + NEO_KHZ800);
  mRgb->begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  mRgb->show();             // Turn OFF all pixels ASAP
  mRgb->setBrightness(50);  // Set BRIGHTNESS to about 1/5 (max = 255)
}

float RgbUltrasonic::GetUltrasonicDistance(void) {  //in cm
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

void RgbUltrasonic::SetRgbColor(E_RGB_INDEX index, long Color) {
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

// void RgbUltrasonic::SetRgbEffect(E_RGB_INDEX index, long Color, uint8_t effect)
// {
//     switch((E_RGB_EFFECT)effect) {
//         case E_EFFECT_BREATHING:
//             for (long i = 5; i < 50; i++) {
//                 SetRgbColor(index, (i<<16)|(i<<8)|i);
//                 delay((i < 18) ? 18: (256/i));
//             }
//             for (long i = 50; i >= 5; i--) {
//                 SetRgbColor(index, (i<<16)|(i<<8)|i);
//                 delay((i < 18) ? 18: (256/i));
//             }
//             break;
//         case E_EFFECT_ROTATE:
//             SetRgbColor(E_RGB_ALL, 0);
//             mRgb->setColor(1, Color);
//             mRgb->setColor(4, Color);
//             mRgb->show();
//             delay(200);
//             mRgb->setColor(1, 0);
//             mRgb->setColor(4, 0);
//             mRgb->setColor(2, Color);
//             mRgb->setColor(5, Color);
//             mRgb->show();
//             delay(200);
//             mRgb->setColor(2, 0);
//             mRgb->setColor(5, 0);
//             mRgb->setColor(3, Color);
//             mRgb->setColor(6, Color);
//             mRgb->show();
//             delay(200);
//             mRgb->setColor(3, 0);
//             mRgb->setColor(6, 0);
//             mRgb->show();
//             break;
//         case E_EFFECT_FLASH:
//             for (byte i = 0; i < 6; i++) {
//                SetRgbColor(E_RGB_ALL, Color);
//                delay(100);
//                SetRgbColor(E_RGB_ALL, 0);
//                delay(100);
//             }
//             break;
//     }
// }
