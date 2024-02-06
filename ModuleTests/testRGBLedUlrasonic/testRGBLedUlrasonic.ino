//code provided by the manufacturer
//modified by Rongzhong Li for better demonstration.
//Feb.16, 2021

#include "RgbUltrasonic.h"

RgbUltrasonic ultrasonic(16, 17);  //(signal, RGB)
//The RGBLED module should be plugged in teh fourth grove socket with D6, D7
#define BUZZER 5  // the PWM pin the ACTIVE buzzer is attached to
void beep(int note, float duration = 10, int pause = 0, byte repeat = 1) {
  if (note == 0) {
    analogWrite(BUZZER, 0);
    delay(duration);
    return;
  }
  int freq = 220 * pow(1.059463, note - 1);  // 220 is note A3
  //1.059463 comes from https://en.wikipedia.org/wiki/Twelfth_root_of_two
  float period = 1000000.0 / freq / 2.0;
  for (byte r = 0; r < repeat; r++) {
    for (float t = 0; t < duration * 1000; t += period * 2) {
      analogWrite(BUZZER, 100);  // Almost any value can be used except 0 and 255. it can be tuned as the amplitude of the note
      // experiment to get the best tone
      delayMicroseconds(period);  // wait for a delayms ms
      analogWrite(BUZZER, 0);     // 0 turns it off
      delayMicroseconds(period);  // wait for a delayms ms
    }
    delay(pause);
  }
}

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.

int interval = 3;
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER, OUTPUT);
  ultrasonic.SetupLED();
  // colorWipe(mRgb.Color(255, 0, 0), 50);          // Red
  // theaterChase(mRgb.Color(127, 127, 127), 50);  // White, half brightness
}
void loop() {
  float distance = ultrasonic.GetUltrasonicDistance();
  Serial.print(distance);
  Serial.print(" cm");
  Serial.println();
  // if (distance > 50) {
  //   ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_WHITE, E_EFFECT_BREATHING);
  // }
  // else
  {
    beep(50 - distance / interval * 2, 20 + distance / 2, 5 + distance);
    Serial.print(50 - distance / interval * 2);
    Serial.print("\t");
    Serial.print(20 + distance / 2);
    Serial.print("\t");
    Serial.println(5 + distance);
    if (distance < 4) {
      //   ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_RED, E_EFFECT_FLASH);
      // }
      // else if (distance < 10 + interval) {
      ultrasonic.SetRgbColor(E_RGB_ALL, RGB_RED);
    } else if (distance < 10 + interval * 3) {
      ultrasonic.SetRgbColor(E_RGB_ALL, RGB_BLUE);
    } else if (distance < 10 + interval * 5) {
      ultrasonic.SetRgbColor(E_RGB_ALL, RGB_GREEN);
    } else if (distance < 10 + interval * 7) {
      ultrasonic.SetRgbColor(E_RGB_ALL, RGB_YELLOW);
    } else if (distance < 10 + interval * 9) {
      ultrasonic.SetRgbColor(E_RGB_ALL, RGB_PURPLE);
    } else if (distance < 10 + interval * 11) {
      ultrasonic.SetRgbColor(E_RGB_ALL, RGB_WHITE);
    }
    // else {
    //   ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_YELLOW, E_EFFECT_ROTATE);
    // }
  }
}
