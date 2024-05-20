/* Customized note and melody player for OpenCat, and MEOW \(=^_^=)/
   It also serves to test the baud rate of the serial moniter and the crystal frequency.
   OpenCat happens to use 57600 baud rate (and a 20MHz crystal on Petoi's dedicated PCB).
   If you cannot see this sentence on serial monitor,
      there's something wrong with your Arduino IDE's configuration.

   The beep() is better if played with a passive buzzer
   The meow() can only be played with an active buzzer on PWM pins
   On most Arduino, the PWM pins are identified with a "~" sign, like ~3, ~5, ~6, ~9, ~10 and ~11.
   If your buzzer cannot meow(), it's probably a passive buzzer.

   Rongzhong Li
   August 2017

   Copyright (c) 2018 Petoi LLC.

  The MIT License

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
#include "PetoiESP32Servo/ESP32Servo.h"
#define BASE_PITCH 1046.50
int loopCounter = 0;
// tone 1 corresponds to A3, tone 2 corresponds to B3, tone 13 corresponds to A4, etc.
// tone: pause,1,  2,  3,  4,  5,  6,  7,  high1,  high2
// code: 0,    1,  3,  5,  6,  8,  10, 12, 13,      15

byte melodyNormalBoot[] = {
  8, 13, 10, 13, 8, 5, 8, 3, 5, 8,  //tone
  4, 4, 8, 8, 4, 8, 8, 8, 8, 4,     //relative duration, 8 means 1/8 note length
};
byte melodyInit[] = {
  15, 12, 15, 12, 15, 12, 8, 10, 13, 12, 10, 15,  //tone
  4, 4, 4, 4, 4, 4, 2, 4, 4, 4, 4, 2,             //relative duration, 8 means 1/8 note length
};
byte melodyLowBattery[] = {
  15, 11, 13, 10, 11,  //tone
  4, 4, 4, 4, 4,       //relative duration, 8 means 1/8 note length
};
byte melodyOnBattery[] = {
  11, 10, 13, 11, 15,  //tone
  8, 8, 8, 8, 8,       //relative duration, 8 means 1/8 note length
};
byte melody1[] = { 15, 8, 10, 12, 13, 15, 8, 8, 17, 13, 15, 17, 19, 20, 8, 8,
                   2, 4, 4, 4, 4, 2, 2, 2, 2, 4, 4, 4, 4, 2, 2, 2 };
byte melodyIRpass[] = {
  //  6 6 5 3 2 1 3 2 1 6 5
  17, 15, 12, 10, 8, 12, 10, 8, 5, 3,
  6, 16, 8, 8, 2, 6, 16, 8, 8, 2
};
byte volumeTest[] = {
  12, 14, 16,
  4, 4, 4
};
void beep(float note, float duration = 50, int pause = 0, byte repeat = 1) {
  if (soundState) {
    for (byte r = 0; r < repeat; r++) {
      if (note > 0) {
        tone(BUZZER, BASE_PITCH * pow(1.05946, note), duration, buzzerVolume / amplifierFactor);
      } else {
        delay(duration);
      }
      delay(pause);
    }
  }
}

template<typename T> int8_t sign(T val) {
  return (T(0) < val) - (val < T(0));
}

void meow(int startF = 1, int endF = 25, float duration = 5) {
  int s = sign(endF - startF);
  float increment = 0.2 * s;
  for (float amp = startF; s * amp < s * endF; amp += increment) {
    beep(amp, duration / 5);
  }
}

void continuousTone(uint16_t tone1, uint16_t duration) {
  if (tone1 < 50 || tone1 > 15000) return;  // these do not play on a piezo
  for (long i = 0; i < duration * 1000L; i += tone1 * 2) {
    digitalWrite(BUZZER, HIGH);
    delayMicroseconds(tone1);
    digitalWrite(BUZZER, LOW);
    delayMicroseconds(tone1);
  }
}

void chirp(int startF = 180, int endF = 200, int duration = 10) {  // Bird chirp
  for (uint8_t i = startF; i < endF; i++) {
    continuousTone(i, duration);
  }
}
void soundFallOver() {
  for (int i = 0; i < 2; i++)
    chirp();
}
void playMelody(byte m[], int len) {
  if (soundState) {
    for (int i = 0; i < len; i++) {
      beep(m[i], 1000 / m[len + i]);
    }
  }
}

void playSound() {
  playMelody(melodyNormalBoot, sizeof(melodyNormalBoot) / 2);
}
