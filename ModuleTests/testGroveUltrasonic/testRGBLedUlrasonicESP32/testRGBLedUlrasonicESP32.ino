// code to test the ultrasonic sensor and the NeoPixel RGB LEDs.
// The LEDs change with the distance and make different sound.
// Adjusted for the ESP32 library
// Rongzhong Li
// Petoi LLC
// Feb.6, 2024

#include "RgbUltrasonic.h"

// #define BiBoard_V0_1  //ESP32 Board with 12 channels of built-in PWM for joints
// #define BiBoard_V0_2
#define BiBoard_V1_0

#if defined BiBoard_V0_1 || defined BiBoard_V0_2
#define UART_RX2 16
#define UART_TX2 17
// comment out the following line if the Buzzer is too loud
// #define BUZZER 25  // the PWM pin the ACTIVE buzzer is attached to
#elif defined BiBoard_V1_0
#define UART_RX2 9
#define UART_TX2 10
// comment out the following line if the Buzzer is too loud
// #define BUZZER 2  // the PWM pin the ACTIVE buzzer is attached to
#endif

RgbUltrasonic ultrasonic(UART_RX2, UART_TX2);  //(signal, RGB) use the Grove Tx Rx
// RgbUltrasonic ultrasonic(27,23);  //(signal, RGB) use the infrared reciever's pin 23 and pwm pin 27
//The RGBLED module should be plugged in the fourth grove socket with D6, D7


#ifdef BUZZER
#define BASE_PITCH 1046.50
void beep(float note, float duration = 50, int pause = 0, byte repeat = 1) {
  for (byte r = 0; r < repeat; r++) {
    if (note == 0)
      delay(duration);
    else
      tone(BUZZER, BASE_PITCH * pow(1.05946, note), duration);
    delay(pause);
  }
}
#endif

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.

int interval = 3;
void setup() {
  Serial.begin(115200);
#ifdef BUZZER
  pinMode(BUZZER, OUTPUT);
#endif
  ultrasonic.SetupLED();
  ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_RED, E_EFFECT_FLASH);
  ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_YELLOW, E_EFFECT_ROTATE);
  ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_WHITE, E_EFFECT_BREATHING);
}
void loop() {
  float distance = ultrasonic.GetUltrasonicDistance();
  Serial.print(distance);
  Serial.print(" cm");
  Serial.println();
  if (distance > 50) {
    ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_WHITE, E_EFFECT_BREATHING);
  } else {
#ifdef BUZZER
    beep(30 - distance / interval * 2, 20 + distance / 2, 5 + distance);
#endif
    Serial.print(30 - distance / interval * 2);
    Serial.print("\t");
    Serial.print(20 + distance / 2);
    Serial.print("\t");
    Serial.println(5 + distance);
    if (distance < 4) {
      ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_RED, E_EFFECT_FLASH);
    } else if (distance < 10 + interval) {
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
    } else {
      ultrasonic.SetRgbEffect(E_RGB_ALL, RGB_YELLOW, E_EFFECT_ROTATE);
    }
  }
}
