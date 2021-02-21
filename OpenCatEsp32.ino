
#include "PetoiESP32MP3.h"      // MP3 Player library
//#include "PetoiESP32OTA.h"      // OTA Service
#include "PetoiESP32SPIFFS.h"   // SPIFFS library for storage on QSPI Flash
#include "PetoiMPU6050DMP.h"    // Simplified MPU6050 DMP library, outputs Euler angles in degrees

#include "PetoiBittleMotion.h"
#include "BiboardPinDef.h"      // Biboard Pin definition file, for reference
#include <IRremote.h>

//char *WiFi_SSID = "Kpower-Engineers_2G";    // Your WiFi SSID, only support 2.4GHz, ESP32 cannot recieve 5GHz signal
//char *WiFi_PASSWORD = "kpower2012";         // Your WiFi password

float ypr[3];                   // Yaw Pitch Raw data in degrees from MPU6050 DMP
//
//IRrecv irrecv(IR_Remote_pin);   // IR in, ref: BiboardPinDef.h
//decode_results results;

PetoiESP32MP3Player MP3Player;
//PetoiESP32OTA OTAService;

void getIMUDataOfYawPitchRaw() { // Get Yaw Pitch Raw data from MPU6050
  getDMPRawResult();
  getDMPReadableYawPitchRaw(ypr);
}

byte melody[] = {8, 13, 10, 13, 8,  0,  5,  8,  3,  5, 8,//tone
                 8, 8,  32, 32, 8, 32, 32, 32, 32, 32, 8 //relative duration, 8 means 1/8 note length
                };
void beep(int note, float duration = 10, int pause = 0, byte repeat = 1 ) {
  if (note == 0) {
    digitalWrite(25, 0);
    delay(duration);
    return;
  }
  int freq = 220 * pow(1.059463, note - 1); // 220 is note A3
  //1.059463 comes from https://en.wikipedia.org/wiki/Twelfth_root_of_two
  float period = 1000000.0 / freq / 2.0;
  for (byte r = 0; r < repeat; r++) {
    for (float t = 0; t < duration * 1000; t += period * 2) {
      digitalWrite(25, 1);      // Almost any value can be used except 0 and 255. it can be tuned as the amplitude of the note
      // experiment to get the best tone
      delayMicroseconds(period);          // wait for a delayms ms
      digitalWrite(25, 0);       // 0 turns it off
      delayMicroseconds(period);          // wait for a delayms ms
    }
    delay(pause);
  }
}
void playMelody(byte m[], int len) {
  for (int i = 0; i < len; i++)
    beep(m[i], 1000 / m[len + i], 100);
}

void setup() {
  Serial.begin(115200);         // Serial start, ESP32 MAX 230400
  pinMode(25, OUTPUT);
  MP3Player.init();             // Setup MP3 decoder, SPIFFS and DAC output
  playMelody(melody, sizeof(melody) / 2);
  MP3Player.mp3PlayBack("/dogbark2.mp3");
  pinMode(25, OUTPUT);
  playMelody(melody, sizeof(melody) / 2);
  MP3Player.mp3PlayBack("/dogbark2.mp3");
  delay(10);
  //OTAService.setupWiFi(WiFi_SSID, WiFi_PASSWORD);   // Setup WiFi connection
  //OTAService.OTAEnable();       // Enable OTA Service
  delay(10);
  mpu_setup();                  // Setup MPU6050 & DMP
  delay(10);
  //  irrecv.enableIRIn();          // Enable IR in
  testRun();

  delay(2000);
}

void loop() {
  getIMUDataOfYawPitchRaw();
  /*   OTAService.handleOTA();
    getIMUDataOfYawPitchRaw();
    if (irrecv.decode(&results)) {
      Serial.println(results.value, HEX);
      irrecv.resume(); // Receive the next value
    } */
}
