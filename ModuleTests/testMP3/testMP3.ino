
#include "PetoiESP32MP3.h"      // MP3 Player library
//#include "PetoiESP32OTA.h"      // OTA Service


//char *WiFi_SSID = "Kpower-Engineers_2G";    // Your WiFi SSID, only support 2.4GHz, ESP32 cannot recieve 5GHz signal
//char *WiFi_PASSWORD = "kpower2012";         // Your WiFi password


//
//IRrecv irrecv(IR_Remote_pin);   // IR in, ref: BiboardPinDef.h
//decode_results results;

PetoiESP32MP3Player MP3Player;
//PetoiESP32OTA OTAService;


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
  MP3Player.mp3PlayBack("/bark.mp3");
  //  pinMode(25, OUTPUT);
  //  digitalWrite(25, 0);
  //  pinMode(25, OUTPUT);
  //  playMelody(melody, sizeof(melody) / 2);
  MP3Player.mp3PlayBack("/viola16_48kbps.mp3");
  //  MP3Player.mp3PlayBack("/pno-cs.mp3");

  //  delay(10);
  //  //OTAService.setupWiFi(WiFi_SSID, WiFi_PASSWORD);   // Setup WiFi connection
  //  //OTAService.OTAEnable();       // Enable OTA Service
  delay(10);

}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    Serial.println(c);
    switch (c) {
      case '0':  MP3Player.mp3PlayBack("/pno-cs.mp3"); break;
      case '1':  MP3Player.mp3PlayBack("/bark.mp3"); break;
      case '2':  MP3Player.mp3PlayBack("/viola16_48kbps.mp3"); break;
      case '3':  MP3Player.mp3PlayBack("/viola24pcm_441k_192kbps.mp3"); break;
      case '4':  MP3Player.mp3PlayBack("/1-10.mp3"); break;
      case '5':  MP3Player.mp3PlayBack("/lowbattery_16b_32k.mp3"); break;
      case '6':  MP3Player.mp3PlayBack("/lowbattery_24b_192k.mp3"); break;
    }
  }
}
