
#include "PetoiESP32MP3.h"      // MP3 Player library
#include "PetoiESP32OTA.h"      // OTA Service
#include "PetoiESP32SPIFFS.h"   // SPIFFS library for storage on QSPI Flash
#include "PetoiMPU6050DMP.h"    // Simplified MPU6050 DMP library, outputs Euler angles in degrees 
#include "PetoiBittleMotion.h"
#include <IRremote.h>
#include "BiboardPinDef.h"      // Biboard Pin definition file, for reference

char *WiFi_SSID = "Kpower-Engineers_2G";    // Your WiFi SSID, only support 2.4GHz, ESP32 cannot recieve 5GHz signal
char *WiFi_PASSWORD = "kpower2012";         // Your WiFi password

float ypr[3];                   // Yaw Pitch Raw data in degrees from MPU6050 DMP

IRrecv irrecv(IR_Remote_pin);   // IR in, ref: BiboardPinDef.h
decode_results results;

PetoiESP32MP3Player MP3Player;
PetoiESP32OTA OTAService;

void getIMUDataOfYawPitchRaw() { // Get Yaw Pitch Raw data from MPU6050
  getDMPRawResult();
  getDMPReadableYawPitchRaw(ypr);
}



void setup() {
  Serial.begin(115200);         // Serial start, ESP32 MAX 230400
  MP3Player.init();             // Setup MP3 decoder, SPIFFS and DAC output
  delay(10);
  //OTAService.setupWiFi(WiFi_SSID, WiFi_PASSWORD);   // Setup WiFi connection
  //OTAService.OTAEnable();       // Enable OTA Service
  delay(10);
  mpu_setup();                  // Setup MPU6050 & DMP
  delay(10);
  irrecv.enableIRIn();          // Enable IR in
  testRun();
}

void loop() {

/*   OTAService.handleOTA();
  getIMUDataOfYawPitchRaw();
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);  
    irrecv.resume(); // Receive the next value
  } */

  
}
