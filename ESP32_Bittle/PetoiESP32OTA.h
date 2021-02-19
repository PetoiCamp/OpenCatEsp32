#ifndef _PETOIESP32OTA_H
#define _PETOIESP32OTA_H

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

class PetoiESP32OTA{

    public:
        PetoiESP32OTA();
        void setupWiFi(char* your_ssid, char* your_password);
        void OTAEnable();
        void handleOTA();
        void printOTAAddress();

    private:
        const char* ssid;
        const char* password;

};

#endif
