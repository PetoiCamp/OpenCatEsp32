#ifndef _PETOIESP32MP3_H
#define _PETOIESP32MP3_H

// Arduino Library
#include <Arduino.h>

// ESP32 WiFi & SPIFFS Filesystem Library
#include <WiFi.h>
#include "SPIFFS.h"

// ESP32 Audio Library
#include "AudioFileSourceSPIFFS.h"
#include "AudioFileSourceID3.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2SNoDAC.h"

class PetoiESP32MP3Player {

    public:
        PetoiESP32MP3Player();
        void init();
        void mp3PlayBack(char *filename);

    private:
        AudioGeneratorMP3 *mp3;
        AudioFileSourceSPIFFS *file;
        AudioOutputI2S *out;

};

#endif