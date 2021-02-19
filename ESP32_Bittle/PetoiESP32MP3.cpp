#include "PetoiESP32MP3.h"

using namespace std;

PetoiESP32MP3Player::PetoiESP32MP3Player(){

}

void PetoiESP32MP3Player::init(){
    
    Serial.begin(115200);       // Enable Serial        
    SPIFFS.begin();             // Enable SPIFFS Filesystem

}

void PetoiESP32MP3Player::mp3PlayBack(char *filename){

    file = new AudioFileSourceSPIFFS(filename); // get file from SPIFFS
    out = new AudioOutputI2S(0, 1, 8, 0);       // USE DAC Pin 25 output
    mp3 = new AudioGeneratorMP3();

    // Serial print MP3 infomation
    Serial.print("Play :");
    Serial.println(*filename);

    mp3->begin(file, out);
    while(mp3->isRunning()){
    if (!mp3->loop()){ 
        mp3->stop();
        break;
    }
  }

  delete file;
  delete out;
  delete mp3;
}
