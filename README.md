# OpenCatESP32

This is a demo for [Petoi BiBoard](https://docs.petoi.com/biboard/biboard-v0) at a super early stage.

Install the ESP32 board:

Add https://dl.espressif.com/dl/package_esp32_index.json to the Additional Boards Manager URL

Search "ESP32" in the Board Manager and install it.


## Configuration:

ESP32 Dev Module

* Upload Speed: 921600

* CPU Frequency: 240MHz(WiFi/BT)

* Flash Frequency: 80MHz

* Flash Mode: QIO

* Flash Size: 16MB

* Partition Scheme: Default 4MB with spiffs (we will add instructions on making larger partitions)

* Core Debug Level: None

* PSRAM: Disabled

Uploading **OpenCatEsp32.ino** to the BiBoard should launch a simple demo that makes Bittle walk (click the picture to play). 

[![BittleESP32](https://github.com/PetoiCamp/NonCodeFiles/blob/master/gif/BiBoard.gif)](https://www.youtube.com/watch?v=GTgps_H990w)

Click the GIF to open the YouTube demo.

 
