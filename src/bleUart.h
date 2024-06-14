#include "esp32-hal.h"
/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE"
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second.
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define HOLD_TIME 1500
#define CONNECTION_ATTEMPT 2
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"            // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // receive
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // transmit


class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};
byte bleMessageShift = 1;
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue().c_str();
    if (rxValue.length() > 0) {
      // long current = millis();
      // PTH("BLE", current - lastSerialTime);
      int buffLen = rxValue.length();
      // Serial.print("From BLE:");
      // Serial.println(rxValue.c_str());
      if (bleMessageShift) {
        cmdLen = 0;
        token = rxValue[0];
        lowerToken = tolower(token);
        terminator = (token >= 'A' && token <= 'Z') ? '~' : '\n';
        serialTimeout = (token == T_SKILL_DATA || lowerToken == T_BEEP) ? SERIAL_TIMEOUT_LONG : SERIAL_TIMEOUT;
      }
      for (int i = bleMessageShift; i < buffLen; i++) {
        newCmd[cmdLen++] = rxValue[i];
      }
      bleMessageShift = 0;
      lastSerialTime = millis();
    }
  }
};

void readBle() {
  if (deviceConnected && !bleMessageShift) {
    while ((char)newCmd[cmdLen - 1] != terminator && long(millis() - lastSerialTime) < serialTimeout) {  //wait until the long message is completed
      delay(SERIAL_TIMEOUT);
    }
    // PTH("* BLE", millis() - lastSerialTime);
    cmdLen = (newCmd[cmdLen - 1] == terminator) ? cmdLen - 1 : cmdLen;
    newCmd[cmdLen] = (token >= 'A' && token <= 'Z') ? '~' : '\0';
    newCmdIdx = 2;
    bleMessageShift = 1;
    // PTL(cmdLen);
  }
}

void bleSetup() {
  //  Serial.print("UUID: ");
  //  Serial.println(SERVICE_UUID);
  // Create the BLE Device
  PTH("BLE: ", strcat(readLongByBytes(EEPROM_BLE_NAME), "_BLE"));
  BLEDevice::init(strcat(readLongByBytes(EEPROM_BLE_NAME), "_BLE"));  //read BLE device name from EEPROM so it's static

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic =
    pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_NOTIFY);

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic =
    pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a BLE client connection to notify...");
}


void bleWrite(String buff) {
  if (deviceConnected) {
    pTxCharacteristic->setValue(buff.c_str());
    pTxCharacteristic->notify();
  }
}

void detectBle() {
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(50);                    // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("Bluetooth BLE disconnected!");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    Serial.println("Bluetooth BLE connected!");
    oldDeviceConnected = deviceConnected;
    delay(HOLD_TIME);
    // for (byte i = 0; i < CONNECTION_ATTEMPT; i++) {// send the keywords a few times on connection so that the app knows it's a Petoi device
    pTxCharacteristic->setValue("Petoi Bittle");
    pTxCharacteristic->notify();
    // delay(100);
    // }
    token = 'd';
    bleWrite(String(T_PAUSE));
  }
}
