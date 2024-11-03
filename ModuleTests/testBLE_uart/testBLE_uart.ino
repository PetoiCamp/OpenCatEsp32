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

#define CONNECTION_NAME "BittleTest"
#define HOLD_TIME 1000
#define CONNECTION_ATTEMPT 5
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"            // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // receive
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // transmit

char *genBleID(int suffixDigits = 2, const char prefix[] = CONNECTION_NAME) {
  int prelen = strlen(prefix);
  char *id = new char[prelen + suffixDigits + 1];
  strcpy(id, prefix);
  for (int i = prelen; i < prelen + suffixDigits; i++) {
    int temp = esp_random() % 16;
    sprintf(id + i, "%X", temp);
  }
  id[prelen + suffixDigits] = '\0';
  Serial.print("Bluetooth name: ");
  Serial.println(id);
  return id;
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue().c_str();
    if (rxValue.length() > 0) {
      Serial.print(rxValue);
      // Serial.println("*********");
      // Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i]);
      // Serial.println();
      // Serial.println("*********");
    } else {
      Serial.print(rxValue.length());
    }
  }
};

void setup() {
  Serial.begin(115200);
  Serial.print("UUID: ");
  Serial.println(SERVICE_UUID);
  // Create the BLE Device
  //  std::string suffix = std::string(CONNECTION_NAME) + genBleID(3);
  BLEDevice::init(genBleID());  //using default parameters
                                //  BLEDevice::init(genBleID(3, "OpenCat"));

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
      BLECharacteristic::PROPERTY_WRITE);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}


void bleWrite(String buff) {
  if (deviceConnected) {
    pTxCharacteristic->setValue(buff.c_str());
    pTxCharacteristic->notify();
  }
}

void loop() {
  if (deviceConnected && Serial.available()) {
    String buff = Serial.readStringUntil('\n');
    bleWrite(buff);
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(50);                    // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("Bluetooth disconnected!");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    Serial.println("Bluetooth connected!");
    oldDeviceConnected = deviceConnected;
    delay(HOLD_TIME);
    for (byte i = 0; i < CONNECTION_ATTEMPT; i++) {
      pTxCharacteristic->setValue("Petoi Bittle");
      pTxCharacteristic->notify();
      delay(100);
    }
  }
}
