#include "soc/gpio_sig_map.h"

void read_sound() {
}

void read_GPS() {
}
#ifdef TOUCH0
void read_touch() {
  byte touchPin[] = {
    TOUCH0,
    TOUCH1,
    TOUCH2,
    TOUCH3,
  };
  for (byte t = 0; t < 4; t++) {
    int touchValue = touchRead(touchPin[t]);  //do something with the touch?
    //    PT(touchValue);
    //    PT('\t');
  }
  //  PTL();
}
#endif
void readEnvironment() {
#ifdef GYRO_PIN
  if (gyroBalanceQ && !(frame % imuSkip))
    imuUpdated = read_mpu6050();
#endif
  read_sound();
  read_GPS();
}

//— read master computer’s signals (middle level) —

//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Richard Li - 2020
//
//This example creates a bridge between Serial and Classical Bluetooth (SSP with authentication)
//and also demonstrate that SerialBT has the same functionalities as a normal Serial

#ifdef BT_SSP
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
boolean confirmRequestPending = true;
boolean BTconnected = false;

void BTConfirmRequestCallback(uint32_t numVal) {
  confirmRequestPending = true;
  Serial.println(numVal);
}

void BTAuthCompleteCallback(boolean success) {
  confirmRequestPending = false;
  if (success) {
    BTconnected = true;
    Serial.println("SSP Pairing success!!");
  } else {
    BTconnected = false;
    Serial.println("SSP Pairing failed, rejected by user!!");
  }
}

void blueSspSetup() {
  PTHL("SSP:\t", strcat(readLongByBytes(EEPROM_BLE_NAME), "_SSP"));
  SerialBT.enableSSP();
  SerialBT.onConfirmRequest(BTConfirmRequestCallback);
  SerialBT.onAuthComplete(BTAuthCompleteCallback);
  SerialBT.begin(strcat(readLongByBytes(EEPROM_BLE_NAME), "_SSP"));  //Bluetooth device name
  Serial.println("The SSP device is started, now you can pair it with Bluetooth!");
}

//void readBlueSSP() {
//  if (confirmRequestPending)
//  {
//    if (Serial.available())
//    {
//      int dat = Serial.read();
//      if (dat == 'Y' || dat == 'y')
//      {
//        SerialBT.confirmReply(true);
//      }
//      else
//      {
//        SerialBT.confirmReply(false);
//      }
//    }
//  }
//  else
//  {
//    if (Serial.available())
//    {
//      SerialBT.write(Serial.read());
//    }
//    if (SerialBT.available())
//    {
//      Serial.write(SerialBT.read());
//    }
//    delay(20);
//  }
//}

//end of Richard Li's code

#endif

template<typename T> void printToAllPorts(T text) {
#ifdef BT_BLE
  if (deviceConnected)
    bleWrite(String(text));
#endif
#ifdef BT_SSP
  if (BTconnected)
    SerialBT.println(text);
#endif
  if (moduleActivatedQ[0])//serial2
    Serial2.println(text);
  PTL(text);
}
