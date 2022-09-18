//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Richard Li - 2020
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP with authentication)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
boolean confirmRequestPending = true;

void BTConfirmRequestCallback(uint32_t numVal)
{
  confirmRequestPending = true;
  Serial.println(numVal);
}

void BTAuthCompleteCallback(boolean success)
{
  confirmRequestPending = false;
  if (success)
  {
    Serial.println("Pairing success!!");
  }
  else
  {
    Serial.println("Pairing failed, rejected by user!!");
  }
}


void setup()
{
  Serial.begin(115200);
  SerialBT.enableSSP();
  SerialBT.onConfirmRequest(BTConfirmRequestCallback);
  SerialBT.onAuthComplete(BTAuthCompleteCallback);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop()
{
//    Serial.print("confirmRequestPending ");
//    Serial.println(confirmRequestPending);
//  if (confirmRequestPending)
//  {
//    if (Serial.available())
//    {
//      int dat = Serial.read();
//      if (dat == 'Y' || dat == 'y')
//      {
//        SerialBT.confirmReply(true);
//        Serial.println("connect true");
//      }
//      else
//      {
//        SerialBT.confirmReply(false);
//        Serial.println("connect false");
//      }
//    }
//  }
//  else
  {
    if (Serial.available()) {
      while (Serial.available())
      {
        SerialBT.write(Serial.read());
      }
      SerialBT.write('\n');
    }
    if (SerialBT.available()) {
      char tk = '\0';
      while (SerialBT.available())
      {
        char temp = SerialBT.read();
        if (tk == '\0')
          tk = temp;
        Serial.write(temp);
      }
      Serial.write('\n');
      SerialBT.println(tk);
    }
    //    delay(20);
  }
}
