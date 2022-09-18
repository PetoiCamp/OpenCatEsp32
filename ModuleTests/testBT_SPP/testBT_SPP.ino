#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
bool BTconnected = false;
void setup() {
  Serial.begin(115200);
  SerialBT.begin("BiBoardSpp"); //Bluetooth device name

  Serial.println(F("\n* Start *"));
  Serial.println(F("petoi"));
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  if (!BTconnected) {

    if (SerialBT.available()) {
      SerialBT.println(F("\n* Start *"));
      SerialBT.println(F("petoi"));
      delay(10);
      BTconnected = true;
    }
  }
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  delay(20);
}
