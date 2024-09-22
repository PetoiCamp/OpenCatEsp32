/* Test flash eeprom

   Rongzhong Li
   Petoi LLC
   Sep. 18, 2024
*/

#define SIZE 128

#include <EEPROM.h>
char buff[SIZE];
void showEEPROM(int len = 4096) {
  EEPROM.begin(len);  //operation on len bytes. if you need to read until address i, use i+1 here)
  for (int addr = 0; addr < len; addr++) {
    int data = EEPROM.read(addr);  //read data from address
    Serial.print(char(data));
    //    Serial.print(" ");
    delay(2);
    if ((addr + 1) % 8 == 0)  //print new line every 8 bytes
    {
      Serial.println("");
    }
  }
  EEPROM.end();
  Serial.println("End read");
}

void format(int len = 4096) {
  Serial.println("Start write");

  EEPROM.begin(len + 1);
  for (int addr = 0; addr < len; addr++) {
    int data = 'A' + addr % 58;  //save char: A-->Z-->[\]^_`-->a-->z
    EEPROM.write(addr, data);
  }
  EEPROM.commit();  //save the change
  EEPROM.end();     //same as EEPROM.commit()
  Serial.println("End write");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  Serial.setTimeout(2);
  Serial.println("");
  Serial.println("Start Demo");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    char operation = Serial.read();
    if (operation == 'w') {
      int address = Serial.parseInt();
      Serial.read();  //read and delete the space
      Serial.print(address);
      EEPROM.begin(SIZE + 1);
      for (int addr = 0; addr < SIZE; addr++) {  //read EEPROM, save to buffer
        buff[addr] = EEPROM.read(addr);
        delay(2);
      }
      EEPROM.end();

      int beginAddr = address;
      while (Serial.available()) {
        char data = Serial.read();
        Serial.print(data);
        buff[address++] = data;
      }
      Serial.println();
      Serial.println("Buffer");  // print buffer
      for (int b = 0; b < SIZE; b++) {
        Serial.print(char(buff[b]));
        if ((b + 1) % 8 == 0)
          Serial.println();
      }
      Serial.println("\nEEPROM:");  //write data
      EEPROM.begin(SIZE + 1);
      for (int addr = beginAddr; addr < SIZE; addr++)
        EEPROM.write(addr, buff[addr]);
      EEPROM.commit();  //save
      EEPROM.end();
      showEEPROM(SIZE);
    } else if (operation == 'r') {  //read data and print
      if (Serial.available())
        showEEPROM(Serial.parseInt());
      else
        showEEPROM(SIZE);
      Serial.println("Buffer");
      for (int b = 0; b < SIZE; b++) {
        Serial.print(char(buff[b]));
        if ((b + 1) % 8 == 0)
          Serial.println();
      }
    } else if (operation == 'f') {  //reformat with known patterns
      if (Serial.available())
        format(Serial.parseInt());
      else
        format(SIZE);
    }
  }
}
