/* Write and read long data string to I2C EEPROM
   maximum is 65536 bit. i.e. address stops at 65535.
   Extra data will wrap over to address 0

   Rongzhong Li
   August 2018

   Copyright (c) 2018 Petoi LLC.

  The MIT license

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include <Wire.h>
#define DEVICE_ADDRESS 0x54  //Address of eeprom chip
#define WIRE_BUFFER 30       //Arduino wire allows 32 byte buffer, with 2 byte for address.
#define WIRE_LIMIT 16        //That leaves 30 bytes for data. use 16 to balance each writes
#define PAGE_LIMIT 32        //AT24C32D 32-byte Page Write Mode. Partial Page Writes Allowed
#define SIZE (65535 / 8)
#define EEPROM_SIZE (65535 / 8)
bool EEPROMOverflow = false;



#define EEPROM_BIRTHMARK_ADDRESS 0
#define EEPROM_IMU 1                  // 2x9 = 18 bytes
#define EEPROM_CALIB 20               // 16 bytes
#define EEPROM_BLE_NAME 36            // 20 bytes
#define EEPROM_BOOTUP_SOUND_STATE 56  // 1 byte
#define EEPROM_BUZZER_VOLUME 57       // 1 byte
#define EEPROM_RESERVED 60
#define SERIAL_BUFF 100

void i2cDetect() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning I2C network...");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("- I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    } else if (error == 4) {
      Serial.print("- Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("- No I2C devices found");
  else
    Serial.println("- done");
}

void i2c_eeprom_write_byte(unsigned int eeaddress, byte data) {
  int rdata = data;
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write((int)(eeaddress >> 8));    // MSB
  Wire.write((int)(eeaddress & 0xFF));  // LSB
  Wire.write(rdata);
  Wire.endTransmission();
  delay(5);  // needs 5ms for write
}

byte i2c_eeprom_read_byte(unsigned int eeaddress) {
  byte rdata = 0xFF;
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write((int)(eeaddress >> 8));    // MSB
  Wire.write((int)(eeaddress & 0xFF));  // LSB
  Wire.endTransmission();
  Wire.requestFrom(DEVICE_ADDRESS, 1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}


//This function will write a 2-byte integer to the EEPROM at the specified address and address + 1
void i2c_eeprom_write_int16(unsigned int eeaddress, int16_t p_value) {
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write((int)(eeaddress >> 8));    // MSB
  Wire.write((int)(eeaddress & 0xFF));  // LSB
  Wire.write(lowByte);
  Wire.write(highByte);
  Wire.endTransmission();
  delay(5);  // needs 5ms for write

  //  EEPROM.update(p_address, lowByte);
  //  EEPROM.update(p_address + 1, highByte);
}

//This function will read a 2-byte integer from the EEPROM at the specified address and address + 1
int16_t i2c_eeprom_read_int16(unsigned int eeaddress) {
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write((int)(eeaddress >> 8));    // MSB
  Wire.write((int)(eeaddress & 0xFF));  // LSB
  Wire.endTransmission();
  Wire.requestFrom(DEVICE_ADDRESS, 2);
  byte lowByte = Wire.read();
  byte highByte = Wire.read();
  return (int16_t((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00));
}


void i2c_eeprom_read_buffer(unsigned int eeaddress, byte *buffer, int length) {
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write((int)(eeaddress >> 8));    // MSB
  Wire.write((int)(eeaddress & 0xFF));  // LSB
  Wire.endTransmission();
  Wire.requestFrom(DEVICE_ADDRESS, length);
  int c = 0;
  for (c = 0; c < length; c++) {
    if (Wire.available()) buffer[c] = Wire.read();
    //    PT((char)buffer[c]);
  }
}


void writeLong(unsigned int eeAddress, char *data, int len) {
  //byte locationInPage = eeAddress % PAGE_LIMIT;
  if (eeAddress + len >= SIZE) {
    PTL();
    PTL("EEPROM overflow!\n");
    beep(10, 100, 100, 5);
    return;
  }
  i2c_eeprom_write_byte(eeAddress++, len);
  //  PTL("write " + String(len) + " bytes");
  int writtenToEE = 0;
  while (len > 0) {
    Wire.beginTransmission(DEVICE_ADDRESS);
    Wire.write((int)((eeAddress) >> 8));  // MSB
    Wire.write((int)((eeAddress)&0xFF));  // LSB
    //    PT("* current address: ");
    //    PT((unsigned int)eeAddress);
    //    PTL("\t0 1 2 3 4 5 6 7 8 9 a b c d e f ");
    //    PT("\t\t\t\t");
    byte writtenToWire = 0;
    do {
      //      PT(data[writtenToEE]);
      //      PT(" ");
      Wire.write((byte)data[writtenToEE]);
      writtenToWire++;
      writtenToEE++;
      eeAddress++;
      len--;
    } while (len > 0 && (eeAddress % PAGE_LIMIT) && writtenToWire < WIRE_LIMIT);
    Wire.endTransmission();
    delay(6);  // needs 5ms for page write
    //    PTL();
    //    PTL("wrote " + String(writtenToWire) + " bytes.");
  }
  //  PTL("finish writing");
}

void readLong(unsigned int eeAddress, char *data) {
  int len = i2c_eeprom_read_byte(eeAddress++);
  PTL("read " + String(len) + " bytes");
  int readFromEE = 0;
  int readToWire = 0;

  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write((int)((eeAddress) >> 8));  // MSB
  Wire.write((int)((eeAddress)&0xFF));  // LSB
  Wire.endTransmission();
  while (len > 0) {
    Wire.requestFrom(DEVICE_ADDRESS, min(WIRE_BUFFER, len));
    readToWire = 0;
    do {
      if (Wire.available()) data[readFromEE] = Wire.read();
      PT((char)data[readFromEE]);
      readFromEE++;
    } while (--len > 0 && ++readToWire < WIRE_BUFFER);
    PTL();
  }
  PTL("finish reading");
}

bool newBoardQ(unsigned int eeaddress) {
  return i2c_eeprom_read_byte(eeaddress) != BIRTHMARK;
}

char data[] = " The quick brown fox jumps over the lazy dog. \
The five boxing wizards jump quickly. Pack my box with five dozen liquor jugs.";  // data to write

//char data[]={16,-3,5,7,9};

void genBleID(int suffixDigits = 2) {
  const char *prefix =
#ifdef BITTLE
    "Bittle"
#elif defined NYBBLE
    "Nybble"
#else
    "Cub"
#endif
    ;
  int prelen = strlen(prefix);
  //  PTL(prelen);
  char *id = new char[prelen + suffixDigits + 1];
  strcpy(id, prefix);
  for (int i = 0; i < suffixDigits; i++) {
    int temp = esp_random() % 16;
    sprintf(id + prelen + i, "%X", temp);
  }
  id[prelen + suffixDigits] = '\0';
  //  for (int i = 0; i < prelen + suffixDigits; i++) {
  //    i2c_eeprom_write_byte(EEPROM_BLE_NAME + 1 + i, id[i]);
  //  }
  Serial.println(id);
  writeLong(EEPROM_BLE_NAME, id, prelen + suffixDigits);
}

char *readBleID() {
  int idLen = i2c_eeprom_read_byte(EEPROM_BLE_NAME);
  char *id = new char[idLen + 1];
  //  readLong(EEPROM_BLE_NAME, id);
  for (int i = 0; i < idLen; i++) {
    id[i] = i2c_eeprom_read_byte(EEPROM_BLE_NAME + 1 + i);
  }
  id[idLen] = '\0';
  return id;
}

void customBleID(char *customName, int8_t len) {
  writeLong(EEPROM_BLE_NAME, customName, len + 1);
}

int dataLen(int8_t p) {
  byte skillHeader = p > 0 ? 4 : 7;
  int frameSize = p > 1 ? WALKING_DOF :  //gait
                    p == 1 ? DOF
                           :  //posture
                    DOF + 4;  //behavior
  int len = skillHeader + abs(p) * frameSize;
  return len;
}
void i2cEepromSetup() {
  newBoard = newBoardQ(EEPROM_BIRTHMARK_ADDRESS);
  if (newBoard) {
    PTLF("Set up the new board...");
    i2c_eeprom_write_byte(EEPROM_BOOTUP_SOUND_STATE, 1);
    i2c_eeprom_write_byte(EEPROM_BUZZER_VOLUME, 5);
    soundState = 1;
    buzzerVolume = 5;
    PTF("Unmute and set volume to 5/10");
#ifndef AUTO_INIT
    playMelody(melodyInit, sizeof(melodyInit) / 2);
#endif
    PTF("- Name the new robot as: ");
#ifdef BT_BLE
    genBleID();
#endif
#ifndef AUTO_INIT
    PTL("- Reset the joints' calibration offsets? (Y/n): ");
    char choice = getUserInputChar();
    PTL(choice);
    if (choice == 'Y' || choice == 'y') {
#else
    PTL("- Reset the joints' calibration offsets...");
#endif
      for (byte c = 0; c < DOF; c++)
        i2c_eeprom_write_byte(EEPROM_CALIB + c, 0);
#ifndef AUTO_INIT
    }
#endif
  } else
    playMelody(melodyNormalBoot, sizeof(melodyNormalBoot) / 2);
}
void copydataFromBufferToI2cEeprom(unsigned int eeAddress, int8_t *newCmd) {
  int len = dataLen(newCmd[0]) + 1;
  if (eeAddress + len >= EEPROM_SIZE) {
    PTL();
    PTLF("I2C EEPROM overflow! Delete some skills!\n");
    EEPROMOverflow = true;
#ifdef BUZZER
    beep(10, 100, 100, 2);
#endif
    return;
  }
  int writtenToEE = 0;
  while (len > 0) {
    Wire.beginTransmission(DEVICE_ADDRESS);
    Wire.write((int)((eeAddress) >> 8));  // MSB
    Wire.write((int)((eeAddress)&0xFF));  // LSB
    byte writtenToWire = 0;
    do {
      Wire.write((byte)newCmd[writtenToEE++]);
      writtenToWire++;
      eeAddress++;
    } while ((--len > 0) && (eeAddress % PAGE_LIMIT) && (writtenToWire < WIRE_LIMIT));  //be careful with the chained conditions
    //self-increment may not work as expected
    Wire.endTransmission();
    delay(6);  // needs 5ms for page write
    //    PTL("\nwrote " + String(writtenToWire) + " bytes.");
  }
  delay(6);
  //  PTLF("finish copying to I2C EEPROM");
}
void loadDataFromI2cEeprom(unsigned int eeAddress) {
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write((int)((eeAddress) >> 8));  // MSB
  Wire.write((int)((eeAddress)&0xFF));  // LSB
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)DEVICE_ADDRESS, (uint8_t)1);
  newCmd[0] = Wire.read();
  int bufferLen = dataLen(newCmd[0]);
  //      int tail = bufferLen;
  int readFromEE = 0;
  int readToWire = 0;
  while (bufferLen > 0) {
    //PTL("request " + String(min(WIRE_BUFFER, len)));
    Wire.requestFrom((uint8_t)DEVICE_ADDRESS, (uint8_t)min(WIRE_BUFFER, bufferLen));
    readToWire = 0;
    do {
      if (Wire.available()) newCmd[1 + readFromEE++] = Wire.read();
      //      PT( (int8_t)newCmd[readFromEE - 1]);
      //      PT('\t');
    } while (--bufferLen > 0 && ++readToWire < WIRE_BUFFER);
    //    PTL();
  }
  //      newCmd[tail] = '\0';
}

void saveCalib(int8_t *var) {
  for (byte s = 0; s < DOF; s++) {
    i2c_eeprom_write_byte(EEPROM_CALIB + s, var[s]);
    calibratedZeroPosition[s] = zeroPosition[s] + float(var[s]) * rotationDirection[s];
  }
}
