// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v6.12)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-10 - Uses the new version of the DMP Firmware V6.12
//                 - Note: I believe the Teapot demo is broken with this versin as
//                 - the fifo buffer structure has changed
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "mpu6050/src/I2Cdev.h"

#include "mpu6050/src/MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
   ========================================================================= */

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL
/* REALACCEL represents the acceleration related to the sensor.
   Even if the robot is not moving, it will be large if the sensor is tilted.
   Because gravity has a component in that tilted direction.
   It's useful to use aaReal.z to determine if the sensor is flipped up-side-down.
*/

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL
/* WORLDACCEL represents the acceleration related to the world reference.
   It will be close to zero as long as the robot is not moved.
   It's useful to detect the wobbling about the sensor's original position.
*/

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 gy;       // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector. unit is radian
int8_t yprTilt[3];

float originalYawDirection;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void print6Axis() {

#ifdef OUTPUT_READABLE_QUATERNION
  // display quaternion values in easy matrix form: w x y z
  Serial.print("quat\t");
  Serial.print(q.w);
  Serial.print("\t");
  Serial.print(q.x);
  Serial.print("\t");
  Serial.print(q.y);
  Serial.print("\t");
  Serial.print(q.z);
  Serial.print("\t");
#endif

#ifdef OUTPUT_READABLE_EULER
  // display Euler angles in degrees
  Serial.print("euler\t");
  Serial.print(euler[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(euler[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(euler[2] * 180 / M_PI);
  Serial.print("\t");
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
  // display angles in degrees
  Serial.print("ypr\t");
  Serial.print(ypr[0]);
  Serial.print("\t");
  Serial.print(ypr[1]);
  Serial.print("\t");
  Serial.print(ypr[2]);
  Serial.print("\t");
  /*
    mpu.dmpGetAccel(&aa, fifoBuffer);
    Serial.print("\tRaw Accl XYZ\t");
    Serial.print(aa.x);
    Serial.print("\t");
    Serial.print(aa.y);
    Serial.print("\t");
    Serial.print(aa.z);
    mpu.dmpGetGyro(&gy, fifoBuffer);
    Serial.print("\tRaw Gyro XYZ\t");
    Serial.print(gy.x);
    Serial.print("\t");
    Serial.print(gy.y);
    Serial.print("\t");
    Serial.print(gy.z);
    Serial.print("\t");
  */
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
  // display initial world-frame acceleration, adjusted to remove gravity
  // and rotated based on known orientation from quaternion
  Serial.print("aworld\t");
  Serial.print(aaWorld.x);
  Serial.print("\t");
  Serial.print(aaWorld.y);
  Serial.print("\t");
  Serial.print(aaWorld.z);
  Serial.print("\t");
#endif

#ifdef OUTPUT_READABLE_REALACCEL
  // display real acceleration, adjusted to remove gravity
  Serial.print("areal\t");
  Serial.print(aaReal.x);
  Serial.print("\t");
  Serial.print(aaReal.y);
  Serial.print("\t");
#endif
  Serial.print("areal.z\t");
  Serial.print(aaReal.z);  //becomes negative when flipped
  Serial.print("\t");

  Serial.println();
}

bool read_IMU() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    for (byte i = 0; i < 3; i++) {  //no need to flip yaw
      ypr[i] *= degPerRad;
#ifdef BiBoard
      ypr[i] = -ypr[i];
#endif
    }
    if (printGyro)
      print6Axis();
    exceptions = aaReal.z < 0 && fabs(ypr[2]) > 85;  //the second condition is used to filter out some noise
    return true;
  }
  return false;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void imuSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  // Serial.begin(115200);
  // while (!Serial)
  // ;  // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.
  int connectAttempt = 0;
  do {
    delay(500);
    // initialize device
    Serial.println(F("Initializing MPU..."));
#if defined CONFIG_DISABLE_HAL_LOCKS && CONFIG_DISABLE_HAL_LOCKS == 1
    Serial.println("OK");
#else
    Serial.println("If the program stucks, modify the header file:\n  https://docs.petoi.com/arduino-ide/upload-sketch-for-biboard#sdkconfig.h");
#endif
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    // verify connection
    Serial.print(F("- Testing MPU connections...attempt "));
    Serial.println(connectAttempt++);

  } while (!mpu.testConnection());
  Serial.println(F("- MPU6050 connection successful"));

  // load and configure the DMP
  Serial.println(F("- Initializing DMP..."));

  devStatus = mpu.dmpInitialize();

  for (byte m = 0; m < 6; m++)
    imuOffset[m] = i2c_eeprom_read_int16(EEPROM_IMU + m * 2);
  // supply the gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(imuOffset[0]);
  mpu.setYAccelOffset(imuOffset[1]);
  mpu.setZAccelOffset(imuOffset[2]);  //gravity
  mpu.setXGyroOffset(imuOffset[3]);   //yaw
  mpu.setYGyroOffset(imuOffset[4]);   //pitch
  mpu.setZGyroOffset(imuOffset[5]);   //roll

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    if (newBoard) {
#ifndef AUTO_INIT
      PTL("- Calibrate the Inertial Measurement Unit (IMU)? (Y/n): ");
      while (!Serial.available());
      char choice = Serial.read();
      Serial.println(choice);
      if (choice == 'Y' || choice == 'y') {
#else
      PTL("- Calibrate the Inertial Measurement Unit (IMU)...");
#endif
        PTLF("\nPut the robot FLAT on the table and don't touch it during calibration.");
#ifndef AUTO_INIT
        beep(8, 500, 500, 5);
#endif
        beep(15, 500, 500, 1);
        mpu.CalibrateAccel(20);
        mpu.CalibrateGyro(20);
        i2c_eeprom_write_int16(EEPROM_IMU, mpu.getXAccelOffset());
        i2c_eeprom_write_int16(EEPROM_IMU + 2, mpu.getYAccelOffset());
        i2c_eeprom_write_int16(EEPROM_IMU + 4, mpu.getZAccelOffset());
        i2c_eeprom_write_int16(EEPROM_IMU + 6, mpu.getXGyroOffset());
        i2c_eeprom_write_int16(EEPROM_IMU + 8, mpu.getYGyroOffset());
        i2c_eeprom_write_int16(EEPROM_IMU + 10, mpu.getZGyroOffset());
#ifndef AUTO_INIT
      }
#endif
      beep(18, 50, 50, 6);
      mpu.PrintActiveOffsets();
    }
    // turn on the DMP, now that it's ready
    Serial.println(F("- Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("- Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("- DMP ready! Waiting for the first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("- DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  delay(10);
  read_IMU();
  exceptions = aaReal.z < 0;
  originalYawDirection = ypr[0];
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void imuExample() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  read_IMU();
  print6Axis();
}
