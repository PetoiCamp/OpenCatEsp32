#include "PetoiBittleMotion.h"
#include "PetoiESP32PWMServo.h"
#include "BiboardPinDef.h"
#include "Instincts.h"

#define DOF 12
#define WALKING_DOF 8
//
//int angleArray1[DOF] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//int angleArray2[DOF] = {145, 145, 145, 145, 145, 145, 145, 145, 145, 145, 145, 145};
//
//int midAngleShift[DOF] = {0, 100, 100, 190, 235, 0, 0, 100, 190, 100, 280, 0};

int caliAngle[DOF] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

ESP32PWMServo *servo = new ESP32PWMServo[DOF];
int middleAngle[] = {145, 145, 145, 145, 145, 145, 145, 145, 145, 145, 145, 145};

int middleShifts[] = {0, 15, 0, 0,
                      //-45, -45, -45, -45,
                      45, 45, -45, -45,
                      -75, -75, -75, -75
                     };
int rotationDirections[] = {1, -1, 1, 1,
                            //1, -1, 1, -1,
                            1, -1, -1, 1,
                            -1, 1, 1, -1
                           };
void servoInit() {
  for (byte s = 0; s < 12; s++) {
    servo[s] = ESP32PWMServo(PETOI_P1S);
    servo[s].attach(PWM_pin[s], s);
    servo[s].write(0 );
    servo[s].write(middleAngle[s] + middleShifts[s] * rotationDirections[s] );
    Serial.print(middleAngle[s]);
    Serial.print("\t");
  }
  Serial.println();

  while (!Serial.available());
  char resetJointCalibrationQ = Serial.read();
}

float angle = 0;
float aStep = 0.5;
int joint = 16;
void testRun() {
  int motionArray[DOF];

  servoInit();
  while (0) {
    if (Serial.available() > 0) {
      String inBuffer = Serial.readStringUntil('\n');
      joint = atoi(inBuffer.c_str());

    }
    if (joint < 12) {
      //servo[joint].write(( angle));
      servo[joint].write(middleAngle[joint] + rotationDirections[joint] * (middleShifts[joint] + angle));
      Serial.println(angle);
    }
    else {
      for (byte s = 0; s < 12; s++) {
        servo[s].write(middleAngle[s] + (s > 7 ? -1 : 1)*rotationDirections[s] * (middleShifts[s] + angle));
      }
      //Serial.println();
    }

    delay(5);
    angle += aStep;
    if (angle > 100 || angle < -100)
      aStep *= -1;

  }
  int period = wkF[0];
  int t = 0;
  while (0) {
    for (int j = 0; j < 8; j++)
      servo[j + 4].write(middleAngle[j + 4] + rotationDirections[j + 4] * (middleShifts[j + 4] + wkF[4 + t * 8 + j]));
    delay(10);
    t++;
    if (t == period)t = 0;
  }
}
