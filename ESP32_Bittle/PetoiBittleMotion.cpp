#include "PetoiBittleMotion.h"
#include "PetoiESP32PWMServo.h"
#include "BiboardPinDef.h"

#define DOF 12
#define WALKING_DOF 8

int angleArray1[DOF] = {0,0,0,0,0,0,0,0,0,0,0,0};
int angleArray2[DOF] = {145,145,145,145,145,145,145,145,145,145,145,145};

int midAngleShift[DOF] = {0, 100, 100, 190, 235, 0, 0, 100, 190, 100, 280, 0};
int caliAngle[DOF] = {0,0,0,0,0,0,0,0,0,0,0,0};

ESP32PWMServo S0 = ESP32PWMServo(PETOI_P1S);
ESP32PWMServo S1 = ESP32PWMServo(PETOI_P1S);
ESP32PWMServo S2 = ESP32PWMServo(PETOI_P1S);
ESP32PWMServo S3 = ESP32PWMServo(PETOI_P1S);
ESP32PWMServo S4 = ESP32PWMServo(PETOI_P1S);
ESP32PWMServo S5 = ESP32PWMServo(PETOI_P1S);
ESP32PWMServo S6 = ESP32PWMServo(PETOI_P1S);
ESP32PWMServo S7 = ESP32PWMServo(PETOI_P1S);
ESP32PWMServo S8 = ESP32PWMServo(PETOI_P1S);
ESP32PWMServo S9 = ESP32PWMServo(PETOI_P1S);
ESP32PWMServo S10 = ESP32PWMServo(PETOI_P1S);
ESP32PWMServo S11 = ESP32PWMServo(PETOI_P1S);

void servoInit(){
  S0.attach(PWM_pin[0]);
  S1.attach(PWM_pin[1]);
  S2.attach(PWM_pin[2]);
  S3.attach(PWM_pin[3]);
  S4.attach(PWM_pin[4]);
  S5.attach(PWM_pin[5]);
  S6.attach(PWM_pin[6]);
  S7.attach(PWM_pin[7]);
  S8.attach(PWM_pin[8]);
  S9.attach(PWM_pin[9]);
  S10.attach(PWM_pin[10]);
  S11.attach(PWM_pin[11]);
}

void servoTeamExec(int *angleArray){

    S0.write(angleArray[0]);
    S1.write(angleArray[1]);
    S2.write(angleArray[2]);
    S3.write(angleArray[3]);
    S4.write(angleArray[4]);
    S5.write(angleArray[5]);
    S6.write(angleArray[6]);
    S7.write(angleArray[7]);
    S8.write(angleArray[8]);
    S9.write(angleArray[9]);
    S10.write(angleArray[10]);
    S11.write(angleArray[11]);

}

void testRun(){

    int motionArray[DOF];

    servoInit();

    // angle transform
    for(int i = 0; i < DOF; i++){
        motionArray[i] = angleArray1[i] + midAngleShift[i] + caliAngle[i];
        if(motionArray[i] <= 0 && motionArray[i] >= 290){
            motionArray[i] = 145;
        }
    }

    servoTeamExec(angleArray2);
    delay(1000);
    servoTeamExec(motionArray);
    delay(10000);
}

// this programme is for test
void servoTeamMiddlePoint(){

    servoTeamExec(angleArray2);

/*     S0.writeMicroseconds(1500);
    S1.writeMicroseconds(1500);
    S2.writeMicroseconds(1500);
    S3.writeMicroseconds(1500);
    S4.writeMicroseconds(1500);
    S5.writeMicroseconds(1500);
    S6.writeMicroseconds(1500);
    S7.writeMicroseconds(1500);
    S8.writeMicroseconds(1500);
    S9.writeMicroseconds(1500);
    S10.writeMicroseconds(1500);
    S11.writeMicroseconds(1500); */

}
