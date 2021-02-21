#ifndef _PETOIMPU6050DMP_H
#define _PETOIMPU6050DMP_H

void dmpDataReady();
void mpu_setup();
void getDMPRawResult();
void getDMPReadableYawPitchRaw(float *ypr);

void getIMUDataOfYawPitchRaw();



#endif
