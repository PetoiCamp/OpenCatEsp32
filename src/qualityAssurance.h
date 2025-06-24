float mean(float *a, int n) {
  float sum = 0;
  for (int i = 0; i < n; i++)
    sum += a[i];
  return sum / n;
}

float sDev(float *a, float m, int n) {
  float sum = 0;
  for (int i = 0; i < n; i++)
    sum += (a[i] - m) * (a[i] - m);
  return sqrt(sum / n);
}
byte DcDcGood[] = { 12, 19,
                    4, 4 };
byte DcDcBad[] = { 19, 16, 12,
                   16, 16, 16 };
byte imuGood[] = { 12, 16, 19,
                   4, 4, 4 };
byte imuBad[] = { 25, 20, 16, 11, 9,
                  16, 16, 16, 16, 16 };  // no intented IMU detected!
byte imuBad1[] = { 19, 17, 16, 14, 12,
                   16, 16, 16, 16, 16 };  // too large error

#define IMU_TEST_TRIGGER 0.5
#define MEAN_THRESHOLD 0.5
#define STD_THRESHOLD 0.2

#ifdef GYRO_PIN
void testIMU() {
  bool intendedIMU = false;
#ifdef IMU_ICM42670
  if (icmQ)
    intendedIMU = true;
#endif
#ifdef IMU_MPU6050
  if (mpuQ)
    intendedIMU = true;
#endif
  if (!intendedIMU) {
    PTL("\nNo intented IMU detected!");
    while (1) {
      playMelody(imuBad, sizeof(imuBad) / 2);
      delay(500);
    }
  }
  PTL("\nIMU test: both mean and standard deviation should be small on Pitch and Roll axis\n");
  // delay(1000);
  int count = 100;
  float **history = new float *[2];
  for (int a = 0; a < 2; a++)
    history[a] = new float[count];
  while (fabs(ypr[1]) > IMU_TEST_TRIGGER || fabs(ypr[2]) > IMU_TEST_TRIGGER) {  // the IMU should converge to a stable state before the statistics test
    delay(IMU_PERIOD);
    print6Axis();
  }
  PTL("Test");
  for (int t = 0; t < count; t++) {
    while (!imuUpdated)  // lock to prevent reading imu when it's still calculating
      delay(1);
    print6Axis();
    for (int a = 0; a < 2; a++)
      history[a][t] = ypr[a + 1];
    imuUpdated = false;
  }
  String axis[] = { "Pitch ", "Roll  " };
  for (int a = 0; a < 2; a++) {
    float m = mean(history[a], count);
    float dev = sDev(history[a], m, count);
    PT(axis[a]);
    PT("(in degrees)\tmean: ");
    PT(m);
    PT("\tstandard deviation: ");
    PT(dev);
    if (fabs(m) > MEAN_THRESHOLD || dev > STD_THRESHOLD) {
      PTL("\tFail!");
      while (!Serial.available()) {
        playMelody(imuBad1, sizeof(imuBad1) / 2);
        delay(500);
      }
      while (Serial.available())
        Serial.read();
    } else {
      PTL("\tPass!");
      playMelody(imuGood, sizeof(imuGood) / 2);
    }
  }
  delay(100);
  for (int a = 0; a < 2; a++)
    delete[] history[a];
  delete[] history;
};
#endif

#ifdef IR_PIN
bool testIR() {
  long start = millis();
  long timer = start;
  int count = 0, right = 0;
  int current = 0;
  int previous = 10;
  PTL("\nInfrared test: catch at least 6 consecutive signals\n");
  while (1) {
    if (count == 10 || millis() - start > 1200 || right > 5) {  // test for 1 second
      PT(right);
      PT("/");
      PT(count);
      PTL(" good");
      if (right > 5)
        return true;
      else
        return false;
    }
    if (millis() - timer > 11 && irrecv.decode(&results)) {
      timer = millis();
      current = IRkey();
      irrecv.resume();  // receive the next value
      if (current == 0)
        continue;

      if (current == 11)
        previous = 10;
      if (current - previous == 1)  // if the reading is continuous, add one to right
        right++;
      PT("count");
      PT(count);
      PT("\tprevious ");
      PT(previous);
      PT("\tcurrent ");
      PT(current);
      PT("\tright ");
      PTL(right);
      previous = (current == 20) ? 10 : current;
      count++;
      //      beep(current, 10);
    }
  }
}
#endif
void testDcDc() {
  const int adcPin = 34;
  const float R1 = 30000.0;
  const float R2 = 51000.0;
  const int adcMaxValue = 4095;
  const float vRef = 3.3;
  while (1) {
    int adcValue = analogRead(adcPin);
    float voltageMeasured = (adcValue / float(adcMaxValue)) * vRef;
    float voltageInput = voltageMeasured * (R1 + R2) / R2;
    if (voltageInput < 5.2) {
      PTL("Wrong DcDc output!");
      playMelody(DcDcBad, sizeof(DcDcBad) / 2);
      delay(500);
    } else {
      PTL("\tDcDc Pass!");
      playMelody(DcDcGood, sizeof(DcDcGood) / 2);
      break;
    }
  }
}
void QA() {
  if (newBoard) {
#ifdef I2C_EEPROM_ADDRESS
    i2c_eeprom_write_byte(EEPROM_BOOTUP_SOUND_STATE, 1);
#else
    config.putBool("bootSndState", 1);
#endif
#ifndef AUTO_INIT
    PTL("Run factory quality assurance program? (Y/n)");
    char choice = getUserInputChar(5);  // auto skip in 5 seconds
    PTL(choice);
    if (choice == 'Y' || choice == 'y')
#endif
    {
#ifdef VOLTAGE
      // testDcDc(); //unnecessary
#endif
#ifdef GYRO_PIN
      testIMU();
#endif
      // tests...
      PTL("\nServo test: all servos should rotate and in sync\n");
      loadBySkillName("ts");  // test EEPROM
      while (!Serial.available()) {
        skill->perform();
#ifdef IR_PIN
        if (testIR()) {
#endif
          PTL("Pass");
          break;
#ifdef IR_PIN
        } else {
          PTL("Fail");
          beep(8, 50);
        }
#endif
      }
      while (Serial.available())
        Serial.read();
    }
#ifdef I2C_EEPROM_ADDRESS
    i2c_eeprom_write_byte(EEPROM_BIRTHMARK_ADDRESS, BIRTHMARK);  // finish the test and mark the board as initialized
#else
    config.putChar("birthmark", BIRTHMARK);
#endif
    playMelody(melodyIRpass, sizeof(melodyIRpass) / 2);
  }
}
