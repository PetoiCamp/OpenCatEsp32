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
byte mpuGood[] = { 12, 16, 19,
                   4, 4, 4 };
byte mpuBad[] = { 19, 17, 16, 14, 12,
                  16, 16, 16, 16, 16 };

#define MEAN_THRESHOLD 0.2
#define STD_THRESHOLD 0.02

#ifdef GYRO_PIN
void testMPU() {
  PTL("\nIMU test: both mean and standard deviation should be small on Pitch and Roll axis\n");
  delay(1000);
  int count = 100;
  float **history = new float *[2];
  for (int a = 0; a < 2; a++)
    history[a] = new float[count];
  for (int t = 0; t < count; t++) {
    delay(5);
    read_IMU();
    print6Axis();
    for (int a = 0; a < 2; a++)
      history[a][t] = ypr[a + 1];
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
      while (1) {
        playMelody(mpuBad, sizeof(mpuBad) / 2);
        delay(500);
      }
    } else {
      PTL("\tPass!");
      playMelody(mpuGood, sizeof(mpuGood) / 2);
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
  bool pass = false;
  PTL("\nInfrared test: catch at least 6 consecutive signals\n");
  while (1) {
    if (count == 10 || millis() - start > 1200 || right > 5) {  //test for 1 second
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
      if (current - previous == 1)  //if the reading is continuous, add one to right
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
void QA() {
  if (newBoard) {
    i2c_eeprom_write_byte(EEPROM_BOOTUP_SOUND_STATE, 1);
#ifndef AUTO_INIT
    PTL("Run factory quality assurance program? (Y/n)");
    char choice = getUserInputChar();
    PTL(choice);
    if (choice != 'Y' && choice != 'y')
      return;
#endif
#ifdef GYRO_PIN
    testMPU();
#endif
    //tests...
    PTL("\nServo test: all servos should rotate and in sync\n");
    loadBySkillName("ts");  //test EEPROM
    while (1) {
      skill->perform();
#ifdef IR_PIN
      if (testIR()) {
#endif
        PTL("Pass");
        playMelody(melodyIRpass, sizeof(melodyIRpass) / 2);
        break;
#ifdef IR_PIN
      } else {
        PTL("Fail");
        beep(8, 50);
      }
#endif
    }
  }
}
