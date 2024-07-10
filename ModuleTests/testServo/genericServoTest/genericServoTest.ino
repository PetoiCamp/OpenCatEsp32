#include "src/servo.h"

int amplitude = 5;
int offset = 5;
int pos = 0;
char token = 'c';

int rest[] = {
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  65,
  65,
  65,
  65,
  -65,
  -65,
  -65,
  -65,
};
int stand[] = {
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  30,
  30,
  30,
  30,
  30,
  30,
  30,
  30,
};

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.setTimeout(2);
  while (!Serial)
    ;
  while (Serial.available() && Serial.read())
    ;
  servoSetup();
  Serial.println("Ready!");
  setAllJoint(servoCalib, 8);
  delay(2000);
  // allRotate();
}
void testAllRotate() {
  for (int a = -amplitude; a < amplitude; a += 2)
    for (byte i = 0; i < 16; i++) {
      setJoint(i, a);
      delay(10);
    }
  for (int a = amplitude; a > -amplitude; a -= 2)
    for (byte i = 0; i < 16; i++) {
      setJoint(i, a);
      delay(10);
    }
}
void testGait() {
  for (pos = -amplitude; pos <= +amplitude; pos += 1) {
    for (byte s = 8; s < DOF; s++)
      setJoint(s, pos * (s % 4 / 2 ? 1 : -1) * (s % 2 ? -1 : 1)
                    + stand[s] + offset);  // tell servo to go to position in variable 'pos'
    delay(10);                             // waits 15ms for the servo to reach the position
  }
  for (pos = +amplitude; pos >= -amplitude; pos -= 1) {
    for (byte s = 8; s < DOF; s++)
      setJoint(s, pos * (s % 4 / 2 ? 1 : -1) * (s % 2 ? -1 : 1)
                    + stand[s] + offset);  // tell servo to go to position in variable 'pos'
    delay(10);                             // waits 15ms for the servo to reach the position
  }
}
void loop() {
  if (Serial.available()) {
    token = Serial.read();
    Serial.println(token);
    if (token == 'c')  //move the joints to calibration posture
      for (byte s = 8; s < DOF; s++)
        setJoint(s, 0);
    else if (token == 'd') {  //move the joints to rest posture, and turn off all the servos
      for (byte s = 8; s < DOF; s++)
        setJoint(s, rest[s]);
      delay(2000);
      shutServos();
    } else if (token == 't')  //move the joints to stand posture
      for (byte s = 8; s < DOF; s++)
        setJoint(s, stand[s]);
  }
  if (token == 'k') {  //make the robot walk (simple walk)
    if (Serial.available()) {
      offset = Serial.parseInt();
      if (Serial.available())
        amplitude = Serial.parseInt();
      Serial.print("Offset: ");
      Serial.print(offset);
      Serial.print("\tAmplitude: ");
      Serial.println(amplitude);
    }
    testGait();
  } else if (token == 'a')  //rotate all the servos around the zero position
    testAllRotate();
}