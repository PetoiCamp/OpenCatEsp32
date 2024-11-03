#define TOUCH_PIN 38
#define BUZZER 2

#define MAX_READING 4096


int8_t touchPadMap[] = { 0, 2, 3, 1 };
void beep(int8_t note, float duration = 10, int pause = 0, byte repeat = 1) {
  if (note == 0) {  //rest note
    digitalWrite(BUZZER, 0);
    delay(duration);
    return;
  }
  int freq = 220 * pow(1.059463, note - 1);  // 1.059463 comes from https://en.wikipedia.org/wiki/Twelfth_root_of_two
  float period = 1000000.0 / freq;
  for (byte r = 0; r < repeat; r++) {
    for (float t = 0; t < duration * 1000; t += period) {
      digitalWrite(BUZZER, 1);  // Almost any value can be used except 0 and 255
      // experiment to get the best tone
      delayMicroseconds(period / 2);  // rise for half period
      digitalWrite(BUZZER, 0);        // 0 turns it off
      delayMicroseconds(period / 2);  // down for half period
    }
    delay(pause);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(2);
  while (!Serial)
    ;
  while (Serial.available()) Serial.read();
  pinMode(BUZZER, OUTPUT);
  pinMode(TOUCH_PIN, INPUT);
  Serial.println("start");
  beep(15, 100);
  beep(17, 100);
  beep(18, 100);
}
int8_t prev = -1;
void loop() {
  // put your main code here, to run repeatedly:
  //stats();
  //  sensorConnectedQ(READING_COUNT);
  int touchReading = analogRead(TOUCH_PIN);
  if (touchReading < 3000) {
    int8_t touchPadIdx = touchPadMap[touchReading / 600];
    if (prev != touchPadIdx) {
      prev = touchPadIdx;
      Serial.println(touchPadIdx);
      beep(touchPadIdx * 2 + 15, 100);
    }
  }
}
