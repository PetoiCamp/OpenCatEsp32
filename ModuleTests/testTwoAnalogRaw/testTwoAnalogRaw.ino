
#define BUZZER 5

#define MAX_READING 4096

#define IN1 34
#define IN2 35

void beep(int8_t note, float duration = 10, int pause = 0, byte repeat = 1) {
  if (note == 0) {  //rest note
    analogWrite(BUZZER, 0);
    delay(duration);
    return;
  }
  int freq = 220 * pow(1.059463, note - 1);  // 1.059463 comes from https://en.wikipedia.org/wiki/Twelfth_root_of_two
  float period = 1000000.0 / freq;
  for (byte r = 0; r < repeat; r++) {
    for (float t = 0; t < duration * 1000; t += period) {
      analogWrite(BUZZER, 150);  // Almost any value can be used except 0 and 255
      // experiment to get the best tone
      delayMicroseconds(period / 2);  // rise for half period
      analogWrite(BUZZER, 0);         // 0 turns it off
      delayMicroseconds(period / 2);  // down for half period
    }
    delay(pause);
  }
}

float mean(int *a, int n) {
  float sum = 0;
  for (int i = 0; i < n; i++)
    sum += a[i];
  return sum / n;
}

float sDev(int *a, float m, int n) {
  float sum = 0;
  for (int i = 0; i < n; i++)
    sum += (a[i] - m) * (a[i] - m);
  return sqrt(sum / n);
}
int offsetA_B1 = 0, offsetA_B2 = 0;
long meanLight1 = 0, meanLight2 = 0;
int correctedReading(int light) {
  return light - (offsetA_B2 + (offsetA_B1 - offsetA_B2) / (meanLight1 - meanLight2) * (light - meanLight2));
  //  resetReading();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(2);
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  delay(100);
  Serial.println("start");
}

void loop() {
  // put your main code here, to run repeatedly:
  //stats();
  //  sensorConnectedQ(READING_COUNT);

  Serial.print(analogRead(IN1));
  Serial.print('\t');
  Serial.print(analogRead(IN2));
  Serial.print('\t');
  Serial.print(0);
  Serial.print('\t');
  Serial.println(MAX_READING);
  //  con ? beep(10, 200) : beep(20, 200);
}
