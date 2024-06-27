#define ANALOG1 34
#define ANALOG2 35
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(2);
  pinMode(ANALOG1, INPUT);
  pinMode(ANALOG2, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(digitalRead(ANALOG1));
  Serial.print('\t');
  Serial.print(digitalRead(ANALOG2));
  Serial.print('\t');

  //  Serial.print('\t');
  //  Serial.print(0);
  //  Serial.print('\t');
  //  Serial.print(4096);

  Serial.println();
  delay(2);
}
