
int sensorValue1;  // variable to store the value coming from the sensor
int sensorValue2;  // variable to store the value coming from the sensor
int sensorValue3;  // variable to store the value coming from the sensor
int sensorValue4;  // variable to store the value coming from the sensor

int sensorPin1 = 36;    // select the input pin for the potentiometer
int sensorPin2 = 39;    // select the input pin for the potentiometer


int sensorPin3 = 34;    // select the input pin for the potentiometer
int sensorPin4 = 35;    // select the input pin for the potentiometer


void setup() {
  Serial.begin(115200);
}
void loop() {

  sensorValue1 = analogRead(sensorPin1);
  sensorValue2 = analogRead(sensorPin2);
  sensorValue3 = analogRead(sensorPin3);
  sensorValue4 = analogRead(sensorPin4);

  Serial.print(sensorValue1);
  Serial.print("\t");
  Serial.print(sensorValue2);
  Serial.print("\t");
  Serial.print(sensorValue3);
  Serial.print("\t");
  Serial.println(sensorValue4);


  //47k  330k+51K
}
