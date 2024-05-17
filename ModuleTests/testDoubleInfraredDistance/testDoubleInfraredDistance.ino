#define SENSOR1 34
#define SENSOR2 35
#define READING_COUNT 30
#define SENSOR_DISPLACEMENT 3.7
#define MAX_READING 4096
float ratio = MAX_READING / 1024;  // the formula was derived on Arduino Uno, whose max reading is 1024

#define PT(s) Serial.print(s)                      // abbreviate print commands
#define PT_FMT(s, format) Serial.print(s, format)  // abbreviate print commands
#define PTL(s) Serial.println(s)
#define PTF(s) Serial.print(F(s))  // trade flash memory for dynamic memory with F() function
#define PTLF(s) Serial.println(F(s))
#define PTH(head, value) \
  { \
    PT(head); \
    PT('\t'); \
    PTL(value); \
  }

int meanA = 0, meanB = 0, diffA_B = 0, actualDiff = 0, last = 0;

void read_doubleInfraredDistance() {
  int rawL = analogRead(SENSOR2) / ratio;  // the signal is positively correlated to distance, but non-linear.
  int rawR = analogRead(SENSOR1) / ratio;
  float dL = rawL < 30 ? rawL / 4.0 : 200.0 / sqrt(1024 + 24 - rawL);  // a formular to turn the correlation linear for near field measurement.
  float dR = rawR < 30 ? rawR / 4.0 : 200.0 / sqrt(1024 + 24 - rawR);
  PT("rawLeft ");
  PT(rawL);
  PT("\trawRight ");
  PT(rawR);
  PT("\tdL ");
  PT(dL);
  PT("\tdR ");
  PTL(dR);
}

void setup() {
  Serial.begin(115200);
}
void loop() {
  read_doubleInfraredDistance();
}