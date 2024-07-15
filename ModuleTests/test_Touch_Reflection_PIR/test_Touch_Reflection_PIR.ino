//connects to the digital Grove socket with I34 and I35
#define DIGITAL1 34
#define DIGITAL2 35

int val;
void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(2);
  pinMode(DIGITAL1, INPUT);
  pinMode(DIGITAL2, INPUT);
}
 
void loop()
{
  val = digitalRead(DIGITAL1);
  Serial.println(val); //prints 1 if there's human or touch event
                         //prints 0 if there's reflection within 4-16mm.
  delay(100);
}
