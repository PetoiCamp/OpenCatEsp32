void setup()
{
Serial.begin(115200);
}
 
void loop()
{
Serial.println("-----------");
Serial.println(esp_random());
Serial.println(random(100));
Serial.println(random(1,100));
delay(1000);
}
