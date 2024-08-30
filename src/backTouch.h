int8_t touchPadMap[] = { 0, 2, 3, 1 };
String touchLocation[] = { "Front Left", "Front Right", "Center", "Back" };

void backTouchSetup() {
  pinMode(BACKTOUCH_PIN, INPUT);
}
int8_t prevTouch = -1;
long lastTouchEvent;
void read_backTouch() {
  // put your main code here, to run repeatedly:
  //stats();
  //  sensorConnectedQ(READING_COUNT);
  int touchReading = analogRead(BACKTOUCH_PIN);
  if (touchReading < 3000) {
    int8_t touchPadIdx = touchPadMap[touchReading / 600];
    if (prevTouch != touchPadIdx) {  //|| millis() - lastTouchEvent > 500) {  // if the touch is different or a repeatitive touch interval is longer than 0.5 second
      prevTouch = touchPadIdx;
      PTHL("Touched:", touchLocation[touchPadIdx]);
      beep(touchPadIdx * 2 + 15, 100);
      switch (touchPadIdx) {
        case 0:
          {
#ifdef ROBOT_ARM
            tQueue->addTask('m', "0,45,1,45,2,0");
#else
            tQueue->addTask('m', "0,120", 500);
            tQueue->addTask('m', "0,0,1,40");
#endif
            break;
          }
        case 1:
          {
#ifdef ROBOT_ARM
            tQueue->addTask('m', "0,-45,1,45,2,0", 1000);
#else
            tQueue->addTask('m', "0,-120", 500);
            tQueue->addTask('m', "0,0,1,40");
#endif
            break;
          }
        case 2:
          {
#ifdef ROBOT_ARM
            tQueue->addTask('m', "0,0,1,0,2,0", 1000);
#else
            tQueue->addTask('k', "str", 1000);
#endif
            break;
          }
        case 3:
          {
#ifdef ROBOT_ARM
            tQueue->addTask('m', "1,60,2,120", 1000);
#else
            tQueue->addTask('k', "scrh", 0);
#endif

#ifdef NYBBLE
            tQueue->addTask('m', "2,-30,2,30,2,0", 500);
#endif
            break;
          }
      }
      // tQueue->addTask('k', "up");
      lastTouchEvent = millis();
    }
  }
}
