int8_t touchPadMap[] = { 0, 2, 3, 1 };
String touchLocation[] = { "Front Left", "Front Right", "Center", "Back" };

void backTouchSetup() {
  pinMode(BACKTOUCH_PIN, INPUT);
}
int8_t prevTouch = -1;
void read_backTouch() {
  // put your main code here, to run repeatedly:
  //stats();
  //  sensorConnectedQ(READING_COUNT);
  int touchReading = analogRead(BACKTOUCH_PIN);
  if (touchReading < 3000) {
    int8_t touchPadIdx = touchPadMap[touchReading / 600];
    if (prevTouch != touchPadIdx) {
      prevTouch = touchPadIdx;
      PTHL("Touched:", touchLocation[touchPadIdx]);
      beep(touchPadIdx * 2 + 15, 100);
      switch (touchPadIdx) {
        case 0:
          {
            tQueue->addTask('i', "0,120,1,-20", 1000);
            break;
          }
        case 1:
          {
            tQueue->addTask('i', "0,-120,1,-20", 1000);
            break;
          }
        case 2:
          {
            tQueue->addTask('k', "tbl", 1000);
            break;
          }
        case 3:
          {
            tQueue->addTask('k', "buttUp", 0);
            tQueue->addTask('m', "2,-30,2,30,2,0", 500);
            break;
          }
      }
      tQueue->addTask('k', "up");
    }
  }
}
