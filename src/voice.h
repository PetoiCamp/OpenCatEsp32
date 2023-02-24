#define SERIAL2_BAUD_RATE 9600
#define MAX_CUSTOMIZED_CMD 10

String customizedCmdList[] = {
  "kbalance", "ksit", "d", "m0 80 0 -80 0 0", "khi", "krc", "kvtF", "kck", "kjy", "kstr"  //define up to 10 customized commands.
};
int listLength = 0;

void voiceSetup() {
  PTLF("Init voice");
  Serial2.begin(SERIAL2_BAUD_RATE);
  Serial2.setTimeout(5);
  listLength = min(int(sizeof(customizedCmdList) / sizeof(customizedCmdList[0])), MAX_CUSTOMIZED_CMD);
  PTLF("Number of customized voice commands on the main board: ");
  PTL(listLength);
}

void read_voice() {
  // put your main code here, to run repeatedly:
  if (Serial2.available()) {
    String raw = Serial2.readStringUntil('\n');
    PTL(raw);
    byte index = (byte)raw[2];  //interpret the 3rd byte as integer
    int shift = -1;
    if (index > 10) {
      if (index < 21) {  //11 ~ 20 are customized commands, and their indexes should be shifted by 11
        index -= 11;
        PT(index);
        PT(' ');
        if (index < listLength) {
          raw = customizedCmdList[index];
          token = raw[0];
          shift = 1;
        } else {
          PTLF("Undefined!");
        }
      } else if (index < 61) {  //21 ~ 60 are preset commands, and their indexes should be shifted by 21.
                                //But we don't need to use their indexes.
        token = T_SKILL;
        shift = 3;
      }
      if (shift > 0)
        tQueue->push_back(new Task(token, raw.c_str() + shift, 2000));
      tQueue->push_back(new Task('p', ""));
    }
  }
}