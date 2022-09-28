/* credit to:
   https://github.com/deviceai/arduino_sensors/blob/master/IRremote/ir_test_code.ino

   Go to the library manager of Arduino IDE (instruction: https://www.arduino.cc/en/Guide/Libraries),
   search and install IRremote.
   Rongzhong Li
   August 2017

  The MIT License

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#define SHORT_ENCODING// activating this line will use a shorter encoding of the HEX values
// the original value is formatted as address  code complement
//                                   2Bytes  1Byte   1Byte

#include "IRremote.h"
//you need to change the timer to //#define IR_USE_TIMER1   // tx = pin 9 in IRremoteBoardDefs.h

//on BiBoard
#define IR_RECEIVE 23 // Signal Pin of IR receiver to Arduino Pin 4
#define BUZZER 25 // the PWM pin the ACTIVE buzzer is attached to

//on Arduino Pro mini test tool
// #define IR_RECEIVE 4 // Signal Pin of IR receiver to Arduino Pin 4
// #define BUZZER 3 // the PWM pin the ACTIVE buzzer is attached to
#define BASE_PITCH 1046.50

#define PT(s) Serial.print(s)  //makes life easier
#define PTL(s) Serial.println(s)
#define PTF(s) Serial.print(F(s))//trade flash memory for dynamic memory with F() function
#define PTLF(s) Serial.println(F(s))

/*-----( Declare objects )-----*/
IRrecv irrecv(IR_RECEIVE);     // create instance of 'irrecv'
decode_results results;      // create instance of 'decode_results'

byte melodyIRpass[] = {
  //  6 6 5 3 2 1 3 2 1 6 5
  17, 15, 12, 10, 8,// 12, 10, 8, 5, 3,
  6, 16, 8,  8,  2,// 6, 16, 8, 8, 2
};

void beep(float note, float duration = 50, int pause = 0, byte repeat = 1 ) {
  for (byte r = 0; r < repeat; r++) {
    if (note)
      tone(BUZZER, BASE_PITCH * pow(1.05946, note), duration);
    else
      tone(BUZZER, 0, duration);
    //      delay(duration);
    delay(pause);
  }
}

void playMelody(byte m[], int len) {
  for (int i = 0; i < len; i++) {
    if (m[i])
      tone(BUZZER, 1046.50 * pow(1.05946, m[i]), // C
           1000.0 / m[len + i]);
    //    else
    delay(1000.0 / m[len + i]);
  }
}

//for QA
int IRkey() // takes action based on IR code received
// describing Remote IR codes.
{
#ifndef SHORT_ENCODING
  switch (results.value) {
    //IR signal    key on IR remote           //key mapping
    case 0xFFA25D: /*PTLF(" CH-");   */       return (2);
    case 0xFF629D: /*PTLF(" CH");  */         return (3);
    case 0xFFE21D: /*PTLF(" CH+"); */         return (4);

    case 0xFF22DD: /*PTLF(" |<<"); */         return (5);
    case 0xFF02FD: /*PTLF(" >>|"); */         return (6);
    case 0xFFC23D: /*PTLF(" >||"); */         return (7);

    case 0xFFE01F: /*PTLF(" -");   */         return (8);
    case 0xFFA857: /*PTLF(" +");  */          return (9);
    case 0xFF906F: /*PTLF(" EQ"); */          return (10);

    case 0xFF6897: /*PTLF(" 0");  */          return (11);
    case 0xFF9867: /*PTLF(" 100+"); */        return (12);
    case 0xFFB04F: /*PTLF(" 200+"); */        return (13);

    case 0xFF30CF: /*PTLF(" 1");  */          return (14);
    case 0xFF18E7: /*PTLF(" 2");  */          return (15);
    case 0xFF7A85: /*PTLF(" 3");  */          return (16);

    case 0xFF10EF: /*PTLF(" 4");  */          return (17);
    case 0xFF38C7: /*PTLF(" 5");  */          return (18);
    case 0xFF5AA5: /*PTLF(" 6");  */          return (19);

    case 0xFF42BD: /*PTLF(" 7");  */          return (20);
    case 0xFF4AB5: /*PTLF(" 8");  */          return (21);
    case 0xFF52AD: /*PTLF(" 9");  */          return (22);

    case 0xFFFFFFFF: return (""); //Serial.println(" REPEAT");
#else
  uint8_t trimmed = (results.value >> 8);
  switch (trimmed) {
    //IR signal    key on IR remote           //key mapping
    case 0xA2: /*PTLF(" CH-");   */       return (2);
    case 0x62: /*PTLF(" CH");  */         return (3);
    case 0xE2: /*PTLF(" CH+"); */         return (4);

    case 0x22: /*PTLF(" |<<"); */         return (5);
    case 0x02: /*PTLF(" >>|"); */         return (6);
    case 0xC2: /*PTLF(" >||"); */         return (7);

    case 0xE0: /*PTLF(" -");   */         return (8);
    case 0xA8: /*PTLF(" +");  */          return (9);
    case 0x90: /*PTLF(" EQ"); */          return (10);

    case 0x68: /*PTLF(" 0");  */          return (11);
    case 0x98: /*PTLF(" 100+"); */        return (12);
    case 0xB0: /*PTLF(" 200+"); */        return (13);

    case 0x30: /*PTLF(" 1");  */          return (14);
    case 0x18: /*PTLF(" 2");  */          return (15);
    case 0x7A: /*PTLF(" 3");  */          return (16);

    case 0x10: /*PTLF(" 4");  */          return (17);
    case 0x38: /*PTLF(" 5");  */          return (18);
    case 0x5A: /*PTLF(" 6");  */          return (19);

    case 0x42: /*PTLF(" 7");  */          return (20);
    case 0x4A: /*PTLF(" 8");  */          return (21);
    case 0x52: /*PTLF(" 9");  */          return (22);

    case 0xFF: return (-1); //Serial.println(" REPEAT");
#endif
    default: {
        //Serial.println(results.value, HEX);
      }
      return (0);                      //Serial.println("null");
  }// End Case
  //delay(100); // Do not get immediate repeat //no need because the main loop is slow
}

bool testIR() {
  long start = millis();
  long timer = start;
  int count = 0, right = 0;
  int current = 0;
  int previous = 10;
  bool pass = false;
  while (1) {
    if (count == 10 || millis() - start > 1000 || right > 5) { //test for 1 second
      PT(right);
      PT("/");
      PT(count);
      PTL(" good");
      if (right > 5) {
        return true;
      }
      else {
        return false;
      }
    }

    if (millis() - timer > 11 && irrecv.decode(&results)) {
      timer = millis();
      current = IRkey();
      irrecv.resume(); // receive the next value
      if (current == 0)
        continue;

      if (current == 11)
        previous = 10;
      if (current - previous == 1)  //if the reading is continuous, add one to right
        right++;
      PT("count"); PT(count); PT("\tprevious "); PT(previous); PT("\tcurrent "); PT(current); PT("\tright "); PTL(right);
      previous = (current == 20) ? 10 : current;
      count++;
      //      beep(current, 10);
    }
  }
}

void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  pinMode(BUZZER, OUTPUT);
  Serial.begin(115200);
  Serial.println("IR Receiver Button Decode");
  irrecv.enableIRIn(); // Start the receiver
  PTL(sizeof(melodyIRpass) / 2);
}/*--(end setup )---*/



void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
  if (testIR()) {
    PTL("Pass");
    playMelody(melodyIRpass, sizeof(melodyIRpass) / 2);
  }
  else {
    PTL("Fail");
    beep(8, 50);
  }
}/* --(end main loop )-- */

/*-----( Function )-----*/
