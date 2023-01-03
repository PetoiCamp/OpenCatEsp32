/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These drivers use I2C to communicate, 2 pins are required to
  interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

/*
  Revised for convenient writing angles and setting different servos with different parameters.

  Rongzhong Li
  Feb.2, 2022
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//#include "src/Adafruit-PWM-Servo-Driver-Library/Adafruit_PWMServoDriver.h"

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

#define SERVO_FREQ 320
#define ANGLE_RANGE 290
#define PWM_NUM 16
// our servo # counter
uint8_t servonum = 0;
long oscillatorFreq = 25000000;

class ServoModel {
  public:
    ServoModel() {};
    ServoModel(int range, int freq, int min, int max): angleRange(range), frequency(freq), minPulse(min), maxPulse(max) {}
    int getAngleRange() {
      return angleRange;
    }
    int getFrequency() {
      return frequency;
    }
    int getMinPulse() {
      return minPulse;
    }
    int getMaxPulse() {
      return maxPulse;
    };
  private:
    int angleRange;
    int frequency;
    int minPulse;
    int maxPulse;
    float k_us2pulse;
    float k_angle2pulse;
};

byte PWM_pin[PWM_NUM] = {12, 11, 3, 4,
                         13, 10, 5, 2,
                         14, 9, 6, 1,
                         15, 8, 7, 0
                        };

#define KNEE P1S//P2K
#define REGULAR P1S

enum ServoModel_t {
  G41 = 0,
  P1S,
  P2K
};

ServoModel_t servoModelList[] = {
  REGULAR, REGULAR, REGULAR, REGULAR,
  REGULAR, REGULAR, REGULAR, REGULAR,
  REGULAR, REGULAR, REGULAR, REGULAR,
  KNEE, KNEE, KNEE, KNEE
};

#define DOF 16

class Petoi_PWMServoDriver: public Adafruit_PWMServoDriver {
  private:
    const static int SERVOMIN = 150; // This is the 'minimum' pulse length count (out of 4096)
    const static int SERVOMAX = 600; // This is the 'maximum' pulse length count (out of 4096)
    const static int USMIN = 600; // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
    const static int USMAX = 2400; // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
    const static int DEFAULT_FREQ = 50; // Analog servos run at ~50 Hz updates
  public:
    int servoPWMFreq;
    float k_us2pulse;
    float b_offset;
    float *k_angle2pulse;

    using Adafruit_PWMServoDriver::Adafruit_PWMServoDriver;

    void setup(ServoModel_t *servoModelList, int8_t numServo = PWM_NUM, long oscillatorFreq = 25000000) {
      servoPWMFreq = SERVO_FREQ;
      setOscillatorFrequency(oscillatorFreq);
      setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
      delay(10);
      //  pulse = angle * k_angle2pulse + b_offset
      k_us2pulse = microsecondsToPulse(1);
      k_angle2pulse = new float[numServo];
      b_offset = servoPWMFreq / DEFAULT_FREQ * SERVOMIN;

      ServoModel servoG41   (180,    SERVO_FREQ,      500,      2500);
      ServoModel servoP1S   (290,    SERVO_FREQ,      500,      2500);//1s/4 = 250ms 250ms/2500us=100Hz
      ServoModel servoP1L   (220,    SERVO_FREQ,      500,      2500);
      for (int s = 0; s < numServo; s++) {
        ServoModel *model;
        switch (servoModelList[s]) {
          case G41:
            model = &servoG41;
            break;
          case P1S:
            model = &servoP1S;
            break;
          case P2K:
            model = &servoP1L;
            break;
        }
        Serial.println(s);
        calcAngleToPulseFactor(s, model->getAngleRange(), model->getMinPulse(), model->getMaxPulse());
//        zeroPosition[s] = model->getAngleRange() / 2 + float(middleShift[s])  * rotationDirection[s];
//        calibratedZeroPosition[s] = zeroPosition[s] + float(servoCalib[s])  * rotationDirection[s];
      }
      Serial.println();
    }

    float microsecondsToPulse(uint16_t microseconds) {
      float pulse = microseconds;
      float pulselength;
      pulselength = 1000000; // 1,000,000 us per second
      // Read prescale
      uint16_t prescale = readPrescale();
      // Calculate the pulse for PWM based on Equation 1 from the datasheet section 7.3.5
      prescale += 1;
      pulselength *= prescale;
      pulselength /= oscillatorFreq;
      pulse /= pulselength;
      return pulse;
    }
    void calcAngleToPulseFactor(uint8_t servoNum, int angleRange = ANGLE_RANGE, int usMin = USMIN, int usMax = USMAX) {
      //  angle *  ______________k_angle2pulse______________ + _______________b_offset_______________
      // (angle * (usMax - usMin) / angleRange) * k_us2pulse + servoPWMFreq / DEFAULT_FREQ * SERVOMIN;
      k_angle2pulse[PWM_pin[servoNum]] = (usMax - usMin) / angleRange * k_us2pulse;
      //      Serial.print(servoNum); Serial.print('\t');
      //      Serial.println(k_angle2pulse[PWM_pin[servoNum]]);
    }
    void writeAngle(uint8_t servoNum, float angle) {
      setPWM(PWM_pin[servoNum], 0, k_angle2pulse[PWM_pin[servoNum]]*angle + b_offset);
      //      Serial.print(PWM_pin[servoNum]); Serial.print('\t');
      //      Serial.println(k_angle2pulse[servoNum]*angle + b_offset);
    }
    void shutServos(byte s0 = 0, byte s1 = PWM_NUM) {
      //todo:
      //shutServos() all shut
      //shutServos(x) shut x
      //shutServos(a, b) shut a to b
      for (byte s = s0; s < s1; s++) {
        setPWM(s, 0, 4096);
      }
    }
};



// called this way, it uses the default address 0x40
Petoi_PWMServoDriver pwm = Petoi_PWMServoDriver();
// you can also call it with a different address you want
//Petoi_PWMServoDriver pwm = Petoi_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Petoi_PWMServoDriver pwm = Petoi_PWMServoDriver(0x40, Wire);


void setup() {
  Serial.begin(115200);
  Serial.println("Set up PCA9685 PWM servo driver...");
  pwm.begin();
  /*
     In theory the internal oscillator (clock) is 25MHz but it really isn't
     that precise. You can 'calibrate' this by tweaking this number until
     you get the PWM update frequency you're expecting!
     The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
     is used for calculating things like writeMicroseconds()
     Analog servos run at ~50 Hz updates, It is important to use an
     oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
     1) Attach the oscilloscope to one of the PWM signal pins and ground on
        the I2C PCA9685 chip you are setting the value for.
     2) Adjust setoscillatorFrequency() until the PWM update frequency is the
        expected value (50Hz for most ESCs)
     Setting the value here is specific to each individual I2C PCA9685 chip and
     affects the calculations for the PWM update frequency.
     Failure to correctly set the int.osc value will cause unexpected PWM results
  */
  pwm.setup(servoModelList);
}

void loop() {
  for (int an = -180; an < 180; an += 90) { //0,90,180,270,
    //    for (servonum = 0; servonum < 16; servonum++)
    pwm.writeAngle(servonum, an);
    Serial.println(an);
    //    calibratedPWM(servonum, an);
    delay(1000);
  }
  for (int an = 180; an > -180; an -= 90) {//180,90
    //    for (servonum = 0; servonum < 16; servonum++)
    pwm.writeAngle(servonum, an);
    Serial.println(an);
    //    calibratedPWM(servonum, an);
    delay(1000);
  }
  //  pwm.shutServos();
  //  delay(3000);
}
