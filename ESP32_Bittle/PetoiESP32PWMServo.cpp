#include "PetoiESP32PWMServo.h"

int ESP32PWMServo::channel_next_free = 0;

ESP32PWMServo::ESP32PWMServo() {
    TAU_USEC = 20000;         // 50Hz standard PWM frequency
    _resetFields();
};

ESP32PWMServo::ESP32PWMServo(servo_signal_type_t servoSignalType){
    if(servoSignalType == DIGITAL_SERVO){
        TAU_USEC = 2500;        // 400Hz High PWM frequency
    }
    else{
        TAU_USEC = 20000;      // 50Hz standard PWM frequency 
    }
    _resetFields();
}

ESP32PWMServo::ESP32PWMServo(digital_servo_model_t servoModel){
    if(servoModel == PETOI_P1S){
        TAU_USEC = 2500;
        _minAngle = 0;
        _maxAngle = 290;
        _minPulseWidth = 500;
        _maxPulseWidth = 2500;
    }
    else if(servoModel == GDW_031 || servoModel == GDW_021){
        TAU_USEC = 2500;
        _minAngle = 0;
        _maxAngle = 160;
        _minPulseWidth = 500;
        _maxPulseWidth = 2500;
    }
    else{
        TAU_USEC = 20000;
    }
}

ESP32PWMServo::ESP32PWMServo(int tau_uSec){
    
    TAU_USEC = tau_uSec;
    _resetFields();
}

ESP32PWMServo::~ESP32PWMServo() {
    detach();
}

bool ESP32PWMServo::attach(int pin, int channel, 
                   int minAngle, int maxAngle, 
                   int minPulseWidth, int maxPulseWidth) 
{
    if(channel == CHANNEL_NOT_ATTACHED) {
        if(channel_next_free == CHANNEL_MAX_NUM) {
            return false;
        }
        _channel = channel_next_free;
        channel_next_free++;
    } else {
        _channel = channel;
    }

    _pin = pin;
    _minAngle = minAngle;
    _maxAngle = maxAngle;
    _minPulseWidth = minPulseWidth;
    _maxPulseWidth = maxPulseWidth;

    long pwm_freq = 1000 * 1000 / TAU_USEC;

    ledcSetup(_channel, pwm_freq, 16); // channel X, 50-400 Hz, 16-bit depth
    ledcAttachPin(_pin, _channel);
    return true;
}

bool ESP32PWMServo::detach() {
    if (!this->attached()) {
        return false;
    }

    if(_channel == (channel_next_free - 1))
        channel_next_free--;

    ledcDetachPin(_pin);
    return true;
}

void ESP32PWMServo::write(int degrees) {
    degrees = constrain(degrees, _minAngle, _maxAngle);
    writeMicroseconds(_angleToUs(degrees));
}

void ESP32PWMServo::writeMicroseconds(int pulseUs) {
    if (!attached()) {
        return;
    }
    pulseUs = constrain(pulseUs, _minPulseWidth, _maxPulseWidth);
    _pulseWidthDuty = _usToDuty(pulseUs);
    ledcWrite(_channel, _pulseWidthDuty);
}

int ESP32PWMServo::read() {
    return _usToAngle(readMicroseconds());
}

int ESP32PWMServo::readMicroseconds() {
    if (!this->attached()) {
        return 0;
    }
    int duty = ledcRead(_channel);
    return _dutyToUs(duty);
}

bool ESP32PWMServo::attached() const { return _pin != PIN_NOT_ATTACHED; }

int ESP32PWMServo::attachedPin() const { return _pin; }

void ESP32PWMServo::_resetFields(void) {
    _pin = PIN_NOT_ATTACHED;
    _pulseWidthDuty = 0;
    _channel = CHANNEL_NOT_ATTACHED;
    _minAngle = MIN_ANGLE;
    _maxAngle = MAX_ANGLE;
    _minPulseWidth = MIN_PULSE_WIDTH;
    _maxPulseWidth = MAX_PULSE_WIDTH;
}
