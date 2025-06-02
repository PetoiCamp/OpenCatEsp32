/*
Copyright (c) 2017 John K. Bennett. All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

* Notes on the implementation:
* The ESP32 supports 16 hardware LED PWM channels that are intended
* to be used for LED brightness control. The low level ESP32 code
* (esp32-hal-ledc.*) allows us to set the PWM frequency and bit-depth,
* and then manipulate them by setting bits in the relevant control
* registers.
*
* Different servos require different pulse widths to vary servo angle, but the range is
* an approximately 500-2500 microsecond pulse every 20ms (50Hz). In general, hobbyist servos
* sweep 180 degrees, so the lowest number in the published range for a particular servo
* represents an angle of 0 degrees, the middle of the range represents 90 degrees, and the top
* of the range represents 180 degrees. So for example, if the range is 1000us to 2000us,
* 1000us would equal an angle of 0, 1500us would equal 90 degrees, and 2000us would equal 180
* degrees. We vary pulse width (recall that the pulse period is already set to 20ms) as follows:
*
* The ESP32 PWM timers allow us to set the timer width (max 20 bits). Thus
* the timer "tick" length is (pulse_period/2**timer_width), and the equation for pulse_high_width
* (the portion of the 20ms cycle that the signal is high) becomes:
*
*                  pulse_high_width  = count * tick_length
*                                    = count * (pulse_period/2**timer_width)
*
*            and   count = (pulse_high_width / (pulse_period/2**timer_width))
*
* So, for example, if I want a 1500us pulse_high_width, I set pulse_period to 20ms (20000us)
* (this value is set in the ledcSetup call), and count (used in the ledcWrite call) to
* 1500/(20000/65536), or 4924. This is the value we write to the timer in the ledcWrite call.
* If we increase the timer_width, the timer_count values need to be adjusted.
*
* The servo signal pins connect to any available GPIO pins on the ESP32, but not all pins are
* GPIO pins.
*
* The ESP32 is a 32 bit processor that includes FP support; this code reflects that fact.
*/

#include "ESP32Servo.h"
#include "Arduino.h"

//
Servo::Servo()
{ // initialize this channel with plausible values, except pin # (we set pin # when attached)
    this->frequency = DEFAULT_REFRESH_CPS;
    this->ticks = DEFAULT_PULSE_WIDTH_TICKS;
    this->timer_width = DEFAULT_TIMER_WIDTH;
    this->pinNumber = -1; // make it clear that we haven't attached a pin to this channel
    this->min = DEFAULT_uS_LOW;
    this->max = DEFAULT_uS_HIGH;
    this->timer_width_ticks = pow(2, this->timer_width);
}
ESP32PWM *Servo::getPwm()
{

    return &pwm;
}

int Servo::attach(int pin)
{

    return (this->attach(pin, DEFAULT_ANGLE_RANGE, DEFAULT_REFRESH_CPS, DEFAULT_uS_LOW, DEFAULT_uS_HIGH));
}

int Servo::attach(int pin, ServoModel model)
{
    return (this->attach(pin, model.getAngleRange(), model.getFrequency(), model.getMinPulse(), model.getMaxPulse()));
}
int Servo::attach(int pin, ServoModel *model)
{
    return (this->attach(pin, model->getAngleRange(), model->getFrequency(), model->getMinPulse(), model->getMaxPulse()));
}

int Servo::attach(int pin, int range, int frequency, int minPulse, int maxPulse)
{

#ifdef ENFORCE_PINS
    // ESP32 Recommend only the following pins 2,4,12-19,21-23,25-27,32-33
    // ESP32-S2 only the following pins 1-21,26,33-42
    if (pwm.hasPwm(pin))
    {
#endif
        // OK to proceed; first check for new/reuse
        if (this->pinNumber < 0) // we are attaching to a new or previously detached pin; we need to initialize/reinitialize
        {
            this->ticks = DEFAULT_PULSE_WIDTH_TICKS;
            this->timer_width = DEFAULT_TIMER_WIDTH;
            this->timer_width_ticks = pow(2, this->timer_width);
        }
        this->pinNumber = pin;
#ifdef ENFORCE_PINS
    }
    else
    {
        Serial.println("This pin can not be a servo: " + String(pin) +
#if defined(ARDUINO_ESP32S2_DEV)
                       "\r\nServo availible on: 1-21,26,33-42"
#else
                       "\r\nServo availible on: 2,4,5,12-19,21-23,25-27,32-33"
#endif
        );
        return 0;
    }
#endif

    // min/max checks. ensure pulse width is valid
    if (minPulse < MIN_PULSE_WIDTH)
        minPulse = MIN_PULSE_WIDTH;
    this->min = minPulse; // store this value in uS

    if (maxPulse > MAX_PULSE_WIDTH)
        maxPulse = MAX_PULSE_WIDTH;
    this->max = maxPulse; // store this value in uS

    this->angleRange = range;
    this->frequency = frequency;
    int period = 1000 * 1000 / frequency;
    // this->ticks = DEFAULT_PULSE_WIDTH / (period / this->timer_width_ticks)
    this->ticks = (int)(DEFAULT_PULSE_WIDTH * (this->timer_width_ticks) / period);
    // Set up this channel
    // if you want anything other than default timer width, you must call setTimerWidth() before attach
    pwm.attachPin(this->pinNumber, this->frequency, this->timer_width); // GPIO pin assigned to channel
                                                                        //        Serial.print(this->pinNumber);
                                                                        //        Serial.println("Attaching servo : "+String(pin)+" on PWM "+String(pwm.getChannel()));

    return 1;
}

void Servo::detach()
{
    if (this->attached())
    {
        // keep track of detached servos channels so we can reuse them if needed
        pwm.detachPin(this->pinNumber);

        this->pinNumber = -1;
    }
}

void Servo::write(int value)
{
    // treat values less than MIN_PULSE_WIDTH (500) as angles in degrees (valid values in microseconds are handled as microseconds)
    if (value < MIN_PULSE_WIDTH)
    {
        if (value < 0)
            value = 0;
        else if (value > angleRange)
            value = angleRange;

        value = map(value, 0, angleRange, this->min, this->max);
    }
    this->writeMicroseconds(value);
}

void Servo::writeMicroseconds(int value)
{
    // calculate and store the values for the given channel
    if (this->attached()) // ensure channel is valid
    {
        if (value)
        {
            if (value < this->min && value != 0) // ensure pulse width is valid
                value = this->min;
            else if ((value > this->max && value < 2800) || value > 3600)
                value = this->max;
        }
        value = usToTicks(value); // convert to ticks
        this->ticks = value;
        // do the actual write
        pwm.write(this->ticks);
    }
}

int Servo::read() // return the value as degrees
{
    return (map(readMicroseconds() + 1, this->min, this->max, 0, angleRange));
}

int Servo::readMicroseconds()
{
    int pulsewidthUsec;
    if (this->attached())
    {
        pulsewidthUsec = ticksToUs(this->ticks);
    }
    else
    {
        pulsewidthUsec = 0;
    }

    return (pulsewidthUsec);
}

float Servo::pulseToAngle(float pulse)
{
    return map(pulse, this->min, this->max, 0, angleRange * 10) / 10.0;
}

bool Servo::attached()
{
    return (pwm.attached());
}

void Servo::setTimerWidth(int value)
{
    // only allow values between 16 and 20
    if (value < 16)
        value = 16;
    else if (value > 20)
        value = 20;

    // Fix the current ticks value after timer width change
    // The user can reset the tick value with a write() or writeUs()
    int widthDifference = this->timer_width - value;
    // if positive multiply by diff; if neg, divide
    if (widthDifference > 0)
    {
        this->ticks = widthDifference * this->ticks;
    }
    else if (widthDifference < 0)
    {
        this->ticks = this->ticks / -widthDifference;
    }

    this->timer_width = value;
    this->timer_width_ticks = pow(2, this->timer_width);

    // If this is an attached servo, clean up
    if (this->attached())
    {
        // detach, setup and attach again to reflect new timer width
        pwm.detachPin(this->pinNumber);
        pwm.attachPin(this->pinNumber, this->frequency, this->timer_width);
    }
}

int Servo::readTimerWidth()
{
    return (this->timer_width);
}

int Servo::usToTicks(int usec)
{
    return (int)((float)usec / ((float)REFRESH_USEC / (float)this->timer_width_ticks) * (((float)this->frequency) / 50.0));
}

int Servo::ticksToUs(int ticks)
{
    return (int)((float)ticks * ((float)REFRESH_USEC / (float)this->timer_width_ticks) / (((float)this->frequency) / 50.0));
}
