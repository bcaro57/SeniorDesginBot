#include <Arduino.h>
#include "LEDHandler.h"

LED::LED(int _green_pin, int _left_pin, int _middle_pin, int _right_pin): GreenLED(_green_pin),
                                                                          LeftLED(_left_pin),
                                                                          MiddleLED(_middle_pin),
                                                                          RightLED(_right_pin)
{}

void LED::init(){

    pinMode(GreenLED, OUTPUT);
    pinMode(LeftLED, OUTPUT);
    pinMode(MiddleLED, OUTPUT);
    pinMode(RightLED, OUTPUT);
}

void LED::turnOn(bool GreenOn, bool LeftOn, bool MiddleOn, bool RightOn){

    digitalWrite(GreenLED, GreenOn);
    digitalWrite(LeftLED, LeftOn);
    digitalWrite(MiddleLED, MiddleOn);
    digitalWrite(RightLED, RightOn);
}

void LED::update(uint8_t buf, int MotorState){

  if (buf == 0x01) {
    turnOn(1, 1, 0, 0);
  }

  else if (buf == 0x02) {
    turnOn(1, 0, 1, 0);
  }

  else if (buf == 0x03) {
    turnOn(1, 0, 0, 1);
  }

  else if (buf == 0x04) {
    turnOn(1, 1, 1, 1);
  }

  else if (MotorState){
    turnOn(1, 1, 0, 0);
  }

  else {
    turnOn(1, 0, 0, 0);
  }
}
