#include "LEDHandler.h"
#include <Arduino.h>

LED::LED(int _notice_pin, int _left_pin, int _middle_pin, int _right_pin) :  int noticeLED(_notice_pin), 
                                                                             int leftLED(_left_pin), 
                                                                             int middleLED(_middle_pin), 
                                                                             int rightLED(_right_pin)
{}

void LED::init(){

    pinMode(noticeLED, OUTPUT);
    pinMode(leftLED, OUTPUT);
    pinMode(middleLED, OUTPUT);
    pinMode(rightLED, OUTPUT);

    char* orderLED = [noticeLED, leftLED, middleLED, rightLED];
}


void LED::change(int* arrayLED){

    for (int i=0, i<length(arrayLED), i++){

        digitalWrite(orderLED[i], arrayLED[i]);
    }
}