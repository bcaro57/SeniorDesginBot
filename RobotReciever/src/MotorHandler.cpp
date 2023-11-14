#include <Arduino.h>
#include "MotorHandler.h"

MotorDriver::MotorDriver(int _rpwm_pin, int _lpwm_pin) : RPWM(_rpwm_pin),
                                                         LPWM(_lpwm_pin)
{}

void MotorDriver::init(){

    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
}

void MotorDriver::setVelocity(int target){

    if (target <= 0){
        analogWrite(LPWM, target);
        analogWrite(RPWM, 0);
    }
    else{
        analogWrite(LPWM, 0);
        analogWrite(RPWM, target);
    }
}