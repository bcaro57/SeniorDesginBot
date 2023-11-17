#include <Arduino.h>
#include "MotorHandler.h"



MotorDriver::MotorDriver(int _rpwm, int _lpwm): RPWM(_rpwm),
                                                LPWM(_lpwm)
{}


void MotorDriver::init(){

    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);

    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
}


void MotorDriver::setDirection(Direction dir){

    current_direction = dir;
}


float MotorDriver::setSpeed(int percentage){

    return speed = percentage*255/100;
}


void MotorDriver::setVelocity(int percentage){
    if (current_direction == Direction::Forward){
        analogWrite(RPWM, setSpeed(percentage));
        analogWrite(LPWM, 0);
    }
    else{
        analogWrite(RPWM, 0);
        analogWrite(LPWM, setSpeed(percentage));
    }
}




// Encoder::Encoder(int _pulse_a, int _pulse_b): pulseA(_pulse_a),
//                                               pulseB(_pulse_b)
// {}

// void Encoder::init(){

//     Direction direction = Direction::Forward;
//     pinMode(pulseB, INPUT);
//     attachInterrupt(digitalPinToInterrupt(pulseA), wheelSpeed, CHANGE);
// }

// void Encoder::wheelSpeed(){


// }


MotorControl::MotorControl(MotorDriver*_LeftMotor, MotorDriver* _MiddleMotor, MotorDriver* _RightMotor): LeftMotor(_LeftMotor),
                                                                                                         MiddleMotor(_MiddleMotor),
                                                                                                         RightMotor(_RightMotor)
{}


void MotorControl::init(){

    LeftMotor->init();
    MiddleMotor->init();
    RightMotor->init();

    LeftMotor->setDirection(Direction::Forward);
    MiddleMotor->setDirection(Direction::Forward);
    RightMotor->setDirection(Direction::Forward);

    setSpeed(80);
}


void MotorControl::setSpeed(int percent){
    
    speed = percent*255/100;
}



void MotorControl::update(uint8_t buf, int MotorState){

    if (buf == 0x01) {
        LeftMotor->setVelocity(0);
        MiddleMotor->setVelocity(0);
        RightMotor->setVelocity(0);    
    }

    else if (buf == 0x02) {
        LeftMotor->setVelocity(speed);
        MiddleMotor->setVelocity(0);
        RightMotor->setVelocity(0); 
    }

    else if (buf == 0x03) {
        LeftMotor->setVelocity(0);
        MiddleMotor->setVelocity(speed);
        RightMotor->setVelocity(0); 
    }

    else if (buf == 0x04) {
        LeftMotor->setVelocity(0);
        MiddleMotor->setVelocity(0);
        RightMotor->setVelocity(speed); 
    }

    else if (MotorState){
        LeftMotor->setVelocity(speed);
        MiddleMotor->setVelocity(speed);
        RightMotor->setVelocity(speed); 
    }

    else {
        LeftMotor->setVelocity(0);
        MiddleMotor->setVelocity(0);
        RightMotor->setVelocity(0); 
    }
}