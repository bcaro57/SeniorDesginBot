#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include "MotorHandler.h"
#include "pindefs.h"

/*


MotorClosedLoop constructor and methods (described in MotorHandler.h file)


*/

// MotorClosedLoop::MotorClosedLoop(MotorDriver* _motor, Encoder* _encoder): Motor(_motor),
//                                                                           MotorEncoder(_encoder)
// {}


// void MotorClosedLoop::init() {
//     Motor->init();
//     MotorEncoder->init();
// }


/*


MotorDriver constructor and methods (described in MotorHandler.h file)


*/

MotorDriver::MotorDriver(int _lpwm, int _rpwm): LPWM(_lpwm),
                                                RPWM(_rpwm)
{}


void MotorDriver::init() {

    pinMode(LPWM, OUTPUT);
    pinMode(RPWM, OUTPUT);
    analogWrite(LPWM, 0);
    analogWrite(RPWM, 0);
}


void MotorDriver::setDirection(Direction dir) {

    current_direction = dir;
}


float MotorDriver::setSpeed(int percentage) {

    return speed = percentage*255/100;
}


void MotorDriver::setVelocity(int percentage) {

    if (percentage >= 0) {
        // setDirection(Direction::Forward);
        analogWrite(RPWM, setSpeed(abs(percentage)));
        analogWrite(LPWM, 0);
    }
    else {
        // setDirection(Direction::Reverse);
        analogWrite(RPWM, 0);
        analogWrite(LPWM, setSpeed(abs(percentage)));
    }
}

Direction MotorDriver::getDirection() {
    return current_direction;
}


/*


Encoder class constructor and methods (described in MotorHandler.h file)


*/

Encoder::Encoder(int _pulse_a, int _pulse_b, Adafruit_MCP23X17* _mcp = NULL): pulseA(_pulse_a),
                                                                              pulseB(_pulse_b),
                                                                              MCP(_mcp)
{}


void Encoder::wheelSpeed() {
    int Lstate = digitalRead(pulseA);   
    if((encoder0PinALast == LOW) && Lstate==HIGH) {     
        int val = digitalRead(pulseB);     
        if(val == LOW && direction == Direction::Forward) {       
            direction = Direction::Reverse; //Reverse     
        }     
        else if(val == HIGH && direction == Direction::Reverse) {       
            direction = Direction::Forward;  //Forward     
        }  
    }   
    encoder0PinALast = Lstate;     

    if(direction == Direction::Reverse) { 
        velocity++; 
        position++;   
    }
    else {
        velocity--;
        position--;
    } 

}


void Encoder::init(){

    Direction direction = Direction::Forward;
    switch(pulseA) {
        case L_ENCODER_A:
            pinMode(pulseB, INPUT);
            attachInterrupt(digitalPinToInterrupt(pulseA), wheelSpeedExt0, CHANGE);
            instances[0] = this;
            break;
        case M_ENCODER_A:
            pinMode(pulseB, INPUT);
            attachInterrupt(digitalPinToInterrupt(pulseA), wheelSpeedExt1, CHANGE);
            instances[1] = this;
            break;
        case R_ENCODER_A:
            pinMode(pulseB, INPUT);
            attachInterrupt(digitalPinToInterrupt(pulseA), wheelSpeedExt2, CHANGE);
            instances[2] = this;
            break;
    }
    
}


long Encoder::getPosition() {
    return position;
}


int Encoder::getVelocity() {
    return velocity;
}


/*


MotorControl constructor and methods (described in MotorHandler.h file)


*/

MotorControl::MotorControl(MotorDriver* _LeftMotor, MotorDriver* _MiddleMotor, MotorDriver* _RightMotor): LeftMotor(_LeftMotor),
                                                                                                          MiddleMotor(_MiddleMotor),
                                                                                                          RightMotor(_RightMotor)
{}


void MotorControl::init() {

    LeftMotor->init();
    MiddleMotor->init();
    RightMotor->init();

    LeftMotor->setVelocity(0);
    MiddleMotor->setVelocity(0);
    RightMotor->setVelocity(0);

    left_was_toggled = false;
    middle_was_toggled = false;
    right_was_toggled = false;

    setSpeed(100);
}


void MotorControl::setSpeed(int percent) {
    
    speed = percent;
}


void MotorControl::update(uint8_t buf, bool ToggleState, bool LToggle, bool MToggle, bool RToggle) {

    switch (buf){
        case 0x00:
            LeftMotor->setVelocity(0);
            MiddleMotor->setVelocity(0);
            RightMotor->setVelocity(0); 
            break;
        case 0x01:
            LeftMotor->setVelocity(0);
            MiddleMotor->setVelocity(0);
            RightMotor->setVelocity(0); 
            break;
        case 0x02:
            LeftMotor->setVelocity(speed);
            MiddleMotor->setVelocity(0);
            RightMotor->setVelocity(0); 
            break;
        case 0x03:
            LeftMotor->setVelocity(0);
            MiddleMotor->setVelocity(speed);
            RightMotor->setVelocity(0); 
            break;
        case 0x04:
            LeftMotor->setVelocity(0);
            MiddleMotor->setVelocity(0);
            RightMotor->setVelocity(speed); 
            break;
    }
    
    if (ToggleState) {
        LeftMotor->setVelocity(speed);
        MiddleMotor->setVelocity(speed);
        RightMotor->setVelocity(speed); 
    }

    if (LToggle) {
        left_was_toggled = true;
        if (LeftMotor->getDirection() == Direction::Forward) {
            LeftMotor->setVelocity(speed);
            MiddleMotor->setVelocity(0);
            RightMotor->setVelocity(0);
        }
        else if (LeftMotor->getDirection() == Direction::Reverse) {
            LeftMotor->setVelocity(-speed);
            MiddleMotor->setVelocity(0);
            RightMotor->setVelocity(0);
        }
    }

    else if (!LToggle && left_was_toggled){
        if (LeftMotor->getDirection() == Direction::Forward && left_was_toggled) {
            LeftMotor->setDirection(Direction::Reverse);
            left_was_toggled = false;
        }
        else if (LeftMotor->getDirection() == Direction::Reverse && left_was_toggled) {
            LeftMotor->setDirection(Direction::Forward);
            left_was_toggled = false;
        }
    }

    if (MToggle) {
        middle_was_toggled = true;
        if (MiddleMotor->getDirection() == Direction::Forward) {
            LeftMotor->setVelocity(0);
            MiddleMotor->setVelocity(speed);
            RightMotor->setVelocity(0);
        }
        else if (MiddleMotor->getDirection() == Direction::Reverse) {
            LeftMotor->setVelocity(0);
            MiddleMotor->setVelocity(-speed);
            RightMotor->setVelocity(0);
        }
    }

    else if (!MToggle && middle_was_toggled){
        if (MiddleMotor->getDirection() == Direction::Forward && middle_was_toggled) {
            MiddleMotor->setDirection(Direction::Reverse);
            middle_was_toggled = false;
        }
        else if (MiddleMotor->getDirection() == Direction::Reverse && middle_was_toggled) {
            MiddleMotor->setDirection(Direction::Forward);
            middle_was_toggled = false;
        }
    }

    if (RToggle) {
        right_was_toggled = true;
        if (RightMotor->getDirection() == Direction::Forward) {
            LeftMotor->setVelocity(0);
            MiddleMotor->setVelocity(0);
            RightMotor->setVelocity(speed);
        }
        else if (RightMotor->getDirection() == Direction::Reverse) {
            LeftMotor->setVelocity(0);
            MiddleMotor->setVelocity(0);
            RightMotor->setVelocity(-speed);
        }
    }

    else if (!RToggle && right_was_toggled){
        if (RightMotor->getDirection() == Direction::Forward && right_was_toggled) {
            RightMotor->setDirection(Direction::Reverse);
            right_was_toggled = false;
        }
        else if (RightMotor->getDirection() == Direction::Reverse && right_was_toggled) {
            RightMotor->setDirection(Direction::Forward);
            right_was_toggled = false;
        }
    }

}