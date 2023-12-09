#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

#include "MotorHandler.h"
#include "pindefs.h"


/*


MotorDriver constructor and methods (described in MotorHandler.h file)


*/

MotorDriver::MotorDriver(int _lpwm, int _rpwm, Adafruit_MCP23X17* _mcp): LPWM(_lpwm),
                                                                         RPWM(_rpwm),
                                                                         MCP(_mcp)
{}


void MotorDriver::init() {
    
    if (!MCP) {
        pinMode(LPWM, OUTPUT);
        pinMode(RPWM, OUTPUT);
        analogWrite(LPWM, 0);
        analogWrite(RPWM, 0);
    }
    else {
        MCP->pinMode(LPWM, OUTPUT);
        MCP->pinMode(RPWM, OUTPUT);
        MCP->digitalWrite(LPWM, 0);
        MCP->digitalWrite(RPWM, 0);
    }
}


void MotorDriver::setDirection(Direction dir) {

    current_direction = dir;
}


float MotorDriver::setSpeed(int percentage) {

    return speed = percentage*255/100;
}


void MotorDriver::setVelocity(int percentage) {
    
    if (!MCP) {
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
    else {
        if (percentage >= 0) {
            // setDirection(Direction::Forward);
            MCP->digitalWrite(RPWM, 1);
            MCP->digitalWrite(LPWM, 0);
        }
        else {
            // setDirection(Direction::Reverse);
            MCP->digitalWrite(RPWM, 0);
            MCP->digitalWrite(LPWM, 1);
        }
    }
}


Direction MotorDriver::getDirection() {
    return current_direction;
}


/*


StepperDriver constructor and methods (described in MotorHandler.h file)


*/


StepperDriver::StepperDriver(int _pulse_pin, int _dir_pin, int _sensor_pin, Adafruit_MCP23X17* _mcp): pulsePin(_pulse_pin),
                                                                                                      dirPin(_dir_pin),
                                                                                                      sensorPin(_sensor_pin),
                                                                                                      MCP(_mcp)
{}


void StepperDriver::init() {
    pinMode(pulsePin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, HIGH);

    // if (MCP->digitalRead(sensorPin) == LOW && !calibrated) {
    // for (int i; i<300; i++) { 
    //     MCP->digitalWrite(pulsePin, HIGH);
    //     delayMicroseconds(500);
    //     MCP->digitalWrite(pulsePin, LOW);
    //     delayMicroseconds(500);
    // }
}


void StepperDriver::movePosition() {
    
    digitalWrite(pulsePin, HIGH);
    delayMicroseconds(500);
    digitalWrite(pulsePin, LOW);
    delayMicroseconds(500);

}


/*


Encoder class constructor and methods (described in MotorHandler.h file)


*/

Encoder::Encoder(pulsePin _pin_loc, int _pulse_a, int _pulse_b, Adafruit_MCP23X17* _mcp): pinLoc(_pin_loc),
                                                                                          pulseA(_pulse_a),
                                                                                          pulseB(_pulse_b),
                                                                                          MCP(_mcp)
{}


void Encoder::wheelSpeed() {
    if (pinLoc == pulsePin::mcp0){ 
        int Lstate = MCP->getCapturedInterrupt();   
        if((encoder0PinALast == LOW) && Lstate==HIGH) {     
            int val = MCP->digitalRead(pulseB);     
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
    else {
        int Lstate = digitalRead(pulseA);   
        if((encoder0PinALast == LOW) && Lstate==HIGH) {     
            int val = MCP->digitalRead(pulseB);     
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

}


void Encoder::init(){

    direction = Direction::Forward;
    switch(pinLoc) {
        
        case pulsePin::feather0:
            MCP->pinMode(pulseB, INPUT);
            attachInterrupt(digitalPinToInterrupt(L_ENCODER_A), wheelSpeedExt0, CHANGE);
            instances[0] = this;
            break;

        case pulsePin::feather1:
            MCP->pinMode(pulseB, INPUT);
            // attachInterrupt(digitalPinToInterrupt(pulseA), wheelSpeedExt1, CHANGE);
            MCP->setupInterruptPin(M_ENCODER_A, CHANGE);
            instances[1] = this;
            break;

        case pulsePin::mcp0:
            MCP->pinMode(pulseB, INPUT);
            attachInterrupt(digitalPinToInterrupt(R_ENCODER_A), wheelSpeedExt2, CHANGE);
            instances[2] = this;
            break;

    }
    
}


long Encoder::getPosition() {
    return position;
}


int Encoder::getVelocity() {
    int true_vel = velocity;
    velocity = 0;
    return true_vel;
}


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


DriveControl constructor and methods (described in MotorHandler.h file)


*/

DriveControl::DriveControl(MotorDriver* _LeftMotor, MotorDriver* _MiddleMotor, MotorDriver* _RightMotor): LeftMotor(_LeftMotor),
                                                                                                          MiddleMotor(_MiddleMotor),
                                                                                                          RightMotor(_RightMotor)
{}


void DriveControl::init() {

    LeftMotor->init();
    MiddleMotor->init();
    RightMotor->init();

    LeftMotor->setVelocity(0);
    MiddleMotor->setVelocity(0);
    RightMotor->setVelocity(0);

    setSpeed(100);
}


void DriveControl::setSpeed(int percent) {
    
    speed = percent;
}


void DriveControl::update(uint8_t buf, bool ToggleState) {

  
    if (ToggleState) {
        LeftMotor->setVelocity(speed);
        MiddleMotor->setVelocity(speed);
        RightMotor->setVelocity(speed); 
    }

}


/*


ActuatorControl constructor and methods (described in MotorHandler.h file)


*/

ActuatorControl::ActuatorControl(MotorDriver* _left_motor, MotorDriver* _middle_motor, MotorDriver* _right_motor, StepperDriver* _dynamic_balancer, Adafruit_MCP23X17* _mcp): LeftMotor(_left_motor),
                                                                                                                                                                              MiddleMotor(_middle_motor),
                                                                                                                                                                              RightMotor(_right_motor),
                                                                                                                                                                              DynamicBalancer(_dynamic_balancer),
                                                                                                                                                                              MCP(_mcp)
{}


void ActuatorControl::init() {

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

    DynamicBalancer->init();
}


void ActuatorControl::setSpeed(int percent) {
    
    speed = percent;
}

void ActuatorControl::update(uint8_t buf, bool LToggle, bool MToggle, bool RToggle) {

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

