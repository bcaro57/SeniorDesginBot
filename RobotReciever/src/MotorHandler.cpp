#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include <AccelStepper.h>

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
            analogWrite(RPWM, setSpeed(abs(percentage)));
            analogWrite(LPWM, 0);
        }
        else {
            analogWrite(RPWM, 0);
            analogWrite(LPWM, setSpeed(abs(percentage)));
            Serial.println(setSpeed(abs(percentage)));
        }
    }
    else {
        if (percentage > 0) {
            MCP->digitalWrite(RPWM, 1);
            MCP->digitalWrite(LPWM, 0);
        }
        else if (percentage < 0) {
            MCP->digitalWrite(RPWM, 0);
            MCP->digitalWrite(LPWM, 1);
        }
        else {
            MCP->digitalWrite(RPWM, 0);
            MCP->digitalWrite(LPWM, 0);
        }
    }
}
Direction MotorDriver::getDirection() {
    return current_direction;
}


/*


StepperDriver constructor and methods (described in MotorHandler.h file)


*/


StepperDriver::StepperDriver(int _pulse_pin, int _dir_pin, int _sensor_pin): pulsePin(_pulse_pin),
                                                                             dirPin(_dir_pin),
                                                                             sensorPin(_sensor_pin)
                                                                                                     
{}
void StepperDriver::init() {

    calibrated = false;
    pinMode(sensorPin, INPUT_PULLUP);
    myStepper = AccelStepper(1, pulsePin, dirPin);
    myStepper.setMaxSpeed(350);
    myStepper.setSpeed(350);
    myStepper.setAcceleration(500);

    while (!calibrated){
        myStepper.setSpeed(-300);
        myStepper.runSpeed();
        if (digitalRead(sensorPin) == 1) {
            myStepper.setCurrentPosition(0);
            delay(100);
            calibrated = true;
        }
    }

    myStepper.setMaxSpeed(8000);
    myStepper.setSpeed(8000);
    myStepper.setAcceleration(2500);
}
void StepperDriver::movePosition(int targetPos) {
    myStepper.run();

    if (!calibrated){
        return;
    }

    else {
        switch(targetPos) {
            case 1:
                myStepper.runToNewPosition(0);
                break;
            case 2:
                myStepper.runToNewPosition(halfLength);
                break;
            case 3:
                myStepper.runToNewPosition(fullLength);
                break;
        }
    }
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

    else{
        LeftMotor->setVelocity(0);
        MiddleMotor->setVelocity(0);
        RightMotor->setVelocity(0);
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

    setSpeed(100);

    DynamicBalancer->init();
}
void ActuatorControl::setSpeed(int percent) {
    
    speed = percent;
}
void ActuatorControl::update(uint8_t buf) {

    unsigned long currentTime = millis();

    switch(buf) {
        case 0x02:
            DynamicBalancer->movePosition(3);
            if (LeftMotor->getDirection() == Direction::Forward) {
                LeftMotor->setDirection(Direction::Reverse);
            }
            else if (LeftMotor->getDirection() == Direction::Reverse) {
                LeftMotor->setDirection(Direction::Forward);
            }
            LeftEventTime = millis();
            break;
        case 0x03:
            DynamicBalancer->movePosition(2);
            if (MiddleMotor->getDirection() == Direction::Forward) {
                MiddleMotor->setDirection(Direction::Reverse);
            }
            else if (MiddleMotor->getDirection() == Direction::Reverse) {
                MiddleMotor->setDirection(Direction::Forward);
            }
            MiddleEventTime = millis();
            break;
        case 0x04:
            DynamicBalancer->movePosition(1);
            if (RightMotor->getDirection() == Direction::Forward) {
                RightMotor->setDirection(Direction::Reverse);
            }
            else if (RightMotor->getDirection() == Direction::Reverse) {
                RightMotor->setDirection(Direction::Forward);
            }
            RightEventTime = millis();
            break;
    }

    if (currentTime < LeftEventTime + waitTime) {
        if (LeftMotor->getDirection() == Direction::Forward) {
            LeftMotor->setVelocity(speed);
        }
        else if (LeftMotor->getDirection() == Direction::Reverse) {
            LeftMotor->setVelocity(-speed);
        }
    }
    else {
        LeftMotor->setVelocity(0);
    }

    if (currentTime < MiddleEventTime + waitTime) {
        if (MiddleMotor->getDirection() == Direction::Forward) {
            MiddleMotor->setVelocity(speed);
        }
        else if (MiddleMotor->getDirection() == Direction::Reverse) {
            MiddleMotor->setVelocity(-speed);
        }
    }
    else {
        MiddleMotor->setVelocity(0);
    }

    if (currentTime < RightEventTime + waitTime) {
        if (RightMotor->getDirection() == Direction::Forward) {
            RightMotor->setVelocity(speed);
        }
        else if (RightMotor->getDirection() == Direction::Reverse) {
            RightMotor->setVelocity(-speed);
        }
    }
    else {
        RightMotor->setVelocity(0);
    }


    // if (LToggle) {
    //     DynamicBalancer->movePosition(3);
    //     left_was_toggled = true;
    //     if (LeftMotor->getDirection() == Direction::Forward) {
    //         LeftMotor->setVelocity(speed);
    //         MiddleMotor->setVelocity(0);
    //         RightMotor->setVelocity(0);
    //     }
    //     else if (LeftMotor->getDirection() == Direction::Reverse) {
    //         LeftMotor->setVelocity(-speed);
    //         MiddleMotor->setVelocity(0);
    //         RightMotor->setVelocity(0);
    //     }
    // }

    // else if (!LToggle && left_was_toggled){

    //     if (LeftMotor->getDirection() == Direction::Forward && left_was_toggled) {
    //         LeftMotor->setDirection(Direction::Reverse);
    //         left_was_toggled = false;
    //     }
    //     else if (MiddleMotor->getDirection() == Direction::Reverse && left_was_toggled) {
    //         LeftMotor->setDirection(Direction::Forward);
    //         left_was_toggled = false;
    //     }
    // }

    // if (MToggle) {
    //     DynamicBalancer->movePosition(2);
    //     middle_was_toggled = true;
    //     if (MiddleMotor->getDirection() == Direction::Forward) {
    //         LeftMotor->setVelocity(0);
    //         MiddleMotor->setVelocity(speed);
    //         RightMotor->setVelocity(0);
    //     }
    //     else if (MiddleMotor->getDirection() == Direction::Reverse) {
    //         LeftMotor->setVelocity(0);
    //         MiddleMotor->setVelocity(-speed);
    //         RightMotor->setVelocity(0);
    //     }
    // }

    // else if (!MToggle && middle_was_toggled){

    //     if (MiddleMotor->getDirection() == Direction::Forward && middle_was_toggled) {
    //         MiddleMotor->setDirection(Direction::Reverse);
    //         middle_was_toggled = false;
    //     }
    //     else if (MiddleMotor->getDirection() == Direction::Reverse && middle_was_toggled) {
    //         MiddleMotor->setDirection(Direction::Forward);
    //         middle_was_toggled = false;
    //     }
    // }

    // if (RToggle) {
    //     DynamicBalancer->movePosition(1);
    //     right_was_toggled = true;
    //     if (RightMotor->getDirection() == Direction::Forward) {
    //         LeftMotor->setVelocity(0);
    //         MiddleMotor->setVelocity(0);
    //         RightMotor->setVelocity(speed);
    //     }
    //     else if (RightMotor->getDirection() == Direction::Reverse) {
    //         LeftMotor->setVelocity(0);
    //         MiddleMotor->setVelocity(0);
    //         RightMotor->setVelocity(-speed);
    //     }
    // }

    // else if (!RToggle && right_was_toggled){
    //     if (RightMotor->getDirection() == Direction::Forward && right_was_toggled) {
    //         RightMotor->setDirection(Direction::Reverse);
    //         right_was_toggled = false;
    //     }
    //     else if (RightMotor->getDirection() == Direction::Reverse && right_was_toggled) {
    //         RightMotor->setDirection(Direction::Forward);
    //         right_was_toggled = false;
    //     }
    // }

}