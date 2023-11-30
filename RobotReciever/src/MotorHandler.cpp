#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include "MotorHandler.h"
#include "pindefs.h"


// MotorClosedLoop::MotorClosedLoop(MotorDriver* _motor, Encoder* _encoder): Motor(_motor),
//                                                                           MotorEncoder(_encoder)
// {}


// void MotorClosedLoop::init(){
//     Motor->init();
//     MotorEncoder->init();
// }



MotorDriver::MotorDriver(int _lpwm, int _rpwm): LPWM(_lpwm),
                                                RPWM(_rpwm)
{}


void MotorDriver::init(){
    // if (!PCF){
        pinMode(LPWM, OUTPUT);
        pinMode(RPWM, OUTPUT);

        analogWrite(LPWM, 0);
        analogWrite(RPWM, 0);
    // }

    // else {
        // PCF->pinMode(LPWM, OUTPUT);
        // PCF->pinMode(RPWM, OUTPUT);

        // PCF->digitalWrite(LPWM, 0);
        // PCF->digitalWrite(RPWM, 0);
    // }
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




// Encoder::Encoder(int _pulse_a, int _pulse_b, Adafruit_MCP23X17* _mcp = NULL): pulseA(_pulse_a),
//                                                                               pulseB(_pulse_b),
//                                                                               MCP(_mcp)
// {}


// void Encoder::wheelSpeed(){
//     int Lstate = digitalRead(pulseA);   
//     if((encoder0PinALast == LOW) && Lstate==HIGH){     
//         int val = digitalRead(pulseB);     
//         if(val == LOW && direction == Direction::Forward) {       
//             direction = Direction::Reverse; //Reverse     
//         }     
//         else if(val == HIGH && direction == Direction::Reverse){       
//             direction = Direction::Forward;  //Forward     
//         }  
//     }   
//     encoder0PinALast = Lstate;     

//     if(direction == Direction::Reverse){ 
//         velocity++; 
//         position++;   
//     }
//     else{
//         velocity--;
//         position--;
//     } 

// }


// void Encoder::init(){

//     Direction direction = Direction::Forward;
//     switch(pulseA){
//         case L_ENCODER_A:
//             pinMode(pulseB, INPUT);
//             attachInterrupt(digitalPinToInterrupt(pulseA), wheelSpeedExt0, CHANGE);
//             instances[0] = this;
//             break;
//         case M_ENCODER_A:
//             pinMode(pulseB, INPUT);
//             attachInterrupt(digitalPinToInterrupt(pulseA), wheelSpeedExt1, CHANGE);
//             instances[1] = this;
//             break;
//         case R_ENCODER_A:
//             pinMode(pulseB, INPUT);
//             attachInterrupt(digitalPinToInterrupt(pulseA), wheelSpeedExt2, CHANGE);
//             instances[2] = this;
//             break;
//     }
    
// }


// long Encoder::getPosition(){
//     return position;
// }


// int Encoder::getVelocity(){
//     return velocity;
// }



MotorControl::MotorControl(MotorDriver*_LeftMotor, MotorDriver* _MiddleMotor, MotorDriver* _RightMotor): LeftMotor(_LeftMotor),
                                                                                                         MiddleMotor(_MiddleMotor),
                                                                                                         RightMotor(_RightMotor)
{}


void MotorControl::init(){

    LeftMotor->init();
    MiddleMotor->init();
    RightMotor->init();

    LeftMotor->setDirection(Direction::Reverse);
    MiddleMotor->setDirection(Direction::Reverse);
    RightMotor->setDirection(Direction::Reverse);

    setSpeed(80);
}


void MotorControl::setSpeed(int percent){
    
    speed = percent;
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