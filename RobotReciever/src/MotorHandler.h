#ifndef __MOTOR_HANDLER_H__
#define __MOTOR_HANDLER_H__

#include <Arduino.h>
#include "pindefs.h"


class MotorDriver{

    private:
        int RPWM;
        int LPWM;
    public:
        MotorDriver(int _rpwm_pin, int _lpwm_pin);

        init();
        setVelocity(int target);
}

// void MotorSetup(){
//   pinMode(MOTOR1_RPWM_Output, OUTPUT); 
//   pinMode(MOTOR1_LPWM_Output, OUTPUT); 
//   pinMode(MOTOR2_RPWM_Output, OUTPUT); 
//   pinMode(MOTOR2_LPWM_Output, OUTPUT); 
//   pinMode(MOTOR3_RPWM_Output, OUTPUT); 
//   pinMode(MOTOR3_LPWM_Output, OUTPUT); 
// }

// void MotorsController(uint8_t buf, int MotorState){
//     if (MotorState){
//         Serial.println("All motors move now!");
        
//         analogWrite(MOTOR1_RPWM_Output, 0);
//         analogWrite(MOTOR1_LPWM_Output, 100);
//         analogWrite(MOTOR2_RPWM_Output, 0);
//         analogWrite(MOTOR2_LPWM_Output, 100);
//         analogWrite(MOTOR3_RPWM_Output, 0);
//         analogWrite(MOTOR3_LPWM_Output, 100);
//     }
//     else{
//         Serial.println("No motors move now.");

//         analogWrite(MOTOR1_RPWM_Output, 0);
//         analogWrite(MOTOR1_LPWM_Output, 0);
//         analogWrite(MOTOR2_RPWM_Output, 0);
//         analogWrite(MOTOR2_LPWM_Output, 0);
//         analogWrite(MOTOR3_RPWM_Output, 0);
//         analogWrite(MOTOR3_LPWM_Output, 0);
//     }
// }  


#endif