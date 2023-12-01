#ifndef PINDEFS_H
#define PINDEFS_H


// LED pins (in the IO expander)
#define GREEN_LED_PIN 11 //B3
#define LEFT_LED_PIN 8   //B0
#define MIDDLE_LED_PIN 9 //B1
#define RIGHT_LED_PIN 10 //B2


// For the drive motors
#define L_MOTOR_LPWM 5
#define L_MOTOR_RPWM 6
#define M_MOTOR_LPWM 9
#define M_MOTOR_RPWM 10
#define R_MOTOR_LPWM 11
#define R_MOTOR_RPWM 12


// For the encoders - all fake values right now
#define L_ENCODER_A 7 
#define L_ENCODER_B 8
#define M_ENCODER_A 9
#define M_ENCODER_B 10
#define R_ENCODER_A 11
#define R_ENCODER_B 12


// For the linear actuator motors
#define L_ACTUATOR_LPWM A0
#define L_ACTUATOR_RPWM A1
#define M_ACTUATOR_LPWM A2
#define M_ACTUATOR_RPWM A3
#define R_ACTUATOR_LPWM A4
#define R_ACTUATOR_RPWM A5



#endif