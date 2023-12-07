#ifndef PINDEFS_H
#define PINDEFS_H


/*

All pin definitions for the feather or expander should be defined here. Anything controller by
the expander should be specifically notified as expander pins and which ones they are

*/


// LED pins (in the IO expander)
#define GREEN_LED_PIN 11        // expander pin B3
#define LEFT_LED_PIN 8          // expander pin B0
#define MIDDLE_LED_PIN 9        // expander pin B1
#define RIGHT_LED_PIN 10        // expander pin B2


// For the drive motors
#define L_MOTOR_LPWM A0
#define L_MOTOR_RPWM A1
#define M_MOTOR_LPWM A2
#define M_MOTOR_RPWM A3
#define R_MOTOR_LPWM A4
#define R_MOTOR_RPWM A5


// For the encoders - all fake values right now, will need interrupt pins for A and normal pins for B
#define L_ENCODER_A 1
#define L_ENCODER_B 13
#define M_ENCODER_A 0
#define M_ENCODER_B 3
#define R_ENCODER_A 0
#define R_ENCODER_B 4



// For the linear actuator motors
#define L_ACTUATOR_LPWM 5
#define L_ACTUATOR_RPWM 6
#define M_ACTUATOR_LPWM 9
#define M_ACTUATOR_RPWM 10
#define R_ACTUATOR_LPWM 11
#define R_ACTUATOR_RPWM 12



#endif