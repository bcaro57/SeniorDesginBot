#ifndef PINDEFS_H
#define PINDEFS_H


/*

All pin definitions for the feather or expander should be defined here. Anything controller by
the expander should be specifically notified as expander pins and which ones they are

*/


// LED pins (in the IO expander)
#define RIGHT_LED_PIN 4         // expander pin A4
#define MIDDLE_LED_PIN 5        // expander pin A5
#define LEFT_LED_PIN 6          // expander pin A6
#define GREEN_LED_PIN 7         // expander pin A7


// For the drive motors
#define L_MOTOR_LPWM A0
#define L_MOTOR_RPWM A1
#define M_MOTOR_LPWM A2
#define M_MOTOR_RPWM A3
#define R_MOTOR_LPWM A4
#define R_MOTOR_RPWM A5


// For the encoders - all ENCODER_B values are on the expander board. also, M_ENCODER_A is on the expander board.
#define L_ENCODER_A 0           // interrupt pin 0, the RX pin
#define L_ENCODER_B 2           // expander pin A2
#define M_ENCODER_A 1           // expander pin IA
#define M_ENCODER_B 1           // expander pin A1
#define R_ENCODER_A 1           // interrupt pin 1, the TX pin
#define R_ENCODER_B 0           // expander pin A0


// For the linear actuator motors
#define L_ACTUATOR_LPWM 10      // expander pin B2
#define L_ACTUATOR_RPWM 11      // expander pin B3
#define M_ACTUATOR_LPWM 12      // expander pin B4
#define M_ACTUATOR_RPWM 13      // expander pin B5
#define R_ACTUATOR_LPWM 14      // expander pin B6
#define R_ACTUATOR_RPWM 15      // expander pin B7


#define STEPPER_PIN_DIR 12       // expander pin B0
#define STEPPER_PIN_PUL 11       // expander pin B1

#define LIMIT_SWITCH_PIN 3      // expander pin A3
 
#endif