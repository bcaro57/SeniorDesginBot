#ifndef LED_HANDLER_H
#define LED_HANDLER_H

#include <Arduino.h>
#include "pindefs.h"

class LED{

  private:
    int noticeLED;
    int leftLED;
    int middleLED;
    int rightLED;

  public:
    LED(int _notice_pin, int _left_pin, int _middle_pin, int _right_pin);

    void init();
    void change(int* arrayLED);
}

// void LEDSetup(){
//   pinMode(GREEN_LED_PIN, OUTPUT);
//   pinMode(LEFT_LED_PIN, OUTPUT);
//   pinMode(MIDDLE_LED_PIN, OUTPUT);
//   pinMode(RIGHT_LED_PIN, OUTPUT);
// }

// void LEDController(uint8_t buf, int MotorState){
//   if (buf == 0x01) {
//     digitalWrite(GREEN_LED_PIN, HIGH);
//     digitalWrite(LEFT_LED_PIN, HIGH);
//     digitalWrite(MIDDLE_LED_PIN, LOW);
//     digitalWrite(RIGHT_LED_PIN, LOW);
//   }

//   else if (buf == 0x02) {
//     digitalWrite(GREEN_LED_PIN, HIGH);
//     digitalWrite(LEFT_LED_PIN, LOW);
//     digitalWrite(MIDDLE_LED_PIN, HIGH);
//     digitalWrite(RIGHT_LED_PIN, LOW);
//   }

//   else if (buf == 0x03) {
//     digitalWrite(GREEN_LED_PIN, HIGH);
//     digitalWrite(LEFT_LED_PIN, LOW);
//     digitalWrite(MIDDLE_LED_PIN, LOW);
//     digitalWrite(RIGHT_LED_PIN, HIGH);
//   }

//   else if (buf == 0x04) {
//     digitalWrite(GREEN_LED_PIN, HIGH);
//     digitalWrite(LEFT_LED_PIN, HIGH);
//     digitalWrite(MIDDLE_LED_PIN, HIGH);
//     digitalWrite(RIGHT_LED_PIN, HIGH);
//   }

//   else if(MotorState){
//     digitalWrite(GREEN_LED_PIN, HIGH);
//     digitalWrite(LEFT_LED_PIN, HIGH);
//     digitalWrite(MIDDLE_LED_PIN, LOW);
//     digitalWrite(RIGHT_LED_PIN, LOW);
//   }

//   else {
//     digitalWrite(GREEN_LED_PIN, HIGH);
//     digitalWrite(LEFT_LED_PIN, LOW);
//     digitalWrite(MIDDLE_LED_PIN, LOW);
//     digitalWrite(RIGHT_LED_PIN, LOW);
//   }
// }

#endif