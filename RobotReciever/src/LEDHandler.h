#ifndef __LED_HANDLER_H__
#define __LED_HANDLER_H__

#include <Arduino.h>

#define GREEN_LED_PIN A1
#define LEFT_LED_PIN A2
#define MIDDLE_LED_PIN A3
#define RIGHT_LED_PIN A4

void LEDSetup(){
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(MIDDLE_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);
}

void LEDController(uint8_t buf, int MotorState){
  if (buf == 0x01) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(LEFT_LED_PIN, HIGH);
    digitalWrite(MIDDLE_LED_PIN, LOW);
    digitalWrite(RIGHT_LED_PIN, LOW);
  }

  else if (buf == 0x02) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(LEFT_LED_PIN, LOW);
    digitalWrite(MIDDLE_LED_PIN, HIGH);
    digitalWrite(RIGHT_LED_PIN, LOW);
  }

  else if (buf == 0x03) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(LEFT_LED_PIN, LOW);
    digitalWrite(MIDDLE_LED_PIN, LOW);
    digitalWrite(RIGHT_LED_PIN, HIGH);
  }

  else if (buf == 0x04) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(LEFT_LED_PIN, HIGH);
    digitalWrite(MIDDLE_LED_PIN, HIGH);
    digitalWrite(RIGHT_LED_PIN, HIGH);
  }

  else if(MotorState){
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(LEFT_LED_PIN, HIGH);
    digitalWrite(MIDDLE_LED_PIN, LOW);
    digitalWrite(RIGHT_LED_PIN, LOW);
  }

  else {
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(LEFT_LED_PIN, LOW);
    digitalWrite(MIDDLE_LED_PIN, LOW);
    digitalWrite(RIGHT_LED_PIN, LOW);
  }
}

#endif