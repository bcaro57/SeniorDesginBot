#include <Arduino.h>

#include "MotorHandler.h"
#include "LEDHandler.h"
#include "RadioHandler.h"
#include "pindefs.h"

LED Led(GREEN_LED_PIN, LEFT_LED_PIN, MIDDLE_LED_PIN, RIGHT_LED_PIN);
MotorDriver LeftMotor(L_MOTOR_RPWM, L_MOTOR_LPWM);
MotorDriver MiddleMotor(M_MOTOR_RPWM, M_MOTOR_LPWM);
MotorDriver RightMotor(R_MOTOR_RPWM, R_MOTOR_RPWM);
MotorControl DriveController(&LeftMotor, &MiddleMotor, &RightMotor);


void setup() {
  RadioInit();
  DriveController.init(); 
  Led.init();
}

void loop() {

  if(rf95.available()) {

    uint8_t buf[1];
    uint8_t len = sizeof(buf);

    if(rf95.recv(buf, &len)) {
      MotorState = ButtonToggle(buf[0]);
      DriveController.update(buf[0], MotorState);
      Led.update(buf[0], MotorState);
    }
    else {
      Serial.println("Receive failed");
    }
  
  }
}