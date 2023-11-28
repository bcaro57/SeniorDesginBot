#include <Arduino.h>
#include <Adafruit_PCF8575.h>

#include "MotorHandler.h"
#include "LEDHandler.h"
#include "RadioHandler.h"
#include "pindefs.h"


// Instance of the IO Expander
Adafruit_PCF8575 pcf;


// Instances of all motors/peripherals
LED Led(GREEN_LED_PIN, LEFT_LED_PIN, MIDDLE_LED_PIN, RIGHT_LED_PIN);

MotorDriver LeftMotor(L_MOTOR_LPWM, L_MOTOR_RPWM);
MotorDriver MiddleMotor(M_MOTOR_LPWM, M_MOTOR_RPWM);
MotorDriver RightMotor(R_MOTOR_LPWM, R_MOTOR_RPWM);

MotorDriver LeftActuator(L_ACTUATOR_LPWM, L_ACTUATOR_RPWM, &pcf);
MotorDriver MiddleActuator(M_ACTUATOR_LPWM, M_ACTUATOR_RPWM, &pcf);
MotorDriver RightActuator(R_ACTUATOR_LPWM, R_ACTUATOR_RPWM, &pcf);


// Instance of higher level controller
MotorControl DriveController(&LeftMotor, &MiddleMotor, &RightMotor);


void setup() {
  Serial.begin(115200);
  RadioInit();
  IOExpanderInit();
  DriveController.init(); 
  Led.init();
}


void loop() {

  if(rf95.available()) {

    uint8_t buf[1];
    uint8_t len = sizeof(buf);

    if(rf95.recv(buf, &len)) {
      MotorState = ButtonToggle(buf[0]);
      Led.update(buf[0], MotorState);
      DriveController.update(buf[0], MotorState);
    }
    else {
      Serial.println("Receive failed");
    }
  }
}