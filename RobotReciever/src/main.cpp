#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

#include "MotorHandler.h"
#include "PeripheralHandler.h"
#include "pindefs.h"


// Instances of all motors/peripherals
Adafruit_MCP23X17 mcp;
LED Led(GREEN_LED_PIN, LEFT_LED_PIN, MIDDLE_LED_PIN, RIGHT_LED_PIN, &mcp);

MotorDriver LeftMotor(L_MOTOR_LPWM, L_MOTOR_RPWM);
MotorDriver MiddleMotor(M_MOTOR_LPWM, M_MOTOR_RPWM);
MotorDriver RightMotor(R_MOTOR_LPWM, R_MOTOR_RPWM);

MotorDriver LeftActuator(L_ACTUATOR_LPWM, L_ACTUATOR_RPWM);
MotorDriver MiddleActuator(M_ACTUATOR_LPWM, M_ACTUATOR_RPWM);
MotorDriver RightActuator(R_ACTUATOR_LPWM, R_ACTUATOR_RPWM);

// Encoder * Encoder::instances [3] = { NULL, NULL, NULL };
// Encoder LeftEncoder(L_ENCODER_A, L_ENCODER_B);
// Encoder RightEncoder(L_ENCODER_A, L_ENCODER_B);
// Encoder MiddleEncoder(L_ENCODER_A, L_ENCODER_B);


// Instance of higher level controller for drive motors and actuators
MotorControl DriveController(&LeftMotor, &MiddleMotor, &RightMotor);
MotorControl ActuatorController(&LeftActuator, &MiddleActuator, &RightActuator);

int bufLen = 3;

void setup() {
  Serial.begin(115200);
  mcpInit(&mcp);
  RadioInit();
  Led.init();
  DriveController.init(); 
  ActuatorController.init();
}


void loop() {

  if(rf95.available()) {

    uint8_t buf[bufLen];
    uint8_t len = sizeof(buf);

    if(rf95.recv(buf, &len)) {
      MotorState = ButtonToggle(buf[0]);
      Led.update(buf[0], MotorState);
      DriveController.setSpeed(buf[2]);
      DriveController.update(buf[0], MotorState);
      ActuatorController.update(buf[1]);
    }
    else {
      Serial.println("Receive failed");
    }
  }
}