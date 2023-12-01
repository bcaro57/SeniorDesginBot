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

Encoder * Encoder::instances [3] = { NULL, NULL, NULL };
Encoder LeftEncoder(L_ENCODER_A, L_ENCODER_B);
Encoder MiddleEncoder(M_ENCODER_A, M_ENCODER_B);
Encoder RightEncoder(R_ENCODER_A, R_ENCODER_B);


// Instances of higher level controller for drive motors and actuators
MotorControl DriveController(&LeftMotor, &MiddleMotor, &RightMotor);
MotorControl ActuatorController(&LeftActuator, &MiddleActuator, &RightActuator);


// Instances of the data handling objects
DataInfo DriveToggle(0x01);
DataInfo LeftActuatorToggle(0x02);
DataInfo MiddleActuatorToggle(0x03);
DataInfo RightActuatorToggle(0x04);


// Defining the length of data being recieved (how many bytes)
int bufLen = 3;

/*

The bytes of data recieved are used in the following manner:

  byte 1 (buf[0])   -> data in charge of the drive motors, specific to the buttons being pressed
  byte 2 (buf[1])   -> data in charge of the linear actuators, specific to the buttons being pressed
  byte 3 (buf[2])   -> data in charge of the speed of the drive motors, specific to the joystick inputs

*/

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
      // setting all button toggle variables
      bool MotorToggle = DriveToggle.ButtonToggle();
      bool LToggle = LeftActuatorToggle.ButtonToggle();
      bool MToggle = MiddleActuatorToggle.ButtonToggle();
      bool RToggle = RightActuatorToggle.ButtonToggle();

      // updating each controller
      Led.update(buf[0], DriveToggle.ButtonToggle());
      DriveController.setSpeed(buf[2]);
      DriveController.update(buf[0], DriveToggle.ButtonToggle());
      ActuatorController.update(buf[1], false, LToggle, MToggle, RToggle);
    }
    else {
      Serial.println("Receive failed");
    }
  }
}