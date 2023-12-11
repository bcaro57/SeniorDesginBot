#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include <AccelStepper.h>

#include "MotorHandler.h"
#include "PeripheralHandler.h"
#include "pindefs.h"

// Instances of all motors/peripherals
Adafruit_MCP23X17 mcp;
LED Led(GREEN_LED_PIN, LEFT_LED_PIN, MIDDLE_LED_PIN, RIGHT_LED_PIN, &mcp);
StepperDriver Balancer(STEPPER_PIN_PUL, STEPPER_PIN_DIR, LIMIT_SWITCH_PIN);

MotorDriver LeftMotor(L_MOTOR_LPWM, L_MOTOR_RPWM);
MotorDriver MiddleMotor(M_MOTOR_LPWM, M_MOTOR_RPWM);
MotorDriver RightMotor(R_MOTOR_LPWM, R_MOTOR_RPWM);

MotorDriver LeftActuator(L_ACTUATOR_LPWM, L_ACTUATOR_RPWM, &mcp);
MotorDriver MiddleActuator(M_ACTUATOR_LPWM, M_ACTUATOR_RPWM, &mcp);
MotorDriver RightActuator(R_ACTUATOR_LPWM, R_ACTUATOR_RPWM, &mcp);

Encoder * Encoder::instances [3] = { NULL, NULL, NULL };
Encoder LeftEncoder(pulsePin::feather0, L_ENCODER_A, L_ENCODER_B, &mcp);
Encoder MiddleEncoder(pulsePin::feather1, M_ENCODER_A, M_ENCODER_B, &mcp);
Encoder RightEncoder(pulsePin::mcp0, R_ENCODER_A, R_ENCODER_B, &mcp);


// Instances of higher level controller for drive motors and actuators
DriveControl DriveController(&LeftMotor, &MiddleMotor, &RightMotor);
ActuatorControl ActuatorController(&LeftActuator, &MiddleActuator, &RightActuator, &Balancer, &mcp);


// Instances of the data handling objects
DataInfo DriveData;
DataInfo LeftActuatorToggle;
DataInfo MiddleActuatorToggle;
DataInfo RightActuatorToggle;


// Defining the length of data being recieved (how many bytes)
int bufLen = 3;

/*

The bytes of data recieved are used in the following manner:

  byte 1 (buf[0])   -> data in charge of the drive motors, specific to the buttons being pressed
  byte 2 (buf[1])   -> data in charge of the linear actuators, specific to the buttons being pressed
  byte 3 (buf[2])   -> data in charge of the speed of the drive motors, specific to the joystick inputs

*/

void setup() {
  // mcp.setupInterrupts(true, false, LOW);

  Serial.begin(115200);
  mcpInit(&mcp);
  RadioInit();
  Led.init();
  DriveController.init(); 
  ActuatorController.init();
  Balancer.init();
  // LeftEncoder.init();
  // MiddleEncoder.init();
  // RightEncoder.init();
}


void loop() {
  Serial.println("we're in the loop");
  if(rf95.available()) {

    uint8_t buf[bufLen];
    uint8_t len = sizeof(buf);

    if(rf95.recv(buf, &len)) {
      // setting the button toggle variable
      bool AutonomousToggle = DriveData.ButtonToggle(buf[0], 0x01);

      int DriveSpeed = DriveData.MapValue(buf[2]);
      // updating each controller
      Led.update(buf[0], AutonomousToggle);
      Led.update(buf[1], AutonomousToggle);
      DriveController.setSpeed(DriveSpeed);
      DriveController.update();
      ActuatorController.update(buf[1]);
    }
    
    else {
      Serial.println("Receive failed");
    }
  }
}