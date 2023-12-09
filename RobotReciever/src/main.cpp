#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

#include "MotorHandler.h"
#include "PeripheralHandler.h"
#include "pindefs.h"

// Instances of all motors/peripherals
Adafruit_MCP23X17 mcp;
LED Led(GREEN_LED_PIN, LEFT_LED_PIN, MIDDLE_LED_PIN, RIGHT_LED_PIN, &mcp);
StepperDriver myStepper(STEPPER_PIN_PUL, STEPPER_PIN_DIR, LIMIT_SWITCH_PIN, NULL);

MotorDriver LeftMotor(L_MOTOR_LPWM, L_MOTOR_RPWM);
MotorDriver MiddleMotor(M_MOTOR_LPWM, M_MOTOR_RPWM);
MotorDriver RightMotor(R_MOTOR_LPWM, R_MOTOR_RPWM);

MotorDriver LeftActuator(L_ACTUATOR_LPWM, L_ACTUATOR_RPWM);
MotorDriver MiddleActuator(M_ACTUATOR_LPWM, M_ACTUATOR_RPWM);
MotorDriver RightActuator(R_ACTUATOR_LPWM, R_ACTUATOR_RPWM);

Encoder * Encoder::instances [3] = { NULL, NULL, NULL };
Encoder LeftEncoder(pulsePin::feather0, L_ENCODER_A, L_ENCODER_B, &mcp);
Encoder MiddleEncoder(pulsePin::feather1, M_ENCODER_A, M_ENCODER_B, &mcp);
Encoder RightEncoder(pulsePin::mcp0, R_ENCODER_A, R_ENCODER_B, &mcp);


// Instances of higher level controller for drive motors and actuators
DriveControl DriveController(&LeftMotor, &MiddleMotor, &RightMotor);
ActuatorControl ActuatorController(&LeftActuator, &MiddleActuator, &RightActuator, &myStepper, &mcp);


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
  // myStepper.init();
  // LeftEncoder.init();
  // MiddleEncoder.init();
  // RightEncoder.init();
  Serial.println("setup is good!");
}


void loop() {
  // myStepper.movePosition();

  if(rf95.available()) {

    uint8_t buf[bufLen];
    uint8_t len = sizeof(buf);

    if(rf95.recv(buf, &len)) {
      // setting the button toggle variable
      bool MotorToggle = DriveData.ButtonToggle(buf[0], 0x01);
      bool LToggle = LeftActuatorToggle.ButtonToggle(buf[1], 0x02);
      bool MToggle = MiddleActuatorToggle.ButtonToggle(buf[1], 0x03);
      bool RToggle = RightActuatorToggle.ButtonToggle(buf[1], 0x04);

      int DriveSpeed = DriveData.MapValue(buf[2]);

      // updating each controller
      Led.update(buf[0], MotorToggle);
      Led.update(buf[1], MotorToggle);
      DriveController.setSpeed(DriveSpeed);

      //   Serial.print("Left encoder position is ");
      // Serial.print(LeftEncoder.getPosition());
      // Serial.print(" and velocity is ");
      // Serial.println(LeftEncoder.getVelocity());
      //       Serial.print("      Middle encoder position is ");
      // Serial.print(MiddleEncoder.getPosition());
      // Serial.print(" and velocity is ");
      // Serial.print(MiddleEncoder.getVelocity());
      //       Serial.print("      Right encoder position is ");
      // Serial.print(RightEncoder.getPosition());
      // Serial.print(" and velocity is ");
      // Serial.println(RightEncoder.getVelocity());

      DriveController.update(buf[0], MotorToggle);
      ActuatorController.update(buf[1], LToggle, MToggle, RToggle);

      Serial.print("The Motor Toggle State is: ");
      Serial.print(MotorToggle);
      Serial.print(", LToggle is: ");
      Serial.print(LToggle);
      Serial.print(", MToggle is: ");
      Serial.print(MToggle);
      Serial.print(", RToggle is: ");
      Serial.print(RToggle);

      Serial.print(", the joystick says: ");
      Serial.println(DriveSpeed);


    }

    else {
      Serial.println("Receive failed");
    }
  }
}