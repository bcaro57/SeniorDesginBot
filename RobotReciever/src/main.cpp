#include <Arduino.h>

#include "MotorHandler.h"
#include "LEDHandler.h"
#include "RadioHandler.h"


void setup() {
  // for the LED
  LEDSetup();

  // for the motors
  MotorSetup();

  // for the transmission
  RadioSetup();
}

void loop() {
  if(rf95.available()) {

    uint8_t buf[1];
    uint8_t len = sizeof(buf);

    if(rf95.recv(buf, &len)) {
      MotorState = ButtonToggle(buf[0]);
      MotorsController(buf[0], MotorState);
      LEDController(buf[0], MotorState);
    }
    else {
      Serial.println("Receive failed");
    }
  
  }
}