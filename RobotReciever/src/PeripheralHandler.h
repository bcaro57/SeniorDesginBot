#ifndef RADIO_HANDLER_H
#define RADIO_HANDLER_H

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

// Comm Pins for the Feather 32u4:
#define RFM95_CS   8
#define RFM95_RST  4
#define RFM95_INT  7

// Operating frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

/*
RadioInit - initializes the radio transmission protocols
*/

void RadioInit(){
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  Serial.println("Feather LoRa TX Connection!");
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

}


/*
ButtonToggle - takes in a specific buffer value and toggles a switch to be on or off (which has some slight debounce)
*/

class DataInfo {

  public:
    DataInfo(uint8_t _buf_data): bufData(_buf_data)
    {}

    bool ButtonToggle() {
      if ((bufData == 0x01) && (currentButtonState)) {
        currentButtonState = !currentButtonState;
        ToggleState = !ToggleState;
        delay(100);
      }
      else if ((bufData == 0x01) && (!currentButtonState)) {
        currentButtonState = !currentButtonState;
      }
      return ToggleState;
    }

  private:
    uint8_t bufData;
    bool currentButtonState;
    bool ToggleState;
};



/*
mcpInit - initializes the IO Expander with the 'begin_I2C' method
*/

void mcpInit(Adafruit_MCP23X17* mcp) {
  if (!mcp->begin_I2C()) {
    Serial.println("Error.");
    while (1);
  } 
}


/*
LED - designed to handle the onboard indicator LEDs to help inform the user of what's going on

inputs into the constructor: each of the LEDs (green, left, middle, right) and the MCP (IO expander)

methods:
  init()            -> initializes the LEDs, checking if they need to be set as expander pins
  mcpHelper()       -> if the expander is registered, treat any pins as expander pins
  turnOn()          -> uses the mcpHelper and the buffer to determine which LEDs are on

variables:
  GreenLED          -> the pin for the green LED
  LeftLED           -> the pin for the LED on the left
  MiddleLED         -> the pin for the LED in the middle
  RightLED          -> the pin for the LED on the right
  MCP               -> the IO expansion board object
*/

class LED {

  public:
    LED(int _green_pin, int _left_pin, int _middle_pin, int _right_pin, Adafruit_MCP23X17* _mcp = NULL): GreenLED(_green_pin),
                                                                                                         LeftLED(_left_pin),
                                                                                                         MiddleLED(_middle_pin),
                                                                                                         RightLED(_right_pin),
                                                                                                         MCP(_mcp)
    {}

    void init() {

      if (!MCP) {
        pinMode(GreenLED, OUTPUT);
        pinMode(LeftLED, OUTPUT);
        pinMode(MiddleLED, OUTPUT);
        pinMode(RightLED, OUTPUT);
      }
      else {
        MCP->pinMode(GreenLED, OUTPUT);
        MCP->pinMode(LeftLED, OUTPUT);
        MCP->pinMode(MiddleLED, OUTPUT);
        MCP->pinMode(RightLED, OUTPUT);
      }
    }
    
    void mcpHelper(int LED, bool state) {

      if (!MCP) {
        digitalWrite(LED, state);
      }
      else {
        MCP->digitalWrite(LED, state);
      }
    }
    
    void turnOn(bool GreenOn, bool LeftOn, bool MiddleOn, bool RightOn) {

      mcpHelper(GreenLED, GreenOn);
      mcpHelper(LeftLED, LeftOn);
      mcpHelper(MiddleLED, MiddleOn);
      mcpHelper(RightLED, RightOn);
    }

    void update(uint8_t buf, int ToggleState) {
      
      if (buf == 0x01) {
        turnOn(1, 1, 1, 1);
      }

      else if (buf == 0x02) {
        turnOn(1, 1, 0, 0);
      }

      else if (buf == 0x03) {
        turnOn(1, 0, 1, 0);
      }

      else if (buf == 0x04) {
        turnOn(1, 0, 0, 1);
      }

      else if (ToggleState) {
        turnOn(1, 1, 1, 1);
      }

      else {
        turnOn(1, 0, 0, 0);
      }
    }

  private:

    int GreenLED;
    int LeftLED;
    int MiddleLED;
    int RightLED;

    Adafruit_MCP23X17* MCP;
};



#endif