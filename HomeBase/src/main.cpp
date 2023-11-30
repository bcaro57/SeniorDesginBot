#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

// Feather 32u4:
#define RFM95_CS   8
#define RFM95_RST  4
#define RFM95_INT  7

// Operating frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Defining the pins being used
#define ALL_BUTTON 9
#define BUTTON_PIN_RIGHT 10
#define BUTTON_PIN_MIDDLE 11
#define BUTTON_PIN_LEFT 12


void setup() {
  // for the button
  pinMode(ALL_BUTTON, INPUT_PULLUP);
  pinMode(BUTTON_PIN_RIGHT, INPUT_PULLUP);
  pinMode(BUTTON_PIN_MIDDLE, INPUT_PULLUP);
  pinMode(BUTTON_PIN_LEFT, INPUT_PULLUP);

  // for the radio stuff
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  Serial.begin(115200);
  delay(100);
  Serial.println("Feather LoRa TX Test!");
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


void loop() {
  int delayTime = 10;
  
  uint8_t buttonValue[1];
  if (digitalRead(BUTTON_PIN_LEFT) == LOW ) {
    Serial.print("The left button is being pressed and the value is ");
    buttonValue[0] = 0x01;
    Serial.println(buttonValue[0]);
    delay(delayTime);
  }
  else if (digitalRead(BUTTON_PIN_MIDDLE) == LOW ) {
    Serial.print("The middle button is being pressed and the value is ");
    buttonValue[0] = 0x02;
    Serial.println(buttonValue[0]);
    delay(delayTime);
  }
  else if (digitalRead(BUTTON_PIN_RIGHT) == LOW ) {
    Serial.print("The right button is being pressed and the value is ");
    buttonValue[0] = 0x03;
    Serial.println(buttonValue[0]);
    delay(delayTime);
  }
  else if (digitalRead(ALL_BUTTON) == LOW ) {
    Serial.print("The 'all' button is being pressed and the value is ");
    buttonValue[0] = 0x04;
    Serial.println(buttonValue[0]);
    delay(delayTime);
  }
  else {
    Serial.print("The buttons are not being pressed and the value is ");
    buttonValue[0] = 0x00;
    Serial.println(buttonValue[0]);
    delay(delayTime);
    }
  rf95.send((uint8_t *)buttonValue, 1);
  rf95.waitPacketSent();
}