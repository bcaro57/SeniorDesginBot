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
#define BUTTON_1 3
#define BUTTON_2 6
#define BUTTON_3 10
#define BUTTON_4 12
#define JOYSTICK A0

// LED pins
#define LED_1 2
#define LED_2 5
#define LED_3 9
#define LED_4 11

int delayTime = 10;
int bufLen = 3;

void setup() {
  // for the button
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
  pinMode(BUTTON_3, INPUT_PULLUP);
  pinMode(BUTTON_4, INPUT_PULLUP);
  pinMode(JOYSTICK, INPUT_PULLUP);

  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);

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

  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
  digitalWrite(LED_3, LOW);
  digitalWrite(LED_4, LOW);
  
  uint8_t buttonValue[bufLen];
  if (digitalRead(BUTTON_1) == LOW ) {
    Serial.print("The 'ALL' button is being pressed and the value is ");
    buttonValue[0] = 0x01;
    digitalWrite(LED_1, HIGH);
    Serial.println(buttonValue[0]);
    delay(delayTime);
  }
  else if (digitalRead(BUTTON_2) == LOW ) {
    Serial.print("The middle button is being pressed and the value is ");
    buttonValue[1] = 0x02;
    Serial.println(buttonValue[0]);
    digitalWrite(LED_2, HIGH);
    delay(delayTime);
  }
  else if (digitalRead(BUTTON_3) == LOW ) {
    Serial.print("The right button is being pressed and the value is ");
    buttonValue[1] = 0x03;
    Serial.println(buttonValue[0]);
    digitalWrite(LED_3, HIGH);
    delay(delayTime);
  }
  else if (digitalRead(BUTTON_4) == LOW ) {
    Serial.print("The 'all' button is being pressed and the value is ");
    buttonValue[1] = 0x04;
    Serial.println(buttonValue[0]);
    digitalWrite(LED_4, HIGH);
    delay(delayTime);
  }
  else {
    Serial.print("The buttons are not being pressed and the value is ");
    buttonValue[0] = 0x00;
    buttonValue[1] = 0x00;
    Serial.println(buttonValue[0]);
    delay(delayTime);
    }
  
  buttonValue[2] = analogRead(JOYSTICK)/4;
  Serial.print("The joystick value is ");
  Serial.println(buttonValue[1]);

  rf95.send((uint8_t *)buttonValue, bufLen);
  rf95.waitPacketSent();
}