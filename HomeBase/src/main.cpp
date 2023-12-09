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
#define BUTTON_LEFT 9
#define BUTTON_MIDDLE 10
#define BUTTON_RIGHT 12
#define BUTTON_BOTTOM 11
#define JOYSTICK A5

// LED pins
#define LED_RED 6
#define LED_GREEN 5
#define LED_BLUE 3

// misc variables
int delayTime = 10;   // length of the delay (10ms)
int bufLen = 3;       // length of the buffer (3 bytes of data)
int joystickNeutral;  // a variable for the joystick to have a deadzone (so that it has a larger "stopped" radius)
int deadzone = 10;    // the deadzone value set for calibrating the center

void setup() {
  // for the buttons
  pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_MIDDLE, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);
  pinMode(BUTTON_BOTTOM, INPUT_PULLUP);
  pinMode(JOYSTICK, INPUT);

  // for the leds
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  // getting the neutral reading for calibration
  joystickNeutral = analogRead(JOYSTICK)/4;
  

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


void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(LED_RED, redValue);
  analogWrite(LED_GREEN, greenValue);
  analogWrite(LED_BLUE, blueValue);
}

void loop() {

  analogWrite(LED_BLUE, 0);
  analogWrite(LED_GREEN, 0);
  analogWrite(LED_RED, 0);
  
  uint8_t buf[bufLen];
  if (digitalRead(BUTTON_LEFT) == LOW ) {
    Serial.print("The 'ALL' button is being pressed and the value is ");
    buf[0] = 0x01;
    setColor(255, 0, 0);
    Serial.println(buf[0]);
    delay(delayTime);
  }
  else if (digitalRead(BUTTON_MIDDLE) == LOW ) {
    Serial.print("The middle button is being pressed and the value is ");
    buf[1] = 0x02;
    Serial.println(buf[1]);
    setColor(0, 255, 0);
    delay(delayTime);
  }
  else if (digitalRead(BUTTON_RIGHT) == LOW ) {
    Serial.print("The right button is being pressed and the value is ");
    buf[1] = 0x03;
    Serial.println(buf[1]);
    setColor(0, 0, 255);
    delay(delayTime);
  }
  else if (digitalRead(BUTTON_BOTTOM) == LOW ) {
    Serial.print("The 'all' button is being pressed and the value is ");
    buf[1] = 0x04;
    Serial.println(buf[1]);
    setColor(255, 255, 255);
    delay(delayTime);
  }
  else {
    Serial.print("The buttons are not being pressed and the value is ");
    buf[0] = 0x00;
    buf[1] = 0x00;
    Serial.println(buf[0]);
    delay(delayTime);
  }
  
  int joystickVal = analogRead(JOYSTICK);   

  if (joystickVal >= joystickNeutral + deadzone || joystickVal <= joystickNeutral - deadzone) {
    buf[2] = map(joystickVal, 0, 1023, 0, 255);
  }
  else {
    buf[2] = 128;
  }

  Serial.print("The joystick value is ");
  Serial.println(buf[2]);

  rf95.send((uint8_t *)buf, bufLen);
  rf95.waitPacketSent();
}