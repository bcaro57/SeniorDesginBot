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
#define GREEN_LED_PIN A1
#define LEFT_LED_PIN A2
#define MIDDLE_LED_PIN A3
#define RIGHT_LED_PIN A4

#define MOTOR1_RPWM_Output 6
#define MOTOR1_LPWM_Output 5
#define MOTOR2_RPWM_Output 10
#define MOTOR2_LPWM_Output 9
#define MOTOR3_RPWM_Output 12
#define MOTOR3_LPWM_Output 11

void setup() {
  // for the LED
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(MIDDLE_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);

  // for the motors
  pinMode(MOTOR1_RPWM_Output, OUTPUT); 
  pinMode(MOTOR1_LPWM_Output, OUTPUT); 
  pinMode(MOTOR2_RPWM_Output, OUTPUT); 
  pinMode(MOTOR2_LPWM_Output, OUTPUT); 
  pinMode(MOTOR3_RPWM_Output, OUTPUT); 
  pinMode(MOTOR3_LPWM_Output, OUTPUT); 

  // for the transmission
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
  if(rf95.available()) {

    uint8_t buf[1];
    uint8_t len = sizeof(buf);

    if(rf95.recv(buf, &len)) {

      if (buf[0] == 0x01) {
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(LEFT_LED_PIN, HIGH);
        digitalWrite(MIDDLE_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, LOW);

        Serial.println("Motor 1 moves now!");
        analogWrite(MOTOR1_RPWM_Output, 0);
        analogWrite(MOTOR1_LPWM_Output, 250);
        analogWrite(MOTOR2_RPWM_Output, 0);
        analogWrite(MOTOR2_LPWM_Output, 0);
        analogWrite(MOTOR3_RPWM_Output, 0);
        analogWrite(MOTOR3_LPWM_Output, 0);
      }

      else if (buf[0] == 0x02) {
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(MIDDLE_LED_PIN, HIGH);
        digitalWrite(RIGHT_LED_PIN, LOW);

        Serial.println("Motor 2 moves now!");
        analogWrite(MOTOR1_RPWM_Output, 0);
        analogWrite(MOTOR1_LPWM_Output, 0);
        analogWrite(MOTOR2_RPWM_Output, 0);
        analogWrite(MOTOR2_LPWM_Output, 250);
        analogWrite(MOTOR3_RPWM_Output, 0);
        analogWrite(MOTOR3_LPWM_Output, 0);
      }

      else if (buf[0] == 0x03) {
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(MIDDLE_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, HIGH);

        Serial.println("Motor 3 moves now!");
        analogWrite(MOTOR1_RPWM_Output, 0);
        analogWrite(MOTOR1_LPWM_Output, 0);
        analogWrite(MOTOR2_RPWM_Output, 0);
        analogWrite(MOTOR2_LPWM_Output, 0);
        analogWrite(MOTOR3_RPWM_Output, 0);
        analogWrite(MOTOR3_LPWM_Output, 250);
      }

      else if (buf[0] == 0x04) {
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(LEFT_LED_PIN, HIGH);
        digitalWrite(MIDDLE_LED_PIN, HIGH);
        digitalWrite(RIGHT_LED_PIN, HIGH);

        Serial.println("All motors move now!");
        analogWrite(MOTOR1_RPWM_Output, 0);
        analogWrite(MOTOR1_LPWM_Output, 250);
        analogWrite(MOTOR2_RPWM_Output, 0);
        analogWrite(MOTOR2_LPWM_Output, 250);
        analogWrite(MOTOR3_RPWM_Output, 0);
        analogWrite(MOTOR3_LPWM_Output, 250);
      }

      else {
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(MIDDLE_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, LOW);

        Serial.println("No motors are moving now.");
        analogWrite(MOTOR1_RPWM_Output, 0);
        analogWrite(MOTOR1_LPWM_Output, 0);
        analogWrite(MOTOR2_RPWM_Output, 0);
        analogWrite(MOTOR2_LPWM_Output, 0);
        analogWrite(MOTOR3_RPWM_Output, 0);
        analogWrite(MOTOR3_LPWM_Output, 0);
      }

    }

    else {
      Serial.println("Receive failed");
    }
  
  }
}