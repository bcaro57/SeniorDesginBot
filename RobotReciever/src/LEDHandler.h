#ifndef LED_HANDLER_H
#define LED_HANDLER_H

class LED{

  public:
    LED(int _green_pin, int _left_pin, int _middle_pin, int _right_pin);

    void init();
    void turnOn(bool GreenOn, bool LeftOn, bool MiddleOn, bool RightOn);
    void update(uint8_t buf, int MotorState);
  private:

    int GreenLED;
    int LeftLED;
    int MiddleLED;
    int RightLED;
};

#endif