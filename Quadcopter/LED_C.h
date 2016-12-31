#ifndef LED_C_H
#define LED_C_H

#include <Arduino.h> //It is very important to remember this! note that if you are using Arduino 1.0 IDE, change "WProgram.h" to "Arduino.h"
#include "PinClass.h"

class LED : public PinClass {
  public:
    LED();
    LED(int pin);
    LED(int pin, bool on_off);
    LED(int pin, unsigned int OnTime, unsigned int brightness, bool sweep);
    LED(int pin, unsigned int OnTime, unsigned int OffTime);
    ~LED();
    int getStatus();
    void p_on();                         // eine Funktion LED einschalten
    void p_off();                        // eine Funktion LED ausschalten
    void toggle();
    void flash();
    void sweep();
    void update();
    void fadeIn(int time);
    void fadeOut(int time);
    void fadeInOut(int time);
    void mblink(int duration);
    void mblink(int duration, int count);

  private:
    bool _status, _sweep;
    int _pin;
    int _add;
    unsigned int _brightness;
    unsigned int _previousMillis, _currentMillis, _OnTime, _OffTime;
};

#endif
