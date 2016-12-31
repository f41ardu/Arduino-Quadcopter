//#if defined(ARDUINO) && ARDUINO >= 100
//#include "Arduino.h"	// for digitalRead, digitalWrite, etc
//#else
//#include "WProgram.h"
//#endif
//#include "class.h"
#include "PinClass.h"
#include "LED_C.h"

LED::LED(): // Constructor
  _status(0)
{
  //leerer Constructor;
}

LED::LED(int pin): PinClass(pin), // Constructor mit Initialisierung
  _status(0), _pin(pin)
{

}

LED::LED(int pin, bool on_off): PinClass(pin), // Constructor mit Initialisierung
  _status(on_off), _pin(pin)
{
  _status ? p_off() : p_on();
}

LED::LED(int pin, unsigned int OnTime, unsigned int OffTime): PinClass(pin), // Contructor
  _OnTime(OnTime), _OffTime(OffTime), _pin(pin)
{
  _sweep = false;
  _previousMillis = millis();
  //   p_off();
}

LED::LED(int pin, unsigned int OffTime, unsigned int brightness, bool sweep): PinClass(pin), // Contructor
  _OffTime(OffTime), _brightness(brightness), _pin(pin), _sweep(sweep)
{
  if (_sweep == true) {
    _previousMillis = millis();
    _add = 1;
    _OffTime = _OffTime / (255 / 5);
  } else {
    _sweep = false;
    _previousMillis = millis();
    _status ? p_off() : p_on();
  }
}


// Destructor
LED::~LED()
{
}

int LED::getStatus()
{
  return _status;
}

void LED::p_on()
{
  on();
  _status = true;
}

void LED::p_off()
{
  off();
  _status = false;
  _brightness = 0;
}

void LED::toggle()
{
  _status ? p_off() : p_on();
  _brightness = 255;
}

void LED::mblink(int duration, int count) {
  for ( byte i = 0; i <= count; i++) {
    blink(duration);
  }
}

void LED::mblink(int duration) {
  blink(duration);
}

void LED::flash() {
  // check to see if it's time to change the state of the LED
  if ((_currentMillis - _previousMillis >= _OnTime))
  {
    toggle();  // Turn it off
    _previousMillis = _currentMillis;  // Remember the time
  }
  else if ((_currentMillis - _previousMillis >= _OffTime))
  {
    toggle();  // turn it on
    _previousMillis = _currentMillis;   // Remember the time
  }
  _currentMillis = millis();
}


void LED::sweep() {
  // check to see if it's time to change the state of the LED

  if ( _currentMillis - _previousMillis >= _OffTime ) {

    _previousMillis = _currentMillis;  // Remember the time
    analogWrite(_pin, _brightness);
    _brightness = _brightness + _add;

    if (_brightness == 0 || _brightness == 255) {
      _add = -_add ;
    }
  }
  _currentMillis = millis();
}

void LED::update() {
  if ( _sweep == true ) {
    sweep();
  } else {
    flash();
  }
}

//assume PWM
void LED::fadeIn(int time) {
  for (byte value = 0 ; value < 255; value += 5) {
    fade(value);
    delay(time / (255 / 5));
  }
  p_on();
}

//assume PWM
void LED::fadeOut(int time) {
  for (byte value = 255; value > 0; value -= 5) {
    fade(value);
    delay(time / (255 / 5));
  }
  p_off();
}

//assume PWM
void LED::fadeInOut(int time) {
  fadeIn(time);
  fadeOut(time);
  p_off();
}
