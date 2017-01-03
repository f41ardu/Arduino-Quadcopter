// http://de.wikibooks.org/wiki/C%2B%2B-Programmierung:_Klassen

//if defined(ARDUINO) && ARDUINO >= 100
//#include "Arduino.h"  // for digitalRead, digitalWrite, etc
//#else
//#include "WProgram.h"
//#endif
#include "PinClass.h"

PinClass::PinClass(): // Constructor
  _pin(0), _status(0)
{
  //leerer Constructor;
}

PinClass::PinClass(int a): // Constructor mit Initialisierung
  _pin(a), _status(0)
{
  init(_pin);
}

PinClass::PinClass(int pin, unsigned int OnTime, unsigned int OffTime): // Contructor
  _OnTime(OnTime), _OffTime(OffTime), _pin(pin)
{
  init(_pin);
  _previousMillis = millis();
 off();
}

PinClass::~PinClass() // Destructor
{
}

void PinClass::init(int pin) // LED intialsieren
{
  _pin = pin;                   // speichert den LED Pin in der privaten Variable _led
  pinMode(_pin, OUTPUT);
}
void PinClass::on() // LED ein
{
  _status = true;
  digitalWrite(_pin, HIGH); //set the pin HIGH and thus turn LED on
}

void PinClass::off() // LED aus
{
  _status = false;
  digitalWrite(_pin, LOW); //set the pin HIGH and thus turn LED off
}

void PinClass::blink(int intervall) // LED einmal intervall ms blinken lassen
{
  on();
  delay(intervall / 2);
  off();
  delay(intervall / 2);
}

void PinClass::fade(int value) // LED Helligkeit setzen
{
  analogWrite(_pin, value);
}

void PinClass::toggle()
{
  _status ? off() : on();
}

void PinClass::timechange(int OnTime, int OffTime) // 
{
  _OnTime=OnTime;
  _OffTime=OffTime;
}

void PinClass::flash() {
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


