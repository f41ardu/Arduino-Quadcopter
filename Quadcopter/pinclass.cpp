// http://de.wikibooks.org/wiki/C%2B%2B-Programmierung:_Klassen

//if defined(ARDUINO) && ARDUINO >= 100
//#include "Arduino.h"	// for digitalRead, digitalWrite, etc
//#else
//#include "WProgram.h"
//#endif
#include "PinClass.h"

PinClass::PinClass(): // Constructor
  _pin(0)
{
  //leerer Constructor;
}

PinClass::PinClass(int a): // Constructor mit Initialisierung
  _pin(a)
{
  init(_pin);
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
  digitalWrite(_pin, HIGH); //set the pin HIGH and thus turn LED on
}

void PinClass::off() // LED aus
{
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


