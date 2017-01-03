// http://de.wikibooks.org/wiki/C%2B%2B-Programmierung:_Klassen

#ifndef PinClass_H
#define PinClass_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"  // for digitalRead, digitalWrite, etc
#else
#include "WProgram.h"
#endif 
//#include <Arduino.h> //It is very important to remember this! note that if you are using Arduino 1.0 IDE, change "WProgram.h" to "Arduino.h"

class PinClass
{
  public:                             // öffentlich
    PinClass();                       // der Default-Konstruktor
    PinClass(int a);                  // weiterer Konstruktor mit Parameter
    PinClass(int a,unsigned int OnTime, unsigned int OffTime);  
    //   PinClass(const LED& a);      // Copy-Konstruktor wird nicht benötigt
       ~PinClass();                   // Class Destruktor

    void init(int pin);               // einen PIN mit einem (Default-) Parameter LED Initialisieresn
    void on();                        // einen PIN einschalten
    void off();                       // einen PIN ausschalten
    void blink(int intervall);        // einen PIN für die länge von intervall (ms) ein- und ausschalten
    void fade(int value);             // eine Spannung (0 .. 5Volt/255) am PIN ausgeben
    void timechange(int OnTime, int OffTime); 
    void toggle();                    // PIN Zustand umschalten (ON/OFF, OFF/ON) 
    void flash();                     // PIN ein und auschalten (OnTime, OffTime) 

  private:                            // privat
     bool _status; 
    int _pin;
    unsigned int _previousMillis, _currentMillis, _OnTime, _OffTime;

};

#endif


