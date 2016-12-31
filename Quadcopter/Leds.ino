
void leds_initialize(){
 pinMode(PIN_LED, OUTPUT);
 digitalWrite(PIN_LED, LOW); 
}
/* 
void leds_status(byte stat){
 digitalWrite(PIN_LED, stat);  
}
*/ 
// this provides a heartbeat on pin 13, so you can tell the software is running.
uint8_t hbval = 128;
int8_t hbdelta = 8;
void heartbeat() {
  static unsigned long last_time = 0;
  unsigned long now = millis();
  if ((now - last_time) < 40)
    return;
  last_time = now;
  if (hbval > 192) hbdelta = -hbdelta;
  if (hbval < 32) hbdelta = -hbdelta;
  hbval += hbdelta;
  analogWrite(PIN_LED, hbval);
}
