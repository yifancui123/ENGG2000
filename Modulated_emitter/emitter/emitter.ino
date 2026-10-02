const int IR_LED_PIN = 3;   // must be a PWM-capable pin (~3, ~9, ~10, ~11 on Nano)
                              // tone() needs Timer2 internally on most boards,
                              // so avoid pins tied to other timers you're using elsewhere

void setup() {
  // Nothing to configure — tone() handles the pin mode itself
}

void loop() {
  tone(IR_LED_PIN, 38000);   // continuous 38kHz square wave, never stops
  // deliberately no noTone() call anywhere — that's what makes it CONTINUOUS
  // rather than pulsed/bursted
}