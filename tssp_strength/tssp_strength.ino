// ===== MQ Sentinel — TSSP58P38 Strength Test (pulse-width method) =====
// TSSP58P38 OUT is a DIGITAL pin; signal strength = LOW pulse width (us).
// We read it with pulseIn()/micros() timing, NOT analogRead().
// (analogRead only ever returns ~0 or ~1023 on a digital pin -> useless here.)

#include "ir_sensor.h"

// 0 = front, 1 = right, 2 = back, 3 = left.
// A2..A5 are used here as DIGITAL inputs (analog pins work as digital too).
int sensorPins[4] = {A2, A3, A4, A5};

void setup() {
  Serial.begin(9600);

  // TSSP58P38 has a push-pull output, so no INPUT_PULLUP needed.
  for (int i = 0; i < 4; i++) {
    pinMode(sensorPins[i], INPUT);
  }
}

void loop() {
  // Strength of each sensor = its LOW pulse width in microseconds.
  // Wider = stronger / more on-axis. 0 = sensor saw nothing this cycle.
  unsigned long front = readStrength(sensorPins[0]);
  unsigned long right = readStrength(sensorPins[1]);
  unsigned long back  = readStrength(sensorPins[2]);
  unsigned long left  = readStrength(sensorPins[3]);

  // Tab-separated output for the Serial Plotter.
  Serial.print("Front:");
  Serial.print(front);
  Serial.print("\tRight:");
  Serial.print(right);
  Serial.print("\tBack:");
  Serial.print(back);
  Serial.print("\tLeft:");
  Serial.println(left);

  delay(20);
}
