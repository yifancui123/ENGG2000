// ===== TSSP58P38 multi-sensor strength reader (power-cycled) =====
// Beacon is continuous, so we blink the sensors' power. On each power-up the
// AGC is fresh and detects the beacon for a short window whose LOW-pulse
// width = strength.
//
// Wiring:
//   All TSSP VS  -> D8 (shared power pin)
//   OUT: front->A0, right->A3, back->A4, left->A5
//   All GND -> GND
//
// To add sensors: raise sensorNum and extend sensorPins in this order:
//   0=front 1=right 2=back 3=left

#include "Arduino.h"
#include "analog_sensor.h"

static const int POWER_PIN = 8;                  // shared VS for all sensors

static const int sensorPins[sensorNum] = {A0};   // e.g. {A0, A3, A4, A5} for all four

static const int OFF_MS = 25;   // powered OFF (resets the AGC)

// One power-cycle on ONE sensor: LOW-pulse width (us) = strength, 0 if unseen.
static unsigned long readStrengthOnce(int powerPin, int outPin) {
  digitalWrite(powerPin, LOW);
  delay(OFF_MS);                                   // reset the AGC
  digitalWrite(powerPin, HIGH);                    // fresh power-up
  unsigned long tStart   = micros();
  unsigned long windowUs = (unsigned long)ON_MS * 1000UL;

  while (digitalRead(outPin) == HIGH) {            // wait for detection
    if (micros() - tStart > windowUs) return 0;
  }
  unsigned long lowStart = micros();
  while (digitalRead(outPin) == LOW) {             // wait for AGC to blind it
    if (micros() - tStart > windowUs) break;
  }
  return micros() - lowStart;
}

void setupSensor() {
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);
  for (int i = 0; i < sensorNum; i++) pinMode(sensorPins[i], INPUT);
  delay(300);
}

// Reads the wired sensors one at a time (~50 ms each). Unwired slots stay 0.
void readStrengths(unsigned long out[sensorNum]) {
  for (int i = 0; i < 4; i++) out[i] = 0;          // default: unseen / not wired
  for (int i = 0; i < sensorNum; i++) {
    out[i] = readStrengthOnce(POWER_PIN, sensorPins[i]);
  }
}