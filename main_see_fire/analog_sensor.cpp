// ===== TSSP58P38 multi-sensor strength reader (power-cycled) =====
// Beacon is continuous, so we blink the sensors' power. On each power-up the
// AGC is fresh and detects the beacon for a short window whose LOW-pulse
// width = strength. All sensors are read together in the same ON window.
//
// Wiring:
//   All TSSP VS  -> D4 (shared power pin)
//   OUT: front->A0, the other three->A1, A2, A3 (temporary pins)
//   All GND -> GND

#include "Arduino.h"
#include "analog_sensor.h"

static const int POWER_PIN = 4;                  // shared VS for all sensors

static const int sensorPins[sensorNum] = {A0, A1, A2, A3};

static const int OFF_MS = 380;   // powered OFF (resets the AGC)

void setupSensor() {
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);
  for (int i = 0; i < sensorNum; i++) pinMode(sensorPins[i], INPUT);
  delay(300);
}

// One power-cycle for all sensors (about 500 ms in total).
// out[i] = first LOW-pulse width in us = strength, 0 if unseen.
void readStrengths(unsigned long out[sensorNum]) {
  unsigned long lowStart[sensorNum];
  bool isLow[sensorNum];
  bool done[sensorNum];
  for (int i = 0; i < sensorNum; i++) {
    out[i] = 0;
    lowStart[i] = 0;
    isLow[i] = false;
    done[i] = false;
  }

  digitalWrite(POWER_PIN, LOW);
  delay(OFF_MS);                                   // reset the AGC
  digitalWrite(POWER_PIN, HIGH);                   // fresh power-up
  unsigned long tStart   = micros();
  unsigned long windowUs = (unsigned long)ON_MS * 1000UL;

  while (micros() - tStart < windowUs) {
    unsigned long now = micros();
    int finished = 0;
    for (int i = 0; i < sensorNum; i++) {
      if (done[i]) {
        finished++;
        continue;
      }
      bool low = (digitalRead(sensorPins[i]) == LOW);
      if (low && !isLow[i]) {                      // falling edge: pulse starts
        lowStart[i] = now;
        isLow[i] = true;
      } else if (!low && isLow[i]) {               // rising edge: pulse ends
        out[i] = now - lowStart[i];
        done[i] = true;
      }
    }
    if (finished == sensorNum) break;
  }

  // went LOW but never came back HIGH (saturated): use the window end
  for (int i = 0; i < sensorNum; i++) {
    if (isLow[i] && !done[i]) out[i] = tStart + windowUs - lowStart[i];
  }
}
