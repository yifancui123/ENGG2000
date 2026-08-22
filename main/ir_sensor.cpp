#include "ir_sensor.h"   // pulls in the prototypes, LENGTH/TOLERANCE, and Arduino.h

// ---- Check ONE sensor. Returns true if it sees a real beacon. ----
bool sawBeacon(int pin) {
#ifdef SIMULATION
  // --- SIM MODE (Wokwi): a pressed button (pin LOW) = "beacon seen". ---
  // A button gives a steady LOW, so we can't measure a 190us pulse here — this
  // only tests the scan/decision LOGIC, not the pulse-length fingerprint.
  return digitalRead(pin) == LOW;
#else
  unsigned long t0 = micros();

  // Step 1: wait for the pin to go LOW (a burst starting).
  // Give up after 12ms (longer than one 5ms beacon period) so we don't hang forever.
  while (digitalRead(pin) == HIGH) {  // find the burst
    if (micros() - t0 > 12000) return false;  // no burst arrived -> no beacon
  }

  // The beacon is CONSTANT for now, so simply seeing a burst = beacon found.
  return true;

  /* ---- Pulse-length "fingerprint" — DISABLED for now ----
     Re-enable this (and LENGTH/TOLERANCE in ir_sensor.h) once the beacon is
     MODULATED and you need to tell it apart from noise/sunlight by burst length.

  // Step 2: it's LOW now — start the stopwatch and wait for it to go back HIGH.
  // To check whether it is a real beacon
  unsigned long startTime = micros();
  while (digitalRead(pin) == LOW) {
    if (micros() - startTime > 5000) break;   // safety, don't get stuck
  }

  // Step 3: measure how long it stayed LOW.
  unsigned long pulse = micros() - startTime;

  // Step 4: the decision — is the length the beacon's fingerprint?
  if (pulse > LENGTH - TOLERANCE && pulse < LENGTH + TOLERANCE)
    return true;    // real beacon
  else
    return false;   // noise / sunlight / nothing
  */
#endif
}
