#ifndef IR_SENSOR_H
#define IR_SENSOR_H

// This header is what the main .ino #includes so it can SEE the functions
// that live in ir_sensor.cpp. (.cpp = the bodies, .h = the "menu" of what's available)
//
// NOTE: keep a header to DECLARATIONS only — no setup()/loop(), no function
// bodies, and do NOT #include <IRremote.hpp> here. That library has real code
// inside and must be included at most once, in a .cpp/.ino, or you get
// "multiple definition" errors. The beacon detection below does NOT need it.

#include <Arduino.h>   // gives digitalRead / micros / etc. inside a .cpp file.
                       // The .ino gets this automatically; a .cpp does NOT.

// ---- SIMULATION toggle ----
// Uncomment to TEST THE LOGIC in Wokwi with pushbuttons
// (button pressed = "this sensor sees the beacon").
// COMMENT IT OUT before flashing real hardware (so real pulse detection runs).
// #define SIMULATION

// Beacon "fingerprint" — DISABLED for now (beacon is constant, so we don't
// filter by burst length yet). Re-enable together with Steps 2-4 in
// ir_sensor.cpp once the beacon is modulated. MEASURE your real beacon first!
// #define LENGTH 190      // expected burst length in microseconds (PLACEHOLDER)
// #define TOLERANCE 60    // how far off is still OK (±)

// Check ONE sensor. Returns true if it sees a real beacon burst.
bool sawBeacon(int millis_scanning, int pin);

#endif
