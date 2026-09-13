#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <Arduino.h>

// Strength of a TSSP58P38 = width of its LOW output pulse (microseconds).
// Wider pulse  = stronger / more on-axis signal.
// Returns 0 if no valid pulse arrives before the timeout (sensor silent/blinded).
unsigned long readStrength(int pin);

// Convenience: did this sensor see the beacon at all?
bool sawBeacon(int pin);

#endif
