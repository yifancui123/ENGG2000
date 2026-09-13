#include "ir_sensor.h"

// ---- Read ONE sensor's signal strength ----
// The TSSP58P38 OUT pin is DIGITAL: it only swings 0V <-> 5V.
// The strength/proximity information is encoded in TIME, not voltage,
// so we must MEASURE the LOW pulse width, NOT use analogRead().
//
// pulseIn(pin, LOW, timeout) waits for a HIGH->LOW->HIGH pulse and
// returns how long the pin stayed LOW, in microseconds.
unsigned long readStrength(int pin) {
  const unsigned long TIMEOUT_US = 30000UL;   // 30 ms: return 0 if no pulse
  return pulseIn(pin, LOW, TIMEOUT_US);
}

// True if the sensor produced at least one valid LOW pulse.
bool sawBeacon(int pin) {
  return readStrength(pin) > 0;
}
