// ===== TSSP58P38 multi-sensor strength reader (power-cycled) =====
// Beacon is continuous, so we blink the sensors' power. On each power-up the
// AGC is fresh and detects the beacon for a short window whose LOW-pulse
// width = strength. All sensors are read together in the same ON window.
//
// Nothing here blocks: loop() must keep running the motor and gyro, so the
// OFF/ON cycle moves forward a little on every sensorsUpdate().
//
// Wiring:
//   All TSSP VS  -> D4 (shared power pin)
//   OUT: front->A0, right->A1, back->A2, left->A3 (body sides)
//   All GND -> GND

#include "Arduino.h"
#include "analog_sensor.h"

static const int POWER_PIN = 4;                  // shared VS for all sensors

static const int sensorPins[sensorNum] = {A0, A1, A2, A3};

static const unsigned long OFF_MS = 380;         // powered OFF (resets the AGC)
static const unsigned long ON_US = (unsigned long)ON_MS * 1000UL;

struct Pulse {
  bool low;                  // went LOW
  bool ended;                // came back HIGH
  unsigned long lowUs;
  unsigned long highUs;
};

static Pulse pulse[sensorNum];
static SensorFrame frame;
static bool frameReady = false;
static bool powered = false;
static unsigned long offStartMs = 0;
static unsigned long onStartUs = 0;
static unsigned long onStartMs = 0;

static void powerOff() {
  for (int i = 0; i < sensorNum; i++) pinMode(sensorPins[i], INPUT);  // never feed a powered-off sensor
  digitalWrite(POWER_PIN, LOW);
  powered = false;
  offStartMs = millis();
}

static void powerOn() {
  memset(pulse, 0, sizeof pulse);
  digitalWrite(POWER_PIN, HIGH);
  for (int i = 0; i < sensorNum; i++) pinMode(sensorPins[i], INPUT_PULLUP);  // an unplugged pin must read HIGH
  powered = true;
  onStartUs = micros();
  onStartMs = millis();
}

static void pollPulses(unsigned long now) {
  for (int i = 0; i < sensorNum; i++) {
    Pulse &p = pulse[i];
    if (p.ended) continue;                       // first pulse only
    bool low = (digitalRead(sensorPins[i]) == LOW);
    if (low && !p.low) {
      p.low = true;
      p.lowUs = now;
    } else if (!low && p.low) {
      p.ended = true;
      p.highUs = now;
    }
  }
}

static void finishFrame() {
  unsigned long endUs = onStartUs + ON_US;       // never came back HIGH: saturated
  for (int i = 0; i < sensorNum; i++) {
    const Pulse &p = pulse[i];
    frame.strength[i] = p.low ? (p.ended ? p.highUs : endUs) - p.lowUs : 0;
  }
  frame.onStartMs = onStartMs;
  frameReady = true;
  powerOff();
}

void setupSensor() {
  pinMode(POWER_PIN, OUTPUT);
  for (int i = 0; i < sensorNum; i++) pinMode(sensorPins[i], INPUT);
  powerOff();
}

void sensorsUpdate() {
  if (!powered) {
    if (millis() - offStartMs >= OFF_MS) powerOn();
    return;
  }
  unsigned long now = micros();
  if (now - onStartUs < ON_US) pollPulses(now);
  else finishFrame();
}

bool sensorsNewFrame() {
  bool ready = frameReady;
  frameReady = false;
  return ready;
}

const SensorFrame &sensorFrame() {
  return frame;
}
