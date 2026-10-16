// test_driver_mode - find out how the DRV8874 is set up, and save it.
// Run this ONCE before the other test sketches (and again if the wiring changes).
//
// Serial Monitor at 115200. Motor clamped, battery ON, hands clear, then send:
//   d  -> detect the driver mode (3 short kicks of the wheel, ~15 s)
//   m  -> show the saved mode
//
// How it works: it tries three input patterns on D5/D6 for 0.15 s each, with
// the driver asleep (coasting) in between. Which patterns move the wheel tells
// us the mode:
//                       D5=0,D6=1   D5=1,D6=1   D5=1,D6=0
//   0 PH/EN (designed)     still       moves       moves
//   1 IN1/IN2              moves       still       moves
//   2 PH/EN, D5/D6 swapped moves       moves       still
// Then it saves the mode to EEPROM and checks that a short pulse stops properly.

#include "driver.h"

const int PIN_ENC_A = 2;
const int PIN_ENC_B = 3;

const long MOVED = 50;             // counts in a 0.15 s kick that mean "moved"

volatile long count = 0;
volatile uint8_t prevAB = 0;
const int8_t QDEC[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void encoderISR() {
  uint8_t ab = ((PIND >> 2) & 1) << 1 | ((PIND >> 3) & 1);
  count += QDEC[(prevAB << 2) | ab];
  prevAB = ab;
}

long readCount() {
  noInterrupts();
  long c = count;
  interrupts();
  return c;
}

// Wait until the wheel has stopped (no counts for 0.5 s). False on timeout.
bool waitStill() {
  unsigned long t0 = millis();
  long last = readCount();
  unsigned long still = millis();
  while (millis() - still < 500) {
    if (millis() - t0 > 15000) return false;
    long now = readCount();
    if (now != last) { last = now; still = millis(); }
  }
  return true;
}

// Put a fixed pattern on D5/D6, wake the driver for 0.15 s, then sleep again.
long kick(int a, int b) {
  coast();
  if (!waitStill()) return -99999;
  digitalWrite(PIN_DRV_A, a);
  digitalWrite(PIN_DRV_B, b);
  long before = readCount();
  digitalWrite(PIN_SLEEP, HIGH);
  delay(150);
  long moved = readCount() - before;
  coast();
  Serial.print(F("#   D5=")); Serial.print(a);
  Serial.print(F(" D6=")); Serial.print(b);
  Serial.print(F(" -> moved ")); Serial.print(moved);
  Serial.println(labs(moved) > MOVED ? F(" counts (MOVES)") : F(" counts (still)"));
  return moved;
}

void detect() {
  Serial.println(F("# detecting - hands clear..."));
  delay(1000);
  long k01 = kick(0, 1);
  long k11 = kick(1, 1);
  long k10 = kick(1, 0);
  if (k01 == -99999 || k11 == -99999 || k10 == -99999) {
    coast();
    Serial.println(F("# FAILED: wheel never stopped between kicks - check wiring"));
    return;
  }
  bool m01 = labs(k01) > MOVED, m11 = labs(k11) > MOVED, m10 = labs(k10) > MOVED;

  int mode = -1;
  if (!m01 &&  m11 &&  m10) mode = 0;
  if ( m01 && !m11 &&  m10) mode = 1;
  if ( m01 &&  m11 && !m10) mode = 2;
  if (mode < 0) {
    Serial.println(F("# UNCLEAR: pattern doesn't match any mode. Nothing saved."));
    Serial.println(F("#   If nothing moved: battery on? nSLEEP wired to D7?"));
    return;
  }

  saveDriverMode(mode);
  driverMode = mode;
  Serial.print(F("# RESULT: mode ")); Serial.print(mode);
  Serial.print(F(" = ")); Serial.println(modeName(mode));
  Serial.println(F("# saved - the other test sketches will use it automatically"));

  // Check: a short forward pulse should move the wheel and then STOP.
  coast();
  waitStill();
  long start = readCount();
  drive(100);
  delay(300);
  brake();
  delay(500);
  long pulse = readCount() - start;
  delay(1000);
  long drift = readCount() - start - pulse;
  Serial.print(F("# check: forward pulse moved ")); Serial.print(pulse);
  Serial.print(F(" counts, then ")); Serial.print(drift);
  Serial.println(F(" counts in the 1 s after the brake"));
  if (labs(drift) > 20) {
    Serial.println(F("# PROBLEM: wheel kept moving after brake. Battery off and tell Claude."));
    coast();
  } else {
    Serial.println(F("# OK: brake works."));
    if (pulse < 0)
      Serial.println(F("# NOTE: forward pulse counted DOWN - swap the D2 and D3 encoder wires."));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_DRV_A, OUTPUT);
  pinMode(PIN_DRV_B, OUTPUT);
  pinMode(PIN_SLEEP, OUTPUT);
  coast();                         // motor off until you send 'd'
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  prevAB = ((PIND >> 2) & 1) << 1 | ((PIND >> 3) & 1);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);
  Serial.println(F("# driver mode test: d = detect, m = show saved mode"));
  Serial.print(F("# saved mode: ")); Serial.println(modeName(readDriverMode()));
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'd') detect();
  if (c == 'm') { Serial.print(F("# saved mode: ")); Serial.println(modeName(readDriverMode())); }
}
