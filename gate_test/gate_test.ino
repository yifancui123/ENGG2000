// ===== TSSP58P38 GATE TEST =====
// GO / NO-GO for the 5-sensor design.
//
// Purpose: confirm the power-cycled TSSP gives a LOW-pulse width (= strength)
// that VARIES SMOOTHLY WITH ANGLE under the continuous 38 kHz beacon.
//   - If strength varies with angle  -> the A1/A2 fine-aim is viable. BUILD IT.
//   - If it's binary (0 or one fixed big number) or doesn't change with angle
//       -> the strength method is dead; rethink (digital + scan-midpoint).
//
// WHY power-cycle: a continuous 38 kHz carrier blinds the TSSP's AGC within a
// fraction of a second. Each power-down resets the AGC; each fresh power-up
// gives a short detection window whose LOW-pulse width encodes signal strength.
//
// HOW TO RUN (manual, no motor needed):
//   1. Fix the beacon ~1 m away. Mount the TSSP on a protractor.
//   2. Serial Monitor @ 115200.
//   3. Point the sensor straight at the beacon (0 deg) -> read pulse_us.
//   4. Rotate the sensor in steps (0, 5, 10, 20, 30, 40 deg, both ways).
//      Record pulse_us at each angle in the Test Data Record.
//   5. PASS  = pulse_us falls off smoothly as you go off-axis.
//      FAIL  = jumps between 0 and a fixed big number, or doesn't change.
//   Watch the min/avg/max summary too: a tight spread = low noise = usable.
//
// Wiring (ONE sensor):
//   TSSP VS  -> D4   (power pin, this sketch toggles it)
//   TSSP OUT -> A0
//   TSSP GND -> GND

const int POWER_PIN  = 4;      // shared VS pin (toggled on/off)
const int SENSOR_PIN = A0;     // TSSP OUT

const unsigned long OFF_MS = 380;   // powered OFF: resets the AGC
const unsigned long ON_MS  = 120;   // powered ON: detection window

// One power-cycle: returns the LOW-pulse width (us) = strength, 0 if unseen.
unsigned long readStrengthOnce() {
  digitalWrite(POWER_PIN, LOW);
  delay(OFF_MS);                          // reset AGC
  digitalWrite(POWER_PIN, HIGH);          // fresh power-up
  unsigned long tStart   = micros();
  unsigned long windowUs = ON_MS * 1000UL;

  while (digitalRead(SENSOR_PIN) == HIGH) {   // wait for detection (goes LOW)
    if (micros() - tStart > windowUs) return 0;   // never saw it this window
  }
  unsigned long lowStart = micros();
  while (digitalRead(SENSOR_PIN) == LOW) {    // time how long it stays LOW
    if (micros() - tStart > windowUs) break;  // window ended (saturated/clipped)
  }
  return micros() - lowStart;
}

// rolling min/avg/max so you can judge noise at each angle
const int N = 8;
unsigned long sum = 0, mn = 0xFFFFFFFF, mx = 0;
int count = 0;

void setup() {
  Serial.begin(115200);
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);
  pinMode(SENSOR_PIN, INPUT);
  delay(300);
  Serial.println(F("# TSSP gate test. pulse_us = strength. 0 = not seen."));
  Serial.println(F("# Rotate the sensor off-axis and watch pulse_us change."));
}

void loop() {
  unsigned long us = readStrengthOnce();
  Serial.print(F("pulse_us = "));
  Serial.println(us);

  sum += us;
  if (us < mn) mn = us;
  if (us > mx) mx = us;
  if (++count >= N) {
    Serial.print(F("   -> last ")); Serial.print(N);
    Serial.print(F(": min=")); Serial.print(mn);
    Serial.print(F(" avg=")); Serial.print(sum / N);
    Serial.print(F(" max=")); Serial.println(mx);
    sum = 0; mn = 0xFFFFFFFF; mx = 0; count = 0;
  }
}
