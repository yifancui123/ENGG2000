// ===== A1 / A2 FINE-AIM TEST =====
// Two front TSSP sensors (A1, A2), tilted apart, aligned with the laser.
//   A1 - A2 (difference) = DIRECTION: which way the beacon is off-centre.
//   A1 + A2 (sum)        = STRENGTH  ~ how on-axis / how close ("distance-ish").
//
// Fine-aim rule:
//   A1 > A2  -> beacon is LEFT  -> rotate left
//   A2 > A1  -> beacon is RIGHT -> rotate right
//   A1 ~ A2  -> CENTERED. If also strong enough -> LOCK (ok to fire).
//
// Both sensors share ONE power pin and are power-cycled together: on each
// fresh power-up the AGC detects the continuous beacon for a short window
// whose LOW-pulse width = strength. We read BOTH pulse widths in the one
// ON window.
//
// HOW TO TEST (bench, no motor):
//   Beacon ~1 m away. Move it left/right across the two sensors.
//   Watch: diff changes sign, |diff| ~ 0 at centre, sum peaks on-axis.
//   Tune CENTER_TOL and LOCK_MIN from the printed numbers.
//
// Wiring:
//   A1 OUT -> A0,  A2 OUT -> A1 (the Arduino A1 pin),  shared VS -> D4,  GND -> GND

const int POWER_PIN = 4;      // shared VS for both sensors (toggled)
const int A1_OUT    = A0;     // sensor A1 output
const int A2_OUT    = A1;     // sensor A2 output (Arduino pin A1)

const unsigned long OFF_MS = 380;   // powered OFF: resets the AGC
const unsigned long ON_MS  = 120;   // powered ON: detection window

// decision thresholds (us) -- PLACEHOLDERS, tune on the bench
const long          CENTER_TOL = 5000;    // |a1-a2| below this = centered
const unsigned long LOCK_MIN   = 40000;   // both above this = strong/on-axis

// Read one power-cycle: a1,a2 = LOW-pulse width (us) = strength, 0 if unseen.
void readA1A2(unsigned long &a1, unsigned long &a2) {
  digitalWrite(POWER_PIN, LOW);
  delay(OFF_MS);                        // reset AGC
  digitalWrite(POWER_PIN, HIGH);        // power both up together

  unsigned long tStart   = micros();
  unsigned long windowUs = ON_MS * 1000UL;

  unsigned long s1 = 0, e1 = 0, s2 = 0, e2 = 0;
  bool low1 = false, done1 = false, low2 = false, done2 = false;

  while (micros() - tStart < windowUs) {
    unsigned long now = micros();
    bool l1 = (digitalRead(A1_OUT) == LOW);
    bool l2 = (digitalRead(A2_OUT) == LOW);

    if (l1 && !low1 && !done1) { s1 = now; low1 = true; }   // A1 falling edge
    if (!l1 && low1 && !done1) { e1 = now; done1 = true; }  // A1 rising edge
    if (l2 && !low2 && !done2) { s2 = now; low2 = true; }   // A2 falling edge
    if (!l2 && low2 && !done2) { e2 = now; done2 = true; }  // A2 rising edge

    if (done1 && done2) break;
  }
  // went LOW but never rose (saturated) -> use window end
  if (low1 && !done1) e1 = tStart + windowUs;
  if (low2 && !done2) e2 = tStart + windowUs;

  a1 = low1 ? (e1 - s1) : 0;
  a2 = low2 ? (e2 - s2) : 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);
  pinMode(A1_OUT, INPUT);
  pinMode(A2_OUT, INPUT);
  delay(300);
  Serial.println(F("# A1/A2 fine-aim test. diff = A1-A2 (dir), sum = A1+A2 (strength)."));
}

void loop() {
  unsigned long a1, a2;
  readA1A2(a1, a2);

  long          diff = (long)a1 - (long)a2;   // >0: A1 stronger -> beacon LEFT
  unsigned long sum  = a1 + a2;               // strength ~ on-axis / closeness

  bool centered = (abs(diff) < CENTER_TOL);
  bool strong   = (a1 > LOCK_MIN && a2 > LOCK_MIN);
  bool lockFire = centered && strong;

  Serial.print(F("A1="));   Serial.print(a1);
  Serial.print(F(" A2="));  Serial.print(a2);
  Serial.print(F(" diff=")); Serial.print(diff);
  Serial.print(F(" sum="));  Serial.print(sum);
  Serial.print(diff >  CENTER_TOL ? F("  -> LEFT")
             : diff < -CENTER_TOL ? F("  -> RIGHT")
             :                      F("  -> CENTERED"));
  Serial.println(lockFire ? F("  [LOCK]") : F(""));
}
