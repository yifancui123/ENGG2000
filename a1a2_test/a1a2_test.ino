// ===== A1 / A2 FINE-AIM TEST =====
// Two front TSSP sensors, tilted apart, aligned with the laser:
//   A1 = LEFT sensor  -> Arduino pin A0   (variable OUT_L)
//   A2 = RIGHT sensor -> Arduino pin A1   (variable OUT_R)
//
//   OUT_L - OUT_R (difference) = DIRECTION: which way the beacon is off-centre.
//   OUT_L + OUT_R (sum)        = STRENGTH  ~ how on-axis / how close.
//
// Fine-aim rule:
//   L > R  -> beacon is LEFT  -> rotate left
//   R > L  -> beacon is RIGHT -> rotate right
//   L ~ R  -> CENTERED. If also strong enough -> LOCK (ok to fire).
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
//   A1 (left)  OUT -> A0
//   A2 (right) OUT -> A1
//   both VS -> D4 (shared power),  both GND -> GND

const int POWER_PIN = 4;      // shared VS for both sensors (toggled)
const int OUT_L     = A0;     // A1 = left  sensor output
const int OUT_R     = A1;     // A2 = right sensor output

const unsigned long OFF_MS = 380;   // powered OFF: resets the AGC
const unsigned long ON_MS  = 120;   // powered ON: detection window

// decision thresholds (us) -- PLACEHOLDERS, tune on the bench
const long          CENTER_TOL = 5000;    // |L-R| below this = centered
const unsigned long LOCK_MIN   = 40000;   // both above this = strong/on-axis

// Read one power-cycle: sL,sR = LOW-pulse width (us) = strength, 0 if unseen.
void readPair(unsigned long &sL, unsigned long &sR) {
  digitalWrite(POWER_PIN, LOW);
  delay(OFF_MS);                        // reset AGC
  digitalWrite(POWER_PIN, HIGH);        // power both up together

  unsigned long tStart   = micros();
  unsigned long windowUs = ON_MS * 1000UL;

  unsigned long sLow = 0, eLow = 0, sRgt = 0, eRgt = 0;
  bool lowL = false, doneL = false, lowR = false, doneR = false;

  while (micros() - tStart < windowUs) {
    unsigned long now = micros();
    bool l = (digitalRead(OUT_L) == LOW);
    bool r = (digitalRead(OUT_R) == LOW);

    if (l && !lowL && !doneL) { sLow = now; lowL = true; }   // left falling edge
    if (!l && lowL && !doneL) { eLow = now; doneL = true; }  // left rising edge
    if (r && !lowR && !doneR) { sRgt = now; lowR = true; }   // right falling edge
    if (!r && lowR && !doneR) { eRgt = now; doneR = true; }  // right rising edge

    if (doneL && doneR) break;
  }
  // went LOW but never rose (saturated) -> use window end
  if (lowL && !doneL) eLow = tStart + windowUs;
  if (lowR && !doneR) eRgt = tStart + windowUs;

  sL = lowL ? (eLow - sLow) : 0;
  sR = lowR ? (eRgt - sRgt) : 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);
  pinMode(OUT_L, INPUT);
  pinMode(OUT_R, INPUT);
  delay(300);
  Serial.println(F("# A1/A2 fine-aim test. diff = L-R (dir), sum = L+R (strength)."));
}

void loop() {
  unsigned long sL, sR;
  readPair(sL, sR);

  long          diff = (long)sL - (long)sR;   // >0: left stronger -> beacon LEFT
  unsigned long sum  = sL + sR;               // strength ~ on-axis / closeness

  bool centered = (abs(diff) < CENTER_TOL);
  bool strong   = (sL > LOCK_MIN && sR > LOCK_MIN);
  bool lockFire = centered && strong;

  Serial.print(F("L="));    Serial.print(sL);
  Serial.print(F(" R="));   Serial.print(sR);
  Serial.print(F(" diff=")); Serial.print(diff);
  Serial.print(F(" sum="));  Serial.print(sum);
  Serial.print(diff >  CENTER_TOL ? F("  -> LEFT")
             : diff < -CENTER_TOL ? F("  -> RIGHT")
             :                      F("  -> CENTERED"));
  Serial.println(lockFire ? F("  [LOCK]") : F(""));
}
