// ===== A0 / A1 FINE-AIM TEST =====
// Two front TSSP sensors, aligned with the laser:
//   A0 = LEFT sensor  -> Arduino pin A0   (variable OUT_L)
//   A1 = RIGHT sensor -> Arduino pin A1   (variable OUT_R)
//
//   OUT_L - OUT_R (difference) = DIRECTION: which way the beacon is off-centre.
//   OUT_L + OUT_R (sum)        = STRENGTH  ~ how on-axis / how close.
//
// Fine-aim rule (seen = reading >= SIGNAL_MIN, saturated still counts):
//   both seen   -> CENTERED
//   only L seen -> beacon is LEFT  -> rotate left
//   only R seen -> beacon is RIGHT -> rotate right
//   neither     -> NO SIGNAL
//
// Both sensors share ONE power pin and are power-cycled together: on each
// fresh power-up the AGC detects the continuous beacon for a short window
// whose LOW-pulse width = strength. We read BOTH pulse widths in the one
// ON window.
//
// NON-BLOCKING: the read is a small state machine, no delay():
//   OFF (380 ms, AGC reset) -> ON (up to 120 ms, timing the pulses) -> result
// Call a1a2Update() every loop(); a1a2Ready() turns true once per new
// reading. The power cycling runs whether or not anything is printed.
//
// HOW TO TEST (bench, no motor):
//   Set DEBUG_PRINT = true. Move the beacon left/right across the sensors.
//   Watch: diff changes sign, |diff| ~ 0 at centre, sum peaks on-axis.
//   Set DEBUG_PRINT = false for the final run (no Serial output).
//
// Wiring:
//   A0 (left)  OUT -> A0
//   A1 (right) OUT -> A1
//   both VS -> D4 (shared power),  both GND -> GND

const int POWER_PIN = 4;      // shared VS for both sensors (toggled)
const int OUT_L     = A0;     // A0 = left  sensor output
const int OUT_R     = A1;     // A1 = right sensor output

// true while testing the sensor range on the bench; false for the final run
const bool DEBUG_PRINT = true;

const unsigned long OFF_MS = 380;   // powered OFF: resets the AGC
const unsigned long ON_MS  = 120;   // powered ON: detection window

// at or above this = sensor sees the beacon (ambient blips are ~0-700 us)
const unsigned long SIGNAL_MIN = 2000;

// ---- read state machine ----
enum ReadState { R_OFF, R_ON };
ReadState     rState = R_OFF;
unsigned long offStart = 0;               // millis() when power went OFF
unsigned long tStart   = 0;               // micros() when power went ON
unsigned long sLow = 0, eLow = 0, sRgt = 0, eRgt = 0;
bool lowL = false, doneL = false, lowR = false, doneR = false;

unsigned long lastL = 0, lastR = 0;       // latest finished reading (us)
bool          fresh = false;              // true until a1a2Get() takes it

// Start a read cycle: power OFF so the AGC resets.
void a1a2Begin() {
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);
  pinMode(OUT_L, INPUT);
  pinMode(OUT_R, INPUT);
  rState   = R_OFF;
  offStart = millis();
}

// Call every loop(). Never waits: does one small step and returns.
void a1a2Update() {
  if (rState == R_OFF) {
    if (millis() - offStart < OFF_MS) return;  // still resetting the AGC
    digitalWrite(POWER_PIN, HIGH);             // power both up together
    tStart = micros();
    sLow = eLow = sRgt = eRgt = 0;
    lowL = doneL = lowR = doneR = false;
    rState = R_ON;
    return;
  }

  // R_ON: sample both outputs once and look for the edges
  unsigned long now      = micros();
  unsigned long windowUs = ON_MS * 1000UL;
  bool l = (digitalRead(OUT_L) == LOW);
  bool r = (digitalRead(OUT_R) == LOW);

  if (l && !lowL && !doneL) { sLow = now; lowL = true; }   // left falling edge
  if (!l && lowL && !doneL) { eLow = now; doneL = true; }  // left rising edge
  if (r && !lowR && !doneR) { sRgt = now; lowR = true; }   // right falling edge
  if (!r && lowR && !doneR) { eRgt = now; doneR = true; }  // right rising edge

  bool timeUp = (now - tStart >= windowUs);
  if (!(doneL && doneR) && !timeUp) return;               // keep timing

  // went LOW but never rose (saturated) -> use window end
  if (lowL && !doneL) eLow = tStart + windowUs;
  if (lowR && !doneR) eRgt = tStart + windowUs;

  lastL = lowL ? (eLow - sLow) : 0;
  lastR = lowR ? (eRgt - sRgt) : 0;
  fresh = true;

  digitalWrite(POWER_PIN, LOW);                            // next AGC reset
  rState   = R_OFF;
  offStart = millis();
}

// True once per finished reading.
bool a1a2Ready() { return fresh; }

// Take the latest reading: sL,sR = LOW-pulse width (us) = strength, 0 if unseen.
void a1a2Get(unsigned long &sL, unsigned long &sR) {
  sL = lastL;
  sR = lastR;
  fresh = false;
}

void setup() {
  if (DEBUG_PRINT) {
    Serial.begin(115200);
    Serial.println(F("# A0/A1 fine-aim test. A0 = left (pin A0), A1 = right (pin A1)."));
    Serial.println(F("# diff = L-R, e = (L-R)/(L+R)."));
  }
  a1a2Begin();
}

void loop() {
  a1a2Update();                  // every loop: keeps power-cycling the sensors
  if (!a1a2Ready()) return;

  unsigned long sL, sR;
  a1a2Get(sL, sR);

  long          diff = (long)sL - (long)sR;   // >0: left stronger -> beacon LEFT
  unsigned long sum  = sL + sR;               // strength ~ on-axis / closeness

  // seen / not seen: works even when a sensor saturates
  bool seenL = (sL >= SIGNAL_MIN);
  bool seenR = (sR >= SIGNAL_MIN);

  // normalized error: distance-independent, 0 when both equal
  float e = (sum > 0) ? (float)diff / (float)sum : 0.0;

  if (!DEBUG_PRINT) return;      // final run: read, but print nothing

  Serial.print(F("A0(L)="));   Serial.print(sL);
  Serial.print(F("  A1(R)=")); Serial.print(sR);
  Serial.print(F("  diff="));  Serial.print(diff);
  Serial.print(F("  e="));     Serial.print(e, 4);
  Serial.println(seenL && seenR ? F("  -> CENTERED")
               : seenL          ? F("  -> LEFT")
               : seenR          ? F("  -> RIGHT")
               :                  F("  -> NO SIGNAL"));
}
