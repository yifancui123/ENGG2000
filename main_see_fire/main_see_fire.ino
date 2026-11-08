// MQ Sentinel - see it, fire the laser (motor logic from simple_pd_fire)
// Loop: every sensor frame is checked for FIRE straight away (moving or not).
// Turns use only frames read once the body is on its hold angle and still.
//   A0 >= A0_SAT_US                                 -> laser ON
//   A0 >= FIRE_MIN_US and A0 >= the strongest side -> laser ON (stays on while
//                                                     A0 keeps seeing it, at least FIRING_MS)
//   A0 strongest but below FIRE_MIN_US             -> fine search: 2 deg steps, hill-climb on A0
//   A1 (right) / A3 (left) / A2 (back) strongest   -> turn -90 / +90 / 180 (45 if ping-pong)
//   nothing seen for 2 s (4 still frames)           -> turn 45 and look again
// Nothing in loop() blocks: gyro, PD and sensors must run every pass.
// Power on with the satellite HANGING and STILL (gyro calibrates for 2 s, LED on).

#include "analog_sensor.h"
#include "gyro.h"
#include "pd_hold.h"

// ---- laser ----
const int LASER_PIN = 12;
const int LED = 13;
const unsigned long FIRING_MS = 2500;  // brief needs 2 s, 0.5 s spare for clock error

// ---- fire rule ----
// A0 at or above this = on the beacon. 8 Nov S2: 96,000 fired across 11 deg,
// 110,000 across about 7 deg, 9,000 below the lowest real saturation (119,268).
const unsigned long FIRE_MIN_US = 110000;

// A0 at or above this is saturated: on the beacon whatever the sides read.
// Close up a side sensor saturates too and can beat A0 by noise (119,268-119,760).
const unsigned long A0_SAT_US = 119000;

// ---- fine search (A0 strongest but below FIRE_MIN_US) ----
const float FINE_STEP = 2;             // deg per step (A0's saturated top is about 6 deg wide)
const byte FINE_MAX = 15;              // give up after 15 steps (30 deg), start again next frame
const float PINGPONG_TURN = 45;        // coarse turn when the last one went the other way

// ---- empty search (nothing seen) ----
const byte EMPTY_FRAMES = 4;           // still frames with nothing seen before a search turn (2 s)
const float SEARCH_TURN = 45;          // sensors are 90 apart: a 45 turn puts one on every blind gap

// ---- direction rule ----
const unsigned long SEEN_MIN_US = 2000;  // at or above this = lit (blips reach ~1200)

// ---- debug printing ----
const bool DEBUG = false;   // prints are commented out: runs hanging with no USB
const bool SENSOR_TEST = false; // true = sensors and laser only: no motor or gyro
const char* SENSOR_NAMES[sensorNum] = { "A0", "A1", "A2", "A3" };

const int SETTLE_DELAY_MS = 3000;      // hands off, let it stop swinging

// ---- gyro calibration check ----
const float GYRO_STILL_DEG = 1.0;      // deg moved in the 1 s check (string twist is about 0.5)
const byte GYRO_TRIES = 5;

// LEFT and RIGHT are the body's own sides, looking out along A0 and the laser.
// A person standing in front of A0 and facing it sees A1 on their left.
enum SensorId { FRONT, RIGHT, BACK, LEFT };
enum Decision { FIRE, FINE, TURN_RIGHT, TURN_LEFT, TURN_BACK, NONE };

bool laserOn = false;
unsigned long laserUntilMs = 0;

float fineDir = 0;                     // +FINE_STEP / -FINE_STEP, 0 = not fine searching
unsigned long lastA0 = 0;              // A0 at the last fine step
byte fineSteps = 0;
int lastCoarse = 0;                    // last 90 turn: +1 left, -1 right, 0 none
byte emptyFrames = 0;                  // still frames in a row with nothing seen
int searchDir = 1;                     // +1 anticlockwise, flips at the wire limit

//---------------------------------------------------------------------------------------------------------
void fireLaser() {
  // if (!laserOn) Serial.println(F("FIRE"));
  laserOn = true;
  laserUntilMs = millis() + FIRING_MS;
  digitalWrite(LASER_PIN, HIGH);
}

void updateLaser() {
  if (laserOn && (long)(millis() - laserUntilMs) >= 0) {
    laserOn = false;
    digitalWrite(LASER_PIN, LOW);
  }
}

void printFrame(const SensorFrame &f) {
  for (int i = 0; i < sensorNum; i++) {
    if (i > 0) Serial.print("\t");
    Serial.print(SENSOR_NAMES[i]);
    Serial.print(":");
    Serial.print(f.strength[i]);
  }
}

// The lit sensor with the highest reading among A1-A3, or FRONT if none is lit.
SensorId strongestSide(const SensorFrame &f) {
  SensorId best = FRONT;
  unsigned long bestStrength = 0;
  for (int i = RIGHT; i <= LEFT; i++) {
    if (f.strength[i] >= SEEN_MIN_US && f.strength[i] > bestStrength) {
      best = (SensorId)i;
      bestStrength = f.strength[i];
    }
  }
  return best;
}

Decision choose(const SensorFrame &f) {
  SensorId side = strongestSide(f);
  unsigned long a0 = f.strength[FRONT];
  if (a0 >= A0_SAT_US) return FIRE;    // saturated A0 always fires
  bool frontStrongest = a0 >= SEEN_MIN_US && (side == FRONT || a0 >= f.strength[side]);
  if (frontStrongest) return a0 >= FIRE_MIN_US ? FIRE : FINE;
  if (side == RIGHT) return TURN_RIGHT;
  if (side == LEFT) return TURN_LEFT;
  if (side == BACK) return TURN_BACK;
  return NONE;
}

void printDecision(Decision d) {
  Serial.print("\t-> ");
  Serial.println(d == FIRE       ? "FIRE"
               : d == FINE       ? "A0 STRONGEST, BELOW FIRE LEVEL: FINE STEP 2"
               : d == TURN_RIGHT ? "A1: TURN -90"
               : d == TURN_LEFT  ? "A3: TURN +90"
               : d == TURN_BACK  ? "A2: TURN 180"
               :                   "NONE: SEARCH 45 AFTER 2 s");
}

// A0 strongest but weak: step 2 deg. Start towards the brighter of A3 (left) / A1 (right),
// then hill-climb: if A0 got weaker than at the last step, turn the other way.
void fineStep(const SensorFrame &f) {
  unsigned long a0 = f.strength[FRONT];
  if (fineDir == 0) {
    fineDir = f.strength[LEFT] >= f.strength[RIGHT] ? FINE_STEP : -FINE_STEP;
    fineSteps = 0;
  } else if (a0 < lastA0) {
    fineDir = -fineDir;
  }
  lastA0 = a0;
  if (++fineSteps > FINE_MAX) {        // not found in 30 deg: stop, start again next frame
    fineDir = 0;
    return;
  }
  pdTurnBy(fineDir);
}

// Side sensor strongest: turn 90 to it, but only 45 if the last coarse turn
// went the other way (beacon near 45 deg: stops the left-right ping-pong)
void coarseTurn(int dir) {
  float deg = (lastCoarse == -dir) ? PINGPONG_TURN : 90;
  lastCoarse = dir;
  pdTurnBy(dir * deg);
}

// Nothing seen for EMPTY_FRAMES still frames: turn 45 the search way.
// Turn round at +-360 so the wire can't wind up (not a 315 turn the other way).
void emptySearch() {
  if (++emptyFrames < EMPTY_FRAMES) return;
  emptyFrames = 0;
  if (fabs(pdHoldAngle() + searchDir * SEARCH_TURN) > 360) searchDir = -searchDir;
  pdTurnBy(searchDir * SEARCH_TURN);
}

// Any decision that isn't a fine step / empty frame ends that search.
void decide(Decision d, const SensorFrame &f) {
  if (d != FINE) fineDir = 0;
  if (d != NONE) emptyFrames = 0;
  if (d != TURN_LEFT && d != TURN_RIGHT) lastCoarse = 0;

  if (d == FINE) fineStep(f);
  else if (d == TURN_RIGHT) coarseTurn(-1);
  else if (d == TURN_LEFT) coarseTurn(+1);
  else if (d == TURN_BACK) pdTurnBy(180);
  else if (d == NONE) emptySearch();
}

// Calibrate until the gyro reads still: after each 2 s calibration (LED on),
// watch 1 s. Moved more than GYRO_STILL_DEG -> the body was moving while
// calibrating: 3 short LED blinks, then calibrate again (up to GYRO_TRIES).
void calibrateStill() {
  for (byte tries = 0; tries < GYRO_TRIES; tries++) {
    digitalWrite(LED, HIGH);
    gyroCalibrate(2000);               // HANGING and STILL
    digitalWrite(LED, LOW);
    unsigned long t0 = millis();
    while (millis() - t0 < 1000) gyroUpdate();   // 1 s check
    if (fabs(gyroAngle()) < GYRO_STILL_DEG) break;
    for (byte i = 0; i < 3; i++) {     // moved: blink, try again
      digitalWrite(LED, HIGH);
      delay(150);
      digitalWrite(LED, LOW);
      delay(150);
    }
  }
  gyroResetAngle();
}

//---------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  pinMode(LED, OUTPUT);

  setupSensor();  // power pin + sensor pins
  if (SENSOR_TEST) {
    Serial.println(F("SENSOR_TEST: A0 front, A1 body right, A2 back, A3 body left"));
    return;
  }

  pdBegin();      // motor pins, brake, encoder interrupts

  if (!gyroBegin()) {
    // Serial.println(F("No gyro, check A4/A5"));
    while (true) {                     // no gyro: LED blinks fast
      digitalWrite(LED, !digitalRead(LED));
      delay(100);
    }
  }
  delay(SETTLE_DELAY_MS);
  calibrateStill();     // satellite hanging STILL, redone if it was moving
  pdHoldHere();
  // Serial.println(F("main_see_fire started"));
}

//---------------------------------------------------------------------------------------------------------
void loop() {
  sensorsUpdate();
  updateLaser();

  if (SENSOR_TEST) {
    if (sensorsNewFrame()) {
      const SensorFrame &f = sensorFrame();
      printFrame(f);
      Decision d = choose(f);
      printDecision(d);
      if (d == FIRE) fireLaser();
    }
    return;
  }

  gyroUpdate();
  pdUpdate();

  if (!sensorsNewFrame()) return;
  const SensorFrame &f = sensorFrame();

  Decision d = choose(f);
  if (d == FIRE) {                     // A0 high: shoot, whatever the body is doing
    fireLaser();
    fineDir = 0;
    lastCoarse = 0;
    emptyFrames = 0;
    return;
  }
  if (!pdStill() || f.onStartMs < pdStillSinceMs()) return;  // moves only from readings taken on target

  // Prints off: it runs hanging with no USB.
  // if (DEBUG) {
  //   printFrame(f);
  //   Serial.print(F("\thold "));
  //   Serial.print(pdHoldAngle(), 1);
  //   Serial.print(F(" gyro "));
  //   Serial.print(gyroAngle(), 1);
  //   printDecision(d);
  // }
  decide(d, f);
}
