// MQ Sentinel - see it, fire the laser (motor logic from simple_pd_fire)
// Loop: every sensor frame is checked for FIRE straight away (moving or not).
// Turns use only frames read once the body is on its hold angle and still.
//   A0 >= A0_SAT_US                                 -> laser ON
//   A0 >= FIRE_MIN_US and A0 >= the strongest side -> laser ON (stays on while
//                                                     A0 keeps seeing it, at least FIRING_MS)
//   A0 strongest but below FIRE_MIN_US             -> stay and hold, look again
//   A1 (right) / A3 (left) / A2 (back) strongest   -> turn -90 / +90 / 180
//   nothing seen                                    -> stay and hold
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
enum Decision { FIRE, WAIT, TURN_RIGHT, TURN_LEFT, TURN_BACK, NONE };

bool laserOn = false;
unsigned long laserUntilMs = 0;

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
  if (frontStrongest) return a0 >= FIRE_MIN_US ? FIRE : WAIT;
  if (side == RIGHT) return TURN_RIGHT;
  if (side == LEFT) return TURN_LEFT;
  if (side == BACK) return TURN_BACK;
  return NONE;
}

void printDecision(Decision d) {
  Serial.print("\t-> ");
  Serial.println(d == FIRE       ? "FIRE"
               : d == WAIT       ? "A0 STRONGEST, BELOW FIRE LEVEL: HOLD"
               : d == TURN_RIGHT ? "A1: TURN -90"
               : d == TURN_LEFT  ? "A3: TURN +90"
               : d == TURN_BACK  ? "A2: TURN 180"
               :                   "NONE: HOLD");
}

void decide(Decision d) {
  if (d == FIRE) fireLaser();          // A0 faces it: shoot (keeps holding)
  else if (d == TURN_RIGHT) pdTurnBy(-90);
  else if (d == TURN_LEFT) pdTurnBy(90);
  else if (d == TURN_BACK) pdTurnBy(180);
  // WAIT / NONE: stay and hold
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
    return;
  }
  if (!pdStill() || f.onStartMs < pdStillSinceMs()) return;  // turns only from readings taken on target

  // Prints off: it runs hanging with no USB.
  // if (DEBUG) {
  //   printFrame(f);
  //   Serial.print(F("\thold "));
  //   Serial.print(pdHoldAngle(), 1);
  //   Serial.print(F(" gyro "));
  //   Serial.print(gyroAngle(), 1);
  //   printDecision(d);
  // }
  decide(d);
}
