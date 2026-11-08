// MQ Sentinel - see it, fire the laser (based on main_sprint2)
// Loop: read the sensors while still.
//   A0 over the threshold: laser on.
//   Otherwise turn 10 degrees towards the strongest of A1-A3 and look again.
//   None of them see it: turn 10 degrees the search way and look again.
// Nothing in loop() blocks: motor, gyro and steady must run every pass.

#include "analog_sensor.h"
#include "motor.h"
#include "gyro.h"
#include "steady.h"

// ---- laser ----
const int LASER_PIN = 12;
const unsigned long FIRING_MS = 2500;  // brief needs 2 s, 0.5 s spare for clock error

// ---- turning ----
const float TURN_STEP = 10;            // deg per turn
const float MAX_ANGLE = 360;           // deg either way, so the wire can't wind up
const byte STARTUP_READS = 4;          // 4 reads = 2 s before the first turn

// ---- fire rule ----
// A0 at or above this = on the beacon. 8 Nov S2: 96,000 fired across 11 deg,
// 110,000 across about 7 deg, 9,000 below the lowest real saturation (119,268).
const unsigned long FIRE_MIN_US = 110000;

// Unpowered sensors all read 119,820-119,980 and look saturated (8 Nov R5-R8)
const unsigned long NO_POWER_US = 119800;

// ---- aim check ----
const float FIRE_TOL = 1.0;            // deg, fire only this close to the held angle

// ---- direction rule ----
const unsigned long SEEN_MIN_US = 2000;  // at or above this = lit (blips reach ~1200)

// ---- debug printing ----
const bool DEBUG = true;
const bool SENSOR_TEST = false; // true = sensors and laser only: no motor or gyro
const char* SENSOR_NAMES[sensorNum] = { "A0", "A1", "A2", "A3" };

const int STARTUP_DELAY_MS = 5000;

// LEFT and RIGHT are the body's own sides, looking out along A0 and the laser.
// A person standing in front of A0 and facing it sees A1 on their left.
enum SensorId { FRONT, RIGHT, BACK, LEFT };

unsigned long ms_at_last_laser_fire = 0;
bool laser_firing = false;

bool still = false;                    // body has finished its turn
unsigned long stillSinceMs = 0;
int searchDir = 1;                     // + = anticlockwise (left), seen from above
byte readsLeft = STARTUP_READS;        // reads with no signal before the next turn

//---------------------------------------------------------------------------------------------------------
void fireLaser() {
  ms_at_last_laser_fire = millis();
  laser_firing = true;
  digitalWrite(LASER_PIN, HIGH);
}

void updateLaser() {
  if (millis() - ms_at_last_laser_fire >= FIRING_MS) {
    laser_firing = false;
    digitalWrite(LASER_PIN, LOW);
    stillSinceMs = millis();           // next decision needs a frame from after the shot
  }
}

void startTurn(float deg) {
  steadyStop();
  steadyTurnBy(deg);
  still = false;
}

void searchStep() {
  if (fabs(gyroAngle() + searchDir * TURN_STEP) > MAX_ANGLE) searchDir = -searchDir;
  startTurn(searchDir * TURN_STEP);
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

// All four at the no-power level: sensor power is lost, not a real reading
bool noPower(const SensorFrame &f) {
  for (int i = 0; i < sensorNum; i++)
    if (f.strength[i] < NO_POWER_US) return false;
  return true;
}

// SENSOR_TEST: print what the robot would do with this frame.
void printDecision(const SensorFrame &f) {
  SensorId side = strongestSide(f);
  Serial.print("\t-> ");
  Serial.println(f.strength[FRONT] >= FIRE_MIN_US ? "FIRE"
               : side == RIGHT ? "A1: TURN CLOCKWISE"
               : side == BACK  ? "A2: TURN (BACK)"
               : side == LEFT  ? "A3: TURN ANTICLOCKWISE"
               :                 "NONE");
}

void decide(const SensorFrame &f) {
  if (f.strength[FRONT] >= FIRE_MIN_US) {
    if (fabs(steadyHoldAngle() - gyroAngle()) < FIRE_TOL) {
      fireLaser();
    } else {
      // A0 sees it from here, so here is the target: hold this angle,
      // fire on the next frame taken once it is still
      steadyStop();
      steadyStartAt(gyroAngle());
      still = false;
    }
    return;
  }

  SensorId side = strongestSide(f);
  if (side == LEFT) {
    readsLeft = 1;
    startTurn(TURN_STEP);              // + = left
  } else if (side == RIGHT) {
    readsLeft = 1;
    startTurn(-TURN_STEP);
  } else if (side == BACK) {
    readsLeft = 1;
    searchStep();                      // either way round reaches it
  } else if (--readsLeft == 0) {
    readsLeft = 1;                     // after the start, one read per step
    searchStep();
  }
}

//---------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  setupSensor();  // power pin + sensor pins
  if (SENSOR_TEST) {
    Serial.println(F("SENSOR_TEST: A0 front, A1 body right, A2 back, A3 body left"));
    return;
  }

  motorBegin();   // motor pins, driver enable, encoder interrupts

  delay(STARTUP_DELAY_MS);

  if (!gyroBegin()) {
    Serial.println(F("No gyro, check A4/A5"));
    while (true) {}
  }
  gyroCalibrate(2000);  // satellite hanging STILL
  steadyStart();        // hold here, first decision comes once this is READY
}

//---------------------------------------------------------------------------------------------------------
void loop() {
  if (SENSOR_TEST) {
    sensorsUpdate();
    if (laser_firing) updateLaser();
    if (sensorsNewFrame()) {
      printFrame(sensorFrame());
      printDecision(sensorFrame());
      if (!laser_firing && sensorFrame().strength[FRONT] >= FIRE_MIN_US) fireLaser();
    }
    return;
  }

  motorUpdate();
  gyroUpdate();
  steadyUpdate();
  sensorsUpdate();

  if (!still && steadyReady()) {
    still = true;
    stillSinceMs = millis();
  }

  if (laser_firing) updateLaser();

  if (!sensorsNewFrame() || laser_firing) return;

  const SensorFrame &f = sensorFrame();
  if (!still || f.onStartMs < stillSinceMs) return;  // taken while turning

  if (DEBUG) {
    printFrame(f);
    Serial.print(F("\thold "));
    Serial.print(steadyHoldAngle(), 1);
    Serial.print(F(" gyro "));
    Serial.print(gyroAngle(), 1);
    printDecision(f);                    // prints "\t-> FIRE / A3: TURN ..." + newline
  }
  if (noPower(f)) {
    Serial.println(F("\tSENSOR POWER LOST - check D4"));
    return;
  }
  decide(f);
}
