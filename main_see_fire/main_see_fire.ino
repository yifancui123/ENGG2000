// MQ Sentinel - see it, fire the laser (based on main_sprint2)
// Loop: read the sensors while still -> A0 over the threshold: laser on,
// otherwise turn 10 degrees and look again.
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

// ---- fire rule ----
// Front pulse must fill at least 80% of the window (96,000 us, same as Sprint 2)
const unsigned long FIRE_MIN_US = (unsigned long)ON_MS * 1000UL * 8 / 10;

// ---- debug printing ----
const bool DEBUG = true;
const char* SENSOR_NAMES[sensorNum] = { "A0", "A1", "A2", "A3" };

const int STARTUP_DELAY_MS = 5000;

enum SensorId { FRONT, RIGHT, BACK, LEFT };

unsigned long ms_at_last_laser_fire = 0;
bool laser_firing = false;

bool still = false;                    // body has finished its turn
unsigned long stillSinceMs = 0;
int searchDir = 1;                     // + = anticlockwise (left), seen from above

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
  Serial.println();
}

void decide(const SensorFrame &f) {
  if (f.strength[FRONT] >= FIRE_MIN_US) {
    fireLaser();
  } else {
    searchStep();
  }
}

//---------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  setupSensor();  // power pin + sensor pins
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

  if (DEBUG) printFrame(f);
  decide(f);
}
