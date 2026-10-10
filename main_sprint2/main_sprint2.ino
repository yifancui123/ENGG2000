// MQ Sentinel - Sprint 2 main.ino
// Loop: motor burst -> stop -> read the sensor -> fire if the rule says so.

#include "analog_sensor.h"
#include "motor_control.h"

// ---- laser ----
const int LASER_PIN = 12;
const unsigned long FIRING_MS = 2000;  // how long the laser stays on per shot

// ---- motor steps (closed loop on the encoder) ----
const int   SCAN_SPEED    = 150;  // max PWM 0-255, normal scanning
const float SCAN_STEP_DEG = 10.0; // turret rotation per step

const int   STALL_SPEED    = 150; // used while stalling (after a shot)
const float STALL_STEP_DEG = 1.0; // creep slowly while the laser is on
const unsigned long STALL_MS = FIRING_MS;   // how long the stall lasts

// ---- fire rule ----
// Front pulse must fill at least 80% of the window (readings saturate near ON_MS*1000)
const unsigned long FIRE_MIN_US = (unsigned long)ON_MS * 1000UL * 8 / 10;
const unsigned long DOMINANCE_X = 2;  // front must be >= this many times any other sensor

// ---- debug printing ----
const bool DEBUG = true;
const char* SENSOR_NAMES[4] = { "Front", "Right", "Back", "Left" };

// latest readings: 0 = front, 1 = right, 2 = back, 3 = left (only sensorNum used)
unsigned long strength[sensorNum];

int STARTUP_DELAY_MS = 5000;

// laser state
unsigned long ms_at_last_laser_fire = 0;
bool laser_firing = false;

// stall state
unsigned long ms_at_stall_begin = 0;
bool isStalling = false;

//---------------------------------------------------------------------------------------------------------
bool shouldFire(const unsigned long s[sensorNum]) {
  if (s[0] < FIRE_MIN_US) return false;           // front sees nothing/too weak

  for (int i = 1; i < sensorNum; i++) {           // skipped when only 1 sensor
    if (s[0] < s[i] * DOMINANCE_X) return false;  // another side is as strong
  }
  return true;
}

void fireLaser() {
  // Serial.println("Firing the lazer.");   // leave off while using the Serial Plotter
  ms_at_last_laser_fire = millis();         // (re)start the laser timer
  laser_firing = true;
}

void updateLaser() {
  if (laser_firing && millis() - ms_at_last_laser_fire >= FIRING_MS) {
    laser_firing = false;
  }
  digitalWrite(LASER_PIN, laser_firing ? HIGH : LOW);
}

void beginStall() {
  ms_at_stall_begin = millis();             // (re)start the stall timer
  isStalling = true;
}

void updateStall() {
  if (isStalling && millis() - ms_at_stall_begin >= STALL_MS) {
    isStalling = false;
  }
}

//---------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  setupMotor();   // motor pins, driver enable, encoder interrupt
  setupSensor();  // power pin + sensor pins

  delay(STARTUP_DELAY_MS);
}

//---------------------------------------------------------------------------------------------------------
void loop() {
  // 1. motor step (small step while stalling, normal scan otherwise).
  //    Returns once the encoder says we're there and stopped.
  if (isStalling) {
    stepDegrees(STALL_STEP_DEG, STALL_SPEED);
  } else {
    stepDegrees(SCAN_STEP_DEG, SCAN_SPEED);
  }

  // 2. look (~50 ms per sensor)
  readStrengths(strength);

  if (DEBUG) {
    for (int i = 0; i < sensorNum; i++) {
      if (i > 0) Serial.print("\t");
      Serial.print(SENSOR_NAMES[i]);
      Serial.print(":");
      Serial.print(strength[i]);
    }
    Serial.println();
  }

  // 3. decide
  if (shouldFire(strength)) {
    fireLaser();
    beginStall();
  }
  updateLaser();
  updateStall();
}