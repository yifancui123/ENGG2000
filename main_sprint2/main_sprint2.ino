// MQ Sentinel - Sprint 2 main.ino
// Loop (once per CYCLE_MS): motor burst -> stop -> read the sensors -> fire if the rule says so.

#include "analog_sensor.h"
#include "motor_control.h"

// ---- laser ----
const int LASER_PIN = 12;
const unsigned long FIRING_MS = 2000;  // how long the laser stays on per shot

// ---- motor burst ----
const int STALL_SPEED = 100;  // PWM 0-255
const int STALL_PULSE_MS = 10;
const int STALL_SETTLE_MS = 10;  


const int SCAN_SPEED = 100;   // PWM 0-255
const int PULSE_MS = 50;         // how long the motor runs each burst (smaller = finer step)

const int SETTLE_MS = 60;             // let the turret stop moving before reading
const unsigned long CYCLE_MS = 1000;  // one burst + one reading per second

// ---- fire rule (TUNE THESE from the Serial Plotter values) ----
// Front pulse must fill at least 80% of the window (readings saturate near ON_MS*1000)
const unsigned long FIRE_MIN_US = (unsigned long)ON_MS * 1000UL * 8 / 10;
const unsigned long DOMINANCE_X = 2;  // front must be >= this many times any other sensor

// ---- startup ----
const unsigned long STARTUP_DELAY_MS = 10000;

// ---- debug printing (turn off once tuned) ----
const bool DEBUG = true;
const char* SENSOR_NAMES[4] = { "Front", "Right", "Back", "Left" };

// latest readings: 0 = front, 1 = right, 2 = back, 3 = left (only sensorNum used)
unsigned long strength[sensorNum];

//---------------------------------------------------------------------------------------------------------
// Fire only if the FRONT sensor clearly sees the beacon and, when other
// sensors are wired, it dominates them (i.e. we are pointing at it).
bool shouldFire(const unsigned long s[sensorNum]) {
  if (s[0] < FIRE_MIN_US) return false;  // front sees nothing/too weak

  for (int i = 1; i < sensorNum; i++) {           // skipped when only 1 sensor
    if (s[0] < s[i] * DOMINANCE_X) return false;  // another side is as strong
  }
  return true;
}
int ms_at_last_laser_fire{};

bool laser_firing = false;

void updateLaser() {
  int ms_since_laser_fired = millis() - ms_at_last_laser_fire;
  if (ms_since_laser_fired < FIRING_MS) {
    laser_firing = true;
  } else {
    laser_firing = false;
  }
  if (laser_firing) {
    digitalWrite(LASER_PIN, HIGH);
  } else {
    digitalWrite(LASER_PIN, LOW);
  }
}
void fireLaser() {
  Serial.println("Firing the lazer.");
  ms_at_last_laser_fire = millis();
}

//---------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  setupMotor();   // motor pins, driver enable, encoder interrupt (motor_control.cpp)
  setupSensor();  // power pin + sensor pins (analog_sensor.cpp)

  //delay(STARTUP_DELAY_MS);
}
bool isStalling = false;
int ms_at_stall_begin = 0;
void beginStall() {
  ms_at_stall_begin = millis();
}
void updateStall() {
  auto ms_since_stall_begin = millis() - ms_at_stall_begin;
  if (ms_since_stall_begin < FIRING_MS) {
    isStalling = true;
  } else {
    isStalling = false;
  }
}
//---------------------------------------------------------------------------------------------------------
void loop() {
  unsigned long cycleStart = millis();

  // 1. motor burst
  if (laser_firing) {
    motorForward(STALL_SPEED);
    delay(STALL_PULSE_MS);
    motorStop();
    delay(STALL_SETTLE_MS);

  } else {
    motorForward(SCAN_SPEED);
    delay(PULSE_MS);
    motorStop();
    delay(SETTLE_MS);
  }

  // 2. look: the motor is stopped while we read (~50 ms per sensor)
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
  // if no sensor, rotate

  // 3. decide
  if (shouldFire(strength)) {
    fireLaser();
    beginStall();
  }
  updateLaser();
  updateStall();

  // 4. wait out the rest of the cycle
  unsigned long elapsed = millis() - cycleStart;
  //if (elapsed < CYCLE_MS) delay(CYCLE_MS - elapsed);
}