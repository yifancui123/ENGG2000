// MQ Sentinel - see it, fire the laser (based on main_sprint2)
// Loop: motor burst -> stop -> read the front sensor -> if seen, laser on.
// While the laser is on the motor stays off: no bursts and no sensor reads.

#include "analog_sensor.h"
#include "motor_control.h"

// ---- laser ----
const int LASER_PIN = 12;
const unsigned long FIRING_MS = 2500;  // brief needs 2 s, 0.5 s spare for clock error

// ---- motor bursts ----
const int SCAN_SPEED = 150;       // PWM 0-255, normal scanning
const int PULSE_MS   = 80;        // how long the motor runs each burst
const int SETTLE_MS  = 20;        // let the turret stop moving before reading

// ---- fire rule ----
// Front pulse must fill at least 80% of the window (96,000 us, same as Sprint 2)
const unsigned long FIRE_MIN_US = (unsigned long)ON_MS * 1000UL * 8 / 10;

// ---- debug printing ----
const bool DEBUG = true;

const int STARTUP_DELAY_MS = 5000;

unsigned long strength[sensorNum];     // strength[0] = front sensor

unsigned long ms_at_last_laser_fire = 0;
bool laser_firing = false;

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
  }
}

//---------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  setupMotor();   // motor pins, driver enable, encoder interrupt
  setupSensor();  // power pin + sensor pin

  delay(STARTUP_DELAY_MS);
}

//---------------------------------------------------------------------------------------------------------
void loop() {
  // laser on: leave the motor off until the shot is finished
  if (laser_firing) {
    updateLaser();
    return;
  }

  // 1. motor burst
  motorForward(SCAN_SPEED);
  delay(PULSE_MS);
  motorStop();
  delay(SETTLE_MS);

  // 2. look (one power cycle, about 500 ms)
  readStrengths(strength);

  if (DEBUG) {
    Serial.print("Front:");
    Serial.println(strength[0]);
  }

  // 3. seen -> fire
  if (strength[0] >= FIRE_MIN_US) {
    fireLaser();
  }
}
