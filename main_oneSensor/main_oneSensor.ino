#include "ir_sensor.h"
#include "motor_control.h"

const unsigned long laserTimer = 2000;

//laser pin
const int LASER_PIN = 12;  // change to whatever pin is actually connected

//ir receiver
const int SENSOR_PIN = 8;

//Spin Speed
const int SPIN_SPEED = 80;

//Countdown: if laser is on, but no beacon is found after 500ms (0.5s), turn off the laser
const unsigned long COUNTDOWN_MS = 500;

//last time seen the beacon
unsigned long lastSeen = 0;

//when beacon was first detected
unsigned long firstSeen = 0;

//wait this long before firing (2s)
const unsigned long FIRE_DELAY_MS = 2000;

//laser status (initially off)
bool laserOn = false;

//---------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  pinMode(LASER_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  setupMotor();  // handles motorPWM/motorDirection/encoder pinMode + attachInterrupt, defined in motor_control.cpp

  digitalWrite(LASER_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  //start spinning slowly in ONE direction and never stop
  motorForward(SPIN_SPEED);
}
//---------------------------------------------------------------------------------------------------------
void loop() {
  if (sawBeacon(SENSOR_PIN)) {
    lastSeen = millis();
    if (firstSeen == 0) {
      firstSeen = millis();
    }
  }

  bool beaconPresent = (lastSeen != 0) && (millis() - lastSeen < COUNTDOWN_MS);
  bool delayElapsed = (firstSeen != 0) && (millis() - firstSeen >= FIRE_DELAY_MS);

  bool shouldBeOn = beaconPresent && delayElapsed;

  if (shouldBeOn && !laserOn) {
    digitalWrite(LASER_PIN, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Beacon in range - laser ON");
    laserOn = true;
  } else if (!beaconPresent && laserOn) {
    digitalWrite(LASER_PIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Beacon lost - laser OFF");
    laserOn = false;
    // reset so next detection starts fresh delay
    firstSeen = 0;
  } else if (!beaconPresent && firstSeen != 0) {
    // beacon lost before delay even finished
    firstSeen = 0;
  }
}