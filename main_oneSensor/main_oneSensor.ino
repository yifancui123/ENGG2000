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
  if(sawBeacon(SENSOR_PIN)){
    lastSeen = millis();
  }

  bool shouldBeOn = (lastSeen != 0) && (millis() - lastSeen < COUNTDOWN_MS);

  if (shouldBeOn && !laserOn) {
    digitalWrite(LASER_PIN, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Beacon in range - laser ON");
    laserOn = true;
  }else if (!shouldBeOn && laserOn){
    digitalWrite(LASER_PIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Beacon lost - laser OFF");
    laserOn = false;
  }
}