#include "ir_sensor.h"
#include "motor_control.h"

const unsigned long laserTimer = 2000;

//laser pin
const int LASER_PIN = 12;  // change to whatever pin is actually connected

//ir receiver
int sensorPins[4] = { 8, 9, 10, 11 };
int sensorLength = 4;

enum State { scanning,
             aligning,
             firing };
State laserState = scanning;

// float targetAngle;
// float laserAngle;
// unsigned long holdStartTime;
//---------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  pinMode(LASER_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  setupMotor();  // handles motorPWM/motorDirection/encoder pinMode + attachInterrupt, defined in motor_control.cpp

  for (int i = 0; i < sensorLength; i++) {
    pinMode(sensorPins[i], INPUT_PULLUP);
  }
}
//---------------------------------------------------------------------------------------------------------
void loop() {
  switch (laserState) {

    case scanning:
      {
        handleScanning();
        break;
      }

    case aligning:
      {
        handleAligning();
        break;
      }

    case firing:
      {
        handleFiring();
        break;
      }
  }
}
//---------------------------------------------------------------------------------------------------------
void fireLaser() {
  digitalWrite(LASER_PIN, HIGH);
  delay(laserTimer);
  digitalWrite(LASER_PIN, LOW);
}
//---------------------------------------------------------------------------------------------------------
void handleScanning() {
  int hitSensor = -1;

  for (int i = 0; i < sensorLength; i++) {
    if (sawBeacon(sensorPins[i])) {
      hitSensor = i;
      break;
    }
  }

  if (hitSensor == -1) {
    Serial.println("No beacon - keep scanning");
    rotateTo(45);
  } else {
    beaconDirection(hitSensor);
    laserState = aligning;
  }
}
//---------------------------------------------------------------------------------------------------------
void beaconDirection(int hitSensor) {
  if (hitSensor == 0) {
    Serial.println("Beacon in FRONT - fire laser!");
  } 
  else if (hitSensor == 1) {
    Serial.println("Beacon on RIGHT - rotate right");
    rotateTo(-90);
  } 
  else if (hitSensor == 2) {
    Serial.println("Beacon BEHIND - rotate around");
    rotateTo(180);
  } 
  else if (hitSensor == 3) {
    Serial.println("Beacon on LEFT - rotate left");
    rotateTo(90);
  }
}
//---------------------------------------------------------------------------------------------------------
void handleAligning(){
  //nothing yet
  laserState = firing;
}
//---------------------------------------------------------------------------------------------------------
void handleFiring(){
  fireLaser();
  laserState = scanning;
}
//---------------------------------------------------------------------------------------------------------