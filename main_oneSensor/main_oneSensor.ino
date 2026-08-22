#include "ir_sensor.h"
#include "motor_control.h"

const unsigned long laserTimer = 2000;

//laser pin
const int LASER_PIN = 12;  // change to whatever pin is actually connected

//ir receiver
const int SENSOR_PIN = 8;
//int sensorLength = 4;

//Spin Speed
const int SPIN_SPEED = 80;

//Countdown: if laser is on, but no beacon is found after 500ms (0.5s), turn off the laser
const unsigned long COUNTDOWN_MS = 500;

//last time seen the beacon
unsigned long lastSeen = 0;

//laser status (initially off)
bool laserOn = false;

/*
enum State { scanning,
             aligning,
             firing };
State laserState = scanning;
*/
// float targetAngle;
// float laserAngle;
// unsigned long holdStartTime;
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
  /*
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
      */
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
    if (sawBeacon(SENSOR_PIN[i])) {
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